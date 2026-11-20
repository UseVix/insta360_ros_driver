#!/usr/bin/env python3
"""Sweep camera driver settings and record the USB latency for each combination.

For every combination of resolution x bitrate x exposure, the driver is started
with measure_latency:=true, the first report windows are discarded as warm-up,
the following ones are parsed from the log, and the driver is stopped with SIGINT
(so it calls StopLiveStreaming/Close) before the next run.

Each run also checks the incoming stream against the request: the resolution
from the stream's SPS, the frame rate from camera timestamps and the bitrate
from bytes received. At the end, a summary says whether each setting is
honoured by the camera (also written to <out>_format_check.txt).

Outputs:
  <out>.csv               one summary row per configuration (appended after every run)
  <out>_windows.csv       one row per report window (raw per-window stats)
  <out>_format_check.txt  whether the camera honours resolution / fps / bitrate
  <out>_logs/             full driver log of every run

Rows already in <out>.csv are skipped, so an interrupted sweep can be resumed by
re-running the same command. Runs that did not finish cleanly are retried.

Example:
  ros2 run insta360_ros_driver latency_sweep.py --out ~/latency_sweep
  ros2 run insta360_ros_driver latency_sweep.py --resolutions 1920x960p30 1440x720p30 \
      --bitrates 524288 4194304 --exposures auto shutter:120 shutter:240 --dry-run
  ros2 run insta360_ros_driver latency_sweep.py --out ~/latency_sweep --summary-only
"""

import argparse
import csv
import itertools
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

DEFAULT_RESOLUTIONS = [
    "3840x1920p30", "2560x1280p30", "2304x1152p30", "1920x960p30",
    "1440x720p30", "960x480p30", "3840x1920p60", "2560x1280p60",
]
DEFAULT_BITRATES = [524288, 2097152, 8388608, 20971520]  # 0.5, 2, 8, 20 Mbit/s
# "auto" is the baseline instead of "unchanged": the camera keeps the last exposure
# settings between runs, so "unchanged" would just repeat the previous run's exposure.
DEFAULT_EXPOSURES = ["auto", "shutter:30", "shutter:60", "shutter:100", "shutter:120",
                     "shutter:240", "shutter:500", "shutter:1000"]

STREAM_RE = re.compile(
    r"\[latency\] stream (?P<stream>\d+) \| cam->callback ms: "
    r"min=(?P<lat_min>\S+) mean=(?P<lat_mean>\S+) p50=(?P<lat_p50>\S+) p99=(?P<lat_p99>\S+) max=(?P<lat_max>\S+) \| "
    r"inter-arrival ms: min=(?P<ia_min>\S+) mean=(?P<ia_mean>\S+) p50=(?P<ia_p50>\S+) p99=(?P<ia_p99>\S+) max=(?P<ia_max>\S+)")
FORMAT_RE = re.compile(
    r"\[stream\] stream (?P<stream>\d+) \| resolution=(?P<resolution>\S+) requested=\S+ \w+ \| "
    r"fps=(?P<fps>\S+) requested=\S+ \w+ \| bitrate=(?P<bitrate>\S+) requested=\S+ ratio=\S+ \w+ \| "
    r"sps_fps=(?P<sps_fps>\S+) frames=(?P<frames>\d+) keyframes=(?P<keyframes>\d+) "
    r"max_packet_bytes=(?P<max_packet_bytes>\d+)")
SYNC_RE = re.compile(r"\[latency\] clock sync: offset=(?P<offset>\S+) ms, sync rtt=(?P<rtt>\S+) ms")
EXPOSURE_NOW_RE = re.compile(r"\[latency\] current exposure_time=(?P<exp>\S+) s")
EXPOSURE_FIRST_RE = re.compile(r"First exposure report from camera: exposure_time=(?P<exp>\S+) s")
EXPOSURE_APPLIED_RE = re.compile(
    r"camera reports mode=(?P<mode>-?\d+) shutter=(?P<shutter>\S+) s iso=(?P<iso>-?\d+) ev=(?P<ev>-?\d+)")
RES_NAME_RE = re.compile(r"^(\d+)x(\d+)p(\d+)$")
STARTED_MARK = "Live streaming started."
FATAL_MARKS = {
    "Failed to start live streaming": "start_failed",
    "No available camera devices found": "no_camera",
    "Failed to open camera": "open_failed",
    "Unknown video_resolution": "bad_resolution",
}

SUMMARY_FIELDS = [
    "timestamp", "config_id", "video_resolution", "video_bitrate", "exposure_mode",
    "shutter_speed_denominator", "status", "windows",
    "lat_mean_ms", "lat_p50_ms", "lat_p99_max_ms", "lat_min_ms", "lat_max_ms", "lat_window_mean_stdev_ms",
    "interarrival_mean_ms", "fps_est", "interarrival_p99_max_ms",
    "sync_rtt_mean_ms", "camera_reported_shutter_s", "measured_exposure_s",
    "actual_resolution", "resolution_check", "actual_fps", "fps_check",
    "actual_bitrate", "bitrate_ratio", "bitrate_check", "sps_fps", "keyframe_interval_frames", "max_packet_bytes",
    "log_file",
]
WINDOW_FIELDS = ["config_id", "window", "stream", "lat_min", "lat_mean", "lat_p50", "lat_p99", "lat_max",
                 "ia_min", "ia_mean", "ia_p50", "ia_p99", "ia_max", "exposure_time_s",
                 "actual_resolution", "actual_fps", "actual_bitrate", "keyframes", "max_packet_bytes"]
# Statuses that are final; anything else (start_timeout, exited, no_camera, ...) is retried on resume.
FINAL_STATUSES = {"ok", "start_failed", "bad_resolution", "skipped_resolution_failed"}


def parse_exposure(spec):
    """'auto' / 'full_auto' / 'unchanged' / 'shutter:120' / 'manual:120' -> (mode, denominator)."""
    if ":" in spec:
        name, den = spec.split(":", 1)
        mode = {"shutter": "shutter_priority", "shutter_priority": "shutter_priority",
                "manual": "manual"}.get(name)
        if mode is None:
            raise argparse.ArgumentTypeError(f"bad exposure spec '{spec}'")
        return mode, int(den)
    return spec, 0


def requested_format(res_name):
    """'1920x960p30' -> ('1920x960', 30.0)."""
    m = RES_NAME_RE.match(res_name)
    return (f"{m[1]}x{m[2]}", float(m[3])) if m else (res_name, float("nan"))


def config_id(res, bitrate, exp_mode, den):
    return f"{res}_{bitrate}_{exp_mode}_{den}"


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def finite(values):
    return [v for v in values if v == v and v not in (float("inf"), float("-inf"))]


def tolerance_check(actual, requested, tol):
    if actual != actual or not requested or requested != requested:
        return "UNKNOWN"
    return "MATCH" if abs(actual - requested) <= tol * requested else "MISMATCH"


def bitrate_check(actual, requested, tol):
    # Encoders undershoot on easy scenes, so BELOW alone does not prove the request was ignored; ABOVE does.
    if actual != actual or not requested:
        return "UNKNOWN"
    if actual > requested * (1 + tol):
        return "ABOVE"
    if actual < requested * (1 - tol):
        return "BELOW"
    return "MATCH"


def find_driver_executable():
    try:
        prefix = subprocess.run(["ros2", "pkg", "prefix", "insta360_ros_driver"], capture_output=True,
                                text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    exe = Path(prefix) / "lib" / "insta360_ros_driver" / "insta360_ros_driver"
    return str(exe) if exe.exists() else None


def find_params_file():
    try:
        prefix = subprocess.run(["ros2", "pkg", "prefix", "--share", "insta360_ros_driver"], capture_output=True,
                                text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    p = Path(prefix) / "config" / "insta360_ros_driver.yaml"
    return str(p) if p.exists() else None


class Run:
    """One driver process; reads its log on a thread and collects parsed reports."""

    def __init__(self, cmd, log_path):
        self.lock = threading.Lock()
        self.windows = []          # dicts from STREAM_RE (all, including warm-up)
        self.formats = []          # dicts from FORMAT_RE (all, including warm-up)
        self.sync_rtts = []
        self.exposures = []
        self.applied_shutter = None
        self.started = False
        self.fatal = None
        self.log = open(log_path, "w")
        env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT="0", RCUTILS_LOGGING_BUFFERED_STREAM="1")
        # The camera SDK prints some log lines in a non-UTF-8 encoding (e.g. GBK); replace undecodable bytes
        # instead of letting the reader thread die, which would stop parsing and stall the driver's stdout pipe.
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     encoding="utf-8", errors="replace",
                                     bufsize=1, env=env, start_new_session=True)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            self._parse_lines()
        except Exception as e:  # keep draining the pipe so the driver never blocks on a full stdout
            with self.lock:
                if self.fatal is None:
                    self.fatal = "log_read_error"
            self.log.write(f"\n[latency_sweep] log reader error: {e!r}\n")
            try:
                while self.proc.stdout.read(65536):
                    pass
            except Exception:
                pass

    def _parse_lines(self):
        for line in self.proc.stdout:
            self.log.write(line)
            self.log.flush()
            with self.lock:
                if STARTED_MARK in line:
                    self.started = True
                for mark, status in FATAL_MARKS.items():
                    if mark in line and self.fatal is None:
                        self.fatal = status
                m = STREAM_RE.search(line)
                if m:
                    w = {k: (int(v) if k == "stream" else fnum(v)) for k, v in m.groupdict().items()}
                    w["exposure_time_s"] = self.exposures[-1] if self.exposures else float("nan")
                    self.windows.append(w)
                    continue
                m = FORMAT_RE.search(line)
                if m:
                    self.formats.append({
                        "stream": int(m["stream"]),
                        "resolution": m["resolution"] if m["resolution"] != "n/a" else "",
                        "fps": fnum(m["fps"]), "bitrate": fnum(m["bitrate"]), "sps_fps": fnum(m["sps_fps"]),
                        "frames": int(m["frames"]), "keyframes": int(m["keyframes"]),
                        "max_packet_bytes": int(m["max_packet_bytes"]),
                    })
                    continue
                m = SYNC_RE.search(line)
                if m:
                    self.sync_rtts.append(fnum(m["rtt"]))
                    continue
                m = EXPOSURE_NOW_RE.search(line) or EXPOSURE_FIRST_RE.search(line)
                if m:
                    self.exposures.append(fnum(m["exp"]))
                    continue
                m = EXPOSURE_APPLIED_RE.search(line)
                if m:
                    self.applied_shutter = fnum(m["shutter"])

    def snapshot(self):
        with self.lock:
            return self.started, self.fatal, len(self.windows)

    def alive(self):
        return self.proc.poll() is None

    def stop(self, timeout):
        """SIGINT so the driver stops streaming and closes the camera; escalate if it hangs."""
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            if self.proc.poll() is not None:
                break
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.proc.wait(wait)
            except subprocess.TimeoutExpired:
                continue
        self.reader.join(timeout=5)
        self.log.close()


def summarize(cid, res, bitrate, exp_mode, den, run, status, args, log_path):
    windows = [w for w in run.windows if w["stream"] == args.stream_index][args.warmup_reports:]
    formats = [f for f in run.formats if f["stream"] == args.stream_index][args.warmup_reports:]
    lat_means = finite([w["lat_mean"] for w in windows])
    ia_means = finite([w["ia_mean"] for w in windows])
    exposures = finite(run.exposures)

    def agg(fn, values, digits=3):
        return round(fn(values), digits) if values else ""

    ia_mean = agg(statistics.mean, ia_means)

    # Stream format: the resolution seen most often, median fps, mean bitrate over the kept windows.
    req_res, req_fps = requested_format(res)
    resolutions = Counter(f["resolution"] for f in formats if f["resolution"])
    actual_res = "|".join(r for r, _ in resolutions.most_common())
    if not actual_res:
        res_check = "UNKNOWN"
    elif len(resolutions) == 1 and actual_res == req_res:
        res_check = "MATCH"
    else:
        res_check = "MISMATCH"
    fps_values = finite([f["fps"] for f in formats])
    actual_fps = statistics.median(fps_values) if fps_values else float("nan")
    br_values = finite([f["bitrate"] for f in formats])
    actual_br = statistics.mean(br_values) if br_values else float("nan")
    sps_fps = finite([f["sps_fps"] for f in formats])
    frames = sum(f["frames"] for f in formats)
    keyframes = sum(f["keyframes"] for f in formats)

    row = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "config_id": cid,
        "video_resolution": res,
        "video_bitrate": bitrate,
        "exposure_mode": exp_mode,
        "shutter_speed_denominator": den,
        "status": status,
        "windows": len(windows),
        "lat_mean_ms": agg(statistics.mean, lat_means),
        "lat_p50_ms": agg(statistics.median, finite([w["lat_p50"] for w in windows])),
        "lat_p99_max_ms": agg(max, finite([w["lat_p99"] for w in windows])),
        "lat_min_ms": agg(min, finite([w["lat_min"] for w in windows])),
        "lat_max_ms": agg(max, finite([w["lat_max"] for w in windows])),
        "lat_window_mean_stdev_ms": agg(statistics.stdev, lat_means) if len(lat_means) > 1 else "",
        "interarrival_mean_ms": ia_mean,
        "fps_est": round(1000.0 / ia_mean, 2) if ia_mean not in ("", 0) else "",
        "interarrival_p99_max_ms": agg(max, finite([w["ia_p99"] for w in windows])),
        "sync_rtt_mean_ms": agg(statistics.mean, finite(run.sync_rtts)),
        "camera_reported_shutter_s": run.applied_shutter if run.applied_shutter is not None else "",
        "measured_exposure_s": round(statistics.median(exposures), 6) if exposures else "",
        "actual_resolution": actual_res,
        "resolution_check": res_check,
        "actual_fps": round(actual_fps, 2) if actual_fps == actual_fps else "",
        "fps_check": tolerance_check(actual_fps, req_fps, args.fps_tolerance),
        "actual_bitrate": round(actual_br) if actual_br == actual_br else "",
        "bitrate_ratio": round(actual_br / bitrate, 3) if actual_br == actual_br else "",
        "bitrate_check": bitrate_check(actual_br, bitrate, args.bitrate_tolerance),
        "sps_fps": round(sps_fps[-1], 3) if sps_fps else "",
        "keyframe_interval_frames": round(frames / keyframes, 1) if keyframes else "",
        "max_packet_bytes": max((f["max_packet_bytes"] for f in formats), default=""),
        "log_file": str(log_path),
    }
    window_rows = []
    for k, w in enumerate(windows):
        r = {"config_id": cid, "window": k, **{f: w[f] for f in WINDOW_FIELDS[2:14]}}
        if k < len(formats):
            f = formats[k]
            r.update({"actual_resolution": f["resolution"], "actual_fps": f["fps"], "actual_bitrate": f["bitrate"],
                      "keyframes": f["keyframes"], "max_packet_bytes": f["max_packet_bytes"]})
        window_rows.append(r)
    return row, window_rows


def migrate_csv(path, fields):
    """Rewrites a CSV written by an older version of this script so its header matches `fields`."""
    if not path.exists() or path.stat().st_size == 0:
        return
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames == fields:
            return
        rows = list(reader)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def open_csv(path, fields):
    migrate_csv(path, fields)
    new = not path.exists() or path.stat().st_size == 0
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    if new:
        writer.writeheader()
        f.flush()
    return f, writer


def format_report(summary_path, report_path):
    """Says, across all runs, whether the camera honours the requested resolution, fps and bitrate."""
    if not summary_path.exists():
        return
    with open(summary_path, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r.get("status") == "ok"]
    lines = ["Stream format check (runs with status ok)", ""]
    if not rows:
        lines.append("No successful runs yet.")
    else:
        # Resolution
        lines.append("Resolution (requested -> received in the stream):")
        by_res = defaultdict(Counter)
        for r in rows:
            by_res[r["video_resolution"]][r.get("actual_resolution") or "unknown"] += 1
        for req, seen in sorted(by_res.items()):
            got = ", ".join(f"{k} ({v} runs)" for k, v in seen.most_common())
            known = set(seen) - {"unknown"}  # runs without stream data (e.g. older CSV rows) do not count
            verdict = "UNKNOWN" if not known else ("OK" if known == {requested_format(req)[0]} else "NOT HONOURED")
            lines.append(f"  {req:<14} -> {got}   {verdict}")
        all_seen = {k for c in by_res.values() for k in c if k != "unknown"}
        if len(by_res) > 1 and len(all_seen) == 1:
            lines.append(f"  => the stream is always {all_seen.pop()}: the resolution setting is ignored")
        lines.append("")

        # Frame rate
        lines.append("Frame rate (requested -> median received):")
        for req in sorted(by_res):
            fps = finite([fnum(r.get("actual_fps")) for r in rows if r["video_resolution"] == req])
            want = requested_format(req)[1]
            if fps:
                med = statistics.median(fps)
                verdict = tolerance_check(med, want, 0.10)
                lines.append(f"  {req:<14} -> {med:6.2f} fps (requested {want:.0f})   "
                             f"{'OK' if verdict == 'MATCH' else 'NOT HONOURED'}")
            else:
                lines.append(f"  {req:<14} -> unknown")
        lines.append("")

        # Bitrate: the reliable test is whether the received bitrate follows the request across runs.
        lines.append("Bitrate (requested -> median received):")
        by_br = defaultdict(list)
        for r in rows:
            v = fnum(r.get("actual_bitrate"))
            if v == v:
                by_br[int(r["video_bitrate"])].append(v)
        medians = {}
        for req in sorted(by_br):
            medians[req] = statistics.median(by_br[req])
            lines.append(f"  {req / 1e6:7.2f} Mbit/s -> {medians[req] / 1e6:7.3f} Mbit/s "
                         f"(x{medians[req] / req:.2f}, {len(by_br[req])} runs)")
        if len(medians) >= 2:
            lo, hi = min(medians), max(medians)
            req_ratio, got_ratio = hi / lo, medians[hi] / medians[lo] if medians[lo] else float("inf")
            if got_ratio >= 1.5:
                lines.append(f"  => received bitrate rises with the request (x{got_ratio:.1f} for x{req_ratio:.0f} "
                             "requested): the bitrate setting is honoured")
            else:
                lines.append(f"  => received bitrate barely changes (x{got_ratio:.2f} for x{req_ratio:.0f} "
                             "requested): the bitrate setting is ignored")
        elif medians:
            lines.append("  => only one bitrate tested; sweep at least two to tell whether it is honoured")
        lines.append("  (a received bitrate BELOW the request can just mean an easy-to-compress scene)")
    text = "\n".join(lines)
    print("\n" + text)
    report_path.write_text(text + "\n")
    print(f"\nFormat check: {report_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="latency_sweep", help="output path prefix (default: ./latency_sweep)")
    ap.add_argument("--resolutions", nargs="+", default=DEFAULT_RESOLUTIONS)
    ap.add_argument("--bitrates", nargs="+", type=int, default=DEFAULT_BITRATES, help="bit/s")
    ap.add_argument("--exposures", nargs="+", default=DEFAULT_EXPOSURES,
                    help="auto | full_auto | adaptive | unchanged | shutter:<N> | manual:<N>  (shutter = 1/N s)")
    ap.add_argument("--iso", type=int, default=0, help="ISO for manual:<N> exposures (0 = camera default)")
    ap.add_argument("--report-period", type=float, default=5.0, help="latency_report_period_s")
    ap.add_argument("--warmup-reports", type=int, default=1, help="report windows discarded at the start")
    ap.add_argument("--reports", type=int, default=4, help="report windows kept per configuration")
    ap.add_argument("--start-timeout", type=float, default=45.0, help="max wait for 'Live streaming started' (s)")
    ap.add_argument("--stop-timeout", type=float, default=15.0, help="max wait for clean shutdown after SIGINT (s)")
    ap.add_argument("--cooldown", type=float, default=3.0, help="pause between runs (s)")
    ap.add_argument("--stream-index", type=int, default=0, help="stream index to evaluate")
    ap.add_argument("--fps-tolerance", type=float, default=0.10,
                    help="fps counts as MATCH within this fraction of the request (default 0.10)")
    ap.add_argument("--bitrate-tolerance", type=float, default=0.50,
                    help="bitrate counts as MATCH within this fraction of the request (default 0.50)")
    ap.add_argument("--params-file", default=None, help="driver YAML (default: installed insta360_ros_driver.yaml)")
    ap.add_argument("--executable", default=None, help="driver executable (default: found via ros2 pkg prefix)")
    ap.add_argument("--no-resume", action="store_true", help="re-run configurations already in the CSV")
    ap.add_argument("--dry-run", action="store_true", help="only list the configurations and estimated time")
    ap.add_argument("--summary-only", action="store_true",
                    help="do not run anything; print the stream format check for an existing <out>.csv")
    args = ap.parse_args()

    exposures = [parse_exposure(e) for e in args.exposures]
    configs = list(itertools.product(args.resolutions, args.bitrates, exposures))

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    summary_path = out.with_name(out.name + ".csv")
    windows_path = out.with_name(out.name + "_windows.csv")
    report_path = out.with_name(out.name + "_format_check.txt")
    log_dir = out.with_name(out.name + "_logs")

    if args.summary_only:
        format_report(summary_path, report_path)
        return 0

    # Runs that did not finish cleanly are retried; their old rows are removed from both CSVs
    # so each configuration appears once.
    done = set()
    if not args.no_resume and summary_path.exists():
        with open(summary_path, newline="") as f:
            rows = list(csv.DictReader(f))
        done = {r["config_id"] for r in rows if r.get("status") in FINAL_STATUSES}
        retry = {r["config_id"] for r in rows} - done
        if retry and not args.dry_run:
            print(f"Retrying {len(retry)} configuration(s) that did not finish cleanly before")
            with open(summary_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
                w.writeheader()
                w.writerows(r for r in rows if r["config_id"] in done)
            if windows_path.exists():
                with open(windows_path, newline="") as f:
                    wrows = list(csv.DictReader(f))
                with open(windows_path, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=WINDOW_FIELDS, extrasaction="ignore")
                    w.writeheader()
                    w.writerows(r for r in wrows if r["config_id"] in done)
    todo = [c for c in configs if config_id(c[0], c[1], *c[2]) not in done]

    # ~5 s driver start delay + startup + (warm-up + kept) windows + shutdown + cooldown
    per_run = 10 + (args.warmup_reports + args.reports) * args.report_period + 5 + args.cooldown
    print(f"{len(configs)} configurations, {len(configs) - len(todo)} already done, {len(todo)} to run "
          f"(~{per_run:.0f} s each, ~{len(todo) * per_run / 60:.0f} min total)")
    if args.dry_run:
        for res, br, (mode, den) in todo:
            print(f"  {config_id(res, br, mode, den)}")
        return 0
    if not todo:
        format_report(summary_path, report_path)
        return 0

    exe = args.executable or find_driver_executable()
    params_file = args.params_file or find_params_file()
    if exe is None:
        exe_cmd = ["ros2", "run", "insta360_ros_driver", "insta360_ros_driver"]
    else:
        exe_cmd = [exe]
    log_dir.mkdir(parents=True, exist_ok=True)

    stop_requested = {"flag": False}

    def on_sigint(signum, frame):
        if stop_requested["flag"]:
            raise KeyboardInterrupt
        stop_requested["flag"] = True
        print("\nStopping after the current run (Ctrl+C again to abort immediately)...", flush=True)

    signal.signal(signal.SIGINT, on_sigint)

    sum_f, sum_w = open_csv(summary_path, SUMMARY_FIELDS)
    win_f, win_w = open_csv(windows_path, WINDOW_FIELDS)
    needed = args.warmup_reports + args.reports
    unsupported = set()  # resolutions the camera refused once; their remaining combinations are skipped
    run = None
    try:
        for i, (res, br, (mode, den)) in enumerate(todo, 1):
            if stop_requested["flag"]:
                break
            cid = config_id(res, br, mode, den)
            print(f"[{i}/{len(todo)}] {cid} ... ", end="", flush=True)
            if res in unsupported:
                sum_w.writerow({"timestamp": datetime.now().isoformat(timespec="seconds"), "config_id": cid,
                                "video_resolution": res, "video_bitrate": br, "exposure_mode": mode,
                                "shutter_speed_denominator": den, "status": "skipped_resolution_failed",
                                "windows": 0})
                sum_f.flush()
                print("skipped (resolution failed to start earlier)")
                continue
            cmd = exe_cmd + ["--ros-args"]
            if params_file:
                cmd += ["--params-file", params_file]
            cmd += ["-p", "measure_latency:=true",
                    "-p", f"latency_report_period_s:={args.report_period}",
                    "-p", f"video_resolution:={res}",
                    "-p", f"video_bitrate:={br}",
                    "-p", f"exposure_mode:={mode}",
                    "-p", f"shutter_speed_denominator:={den}",
                    "-p", "check_stream_format:=true",
                    "-p", f"stream_check_fps_tolerance:={args.fps_tolerance}",
                    "-p", f"stream_check_bitrate_tolerance:={args.bitrate_tolerance}"]
            if mode == "manual" and args.iso > 0:
                cmd += ["-p", f"iso:={args.iso}"]

            log_path = log_dir / f"{cid}.log"
            run = Run(cmd, log_path)
            t0 = time.monotonic()
            status = "ok"
            while True:
                started, fatal, n = run.snapshot()
                elapsed = time.monotonic() - t0
                if fatal:
                    status = fatal
                    break
                if not run.alive():
                    status = "exited"
                    break
                if not started and elapsed > args.start_timeout:
                    status = "start_timeout"
                    break
                if n >= needed:
                    time.sleep(0.5)  # the [stream] line is logged right after the [latency] line
                    break
                if started and elapsed > args.start_timeout + needed * args.report_period * 2 + 10:
                    status = "no_reports" if n == 0 else "partial"
                    break
                if stop_requested["flag"] and n < needed:
                    status = "aborted"
                    break
                time.sleep(0.2)
            run.stop(args.stop_timeout)

            row, window_rows = summarize(cid, res, br, mode, den, run, status, args, log_path)
            if status != "aborted":  # aborted runs are left out so a resume re-runs them
                sum_w.writerow(row)
                sum_f.flush()
                win_w.writerows(window_rows)
                win_f.flush()
            fmt = ""
            if row["actual_resolution"] or row["actual_fps"] != "":
                fmt = (f"  | stream {row['actual_resolution'] or '?'} {row['resolution_check']}, "
                       f"{row['actual_fps']} fps {row['fps_check']}, "
                       f"{row['actual_bitrate']} bit/s {row['bitrate_check']}")
            print(f"{status}  latency mean={row['lat_mean_ms']} ms p50={row['lat_p50_ms']} ms "
                  f"p99={row['lat_p99_max_ms']} ms  exposure={row['measured_exposure_s']} s{fmt}", flush=True)
            if status in ("start_failed", "bad_resolution"):
                unsupported.add(res)
            if status in ("no_camera", "open_failed"):
                print("Camera not available; stopping sweep.")
                break
            time.sleep(args.cooldown)
    except KeyboardInterrupt:
        print("\nAborted.")
        if run is not None:
            try:
                run.stop(args.stop_timeout)
            except Exception:
                pass
    finally:
        sum_f.close()
        win_f.close()
    print(f"Summary: {summary_path}\nWindows: {windows_path}\nLogs:    {log_dir}")
    format_report(summary_path, report_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
