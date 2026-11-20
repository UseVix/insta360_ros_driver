#!/usr/bin/env python3
"""Sweep camera driver settings and record the USB latency for each combination.

For every combination of resolution x bitrate x exposure, the driver is started
with measure_latency:=true, the first report windows are discarded as warm-up,
the following ones are parsed from the log, and the driver is stopped with SIGINT
(so it calls StopLiveStreaming/Close) before the next run.

Outputs:
  <out>.csv           one summary row per configuration (appended after every run)
  <out>_windows.csv   one row per latency report window (raw per-window stats)
  <out>_logs/         full driver log of every run

Rows already in <out>.csv are skipped, so an interrupted sweep can be resumed by
re-running the same command.

Example:
  ros2 run insta360_ros_driver latency_sweep.py --out ~/latency_sweep
  ros2 run insta360_ros_driver latency_sweep.py --resolutions 1920x960p30 1440x720p30 \
      --bitrates 524288 4194304 --exposures auto shutter:120 shutter:240 --dry-run
"""

import argparse
import csv
import itertools
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
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
SYNC_RE = re.compile(r"\[latency\] clock sync: offset=(?P<offset>\S+) ms, sync rtt=(?P<rtt>\S+) ms")
EXPOSURE_NOW_RE = re.compile(r"\[latency\] current exposure_time=(?P<exp>\S+) s")
EXPOSURE_FIRST_RE = re.compile(r"First exposure report from camera: exposure_time=(?P<exp>\S+) s")
EXPOSURE_APPLIED_RE = re.compile(
    r"camera reports mode=(?P<mode>-?\d+) shutter=(?P<shutter>\S+) s iso=(?P<iso>-?\d+) ev=(?P<ev>-?\d+)")
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
    "sync_rtt_mean_ms", "camera_reported_shutter_s", "measured_exposure_s", "log_file",
]
WINDOW_FIELDS = ["config_id", "window", "stream", "lat_min", "lat_mean", "lat_p50", "lat_p99", "lat_max",
                 "ia_min", "ia_mean", "ia_p50", "ia_p99", "ia_max", "exposure_time_s"]


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


def config_id(res, bitrate, exp_mode, den):
    return f"{res}_{bitrate}_{exp_mode}_{den}"


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def finite(values):
    return [v for v in values if v == v and v not in (float("inf"), float("-inf"))]


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
        self.sync_rtts = []
        self.exposures = []
        self.applied_shutter = None
        self.started = False
        self.fatal = None
        self.log = open(log_path, "w")
        env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT="0", RCUTILS_LOGGING_BUFFERED_STREAM="1")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     bufsize=1, env=env, start_new_session=True)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
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


def summarize(cid, res, bitrate, exp_mode, den, run, status, warmup, stream_filter, log_path):
    windows = [w for w in run.windows if w["stream"] == stream_filter][warmup:]
    lat_means = finite([w["lat_mean"] for w in windows])
    ia_means = finite([w["ia_mean"] for w in windows])
    exposures = finite(run.exposures)

    def agg(fn, values):
        return round(fn(values), 3) if values else ""

    ia_mean = agg(statistics.mean, ia_means)
    return {
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
        "log_file": str(log_path),
    }, windows


def open_csv(path, fields):
    new = not path.exists() or path.stat().st_size == 0
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=fields)
    if new:
        writer.writeheader()
        f.flush()
    return f, writer


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
    ap.add_argument("--params-file", default=None, help="driver YAML (default: installed insta360_ros_driver.yaml)")
    ap.add_argument("--executable", default=None, help="driver executable (default: found via ros2 pkg prefix)")
    ap.add_argument("--no-resume", action="store_true", help="re-run configurations already in the CSV")
    ap.add_argument("--dry-run", action="store_true", help="only list the configurations and estimated time")
    args = ap.parse_args()

    exposures = [parse_exposure(e) for e in args.exposures]
    configs = list(itertools.product(args.resolutions, args.bitrates, exposures))

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    summary_path = out.with_name(out.name + ".csv")
    windows_path = out.with_name(out.name + "_windows.csv")
    log_dir = out.with_name(out.name + "_logs")

    done = set()
    if not args.no_resume and summary_path.exists():
        with open(summary_path, newline="") as f:
            done = {row["config_id"] for row in csv.DictReader(f)}
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
                    "-p", f"shutter_speed_denominator:={den}"]
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
                    break
                if started and elapsed > args.start_timeout + needed * args.report_period * 2 + 10:
                    status = "no_reports" if n == 0 else "partial"
                    break
                if stop_requested["flag"] and n < needed:
                    status = "aborted"
                    break
                time.sleep(0.2)
            run.stop(args.stop_timeout)

            row, windows = summarize(cid, res, br, mode, den, run, status, args.warmup_reports,
                                     args.stream_index, log_path)
            if status != "aborted":  # aborted runs are left out so a resume re-runs them
                sum_w.writerow(row)
                sum_f.flush()
                for k, w in enumerate(windows):
                    win_w.writerow({"config_id": cid, "window": k, **{f: w[f] for f in WINDOW_FIELDS[2:]}})
                win_f.flush()
            print(f"{status}  latency mean={row['lat_mean_ms']} ms p50={row['lat_p50_ms']} ms "
                  f"p99={row['lat_p99_max_ms']} ms  fps={row['fps_est']}  exposure={row['measured_exposure_s']} s",
                  flush=True)
            if status in ("start_failed", "bad_resolution"):
                unsupported.add(res)
            if status in ("no_camera", "open_failed"):
                print("Camera not available; stopping sweep.")
                break
            time.sleep(args.cooldown)
    except KeyboardInterrupt:
        print("\nAborted.")
        try:
            run.stop(args.stop_timeout)
        except Exception:
            pass
    finally:
        sum_f.close()
        win_f.close()
    print(f"Summary: {summary_path}\nWindows: {windows_path}\nLogs:    {log_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
