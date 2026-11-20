#!/usr/bin/env python3
"""Find which camera settings affect USB latency, then sweep the ones that do.

Axes (the first value of each list is the baseline):
  exposure, video sub-mode, codec (H.264/H.265), and the five image settings
  (contrast, saturation, brightness, sharpness, white balance). Resolution and
  bitrate are axes too, but have a single value by default.

Phase 1, screening: each axis is varied on its own while every other axis stays at
  its baseline. The baseline itself is run several times, spread over the phase,
  to measure run-to-run noise. An axis "has an effect" if any of its supported
  values moves the mean latency by more than max(--min-effect-ms,
  --noise-factor x baseline stdev) or changes the frame rate by more than
  --fps-effect. Values the camera refuses (start failure, rejected setting,
  codec not switchable) are left out.
Phase 2, full sweep: every combination of the axes that had an effect (with their
  supported values); all other axes stay at baseline. Runs already done in
  screening are reused.

The codec cannot be set through the SDK, only on the camera's menu. Codecs are tested in
the order given (default H.264, the baseline, then H.265). Whenever the camera is not on
the codec a run needs, the script asks you to switch it on the camera and press Enter
('s' skips that codec, 'q' stops). The switch is verified from the driver log.
Runs are grouped so that you switch as few times as possible.

Each run starts the driver with measure_latency:=true, discards the first report
window(s), parses the following ones, and stops the driver with SIGINT. It also
records the stream format actually received (resolution from the SPS, fps,
bitrate). At the end the camera is put back to the baseline sub-mode and image
settings (--no-restore to skip).

Outputs:
  <out>.csv           one row per run (phase, axis values, latency, stream format)
  <out>_windows.csv   one row per report window
  <out>_report.txt    screening verdicts, best configurations, stream format check
  <out>_logs/         full driver log of every run

Re-running the same command resumes: finished runs are skipped, runs that did not
finish cleanly are retried.

Examples:
  ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --dry-run
  ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep
  ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --phase screen
  ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --codecs current --sub-modes normal hdr
  ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --summary-only
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

RESOLUTIONS = ["3840x1920p30", "2560x1280p30", "2304x1152p30", "1920x960p30",
               "1440x720p30", "960x480p30", "3840x1920p60", "2560x1280p60"]
SUB_MODES = ["normal", "hdr", "pure", "liveview", "loop_recording", "timelapse", "timeshift",
             "bullet_time", "slow_motion", "selfie", "super", "dash_cam", "ptz"]
WHITE_BALANCES = ["auto", "2700k", "4000k", "5000k", "6500k", "7500k"]
CODECS = ["h264", "h265"]


class Axis:
    def __init__(self, name, short, flag, default, group, help_text):
        self.name, self.short, self.flag, self.default = name, short, flag, default
        self.group = group  # which driver setting call this axis goes through (for rejected settings)
        self.help = help_text


AXES = [
    Axis("resolution", "res", "--resolutions", ["1920x960p30"], "stream", "|".join(RESOLUTIONS)),
    Axis("bitrate", "br", "--bitrates", ["524288"], "stream", "bit/s"),
    Axis("exposure", "exp", "--exposures",
         ["auto", "shutter:30", "shutter:60", "shutter:100", "shutter:120", "shutter:240", "shutter:500",
          "shutter:1000"], "exposure",
         "auto | full_auto | adaptive | shutter:<N> | manual:<N> (shutter = 1/N s)"),
    Axis("sub_mode", "sub", "--sub-modes", ["normal", "hdr", "pure", "liveview"], "sub_mode", "|".join(SUB_MODES)),
    Axis("codec", "codec", "--codecs", ["h264", "h265"], "codec",
         "h264 | h265 in the order to test (first = baseline), or 'current' to keep whatever the "
         "camera uses (no prompts)"),
    Axis("contrast", "con", "--contrast", ["64", "0", "256"], "image", "0..256 (SDK default 64)"),
    Axis("saturation", "sat", "--saturation", ["64", "0", "256"], "image", "0..256 (SDK default 64)"),
    Axis("brightness", "bri", "--brightness", ["0", "-256", "256"], "image", "-256..256 (SDK default 0)"),
    Axis("sharpness", "shp", "--sharpness", ["3", "0", "6"], "image", "0..6 (SDK default 3)"),
    Axis("white_balance", "wb", "--white-balance", ["auto", "2700k", "7500k"], "image", "|".join(WHITE_BALANCES)),
]
AXIS_NAMES = [a.name for a in AXES]
AXIS_BY_NAME = {a.name: a for a in AXES}

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
CODEC_RE = re.compile(r"Camera encode type: (?P<codec>h26[45])")
RES_NAME_RE = re.compile(r"^(\d+)x(\d+)p(\d+)$")
STARTED_MARK = "Live streaming started."
REJECT_MARKS = {
    "SetExposureSettings(LIVE_STREAM) failed": "exposure",
    "SetVideoSubMode(": "sub_mode",          # line ends with "failed"
    "SetCaptureSettings(LIVE_STREAM) failed": "image",
}
FATAL_MARKS = {
    "Failed to start live streaming": "start_failed",
    "No available camera devices found": "no_camera",
    "Failed to open camera": "open_failed",
    "Unknown video_resolution": "bad_value",
    "Unknown video_sub_mode": "bad_value",
    "Unknown image_white_balance": "bad_value",
    "Image setting out of range": "bad_value",
}
# Statuses that are final; anything else (start_timeout, exited, no_camera, ...) is retried on resume.
FINAL_STATUSES = {"ok", "start_failed", "bad_value", "codec_unavailable"}

METRIC_FIELDS = [
    "lat_mean_ms", "lat_p50_ms", "lat_p99_max_ms", "lat_min_ms", "lat_max_ms", "lat_window_mean_stdev_ms",
    "interarrival_mean_ms", "fps_est", "interarrival_p99_max_ms",
    "sync_rtt_mean_ms", "camera_reported_shutter_s", "measured_exposure_s",
    "actual_resolution", "resolution_check", "actual_fps", "fps_check",
    "actual_bitrate", "bitrate_ratio", "bitrate_check", "sps_fps", "keyframe_interval_frames", "max_packet_bytes",
]
SUMMARY_FIELDS = (["timestamp", "phase", "rep", "config_id"] + AXIS_NAMES +
                  ["status", "settings_rejected", "actual_codec", "windows"] + METRIC_FIELDS + ["log_file"])
WINDOW_FIELDS = ["config_id", "phase", "rep", "window", "stream", "lat_min", "lat_mean", "lat_p50", "lat_p99",
                 "lat_max", "ia_min", "ia_mean", "ia_p50", "ia_p99", "ia_max", "exposure_time_s",
                 "actual_resolution", "actual_fps", "actual_bitrate", "keyframes", "max_packet_bytes"]


# ----------------------------------------------------------------------------- helpers

def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def finite(values):
    return [v for v in values if v == v and v not in (float("inf"), float("-inf"))]


def requested_format(res_name):
    """'1920x960p30' -> ('1920x960', 30.0)."""
    m = RES_NAME_RE.match(res_name)
    return (f"{m[1]}x{m[2]}", float(m[3])) if m else (res_name, float("nan"))


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


def parse_exposure(spec):
    """'auto' / 'full_auto' / 'adaptive' / 'shutter:120' / 'manual:120' -> (mode, denominator)."""
    if ":" in spec:
        name, den = spec.split(":", 1)
        mode = {"shutter": "shutter_priority", "manual": "manual"}.get(name)
        if mode is None or not den.isdigit() or int(den) <= 0:
            raise ValueError(f"bad exposure '{spec}'")
        return mode, int(den)
    if spec not in ("auto", "full_auto", "adaptive"):
        raise ValueError(f"bad exposure '{spec}'")
    return spec, 0


def validate_values(axis, values):
    def int_range(lo, hi):
        for v in values:
            if not re.fullmatch(r"-?\d+", v) or not lo <= int(v) <= hi:
                raise ValueError(f"{axis.flag}: '{v}' is not an integer in {lo}..{hi}")
    if axis.name == "resolution":
        bad = [v for v in values if v not in RESOLUTIONS]
        if bad:
            raise ValueError(f"{axis.flag}: unknown {bad} (use {axis.help})")
    elif axis.name == "bitrate":
        int_range(1, 10 ** 9)
    elif axis.name == "exposure":
        for v in values:
            parse_exposure(v)
    elif axis.name == "sub_mode":
        bad = [v for v in values if v not in SUB_MODES]
        if bad:
            raise ValueError(f"{axis.flag}: unknown {bad} (use {axis.help})")
    elif axis.name == "codec":
        if values != ["current"] and any(v not in CODECS for v in values):
            raise ValueError(f"{axis.flag}: use h264 / h265, or 'current'")
    elif axis.name in ("contrast", "saturation"):
        int_range(0, 256)
    elif axis.name == "brightness":
        int_range(-256, 256)
    elif axis.name == "sharpness":
        int_range(0, 6)
    elif axis.name == "white_balance":
        bad = [v for v in values if v not in WHITE_BALANCES]
        if bad:
            raise ValueError(f"{axis.flag}: unknown {bad} (use {axis.help})")
    if len(set(values)) != len(values):
        raise ValueError(f"{axis.flag}: duplicate values")


def config_id(cfg):
    return ";".join(f"{a.short}={cfg[a.name]}" for a in AXES)


def driver_params(cfg, args):
    mode, den = parse_exposure(cfg["exposure"])
    p = {
        "measure_latency": "true",
        "latency_report_period_s": args.report_period,
        "check_stream_format": "true",
        "stream_check_fps_tolerance": args.fps_tolerance,
        "stream_check_bitrate_tolerance": args.bitrate_tolerance,
        "video_resolution": cfg["resolution"],
        "video_bitrate": cfg["bitrate"],
        "exposure_mode": mode,
        "shutter_speed_denominator": den,
        "video_sub_mode": cfg["sub_mode"],
        "apply_image_settings": "true",
        "image_contrast": cfg["contrast"],
        "image_saturation": cfg["saturation"],
        "image_brightness": cfg["brightness"],
        "image_sharpness": cfg["sharpness"],
        "image_white_balance": cfg["white_balance"],
    }
    if mode == "manual" and args.iso > 0:
        p["iso"] = args.iso
    return p


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


# ----------------------------------------------------------------------------- one driver run

class Run:
    """One driver process; reads its log on a thread and collects parsed reports."""

    def __init__(self, cmd, log_path):
        self.lock = threading.Lock()
        self.windows = []          # dicts from STREAM_RE (all, including warm-up)
        self.formats = []          # dicts from FORMAT_RE (all, including warm-up)
        self.sync_rtts = []
        self.exposures = []
        self.applied_shutter = None
        self.codec = None
        self.rejected = set()
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
                for mark, group in REJECT_MARKS.items():
                    if mark in line and "failed" in line:
                        self.rejected.add(group)
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
                m = CODEC_RE.search(line)
                if m:
                    self.codec = m["codec"]
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
            return self.started, self.fatal, len(self.windows), self.codec

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


def summarize(cfg, phase, rep, run, status, args, log_path):
    windows = [w for w in run.windows if w["stream"] == args.stream_index][args.warmup_reports:]
    formats = [f for f in run.formats if f["stream"] == args.stream_index][args.warmup_reports:]
    lat_means = finite([w["lat_mean"] for w in windows])
    ia_means = finite([w["ia_mean"] for w in windows])
    exposures = finite(run.exposures)

    def agg(fn, values, digits=3):
        return round(fn(values), digits) if values else ""

    ia_mean = agg(statistics.mean, ia_means)
    req_res, req_fps = requested_format(cfg["resolution"])
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
    req_br = int(cfg["bitrate"])
    sps_fps = finite([f["sps_fps"] for f in formats])
    frames = sum(f["frames"] for f in formats)
    keyframes = sum(f["keyframes"] for f in formats)
    cid = config_id(cfg)

    row = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "phase": phase, "rep": rep, "config_id": cid, **cfg,
        "status": status,
        "settings_rejected": "|".join(sorted(run.rejected)),
        "actual_codec": run.codec or "",
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
        "bitrate_ratio": round(actual_br / req_br, 3) if actual_br == actual_br else "",
        "bitrate_check": bitrate_check(actual_br, req_br, args.bitrate_tolerance),
        "sps_fps": round(sps_fps[-1], 3) if sps_fps else "",
        "keyframe_interval_frames": round(frames / keyframes, 1) if keyframes else "",
        "max_packet_bytes": max((f["max_packet_bytes"] for f in formats), default=""),
        "log_file": str(log_path),
    }
    window_rows = []
    for k, w in enumerate(windows):
        r = {"config_id": cid, "phase": phase, "rep": rep, "window": k,
             **{f: w[f] for f in WINDOW_FIELDS[4:16]}}
        if k < len(formats):
            f = formats[k]
            r.update({"actual_resolution": f["resolution"], "actual_fps": f["fps"], "actual_bitrate": f["bitrate"],
                      "keyframes": f["keyframes"], "max_packet_bytes": f["max_packet_bytes"]})
        window_rows.append(r)
    return row, window_rows


# ----------------------------------------------------------------------------- results store

class Store:
    def __init__(self, summary_path, windows_path):
        self.summary_path, self.windows_path = summary_path, windows_path
        self.rows = []
        if summary_path.exists() and summary_path.stat().st_size > 0:
            with open(summary_path, newline="") as f:
                reader = csv.DictReader(f)
                if reader.fieldnames != SUMMARY_FIELDS:
                    raise SystemExit(f"{summary_path} was written by an older version of this script "
                                     "(different columns). Use a new --out.")
                self.rows = list(reader)

    def drop_unfinished(self):
        """Removes rows of runs that did not finish cleanly (and their windows) so they are retried."""
        keep = [r for r in self.rows if r["status"] in FINAL_STATUSES]
        dropped = len(self.rows) - len(keep)
        if not dropped:
            return 0
        keys = {(r["phase"], r["config_id"], r["rep"]) for r in keep}
        self.rows = keep
        self._rewrite(self.summary_path, SUMMARY_FIELDS, self.rows)
        if self.windows_path.exists():
            with open(self.windows_path, newline="") as f:
                wrows = [r for r in csv.DictReader(f) if (r["phase"], r["config_id"], r["rep"]) in keys]
            self._rewrite(self.windows_path, WINDOW_FIELDS, wrows)
        return dropped

    @staticmethod
    def _rewrite(path, fields, rows):
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

    def has(self, phase, cid, rep):
        return any(r["phase"] == phase and r["config_id"] == cid and r["rep"] == str(rep) for r in self.rows)

    def ok_anywhere(self, cid):
        return any(r["config_id"] == cid and r["status"] == "ok" for r in self.rows)

    def append(self, row, window_rows):
        row = {k: str(v) for k, v in row.items()}
        self.rows.append(row)
        for path, fields, rows in ((self.summary_path, SUMMARY_FIELDS, [row]),
                                   (self.windows_path, WINDOW_FIELDS, window_rows)):
            new = not path.exists() or path.stat().st_size == 0
            with open(path, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
                if new:
                    w.writeheader()
                w.writerows(rows)


# ----------------------------------------------------------------------------- planning & analysis

def baseline_codec(values):
    """Baseline codec: the first one listed (H.264 by default), or 'current' to never ask for a switch."""
    return values[0]


def make_baseline(axis_values, codec_baseline):
    base = {name: vals[0] for name, vals in axis_values.items()}
    base["codec"] = codec_baseline
    return base


def screening_plan(axis_values, codec_baseline, repeats):
    """List of (cfg, rep). Codec runs come last; baseline repeats are spread over the other runs."""
    base = make_baseline(axis_values, codec_baseline)
    single = []
    for name in AXIS_NAMES:
        if name == "codec":
            continue
        for v in axis_values[name][1:]:
            single.append(dict(base, **{name: v}))
    plan = [(dict(base), 0)]
    if repeats > 1:
        # insert the remaining baseline repeats evenly, the last one after all single-axis runs
        step = len(single) / (repeats - 1) if single else 0
        positions = [round(step * k) for k in range(1, repeats)]
        rep = 1
        for i, cfg in enumerate(single):
            while rep < repeats and positions[rep - 1] == i:
                plan.append((dict(base), rep))
                rep += 1
            plan.append((cfg, 0))
        while rep < repeats:
            plan.append((dict(base), rep))
            rep += 1
    else:
        plan += [(cfg, 0) for cfg in single]
    if codec_baseline != "current":
        for v in axis_values["codec"]:
            if v != codec_baseline:
                plan.append((dict(base, codec=v), 0))
    return plan


def analyse_screening(store, axis_values, codec_baseline, args):
    """Returns (verdicts, full_axes, lines). verdicts: axis -> dict."""
    base = make_baseline(axis_values, codec_baseline)
    base_id = config_id(base)
    rows = [r for r in store.rows if r["phase"] == "screen"]
    base_rows = [r for r in rows if r["config_id"] == base_id and r["status"] == "ok"]
    base_lat = finite([fnum(r["lat_mean_ms"]) for r in base_rows])
    base_fps = finite([fnum(r["actual_fps"]) if r["actual_fps"] else fnum(r["fps_est"]) for r in base_rows])
    base_rejected = set().union(*[set(filter(None, r["settings_rejected"].split("|"))) for r in base_rows]) \
        if base_rows else set()
    lines = ["SCREENING (each axis varied alone, everything else at baseline)", ""]
    if not base_lat:
        lines.append("No successful baseline run yet.")
        return {}, {}, lines
    b_mean = statistics.mean(base_lat)
    noise = statistics.stdev(base_lat) if len(base_lat) > 1 else 0.0
    threshold = max(args.min_effect_ms, args.noise_factor * noise)
    b_fps = statistics.median(base_fps) if base_fps else float("nan")
    lines.append(f"Baseline: {base_id}")
    lines.append(f"  mean latency {b_mean:.1f} ms over {len(base_lat)} runs (run-to-run stdev {noise:.1f} ms), "
                 f"fps {b_fps:.2f}")
    lines.append(f"  effect threshold: |change| > {threshold:.1f} ms "
                 f"(max of {args.min_effect_ms:g} ms and {args.noise_factor:g} x stdev), or fps change > "
                 f"{args.fps_effect * 100:.0f} %")
    lines.append("")

    verdicts, full_axes = {}, {}
    for axis in AXES:
        name = axis.name
        values = axis_values[name] if name != "codec" else (
            [] if codec_baseline == "current" else [codec_baseline] + [v for v in axis_values[name]
                                                                     if v != codec_baseline])
        if len(values) < 2:
            continue
        group = axis.group
        if group in base_rejected:
            verdicts[name] = {"verdict": "untestable"}
            lines.append(f"{name}: UNTESTABLE - the camera rejects this setting even at baseline")
            lines.append("")
            continue
        supported, effect, incomplete = [], False, False
        detail = []
        for v in values[1:]:
            cid = config_id(dict(base, **{name: v}))
            vrows = [r for r in rows if r["config_id"] == cid]
            ok = [r for r in vrows if r["status"] == "ok"]
            if not vrows:
                incomplete = True
                detail.append(f"    {v:<14} not run yet")
                continue
            if not ok:
                detail.append(f"    {v:<14} UNSUPPORTED ({vrows[-1]['status']})")
                continue
            rejected = set(filter(None, ok[-1]["settings_rejected"].split("|")))
            if group in rejected:
                detail.append(f"    {v:<14} UNSUPPORTED (camera rejected the setting)")
                continue
            lat = finite([fnum(r["lat_mean_ms"]) for r in ok])
            fps = finite([fnum(r["actual_fps"]) if r["actual_fps"] else fnum(r["fps_est"]) for r in ok])
            if not lat:
                detail.append(f"    {v:<14} no latency data")
                continue
            supported.append(v)
            d = statistics.mean(lat) - b_mean
            fps_v = statistics.median(fps) if fps else float("nan")
            fps_change = abs(fps_v - b_fps) / b_fps if b_fps == b_fps and fps_v == fps_v and b_fps else 0.0
            hit = abs(d) > threshold or fps_change > args.fps_effect
            effect |= hit
            extra = ""
            if name == "exposure" and ok[-1]["measured_exposure_s"]:
                extra = f", measured exposure {fnum(ok[-1]['measured_exposure_s']):.6f} s"
            detail.append(f"    {v:<14} {statistics.mean(lat):7.1f} ms ({d:+6.1f} ms), fps {fps_v:6.2f}{extra}"
                          f"{'   <- EFFECT' if hit else ''}")
        if incomplete:
            verdict = "incomplete"
        elif not supported:
            verdict = "untestable"
        else:
            verdict = "effect" if effect else "no effect"
        verdicts[name] = {"verdict": verdict, "supported": supported}
        label = {"effect": "EFFECT -> included in the full sweep",
                 "no effect": "no effect -> fixed at baseline",
                 "untestable": "UNTESTABLE (no supported value besides baseline) -> fixed at baseline",
                 "incomplete": "incomplete (screening not finished)"}[verdict]
        lines.append(f"{name}: {label}")
        lines.append(f"    {values[0]:<14} {b_mean:7.1f} ms (baseline)")
        lines += detail
        lines.append("")
        if verdict == "effect":
            full_axes[name] = [values[0]] + supported
    return verdicts, full_axes, lines


def full_plan(axis_values, codec_baseline, full_axes, camera_codec):
    base = make_baseline(axis_values, codec_baseline)
    names = list(full_axes)
    if "codec" in names:
        # codec outermost, starting with the codec the camera is currently set to
        codecs = full_axes["codec"]
        if camera_codec in codecs:
            codecs = [camera_codec] + [c for c in codecs if c != camera_codec]
        names = ["codec"] + [n for n in names if n != "codec"]
        full_axes = dict(full_axes, codec=codecs)
    configs = []
    for combo in itertools.product(*[full_axes[n] for n in names]):
        configs.append(dict(base, **dict(zip(names, combo))))
    return configs


def best_lines(store, top):
    by_cfg = defaultdict(list)
    cfgs = {}
    for r in store.rows:
        if r["status"] == "ok" and r["lat_mean_ms"]:
            by_cfg[r["config_id"]].append(fnum(r["lat_mean_ms"]))
            cfgs[r["config_id"]] = r
    if not by_cfg:
        return []
    ranked = sorted(by_cfg, key=lambda c: statistics.mean(by_cfg[c]))
    lines = [f"LOWEST LATENCY CONFIGURATIONS (all successful runs, top {min(top, len(ranked))})", ""]
    for i, cid in enumerate(ranked[:top], 1):
        r = cfgs[cid]
        lat = statistics.mean(by_cfg[cid])
        fps = r["actual_fps"] or r["fps_est"]
        lines.append(f"  {i:2d}. {lat:7.1f} ms  fps {fps:>6}  {cid}")
    lines.append("")
    return lines


def format_lines(store):
    """Whether the camera honours the requested resolution, fps and bitrate."""
    rows = [r for r in store.rows if r["status"] == "ok"]
    if not rows:
        return []
    lines = ["STREAM FORMAT CHECK (successful runs)", ""]
    by_res = defaultdict(Counter)
    for r in rows:
        by_res[r["resolution"]][r["actual_resolution"] or "unknown"] += 1
    lines.append("Resolution (requested -> received in the stream):")
    for req, seen in sorted(by_res.items()):
        got = ", ".join(f"{k} ({v} runs)" for k, v in seen.most_common())
        known = set(seen) - {"unknown"}
        verdict = "UNKNOWN" if not known else ("OK" if known == {requested_format(req)[0]} else "NOT HONOURED")
        lines.append(f"  {req:<14} -> {got}   {verdict}")
    all_seen = {k for c in by_res.values() for k in c if k != "unknown"}
    if len(by_res) > 1 and len(all_seen) == 1:
        lines.append(f"  => the stream is always {all_seen.pop()}: the resolution setting is ignored")
    lines.append("Frame rate (requested -> median received):")
    for req in sorted(by_res):
        fps = finite([fnum(r["actual_fps"]) for r in rows if r["resolution"] == req])
        want = requested_format(req)[1]
        if fps:
            med = statistics.median(fps)
            ok = tolerance_check(med, want, 0.10) == "MATCH"
            lines.append(f"  {req:<14} -> {med:6.2f} fps (requested {want:.0f})   {'OK' if ok else 'NOT HONOURED'}")
    by_br = defaultdict(list)
    for r in rows:
        v = fnum(r["actual_bitrate"])
        if v == v:
            by_br[int(r["bitrate"])].append(v)
    if by_br:
        lines.append("Bitrate (requested -> median received):")
        medians = {req: statistics.median(v) for req, v in sorted(by_br.items())}
        for req, med in medians.items():
            lines.append(f"  {req / 1e6:7.2f} Mbit/s -> {med / 1e6:7.3f} Mbit/s (x{med / req:.2f})")
        if len(medians) >= 2:
            lo, hi = min(medians), max(medians)
            got = medians[hi] / medians[lo] if medians[lo] else float("inf")
            lines.append(f"  => x{got:.2f} received for x{hi / lo:.0f} requested: the bitrate setting is "
                         f"{'honoured' if got >= 1.5 else 'ignored'}")
        else:
            lines.append("  (one bitrate tested; sweep two or more to tell whether it is honoured)")
    lines.append("")
    return lines


def write_report(store, axis_values, codec_baseline, args, report_path, quiet=False):
    lines = []
    if codec_baseline is not None:
        _, _, screen = analyse_screening(store, axis_values, codec_baseline, args)
        lines += screen
    lines += best_lines(store, 10)
    lines += format_lines(store)
    text = "\n".join(lines).rstrip() + "\n"
    report_path.write_text(text)
    if not quiet:
        print("\n" + text + f"\nReport: {report_path}", flush=True)


# ----------------------------------------------------------------------------- main

class Sweeper:
    def __init__(self, args, store, axis_values, log_dir, exe_cmd, params_file):
        self.args, self.store, self.axis_values = args, store, axis_values
        self.log_dir, self.exe_cmd, self.params_file = log_dir, exe_cmd, params_file
        self.camera_codec = None        # last codec seen in a driver log
        self.unavailable_codecs = set()
        self.stop_requested = False
        self.quit = False
        self.touched_camera_modes = False
        self.count = 0

    def prompt(self, text):
        """Returns 'go', 'skip' or 'quit'. EOF (no terminal attached) counts as skip."""
        print("\n" + "=" * 70 + "\n" + text + "\n  Enter = done, s = skip this codec, q = stop the sweep\n" + "=" * 70,
              flush=True)
        try:
            answer = input("> ").strip().lower()
        except EOFError:
            print("(no input available - skipping)", flush=True)
            return "skip"
        if self.stop_requested or answer == "q":
            return "quit"
        return "skip" if answer == "s" else "go"

    def launch(self, cfg, log_path, expected_codec):
        cmd = self.exe_cmd + ["--ros-args"]
        if self.params_file:
            cmd += ["--params-file", self.params_file]
        for k, v in driver_params(cfg, self.args).items():
            cmd += ["-p", f"{k}:={v}"]
        run = Run(cmd, log_path)
        t0 = time.monotonic()
        needed = self.args.warmup_reports + self.args.reports
        status = "ok"
        while True:
            started, fatal, n, codec = run.snapshot()
            elapsed = time.monotonic() - t0
            if expected_codec and codec and codec != expected_codec:
                status = "codec_mismatch"
                break
            if fatal:
                status = fatal
                break
            if not run.alive():
                status = "exited"
                break
            if not started and elapsed > self.args.start_timeout:
                status = "start_timeout"
                break
            if n >= needed:
                time.sleep(0.5)  # the [stream] line is logged right after the [latency] line
                break
            if started and elapsed > self.args.start_timeout + needed * self.args.report_period * 2 + 10:
                status = "no_reports" if n == 0 else "partial"
                break
            if self.stop_requested and n < needed:
                status = "aborted"
                break
            time.sleep(0.2)
        run.stop(self.args.stop_timeout)
        if run.codec:
            self.camera_codec = run.codec
        return run, status

    def execute(self, cfg, phase, rep, total):
        """Runs one configuration (asking for a codec switch if needed) and stores the result."""
        self.count += 1
        cid = config_id(cfg)
        want = None if cfg["codec"] == "current" else cfg["codec"]
        label = f"[{phase} {self.count}/{total}] {cid}" + (f" (rep {rep})" if rep else "")
        if want in self.unavailable_codecs:
            self.record_codec_unavailable(cfg, phase, rep, label)
            return
        for attempt in range(3):
            if want and self.camera_codec and self.camera_codec != want:
                answer = self.prompt(f"The next run needs {want.upper()}, the camera is set to "
                                     f"{self.camera_codec.upper()}.\n  Switch the codec on the camera's menu "
                                     f"to {want.upper()}, then press Enter.")
                if answer == "quit":
                    self.quit = True
                    return
                if answer == "skip":
                    self.unavailable_codecs.add(want)
                    self.record_codec_unavailable(cfg, phase, rep, label)
                    return
            print(f"{label} ... ", end="", flush=True)
            safe = re.sub(r"[^A-Za-z0-9=._-]+", "_", cid)
            log_path = self.log_dir / f"{phase}_{safe}_rep{rep}.log"
            run, status = self.launch(cfg, log_path, want)
            if cfg["sub_mode"] != self.axis_values["sub_mode"][0] or any(
                    cfg[n] != self.axis_values[n][0] for n in ("contrast", "saturation", "brightness", "sharpness",
                                                               "white_balance")):
                self.touched_camera_modes = True
            if status == "codec_mismatch":
                print(f"camera is still on {run.codec}", flush=True)
                continue
            row, window_rows = summarize(cfg, phase, rep, run, status, self.args, log_path)
            if status != "aborted":  # aborted runs are left out so a resume re-runs them
                self.store.append(row, window_rows)
            fmt = ""
            if row["actual_resolution"] or row["actual_fps"] != "":
                fmt = (f" | {row['actual_codec']} {row['actual_resolution'] or '?'}, "
                       f"{row['actual_fps']} fps, {row['actual_bitrate']} bit/s")
            rej = f" | REJECTED: {row['settings_rejected']}" if row["settings_rejected"] else ""
            print(f"{status}  latency mean={row['lat_mean_ms']} ms p99={row['lat_p99_max_ms']} ms  "
                  f"exposure={row['measured_exposure_s']} s{fmt}{rej}", flush=True)
            if status in ("no_camera", "open_failed"):
                print("Camera not available; stopping.", flush=True)
                self.quit = True
            time.sleep(self.args.cooldown)
            return
        print(f"\nCodec {want} could not be confirmed after 3 attempts; marking it unavailable.", flush=True)
        self.unavailable_codecs.add(want)
        self.record_codec_unavailable(cfg, phase, rep, label)

    def record_codec_unavailable(self, cfg, phase, rep, label):
        print(f"{label} ... codec_unavailable", flush=True)
        self.store.append({"timestamp": datetime.now().isoformat(timespec="seconds"), "phase": phase, "rep": rep,
                           "config_id": config_id(cfg), **cfg, "status": "codec_unavailable"}, [])

    def restore(self):
        """Puts the camera back to the baseline sub-mode / image settings with a short driver run."""
        base = {n: v[0] for n, v in self.axis_values.items()}
        base["codec"] = "current"
        print("\nRestoring the camera to the baseline sub-mode and image settings "
              f"({base['sub_mode']}, contrast {base['contrast']}, saturation {base['saturation']}, "
              f"brightness {base['brightness']}, sharpness {base['sharpness']}, wb {base['white_balance']}) ... ",
              end="", flush=True)
        cmd = self.exe_cmd + ["--ros-args"]
        if self.params_file:
            cmd += ["--params-file", self.params_file]
        for k, v in driver_params(base, self.args).items():
            cmd += ["-p", f"{k}:={v}"]
        run = Run(cmd, self.log_dir / "restore.log")
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.args.start_timeout:
            started, fatal, _, _ = run.snapshot()
            if started or fatal or not run.alive():
                break
            time.sleep(0.2)
        run.stop(self.args.stop_timeout)
        print("done" if run.started else "failed (see restore.log)", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="latency_sweep", help="output path prefix (default: ./latency_sweep)")
    for a in AXES:
        ap.add_argument(a.flag, dest=a.name, nargs="+", default=a.default,
                        help=f"{a.help}. Default: {' '.join(a.default)} (first = baseline)")
    ap.add_argument("--phase", choices=["all", "screen", "full"], default="all",
                    help="screen only, full sweep only (needs finished screening), or both (default)")
    ap.add_argument("--baseline-repeats", type=int, default=3, help="baseline runs during screening (default 3)")
    ap.add_argument("--min-effect-ms", type=float, default=10.0,
                    help="smallest latency change counted as an effect (default 10 ms)")
    ap.add_argument("--noise-factor", type=float, default=3.0,
                    help="effect must also exceed this many baseline stdevs (default 3)")
    ap.add_argument("--fps-effect", type=float, default=0.10,
                    help="an fps change larger than this fraction also counts as an effect (default 0.10)")
    ap.add_argument("--max-full-runs", type=int, default=300,
                    help="refuse a full sweep larger than this many runs (default 300, ~3.5 h)")
    ap.add_argument("--iso", type=int, default=0, help="ISO for manual:<N> exposures (0 = camera default)")
    ap.add_argument("--report-period", type=float, default=5.0, help="latency_report_period_s")
    ap.add_argument("--warmup-reports", type=int, default=1, help="report windows discarded at the start")
    ap.add_argument("--reports", type=int, default=4, help="report windows kept per run")
    ap.add_argument("--start-timeout", type=float, default=45.0, help="max wait for 'Live streaming started' (s)")
    ap.add_argument("--stop-timeout", type=float, default=15.0, help="max wait for clean shutdown after SIGINT (s)")
    ap.add_argument("--cooldown", type=float, default=3.0, help="pause between runs (s)")
    ap.add_argument("--stream-index", type=int, default=0, help="stream index to evaluate")
    ap.add_argument("--fps-tolerance", type=float, default=0.10, help="fps MATCH tolerance (fraction)")
    ap.add_argument("--bitrate-tolerance", type=float, default=0.50, help="bitrate MATCH tolerance (fraction)")
    ap.add_argument("--params-file", default=None, help="driver YAML (default: installed insta360_ros_driver.yaml)")
    ap.add_argument("--executable", default=None, help="driver executable (default: found via ros2 pkg prefix)")
    ap.add_argument("--no-restore", action="store_true",
                    help="do not reset the camera's sub-mode / image settings to baseline at the end")
    ap.add_argument("--dry-run", action="store_true", help="only print the plan and estimated time")
    ap.add_argument("--summary-only", action="store_true", help="only print the report for an existing <out>.csv")
    args = ap.parse_args()

    axis_values = {a.name: list(getattr(args, a.name)) for a in AXES}
    try:
        for a in AXES:
            validate_values(a, axis_values[a.name])
    except ValueError as e:
        ap.error(str(e))

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    summary_path = out.with_name(out.name + ".csv")
    windows_path = out.with_name(out.name + "_windows.csv")
    report_path = out.with_name(out.name + "_report.txt")
    log_dir = out.with_name(out.name + "_logs")
    store = Store(summary_path, windows_path)

    codec_baseline = baseline_codec(axis_values["codec"])
    per_run = 10 + (args.warmup_reports + args.reports) * args.report_period + 5 + args.cooldown

    if args.summary_only:
        write_report(store, axis_values, codec_baseline, args, report_path)
        return 0

    # ---- plan / dry run
    s_plan = screening_plan(axis_values, codec_baseline, args.baseline_repeats)
    s_todo = [(c, r) for c, r in s_plan if not store.has("screen", config_id(c), r)]
    if args.dry_run:
        print(f"Screening: {len(s_plan)} runs ({len(s_plan) - len(s_todo)} done), ~{len(s_todo) * per_run / 60:.0f} min")
        for cfg, rep in s_todo:
            print(f"  {config_id(cfg)}" + (f" (rep {rep})" if rep else ""))
        if not s_todo:
            _, full_axes, _ = analyse_screening(store, axis_values, codec_baseline, args)
            f_plan = full_plan(axis_values, codec_baseline, full_axes, None)
            f_todo = [c for c in f_plan if not store.ok_anywhere(config_id(c))]
            print(f"Full sweep over {list(full_axes) or 'nothing'}: {len(f_plan)} configurations, "
                  f"{len(f_todo)} to run, ~{len(f_todo) * per_run / 60:.0f} min")
        else:
            worst = 1
            for name, vals in axis_values.items():
                worst *= len(vals) if not (name == "codec" and vals == ["current"]) else 1
            print(f"Full sweep: decided after screening (at most {worst} configurations if every axis has an effect)")
        return 0

    dropped = store.drop_unfinished()
    if dropped:
        print(f"Retrying {dropped} run(s) that did not finish cleanly before", flush=True)

    exe = args.executable or find_driver_executable()
    exe_cmd = [exe] if exe else ["ros2", "run", "insta360_ros_driver", "insta360_ros_driver"]
    params_file = args.params_file or find_params_file()
    log_dir.mkdir(parents=True, exist_ok=True)
    sweeper = Sweeper(args, store, axis_values, log_dir, exe_cmd, params_file)

    def on_sigint(signum, frame):
        if sweeper.stop_requested:
            raise KeyboardInterrupt
        sweeper.stop_requested = True
        print("\nStopping after the current run (Ctrl+C again to abort immediately)...", flush=True)

    signal.signal(signal.SIGINT, on_sigint)

    try:
        # ---- phase 1: screening
        if args.phase in ("all", "screen"):
            s_todo = [(c, r) for c, r in s_plan if not store.has("screen", config_id(c), r)]
            print(f"\nScreening: {len(s_plan)} runs, {len(s_todo)} to do (~{len(s_todo) * per_run / 60:.0f} min)",
                  flush=True)
            sweeper.count = 0
            for cfg, rep in s_todo:
                if sweeper.stop_requested or sweeper.quit:
                    break
                sweeper.execute(cfg, "screen", rep, len(s_todo))
            write_report(store, axis_values, codec_baseline, args, report_path, quiet=True)  # printed at the end

        # ---- phase 2: full sweep
        if args.phase in ("all", "full") and not (sweeper.stop_requested or sweeper.quit):
            if any(not store.has("screen", config_id(c), r) for c, r in s_plan):
                print("Screening is not complete; run it first (--phase screen or --phase all).", flush=True)
                return 1
            verdicts, full_axes, screen_lines = analyse_screening(store, axis_values, codec_baseline, args)
            print("\n" + "\n".join(screen_lines), flush=True)
            if any(v["verdict"] == "incomplete" for v in verdicts.values()):
                print("Screening is not complete; finish it before the full sweep.")
                return 1
            if not full_axes:
                print("No axis has an effect on latency -> nothing to sweep in the full phase.", flush=True)
            else:
                f_plan = full_plan(axis_values, codec_baseline, full_axes, sweeper.camera_codec)
                f_todo = [c for c in f_plan if not store.ok_anywhere(config_id(c))]
                print(f"Full sweep over {', '.join(full_axes)}: {len(f_plan)} configurations, "
                      f"{len(f_plan) - len(f_todo)} already measured, {len(f_todo)} to run "
                      f"(~{len(f_todo) * per_run / 60:.0f} min)", flush=True)
                if len(f_todo) > args.max_full_runs:
                    print(f"That is more than --max-full-runs {args.max_full_runs}. Narrow the value lists of "
                          f"those axes, or raise --max-full-runs to run it anyway.", flush=True)
                else:
                    sweeper.count = 0
                    for cfg in f_todo:
                        if sweeper.stop_requested or sweeper.quit:
                            break
                        sweeper.execute(cfg, "full", 0, len(f_todo))
    except KeyboardInterrupt:
        print("\nAborted.", flush=True)
    finally:
        if sweeper.touched_camera_modes and not args.no_restore:
            try:
                sweeper.restore()
            except KeyboardInterrupt:
                pass
        if (codec_baseline not in (None, "current") and sweeper.camera_codec
                and sweeper.camera_codec != codec_baseline):
            print(f"\nNote: the camera is still set to {sweeper.camera_codec.upper()}; it was "
                  f"{codec_baseline.upper()} when the sweep started.", flush=True)
        print(f"\nResults: {summary_path}\nWindows: {windows_path}\nLogs:    {log_dir}", flush=True)
        write_report(store, axis_values, codec_baseline, args, report_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
