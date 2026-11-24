#!/bin/bash
# Toggles GStreamer encode/decode timing (the built-in "latency" tracer) in the CURRENT shell.
# Must be sourced:  source scripts/gst_timing.sh
# GStreamer processes started from this shell afterwards log per-element times (in ns) to stderr.

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "gst_timing.sh must be sourced: source $0" >&2
    exit 2
fi

if [ -n "${GST_TRACERS-}" ]; then
    unset GST_TRACERS GST_DEBUG
    echo "GStreamer timing OFF"
else
    export GST_TRACERS="latency(flags=pipeline+element)"
    export GST_DEBUG="GST_TRACER:7"
    echo "GStreamer timing ON"
fi
