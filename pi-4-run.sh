#!/bin/bash
# Run the Edge AI Multi-Sport Tracker on Raspberry Pi 5 with Arducam
#
# Camera strategy: OpenCV reads a GStreamer libcamerasrc pipeline directly.
# This is the native, zero-overhead way to use the Pi 5 libcamera stack
# (IMX519, IMX477, IMX708, etc.) — no V4L2 workarounds needed.
#
# Prerequisites:
#   1. Run ./pi-1-install-dependencies.sh
#   2. Run ./pi-2-build-opencv.sh   (must show GStreamer: YES)
#   3. Run ./pi-3-build.sh
#
# Usage:
#   ./pi-4-run.sh [OPTIONS]
#
# Options:
#   --estimator TYPE   kalman_cv | kalman_ca | imm  (default: imm)
#   --motor TYPE       mock | simplefoc             (default: mock)
#   --serial-port DEV  Serial port for simplefoc    (default: /dev/ttyACM0)
#   --width W          Capture width  in pixels     (default: 1280)
#   --height H         Capture height in pixels     (default: 720)
#   --fps F            Target frames per second      (default: 30)
#   --display          Enable display window (requires monitor / X11)
#   --help             Show this message

set -e

# ── Defaults ──────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/server/build"
EXECUTABLE="$BUILD_DIR/bin/tracker_server"

ESTIMATOR="imm"
MOTOR="simplefoc"
SERIAL_PORT="/dev/ttyACM0"
CAP_WIDTH=1280
CAP_HEIGHT=720
CAP_FPS=30
DISPLAY_ENABLED=true    # display enabled by default

# ── Argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --estimator)  ESTIMATOR="$2";    shift 2 ;;
        --motor)      MOTOR="$2";        shift 2 ;;
        --serial-port) SERIAL_PORT="$2"; shift 2 ;;
        --width)      CAP_WIDTH="$2";   shift 2 ;;
        --height)     CAP_HEIGHT="$2"; shift 2 ;;
        --fps)        CAP_FPS="$2";    shift 2 ;;
        --display)    DISPLAY_ENABLED=true; shift ;;
        --help)
            grep '^#' "$0" | grep -v '!/bin' | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Unknown option: $1  (run with --help for usage)"
            exit 1
            ;;
    esac
done

# ── Build if necessary ────────────────────────────────────────────────────────
if [ ! -f "$EXECUTABLE" ]; then
    echo ">>> Binary not found — building project first..."
    "$SCRIPT_DIR/pi-3-build.sh"
    echo ""
fi

# ── Verify libcamerasrc is available ─────────────────────────────────────────
# libcamerasrc + queue + appsink is the confirmed working pipeline on RPi5/PiSP.
# The queue element is critical: it forces a DMA-BUF → system RAM buffer copy
# so that appsink (and OpenCV) can access the frame data in CPU memory.
# Without queue, appsink receives DMA-BUF-only memory it cannot read → error.
echo ">>> Checking GStreamer libcamerasrc element..."
if ! gst-inspect-1.0 libcamerasrc &>/dev/null 2>&1; then
    echo ""
    echo "ERROR: GStreamer element 'libcamerasrc' not found."
    echo "  sudo apt install -y gstreamer1.0-libcamera"
    exit 1
fi
echo "    libcamerasrc: OK"

# ── Build GStreamer pipeline ──────────────────────────────────────────────────
# colorimetry=bt709 + format=NV12 forces the PiSP ISP into processed output mode.
# queue after caps: copies buffers from GPU/DMA-BUF into system RAM.
# videoconvert WITHOUT an explicit format capsfilter: OpenCV sets its own caps
# on appsink (usually BGRx) and videoconvert fulfils that request. Having an
# explicit 'video/x-raw,format=BGR' capsfilter *before* appsink conflicts with
# whatever format OpenCV writes onto appsink, preventing caps negotiation.
PIPELINE="libcamerasrc ! video/x-raw,colorimetry=bt709,format=NV12,width=${CAP_WIDTH},height=${CAP_HEIGHT},framerate=${CAP_FPS}/1 ! queue ! videoconvert ! appsink drop=true max-buffers=2 sync=false"

echo ">>> Camera pipeline:"
echo "    $PIPELINE"

# ── Display / viz flag ────────────────────────────────────────────────────────
VIZ_FLAG=""
if [ "$DISPLAY_ENABLED" = false ]; then
    VIZ_FLAG="--no-viz"
fi

# ── Environment ───────────────────────────────────────────────────────────────
if [ "$DISPLAY_ENABLED" = false ]; then
    export DISPLAY=""
else
    export DISPLAY="${DISPLAY:-:0}"
    export QT_QPA_PLATFORM=xcb
fi

# Silence GStreamer debug noise; set to 2 for troubleshooting pipeline issues
export GST_DEBUG=0
export G_MESSAGES_DEBUG=""
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# ── Launch ────────────────────────────────────────────────────────────────────
echo ""
echo "=== Starting Edge AI Multi-Sport Tracker ==="
echo "    Estimator : $ESTIMATOR"
echo "    Motor     : $MOTOR$([ "$MOTOR" = "simplefoc" ] && echo " ($SERIAL_PORT)" || true)"
echo "    Camera    : libcamerasrc + queue (Arducam IMX519)"
echo "    Resolution: ${CAP_WIDTH}x${CAP_HEIGHT} @ ${CAP_FPS} fps"
echo "    Display   : $([ "$DISPLAY_ENABLED" = true ] && echo "enabled" || echo "disabled (headless)")"
echo ""

cd "$BUILD_DIR"

# shellcheck disable=SC2086
./bin/tracker_server \
    --estimator   "$ESTIMATOR" \
    --motor       "$MOTOR" \
    --serial-port "$SERIAL_PORT" \
    --camera      "$PIPELINE" \
    $VIZ_FLAG
