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
#   3. Run ./pi-3-build.sh  (or ./pi-3-build.sh --hailo for NPU)
#
# Usage:
#   ./pi-4-run.sh [OPTIONS]
#
# Options:
#   --config FILE      Path to config file           (default: tracker.conf)
#   --vision TYPE      yolo | hailo | color_based    (default: from config)
#   --estimator TYPE   kalman_cv | kalman_ca | imm   (default: from config)
#   --motor TYPE       mock | simplefoc              (default: from config)
#   --serial-port DEV  Serial port for simplefoc     (default: from config)
#   --width W          Capture width  in pixels      (default: from config)
#   --height H         Capture height in pixels      (default: from config)
#   --fps F            Target frames per second       (default: from config)
#   --display          Enable display window (requires monitor / X11)
#   --no-viz           Disable display (headless mode)
#   --web-stream       Enable web dashboard on port 8080
#   --port P           Web streaming port            (default: from config)
#   --help             Show this message

set -e

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/server/build"
EXECUTABLE="$BUILD_DIR/bin/tracker_server"

# ── Load config file (sets all defaults) ─────────────────────────────────────
CONFIG_FILE="$SCRIPT_DIR/tracker.conf"
# Allow --config to appear as the very first argument before sourcing
if [[ "$1" == "--config" && -n "$2" ]]; then
    CONFIG_FILE="$2"
fi
if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Config file not found: $CONFIG_FILE"
    exit 1
fi
# shellcheck source=tracker.conf
source "$CONFIG_FILE"

# Model paths — binary runs from server/build/bin/ and resolves ../../models/
BUILD_MODELS_DIR="$SCRIPT_DIR/server/build/models"
YOLO_MODEL="$BUILD_MODELS_DIR/yolov8n.onnx"
HAILO_MODEL="$BUILD_MODELS_DIR/yolov8n.hef"

# ── Argument parsing (overrides config values) ───────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --config)     CONFIG_FILE="$2"; shift 2 ;;  # already handled above, skip
        --estimator)  ESTIMATOR="$2";    shift 2 ;;
        --motor)      MOTOR="$2";        shift 2 ;;
        --serial-port) SERIAL_PORT="$2"; shift 2 ;;
        --vision)     VISION="$2";      shift 2 ;;
        --width)      CAP_WIDTH="$2";   shift 2 ;;
        --height)     CAP_HEIGHT="$2"; shift 2 ;;
        --fps)        CAP_FPS="$2";    shift 2 ;;
        --display)    DISPLAY_ENABLED=true; shift ;;
        --no-viz)     DISPLAY_ENABLED=false; shift ;;
        --web-stream) WEB_STREAM=true; shift ;;
        --port)       WEB_PORT="$2"; shift 2 ;;
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
# Rebuild when: binary missing, OR hailo requested but HEF not yet downloaded
# (the latter means a prior non-hailo build exists and needs to be replaced).
NEEDS_BUILD=false
[ ! -f "$EXECUTABLE" ] && NEEDS_BUILD=true
[ "$VISION" = "hailo" ] && [ ! -f "$HAILO_MODEL" ] && NEEDS_BUILD=true

if [ "$NEEDS_BUILD" = true ]; then
    if [ "$VISION" = "hailo" ]; then
        echo ">>> Building with Hailo NPU support..."
        "$SCRIPT_DIR/pi-3-build.sh" --hailo
    else
        echo ">>> Binary not found — building project first..."
        "$SCRIPT_DIR/pi-3-build.sh"
    fi
    echo ""
fi

# ── Select model path based on vision backend ────────────────────────────────
if [ "$VISION" = "hailo" ]; then
    if [ ! -f "$HAILO_MODEL" ]; then
        echo "ERROR: Hailo HEF model still not found at $HAILO_MODEL after build."
        echo "       Compile manually with the Hailo Model Zoo and place it there."
        exit 1
    fi
elif [ "$VISION" = "color_based" ]; then
    : # No model file needed for color_based
else
    VISION="yolo"
    if [ ! -f "$YOLO_MODEL" ]; then
        echo "ERROR: ONNX model not found at $YOLO_MODEL"
        echo "       Run ./pi-3-build.sh or copy yolov8n.onnx to server/build/models/"
        exit 1
    fi
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
# af-mode=2 = continuous autofocus (required for Arducam IMX519 motorised lens).
PIPELINE="libcamerasrc af-mode=2 ! video/x-raw,colorimetry=bt709,format=NV12,width=${CAP_WIDTH},height=${CAP_HEIGHT},framerate=${CAP_FPS}/1 ! queue ! videoconvert ! appsink drop=true max-buffers=2 sync=false"

echo ">>> Camera pipeline:"
echo "    $PIPELINE"

# ── Display / viz flag ────────────────────────────────────────────────────────
VIZ_FLAG=""
if [ "$DISPLAY_ENABLED" = false ]; then
    VIZ_FLAG="--no-viz"
fi

# ── Web stream flag ───────────────────────────────────────────────────────────
STREAM_FLAGS=""
if [ "$WEB_STREAM" = true ]; then
    STREAM_FLAGS="--web-stream --port $WEB_PORT"
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
# HailoRT shared library lives in /usr/local/lib — add it when running hailo.
if [ "$VISION" = "hailo" ]; then
    export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
fi
# Do NOT prepend /usr/local/lib globally here — it may contain an older OpenCV build
# that would shadow the system 4.10.0 libs the binary was compiled against.
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# ── Launch ────────────────────────────────────────────────────────────────────
echo ""
echo "=== Starting Edge AI Multi-Sport Tracker ==="
echo "    Vision    : $VISION"
echo "    Estimator : $ESTIMATOR"
echo "    Motor     : $MOTOR$([ "$MOTOR" = "simplefoc" ] && echo " ($SERIAL_PORT)" || true)"
echo "    Camera    : libcamerasrc + queue (Arducam IMX519)"
echo "    Resolution: ${CAP_WIDTH}x${CAP_HEIGHT} @ ${CAP_FPS} fps"
echo "    Display   : $([ "$DISPLAY_ENABLED" = true ] && echo "enabled" || echo "disabled (headless)")"
if [ "$WEB_STREAM" = true ]; then
    PI_IP=$(hostname -I | awk '{print $1}')
    echo "    Web UI    : http://${PI_IP}:${WEB_PORT}"
fi
echo ""

cd "$BUILD_DIR"

if [ "$VISION" = "hailo" ]; then
    MODEL_PATH="$HAILO_MODEL"
elif [ "$VISION" = "color_based" ]; then
    MODEL_PATH=""
else
    MODEL_PATH="$YOLO_MODEL"
fi

# shellcheck disable=SC2086
if [ -n "$MODEL_PATH" ]; then
    ./bin/tracker_server \
        --vision      "$VISION" \
        --model       "$MODEL_PATH" \
        --estimator   "$ESTIMATOR" \
        --motor       "$MOTOR" \
        --serial-port "$SERIAL_PORT" \
        --camera      "$PIPELINE" \
        $VIZ_FLAG \
        $STREAM_FLAGS
else
    ./bin/tracker_server \
        --vision      "$VISION" \
        --estimator   "$ESTIMATOR" \
        --motor       "$MOTOR" \
        --serial-port "$SERIAL_PORT" \
        --camera      "$PIPELINE" \
        $VIZ_FLAG \
        $STREAM_FLAGS
fi
