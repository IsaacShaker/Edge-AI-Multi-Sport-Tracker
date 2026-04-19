#!/bin/bash
# Build the Edge AI Multi-Sport Tracker server on Raspberry Pi 5
# Applies ARM Cortex-A76 optimizations; links against locally-installed OpenCV
#
# Usage:
#   ./pi-3-build.sh           # build with YOLO (OpenCV DNN) only
#   ./pi-3-build.sh --hailo   # build with Hailo NPU support (requires HailoRT SDK)

set -e

echo "=== Building Edge AI Multi-Sport Tracker (Raspberry Pi 5) ==="
echo ""

# ── Parse args ────────────────────────────────────────────────────────────────
USE_HAILO=OFF
for arg in "$@"; do
    case $arg in
        --hailo) USE_HAILO=ON ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

# ── Locate sources ────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVER_DIR="$SCRIPT_DIR/server"
BUILD_DIR="$SERVER_DIR/build"

if [ ! -f "$SERVER_DIR/CMakeLists.txt" ]; then
    echo "ERROR: Cannot find $SERVER_DIR/CMakeLists.txt"
    echo "       Run this script from the project root directory."
    exit 1
fi

# ── Verify OpenCV is available ────────────────────────────────────────────────
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

if ! pkg-config --exists opencv4 2>/dev/null; then
    echo "ERROR: OpenCV 4 not found. Run ./pi-2-build-opencv.sh first."
    exit 1
fi

OPENCV_VER=$(pkg-config --modversion opencv4)
OPENCV_PREFIX=$(pkg-config --variable=prefix opencv4)
# Derive the cmake config dir from the actual lib dir reported by pkg-config.
# On Debian/Ubuntu aarch64 the layout is <prefix>/lib/aarch64-linux-gnu/cmake/opencv4.
# This avoids picking up a stale /usr/local/lib/cmake/opencv4 from an older manual build.
OPENCV_LIB_DIR=$(pkg-config --variable=libdir opencv4)
OPENCV_CMAKE_DIR="$OPENCV_LIB_DIR/cmake/opencv4"
echo "    OpenCV version: $OPENCV_VER"
echo "    Prefix        : $OPENCV_PREFIX"
echo "    CMake dir     : $OPENCV_CMAKE_DIR"
echo ""

# ── Build directory ───────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# ── CMake configure ───────────────────────────────────────────────────────────
echo ">>> Running CMake..."
echo "    Hailo NPU support: $USE_HAILO"
cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_CXX_FLAGS="-march=armv8-a+crc+simd -mtune=cortex-a76 -O3" \
    -D CMAKE_C_FLAGS="-march=armv8-a+crc+simd -mtune=cortex-a76 -O3" \
    -D OpenCV_DIR="$OPENCV_CMAKE_DIR" \
    -D WITH_HAILO="$USE_HAILO" \
    "$SERVER_DIR"

# ── Compile ───────────────────────────────────────────────────────────────────
# Use all-but-one core to prevent OOM on Pi
NUM_CORES=$(nproc)
BUILD_CORES=$(( NUM_CORES > 1 ? NUM_CORES - 1 : 1 ))
echo ""
echo ">>> Compiling with $BUILD_CORES / $NUM_CORES cores..."
make -j"$BUILD_CORES"

# ── Verify output ─────────────────────────────────────────────────────────────
BINARY="$BUILD_DIR/bin/tracker_server"
if [ ! -f "$BINARY" ]; then
    echo ""
    echo "ERROR: Expected binary not found at $BINARY"
    exit 1
fi

echo ""
echo "=== Build complete! ==="
echo "    Binary: $BINARY"
echo ""

# ── Download YOLOv8n ONNX model if not already present ───────────────────────
# Model lives in server/build/models/ — the binary resolves ../../models/ from bin/
# The ONNX is not on the releases page, so we download the .pt and export it.
MODELS_DIR="$BUILD_DIR/models"
ONNX_MODEL="$MODELS_DIR/yolov8n.onnx"

if [ ! -f "$ONNX_MODEL" ] || [ "$(wc -c < "$ONNX_MODEL")" -lt 1000000 ]; then
    echo "=== Downloading and exporting YOLOv8n ONNX model ==="
    mkdir -p "$MODELS_DIR"
    PT_URL="https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.pt"
    PT_FILE="$MODELS_DIR/yolov8n.pt"
    if command -v curl &>/dev/null; then
        curl -L "$PT_URL" -o "$PT_FILE"
    else
        wget -q "$PT_URL" -O "$PT_FILE"
    fi
    # Export to ONNX using ultralytics Python package
    python3 -c "from ultralytics import YOLO; YOLO('$PT_FILE').export(format='onnx', imgsz=640, opset=12, simplify=True)" \
        && mv "${PT_FILE%.pt}.onnx" "$ONNX_MODEL" \
        && echo "    Model saved to $ONNX_MODEL"
    rm -f "$PT_FILE"
else
    echo "YOLOv8n model already exists: $ONNX_MODEL"
fi

# ── Download Hailo HEF model if building with Hailo support ──────────────────
# Pre-compiled yolov8n HEF for Hailo-8L from the Hailo Model Zoo S3 bucket.
# The model detects all 80 COCO classes (including sports ball, class 32).
if [ "$USE_HAILO" = "ON" ]; then
    HEF_MODEL="$MODELS_DIR/yolov8n.hef"
    HEF_URL="https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/yolov8n.hef"
    if [ ! -f "$HEF_MODEL" ] || [ "$(wc -c < "$HEF_MODEL")" -lt 100000 ]; then
        echo "=== Downloading YOLOv8n HEF for Hailo-8L ==="
        mkdir -p "$MODELS_DIR"
        if command -v curl &>/dev/null; then
            curl -L "$HEF_URL" -o "$HEF_MODEL"
        else
            wget -q "$HEF_URL" -O "$HEF_MODEL"
        fi
        if [ ! -f "$HEF_MODEL" ] || [ "$(wc -c < "$HEF_MODEL")" -lt 100000 ]; then
            echo ""
            echo "WARNING: HEF download failed or file is too small."
            echo "         Compile manually with the Hailo Model Zoo and place at:"
            echo "         $HEF_MODEL"
        else
            echo "    HEF model saved to $HEF_MODEL"
        fi
    else
        echo "YOLOv8n HEF model already exists: $HEF_MODEL"
    fi
fi

echo ""
echo "Next step:"
echo "  ./pi-4-run.sh            # Run with Arducam (headless, no display)"
echo "  ./pi-4-run.sh --display  # Run with display (requires X11 / connected monitor)"
