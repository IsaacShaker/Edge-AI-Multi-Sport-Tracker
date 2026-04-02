#!/bin/bash
# Build the Edge AI Multi-Sport Tracker server on Raspberry Pi 5
# Applies ARM Cortex-A76 optimizations; links against locally-installed OpenCV

set -e

echo "=== Building Edge AI Multi-Sport Tracker (Raspberry Pi 5) ==="
echo ""

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
# OpenCV installed to /usr/local — add its pkgconfig dir to the search path
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# Also tell cmake where to find the OpenCV config directly
OPENCV_CMAKE_DIR="/usr/local/lib/cmake/opencv4"

if ! pkg-config --exists opencv4 2>/dev/null; then
    if [ ! -f "$OPENCV_CMAKE_DIR/OpenCVConfig.cmake" ]; then
        echo "ERROR: OpenCV 4 not found. Run ./pi-2-build-opencv.sh first."
        exit 1
    fi
    echo "    OpenCV found via cmake config (pkg-config path not set)"
else
    OPENCV_VER=$(pkg-config --modversion opencv4)
    echo "    OpenCV version: $OPENCV_VER"
    echo "    Prefix        : $(pkg-config --variable=prefix opencv4)"
fi
echo ""

# ── Build directory ───────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# ── CMake configure ───────────────────────────────────────────────────────────
echo ">>> Running CMake..."
cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_CXX_FLAGS="-march=armv8-a+crc+simd -mtune=cortex-a76 -O3" \
    -D CMAKE_C_FLAGS="-march=armv8-a+crc+simd -mtune=cortex-a76 -O3" \
    -D OpenCV_DIR="$OPENCV_CMAKE_DIR" \
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
echo "Next step:"
echo "  ./pi-4-run.sh            # Run with Arducam (headless, no display)"
echo "  ./pi-4-run.sh --display  # Run with display (requires X11 / connected monitor)"
