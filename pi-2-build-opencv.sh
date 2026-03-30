#!/bin/bash
# Build OpenCV from source on Raspberry Pi 5 with Arducam / libcamera support
# Optimized for: aarch64 (Cortex-A76), GStreamer, V4L2, Python bindings
# Target OS: Raspberry Pi OS Bookworm (Debian 12, 64-bit)

set -e

echo "=== Building OpenCV for Raspberry Pi 5 ==="
echo "    Optimized for aarch64, libcamera/V4L2, GStreamer, Python bindings"
echo ""

# ── Configuration ─────────────────────────────────────────────────────────────
OPENCV_VERSION="4.10.0"
BUILD_DIR="${HOME}/opencv-build"
INSTALL_PREFIX="/usr/local"
NUM_CORES=$(nproc)
# Leave one core free to prevent OOM kill on less-RAM Pi configurations
BUILD_CORES=$(( NUM_CORES > 1 ? NUM_CORES - 1 : 1 ))

echo "    OpenCV version : $OPENCV_VERSION"
echo "    Build directory: $BUILD_DIR"
echo "    Install prefix : $INSTALL_PREFIX"
echo "    Build cores    : $BUILD_CORES / $NUM_CORES"
echo ""

# ── Guard rails ───────────────────────────────────────────────────────────────
if [ "$EUID" -eq 0 ]; then
    echo "ERROR: Do not run as root."
    exit 1
fi

FREE_DISK=$(df -BG . | awk 'NR==2 {gsub("G",""); print $4}')
if [ "$FREE_DISK" -lt 4 ]; then
    echo "ERROR: At least 4 GB of free disk space is required (found ${FREE_DISK} GB)."
    exit 1
fi

# Confirm swap is adequate (OpenCV build peaks at ~2 GB RAM + swap)
TOTAL_MEM=$(free -m | awk '/Mem/ {print $2}')
TOTAL_SWAP=$(free -m | awk '/Swap/ {print $2}')
TOTAL=$(( TOTAL_MEM + TOTAL_SWAP ))
if [ "$TOTAL" -lt 3000 ]; then
    echo "WARNING: Combined RAM + swap is only ${TOTAL} MB."
    echo "         Run pi-1-install-dependencies.sh first to increase swap."
    read -rp "Continue anyway? [y/N] " yn
    [[ "$yn" =~ ^[Yy]$ ]] || exit 1
fi

# ── Locate Python 3 executable ────────────────────────────────────────────────
PYTHON3_BIN=$(which python3)
PYTHON3_VER=$("$PYTHON3_BIN" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
PYTHON3_INC=$("$PYTHON3_BIN" -c "import sysconfig; print(sysconfig.get_path('include'))")
PYTHON3_NUMPY=$("$PYTHON3_BIN" -c "import numpy; print(numpy.get_include())" 2>/dev/null || echo "")

echo "    Python3 executable : $PYTHON3_BIN  (v${PYTHON3_VER})"
echo "    Python3 include    : $PYTHON3_INC"
[ -n "$PYTHON3_NUMPY" ] && echo "    NumPy include      : $PYTHON3_NUMPY"
echo ""

# ── Source download ───────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -d "opencv" ]; then
    echo ">>> Cloning OpenCV $OPENCV_VERSION..."
    git clone --depth 1 --branch "$OPENCV_VERSION" \
        https://github.com/opencv/opencv.git
else
    echo ">>> OpenCV source already present, skipping clone."
fi

if [ ! -d "opencv_contrib" ]; then
    echo ">>> Cloning opencv_contrib $OPENCV_VERSION..."
    git clone --depth 1 --branch "$OPENCV_VERSION" \
        https://github.com/opencv/opencv_contrib.git
else
    echo ">>> opencv_contrib already present, skipping clone."
fi

# ── CMake configure ───────────────────────────────────────────────────────────
cd "$BUILD_DIR/opencv"

# Always start with a clean build dir to avoid stale cache issues
if [ -d "build" ]; then
    echo ">>> Removing previous build directory for a clean configure..."
    rm -rf build
fi
mkdir build
cd build

echo ""
echo ">>> Configuring OpenCV with cmake..."

# Verify GStreamer dev headers are present — OpenCV silently disables GStreamer
# if gstreamer-app-1.0 pkg-config entry is missing.
echo "    Checking GStreamer development headers..."
for gst_pkg in gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0 gstreamer-app-1.0; do
    if ! pkg-config --exists "$gst_pkg" 2>/dev/null; then
        echo "ERROR: pkg-config package '$gst_pkg' not found."
        echo "       Run ./pi-1-install-dependencies.sh first to install GStreamer dev libs."
        exit 1
    fi
done
echo "    GStreamer headers: OK"
LIBCAMERA_FLAG="-DWITH_LIBCAMERA=OFF"
if pkg-config --exists libcamera 2>/dev/null; then
    LIBCAMERA_FLAG="-DWITH_LIBCAMERA=ON"
    echo "    libcamera detected — enabling WITH_LIBCAMERA"
fi

cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    \
    `# ── ARM / Pi 5 CPU optimizations ──────────────────────────────────────` \
    -D CMAKE_CXX_FLAGS="-march=armv8-a+crc+simd -mtune=cortex-a76" \
    -D CMAKE_C_FLAGS="-march=armv8-a+crc+simd -mtune=cortex-a76" \
    -D ENABLE_NEON=ON \
    -D CPU_BASELINE=NEON \
    -D CPU_DISPATCH="" \
    -D WITH_OPENMP=ON \
    \
    `# ── Camera backends ────────────────────────────────────────────────────` \
    -D WITH_V4L=ON \
    -D WITH_LIBV4L=ON \
    "$LIBCAMERA_FLAG" \
    \
    `# ── GStreamer (libcamera → GStreamer → OpenCV pipeline) ─────────────────` \
    -D WITH_GSTREAMER=ON \
    \
    `# ── Display backend: Qt5 (consistent with laptop build)` \
    -D WITH_GTK=OFF \
    -D WITH_QT=ON \
    -D WITH_OPENGL=ON \
    \
    `# ── Codecs / image I-O ─────────────────────────────────────────────────` \
    -D WITH_FFMPEG=ON \
    -D WITH_JPEG=ON \
    -D WITH_PNG=ON \
    -D WITH_TIFF=ON \
    -D WITH_OPENEXR=ON \
    \
    `# ── Math / linear algebra ──────────────────────────────────────────────` \
    -D WITH_EIGEN=ON \
    -D WITH_TBB=ON \
    -D WITH_OPENBLAS=ON \
    \
    `# ── Python bindings ────────────────────────────────────────────────────` \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE="$PYTHON3_BIN" \
    -D PYTHON3_INCLUDE_DIR="$PYTHON3_INC" \
    ${PYTHON3_NUMPY:+-D PYTHON3_NUMPY_INCLUDE_DIRS="$PYTHON3_NUMPY"} \
    \
    `# ── Trim unused modules to reduce build time / RAM ─────────────────────` \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_opencv_apps=OFF \
    -D BUILD_opencv_java=OFF \
    -D BUILD_opencv_js=OFF \
    \
    `# ── contrib modules (needed for tracking, aruco, etc.) ──────────────────` \
    -D OPENCV_EXTRA_MODULES_PATH="$BUILD_DIR/opencv_contrib/modules" \
    -D BUILD_opencv_xfeatures2d=ON \
    -D BUILD_opencv_tracking=ON \
    -D BUILD_opencv_sfm=OFF \
    -D BUILD_opencv_dnn_superres=OFF \
    \
    `# ── Disable removed/unavailable libs ───────────────────────────────────` \
    -D WITH_GLOG=OFF \
    -D WITH_GFLAGS=OFF \
    ..

# ── Build ─────────────────────────────────────────────────────────────────────
echo ""
echo ">>> Building OpenCV with $BUILD_CORES cores..."
echo "    This typically takes 30–60 minutes on a Raspberry Pi 5."
TIME_START=$(date +%s)

make -j"$BUILD_CORES"

TIME_END=$(date +%s)
ELAPSED=$(( (TIME_END - TIME_START) / 60 ))
echo ""
echo "    Build finished in ~${ELAPSED} minutes."

# ── Install ───────────────────────────────────────────────────────────────────
echo ""
echo ">>> Installing OpenCV to $INSTALL_PREFIX..."
sudo make install
sudo ldconfig

# ── Link Python bindings into site-packages if needed ────────────────────────
PYTHON_SITE=$("$PYTHON3_BIN" -c "import site; print(site.getsitepackages()[0])")
CV2_SO=$(find "$INSTALL_PREFIX/lib/python${PYTHON3_VER}" -name "cv2*.so" 2>/dev/null | head -1)
if [ -n "$CV2_SO" ] && [ ! -f "$PYTHON_SITE/cv2.so" ]; then
    echo ">>> Linking cv2 module into Python site-packages..."
    sudo ln -sf "$CV2_SO" "$PYTHON_SITE/cv2.so"
fi

# ── Verify installation ───────────────────────────────────────────────────────
echo ""
echo ">>> Verifying OpenCV installation..."
INSTALLED_VER=$(pkg-config --modversion opencv4 2>/dev/null || echo "not found via pkg-config")
echo "    pkg-config version : $INSTALLED_VER"

CV2_PYTHON=$("$PYTHON3_BIN" -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "import failed")
echo "    Python cv2 version : $CV2_PYTHON"

GS_SUPPORT=$("$PYTHON3_BIN" -c "import cv2; info=cv2.getBuildInformation(); print('YES' if 'GStreamer' in info and 'YES' in info[info.find('GStreamer'):info.find('GStreamer')+30] else 'NO')" 2>/dev/null || echo "unknown")
echo "    GStreamer support  : $GS_SUPPORT"

echo ""
echo "=== OpenCV $OPENCV_VERSION built and installed successfully! ==="
echo ""
echo "Next step:"
echo "  ./pi-3-build.sh   # Build the tracker project"
