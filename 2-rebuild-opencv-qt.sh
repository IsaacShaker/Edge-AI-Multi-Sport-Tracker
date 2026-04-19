#!/bin/bash
# Rebuild OpenCV with Qt backend for Wayland compatibility

set -e

echo "=== Rebuilding OpenCV with Qt backend ==="
echo "This will fix display issues on Wayland systems"
echo ""

# Install Qt dependencies
echo "Installing Qt development libraries..."
sudo apt update
sudo apt install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5opengl5-dev \
    cmake \
    build-essential \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev

# Create build directory
OPENCV_VERSION="4.10.0"
BUILD_DIR="/tmp/opencv-build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Download OpenCV source
if [ ! -d "opencv" ]; then
    echo "Downloading OpenCV $OPENCV_VERSION..."
    git clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv.git
    git clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv_contrib.git
else
    echo "OpenCV source already present, skipping clone."
fi

cd opencv
mkdir -p build
cd build

# Configure with Qt backend
echo "Configuring OpenCV with Qt backend..."
cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_QT=ON \
    -D WITH_GTK=OFF \
    -D WITH_OPENGL=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE=$(which python3) \
    ..

# Build (this will take 10-30 minutes depending on your CPU)
echo "Building OpenCV (this may take a while)..."
make -j$(nproc)

# Install
echo "Installing OpenCV..."
sudo make install
sudo ldconfig

echo ""
echo "=== OpenCV rebuilt successfully with Qt backend! ==="
echo "Now rebuild your tracker:"
echo "  ./3-build.sh"
echo ""
echo "Then run:"
echo "  ./4-run.sh"
