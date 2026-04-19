#!/bin/bash

# Build script for Edge AI Multi-Sport Tracker Server

set -e

echo "=== Building Edge AI Multi-Sport Tracker Server ==="

# Navigate to server directory
cd "$(dirname "$0")/server"

# Create build directory
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

# Run CMake
echo "Running CMake..."
cmake ..

# Build
echo "Building..."
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
echo "Binary location: $(pwd)/bin/tracker_server"
echo ""

# Download YOLOv8n ONNX model if not already present
MODELS_DIR="$(dirname "$0")/models"
ONNX_MODEL="$MODELS_DIR/yolo11n.onnx"
ONNX_URL="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"

if [ ! -f "$ONNX_MODEL" ]; then
    echo "=== Downloading YOLO11n ONNX model ==="
    mkdir -p "$MODELS_DIR"
    if command -v curl &>/dev/null; then
        curl -L "$ONNX_URL" -o "$ONNX_MODEL" && echo "Model saved to $ONNX_MODEL"
    elif command -v wget &>/dev/null; then
        wget -q "$ONNX_URL" -O "$ONNX_MODEL" && echo "Model saved to $ONNX_MODEL"
    else
        echo "ERROR: neither curl nor wget found. Download manually:"
        echo "  $ONNX_URL -> $ONNX_MODEL"
        exit 1
    fi
else
    echo "ONNX model already exists: $ONNX_MODEL"
fi

echo ""
echo "To run the tracker:"
echo "  ./4-run.sh"
