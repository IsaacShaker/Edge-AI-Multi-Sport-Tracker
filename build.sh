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
echo "To run:"
echo "  cd $(pwd)"
echo "  ./bin/tracker_server --help"
