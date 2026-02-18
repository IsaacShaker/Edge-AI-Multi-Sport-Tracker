#!/bin/bash

# Edge-AI Multi-Sport Tracker Setup Script
# Compatible with Linux, macOS, and Raspberry Pi

set -e  # Exit on error

echo "=========================================="
echo "Edge-AI Multi-Sport Tracker Setup"
echo "=========================================="
echo ""

# Detect OS and Architecture
OS="$(uname -s)"
ARCH="$(uname -m)"
IS_RPI=false

# Check if running on Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(tr -d '\0' < /proc/device-tree/model)
    if [[ $MODEL == *"Raspberry Pi"* ]]; then
        IS_RPI=true
        echo "Detected Raspberry Pi: $MODEL"
    fi
fi

echo "Platform: $OS ($ARCH)"
echo ""

# Check Python version
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed"
    echo "Please install Python 3.8 or higher"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"
echo ""

# Create virtual environment
echo "Creating virtual environment..."
if [ -d ".venv" ]; then
    echo "Warning: Virtual environment already exists at .venv/"
    read -p "Do you want to recreate it? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf .venv
        python3 -m venv .venv
        echo "Virtual environment recreated"
    else
        echo "Using existing virtual environment"
    fi
else
    python3 -m venv .venv
    echo "Virtual environment created at .venv/"
fi
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source .venv/bin/activate
echo "Virtual environment activated"
echo ""

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip
echo ""

# Install dependencies from requirements.txt
echo "Installing Python packages from requirements.txt..."
pip install -r requirements.txt
echo "Python packages installed"
echo ""

# Handle Raspberry Pi specific packages
if [ "$IS_RPI" = true ]; then
    echo "Installing Raspberry Pi specific packages..."
    
    # Check if picamera2 is available
    if python3 -c "import picamera2" 2>/dev/null; then
        echo "picamera2 is already installed"
    else
        echo "Installing picamera2 via apt..."
        if command -v apt &> /dev/null; then
            sudo apt update
            sudo apt install -y python3-picamera2 python3-libcamera
            echo "picamera2 installed"
        else
            echo "Warning: apt not available, cannot install picamera2"
            echo "Please install manually: sudo apt install -y python3-picamera2"
        fi
    fi
    echo ""
fi

# Download YOLOv8 weights if not present
echo "Checking YOLOv8 weights..."
if [ ! -f "yolov8n.pt" ]; then
    echo "YOLOv8n weights not found. They will be downloaded automatically on first run."
else
    echo "YOLOv8n weights found: yolov8n.pt"
fi
echo ""

# Verify installation
echo "Verifying installation..."
python3 << 'PYTHON_CODE'
import sys
try:
    import cv2
    import numpy
    import imutils
    import ultralytics
    import serial
    print("All core packages imported successfully")
    print(f"  - OpenCV: {cv2.__version__}")
    print(f"  - NumPy: {numpy.__version__}")
    print(f"  - Ultralytics: {ultralytics.__version__}")
    
    # Check optional packages
    try:
        import picamera2
        print(f"  - Picamera2: Available")
    except ImportError:
        print(f"  - Picamera2: Not installed (only needed for Raspberry Pi)")
    
except ImportError as e:
    print(f"Error importing package: {e}", file=sys.stderr)
    sys.exit(1)
PYTHON_CODE

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Setup completed successfully!"
    echo "=========================================="
    echo ""
    echo "To activate the virtual environment:"
    echo "  source .venv/bin/activate"
    echo ""
    echo "To run the tracker:"
    echo "  python compute-vision/cv-kinematic-tracker.py"
    echo ""
    echo "To run with YOLOv8:"
    echo "  python compute-vision/cv-kinematic-prose-tracker-v3v1.py"
    echo ""
else
    echo ""
    echo "Setup completed with errors"
    echo "Please check the error messages above"
    exit 1
fi
