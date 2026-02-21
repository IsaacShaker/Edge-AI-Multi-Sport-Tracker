#!/bin/bash
# Install all dependencies required to build and run the Edge AI Multi-Sport Tracker

set -e  # Exit on error

echo "=== Installing Edge AI Multi-Sport Tracker Dependencies ==="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
   echo "Please do not run as root. The script will ask for sudo when needed."
   exit 1
fi

# Detect OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$ID
else
    echo "Cannot detect OS. This script supports Ubuntu/Debian-based systems."
    exit 1
fi

echo "Detected OS: $OS"
echo ""

# Update package lists
echo "Updating package lists..."
sudo apt update

echo ""
echo "Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git

echo ""
echo "Installing OpenCV and dependencies..."
sudo apt install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv

echo ""
echo "Installing Python dependencies..."
# Install pip if not present
if ! command -v pip3 &> /dev/null; then
    echo "Installing pip3..."
    sudo apt install -y python3-pip
fi

# Create virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv .venv
fi

# Activate virtual environment and install requirements
echo "Installing Python packages..."
source .venv/bin/activate
pip install --upgrade pip
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
else
    echo "Warning: requirements.txt not found, skipping Python package installation"
fi

echo ""
echo "Installing serial communication libraries (for gimbal control)..."
sudo apt install -y \
    libserial-dev

echo ""
echo "=== Dependency installation complete! ==="
echo ""
echo "Note: If you're using Wayland and encounter display issues:"
echo "  Run: ./rebuild-opencv-qt.sh"
echo "  This rebuilds OpenCV with Qt backend (fixes GTK/Wayland conflicts)"
echo ""
echo "Optional: Install CUDA for GPU acceleration (if you have an NVIDIA GPU):"
echo "  Follow instructions at: https://developer.nvidia.com/cuda-downloads"
echo ""
echo "To build the project, run:"
echo "  ./setup.sh"
echo ""
echo "To run the tracker, use:"
echo "  ./run-tracker.sh"
