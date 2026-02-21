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
echo "Installing Qt5 libraries (required for OpenCV Qt backend)..."
sudo apt install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5opengl5-dev \
    qtwayland5

echo ""
echo "Installing other development dependencies..."
sudo apt install -y \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    libtbb2 \
    libtbb-dev \
    libdc1394-dev

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
if [ -f "python/requirements.txt" ]; then
    pip install -r python/requirements.txt
else
    echo "Warning: python/requirements.txt not found, skipping Python package installation"
fi

echo ""
echo "Installing serial communication libraries (for gimbal control)..."
sudo apt install -y \
    libserial-dev

echo ""
echo "=== Dependency installation complete! ==="
echo ""
echo "Next steps:"
echo "  1. Rebuild OpenCV with Qt backend (required for portability):"
echo "     ./2-rebuild-opencv-qt.sh"
echo ""
echo "  2. Build the C++ tracker server:"
echo "     ./3-build.sh"
echo ""
echo "  3. Run the tracker:"
echo "     ./4-run.sh"
echo ""
echo "Optional: For Python legacy scripts, run:"
echo "  cd python && ./setup.sh"
