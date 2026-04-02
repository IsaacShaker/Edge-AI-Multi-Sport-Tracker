#!/bin/bash
# Install all dependencies for the Edge AI Multi-Sport Tracker on Raspberry Pi 5
# Target OS: Raspberry Pi OS Bookworm (Debian 12), 64-bit (aarch64)
# Camera: Arducam (via libcamera / V4L2)

set -e

echo "=== Edge AI Multi-Sport Tracker — Raspberry Pi 5 Dependency Installer ==="
echo "Target: Raspberry Pi OS Bookworm (64-bit), Arducam camera"
echo ""

# ── Guard rails ───────────────────────────────────────────────────────────────
if [ "$EUID" -eq 0 ]; then
    echo "ERROR: Do not run as root. The script will use sudo when needed."
    exit 1
fi

if [ "$(uname -m)" != "aarch64" ]; then
    echo "WARNING: This script is designed for aarch64 (Pi 5). Detected: $(uname -m)"
    read -rp "Continue anyway? [y/N] " yn
    [[ "$yn" =~ ^[Yy]$ ]] || exit 1
fi

# ── System update ─────────────────────────────────────────────────────────────
echo ""
echo ">>> Updating package lists..."
sudo apt update

# ── Swap — building OpenCV needs headroom ─────────────────────────────────────
# Install dphys-swapfile first in case it's missing
if ! command -v dphys-swapfile &>/dev/null; then
    echo ">>> Installing dphys-swapfile..."
    sudo apt install -y dphys-swapfile
fi

CURRENT_SWAP=$(free -m | awk '/Swap/ {print $2}')
if [ "$CURRENT_SWAP" -lt 2048 ]; then
    echo ">>> Increasing swap to 2 GB for the OpenCV build..."
    sudo dphys-swapfile swapoff 2>/dev/null || true
    sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
    # If config key doesn't exist yet, add it
    grep -q '^CONF_SWAPSIZE=' /etc/dphys-swapfile || echo 'CONF_SWAPSIZE=2048' | sudo tee -a /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    echo "    Swap is now $(free -m | awk '/Swap/ {print $2}') MB"
fi

# ── Build tools ───────────────────────────────────────────────────────────────
echo ""
echo ">>> Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    cmake-curses-gui \
    ninja-build \
    git \
    pkg-config \
    wget \
    curl \
    unzip

# ── OpenCV build dependencies ─────────────────────────────────────────────────
echo ""
echo ">>> Installing OpenCV build dependencies..."
sudo apt install -y \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libopenexr-dev \
    libopenblas-dev \
    gfortran \
    libeigen3-dev \
    libtbb-dev \
    libhdf5-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libgflags-dev \
    libgoogle-glog-dev

# ── Qt5 for OpenCV GUI backend ───────────────────────────────────────────────
echo ""
echo ">>> Installing Qt5 (OpenCV display backend)..."
sudo apt install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5opengl5-dev

# ── GStreamer — enables libcamera → GStreamer → OpenCV pipeline ───────────────
echo ""
echo ">>> Installing GStreamer..."
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools

# gstreamer1.0-libcamera provides the libcamerasrc element on Pi OS —
# install separately so a missing package doesn't abort the whole block
sudo apt install -y gstreamer1.0-libcamera 2>/dev/null || \
    echo "    NOTE: gstreamer1.0-libcamera not found via apt — libcamerasrc may already be bundled in gstreamer1.0-plugins-bad"

# ── libcamera (Raspberry Pi 5 camera stack) ───────────────────────────────────
echo ""
echo ">>> Installing libcamera and V4L2..."
sudo apt install -y \
    libcamera-dev \
    libcamera-apps \
    libcamera-tools \
    v4l-utils \
    libv4l-dev \
    python3-libcamera \
    rpicam-apps

# Enable V4L2 compatibility layer (exposes camera as /dev/video0)
if ! grep -q "bcm2835-v4l2\|libcamera" /etc/modules 2>/dev/null; then
    echo ""
    echo ">>> Enabling libcamera V4L2 compatibility layer at boot..."
    echo "libcamera" | sudo tee -a /etc/modules
fi

# ── Arducam driver / SDK ──────────────────────────────────────────────────────
echo ""
echo ">>> Installing Arducam drivers..."
echo "    NOTE: Choose the correct driver package for your Arducam model."
echo "    Common models and their drivers:"
echo "      IMX519 / 64MP / Multi-cam → Pivariety driver"
echo "      IMX477 (HQ cam) / IMX708  → native libcamera (no extra driver needed)"
echo ""

# Install Arducam's universal Pivariety driver installer
ARDUCAM_INSTALLER="install_pivariety_pkgs.sh"
ARDUCAM_INSTALLER_URL="https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/${ARDUCAM_INSTALLER}"

if [ ! -f "/tmp/${ARDUCAM_INSTALLER}" ]; then
    echo "    Downloading Arducam installer..."
    wget -q -O "/tmp/${ARDUCAM_INSTALLER}" "${ARDUCAM_INSTALLER_URL}" || {
        echo "    WARNING: Could not download Arducam installer (check internet connection)."
        echo "    Skipping Arducam-specific driver install — standard libcamera will be used."
    }
fi

if [ -f "/tmp/${ARDUCAM_INSTALLER}" ]; then
    chmod +x "/tmp/${ARDUCAM_INSTALLER}"
    echo ""
    echo "    Installing Arducam kernel driver (Pivariety)..."
    sudo bash "/tmp/${ARDUCAM_INSTALLER}" -p kernel_driver || \
        echo "    WARNING: Arducam kernel driver install failed. May not be needed for your model."

    echo "    Installing arducam_config_parser..."
    sudo bash "/tmp/${ARDUCAM_INSTALLER}" -p arducam_config_parser || \
        echo "    WARNING: arducam_config_parser install failed."
fi

# ── Python ────────────────────────────────────────────────────────────────────
echo ""
echo ">>> Installing Python 3 and virtual environment support..."
sudo apt install -y \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-numpy

if [ ! -d ".venv" ]; then
    echo "    Creating Python virtual environment..."
    python3 -m venv .venv
fi

echo "    Installing Python packages..."
source .venv/bin/activate
pip install --upgrade pip
if [ -f "python/requirements.txt" ]; then
    # Install CPU-only torch/torchvision to save space on Pi
    pip install --upgrade \
        numpy \
        imutils \
        pyserial \
        ultralytics
    # Install torch separately with Pi-compatible index
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu || \
        pip install torch torchvision  # fallback to default
else
    echo "    WARNING: python/requirements.txt not found, skipping."
fi
deactivate

# ── Serial (gimbal control) ───────────────────────────────────────────────────
echo ""
echo ">>> Installing serial communication libraries..."
sudo apt install -y libserial-dev

# Add current user to dialout group for serial port access
if ! groups | grep -q dialout; then
    echo "    Adding $USER to 'dialout' group for serial port access..."
    sudo usermod -aG dialout "$USER"
    echo "    NOTE: Log out and back in (or reboot) for group change to take effect."
fi

# ── /boot/firmware/config.txt camera configuration ───────────────────────────
echo ""
echo ">>> Checking /boot/firmware/config.txt for camera settings..."
CONFIG_FILE="/boot/firmware/config.txt"
if [ -f "$CONFIG_FILE" ]; then
    if ! grep -q "camera_auto_detect\|start_x" "$CONFIG_FILE"; then
        echo "    Enabling camera in config.txt..."
        echo "" | sudo tee -a "$CONFIG_FILE"
        echo "# Arducam / libcamera" | sudo tee -a "$CONFIG_FILE"
        echo "camera_auto_detect=1" | sudo tee -a "$CONFIG_FILE"
    else
        echo "    Camera entry already present in config.txt."
    fi
else
    echo "    WARNING: $CONFIG_FILE not found — are you on a real Raspberry Pi?"
fi

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "=== Dependency installation complete! ==="
echo ""
echo "Next steps:"
echo "  1. Verify camera is detected:     rpicam-hello --list-cameras"
echo "                                    v4l2-ctl --list-devices"
echo "  2. Build OpenCV for Pi:           ./pi-2-build-opencv.sh"
echo "  3. Build the project:             ./pi-3-build.sh"
echo "  4. Run with Arducam:              ./pi-4-run.sh"
echo ""
echo "If this is your first install, a REBOOT is recommended before proceeding:"
echo "  sudo reboot"
