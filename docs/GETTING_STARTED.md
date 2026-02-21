# Edge AI Multi-Sport Tracker - Getting Started

## Quick Start (Mock Mode - No Hardware Required)

This guide will get you running the tracker in simulation mode without any hardware.

### Step 1: Install Dependencies

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y libopencv-dev
```

**macOS:**
```bash
brew install cmake opencv
```

### Step 2: Build the Server

```bash
# From repository root
chmod +x 1-install-dependencies.sh 2-rebuild-opencv-qt.sh 3-build.sh 4-run.sh

# Follow the numbered workflow
./1-install-dependencies.sh
./2-rebuild-opencv-qt.sh
./3-build.sh
./4-run.sh
```

This will:
- Create a `server/build` directory
- Configure with CMake
- Compile all source files
- Create the `tracker_server` executable

### Step 3: Run in Mock Mode

```bash
cd server/build
./bin/tracker_server --vision color_based --estimator kalman_cv --motor mock
```

You should see:
```
=== Configuration ===
Vision:    color_based
Estimator: kalman_cv
Motor:     mock
=====================

=== Initializing Tracker Server ===
Creating vision detector: color_based
Creating state estimator: kalman_cv
Creating motor controller: mock
[MockController] Initialized
[MockController] Connecting...
[MockController] Connected (simulated)
=== Tracker Server Initialized ===
Starting tracker server...
Tracker loop started
```

### Step 4: Test with Video

Press 'q' to quit the visualization window.

## Understanding the Output

Every 2 seconds, you'll see statistics:
```
FPS: 29.8 | Frames: 150 | Detections: 42 | Avg Conf: 0.87
```

- **FPS**: Frames per second being processed
- **Frames**: Total frames processed since start
- **Detections**: Number of successful object detections
- **Avg Conf**: Average detection confidence (0-1)

## Next Steps

### Try Different Configurations

**IMM Estimator (multiple models):**
```bash
./bin/tracker_server --estimator imm --motor mock
```

**No visualization (headless mode):**
```bash
./bin/tracker_server --no-viz --motor mock
```

### Connect Real Hardware

1. Connect your SimpleFOC gimbal via USB
2. Find the port: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
3. Run with real motor:

```bash
./bin/tracker_server --motor simplefoc --vision color_based
```

Edit [server/src/motor/simplefoc_controller.cpp](server/src/motor/simplefoc_controller.cpp) to set your serial port if needed.

### Customize Detection

For an orange basketball:
```cpp
// In your config or edit color_based_detector.cpp
hue_min = 0
hue_max = 30
sat_min = 100
sat_max = 255
val_min = 100
val_max = 255
```

For a tennis ball (yellow-green):
```cpp
hue_min = 20
hue_max = 40
```

## Troubleshooting

### Build Errors

**"OpenCV not found":**
```bash
# Ubuntu
sudo apt install libopencv-dev

# macOS
brew install opencv
```

**CMake too old:**
```bash
# Need CMake 3.16+
cmake --version

# Ubuntu 20.04+ has it by default
# Otherwise install from cmake.org
```

### Runtime Errors

**"Failed to open camera 0":**
Try different camera ID:
```bash
./bin/tracker_server --camera 1 --motor mock
```

List available cameras:
```bash
ls /dev/video*
```

**No detections:**
- Ensure good lighting
- Check HSV color ranges match your ball
- Try lower confidence threshold

### Permission Denied (Serial Port)

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## Development

### Add a Custom Estimator

1. Create header in `server/include/estimation/my_estimator.h`
2. Implement in `server/src/estimation/my_estimator.cpp`
3. Register in `server/src/factories/estimator_factory.cpp`
4. Rebuild and use: `--estimator my_estimator`

### Enable Debug Output

Add to your code:
```cpp
#define DEBUG_TRACKER
std::cout << "[DEBUG] State: " << state.position.x << std::endl;
```

## Architecture Overview

```
Camera → Vision → Estimator → Motor Controller → Gimbal
           ↓          ↓              ↓
        Detected   Filtered      Pan/Tilt
         Ball      Position       Angles
```

Each component is **swappable** via the factory pattern!

## Resources

- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed system design
- [README.md](README.md) - Full documentation
- `server/src/main.cpp` - Application entry point
- `server/include/interfaces/` - Interface definitions

---

**Happy Tracking! 🎾🏀⚽**
