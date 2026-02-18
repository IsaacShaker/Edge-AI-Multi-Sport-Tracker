# Quick Reference Card

## Essential Commands

### Building
```bash
./build.sh                    # Build the server
cd server/build && make       # Rebuild after changes
```

### Running
```bash
# Default (color detection, IMM, mock motor)
./server/build/bin/tracker_server

# Custom configuration
./server/build/bin/tracker_server --vision color_based --estimator kalman_cv --motor mock

# Real hardware
./server/build/bin/tracker_server --motor simplefoc

# Headless mode
./server/build/bin/tracker_server --no-viz
```

### Help
```bash
./server/build/bin/tracker_server --help
```

## Available Components

### Vision Detectors (`--vision`)
- `color_based` - HSV color segmentation (✅ Working)
- `yolo` - YOLO object detection (🚧 Stub)

### State Estimators (`--estimator`)
- `kalman_cv` - Constant velocity Kalman (✅ Working)
- `kalman_ca` - Constant acceleration Kalman (🚧 Stub)
- `imm` - Interacting Multiple Model (🚧 Stub)

### Motor Controllers (`--motor`)
- `mock` - Simulated controller (✅ Working)
- `simplefoc` - Real SimpleFOC gimbal (🚧 Partial - Linux only)

## Key Files

### Interfaces (What to implement)
```
server/include/interfaces/
├── i_vision_detector.h       # Vision detector interface
├── i_state_estimator.h       # State estimator interface
├── i_motor_controller.h      # Motor controller interface
└── types.h                   # Common data structures
```

### Factories (Where to register)
```
server/src/factories/
├── vision_factory.cpp
├── estimator_factory.cpp
└── motor_factory.cpp
```

### Main Loop
```
server/src/core/tracker_server.cpp    # Main tracking logic
server/src/main.cpp                   # Entry point
```

## Data Flow

```
Camera → Vision Detector → State Estimator → Motor Controller
           ↓                   ↓                    ↓
        Detection         Estimated State      Gimbal Angles
    (x, y, radius)    (x, y, z, vx, vy, vz)   (pan, tilt)
```

## Key Data Structures

### Detection
```cpp
Detection {
    BoundingBox bbox;
    Point2D center;
    float radius;
    float confidence;
    string label;
}
```

### EstimatedState
```cpp
EstimatedState {
    Point3D position;      // x, y, z (distance)
    Velocity velocity;     // vx, vy, vz
    Acceleration accel;    // ax, ay, az
    float confidence;
}
```

### GimbalAngles
```cpp
GimbalAngles {
    float pan;    // radians
    float tilt;   // radians
}
```

## Adding a Component

### 1. Create Header
```cpp
// server/include/vision/my_detector.h
class MyDetector : public VisionDetectorBase {
public:
    bool initialize(const Config& config) override;
    std::vector<Detection> detect(...) override;
    std::string getType() const override { return "my_detector"; }
};
```

### 2. Implement
```cpp
// server/src/vision/my_detector.cpp
bool MyDetector::initialize(const Config& config) {
    // Your init code
    return true;
}
```

### 3. Register
```cpp
// In server/src/factories/vision_factory.cpp
registerCreator("my_detector", [](const VisionConfig& cfg) {
    return std::make_shared<MyDetector>();
});
```

### 4. Use
```bash
./tracker_server --vision my_detector
```

## Configuration Structures

### VisionConfig
```cpp
VisionConfig {
    string model_type;
    string model_path;
    float confidence_threshold;
    int input_width, input_height;
    // Color detection
    int hue_min, hue_max;
    int sat_min, sat_max;
    int val_min, val_max;
}
```

### EstimatorConfig
```cpp
EstimatorConfig {
    string estimator_type;
    float fps;
    float process_noise_pos;
    float process_noise_vel;
    float measurement_noise_pos;
    float ball_diameter_mm;
    float focal_length_px;
}
```

### MotorConfig
```cpp
MotorConfig {
    string controller_type;
    string serial_port;
    int baudrate;
    float pan_min_rad, pan_max_rad;
    float tilt_min_rad, tilt_max_rad;
    float kp_pan, ki_pan, kd_pan;
}
```

## Directory Map

```
server/
├── include/              # All headers
│   ├── interfaces/       # Pure virtual interfaces
│   ├── vision/          # Vision detector headers
│   ├── estimation/      # Estimator headers
│   ├── motor/           # Motor controller headers
│   ├── factories/       # Factory headers
│   └── core/            # Server core headers
│
└── src/                 # All implementations
    ├── vision/          # Vision implementations
    ├── estimation/      # Estimator implementations
    ├── motor/           # Motor implementations
    ├── factories/       # Factory implementations
    ├── core/            # Server core implementation
    └── main.cpp         # Entry point
```

## Common Tasks

### Test with Different Colors

Edit HSV ranges in color detector or via config:
```cpp
// Orange ball
hue_min = 0, hue_max = 30

// Tennis ball (yellow-green)
hue_min = 20, hue_max = 40

// Blue ball
hue_min = 100, hue_max = 130
```

### Change Camera
```bash
./tracker_server --camera 1  # Use /dev/video1
```

### Debug Output
Add to your code:
```cpp
std::cout << "[DEBUG] Variable: " << value << std::endl;
```

### Check Serial Port
```bash
ls /dev/ttyUSB*     # USB serial
ls /dev/ttyACM*     # ACM serial
ls /dev/video*      # Cameras
```

## Performance Tips

1. **Reduce resolution** for faster processing
2. **Use GPU** for YOLO if available (CUDA)
3. **Optimize HSV ranges** to reduce false positives
4. **Lower FPS** if CPU-bound (change target_fps)
5. **Profile** with tools like `perf` or `valgrind`

## Debugging

### Build Errors
```bash
# Clean rebuild
rm -rf server/build
./build.sh
```

### Runtime Crashes
```bash
# Run with debugger
gdb ./server/build/bin/tracker_server
run --motor mock
```

### No Detections
1. Check lighting
2. Verify HSV ranges
3. Lower confidence threshold
4. Test with known good footage

## Resources

- [ARCHITECTURE.md](ARCHITECTURE.md) - System design
- [GETTING_STARTED.md](GETTING_STARTED.md) - Tutorials
- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - What was changed
- [README.md](README.md) - Full documentation

## Status Summary

| Component | Status | Notes |
|-----------|--------|-------|
| Core server | ✅ Working | Main loop functional |
| Color detector | ✅ Working | HSV segmentation |
| Kalman CV | ✅ Working | Constant velocity |
| Mock motor | ✅ Working | For testing |
| SimpleFOC | 🚧 Partial | Linux serial only |
| YOLO | 🚧 Stub | Interface ready |
| Kalman CA | 🚧 Stub | Interface ready |
| IMM | 🚧 Stub | Interface ready |
| Web stream | 📋 Planned | Not started |

---

**Quick Links**:
- Build: `./build.sh`
- Run: `./server/build/bin/tracker_server`
- Help: `./server/build/bin/tracker_server --help`
