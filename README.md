# Edge AI Multi-Sport Tracker

A high-performance C++ server for real-time sports tracking using computer vision, state estimation, and gimbal control.

## Quick Start

**For new developers, follow this sequence:**

```bash
# 1. Install system dependencies (build tools, Qt5 libraries)
./1-install-dependencies.sh

# 2. Rebuild OpenCV with Qt backend (one-time setup for portability)
./2-rebuild-opencv-qt.sh

# 3. Build the C++ tracker server
./3-build.sh

# 4. Run the tracker
./4-run.sh
```

The tracker will start with default settings (color-based detection, IMM estimator, mock motor). A window will display the camera feed with real-time tracking overlays.

**Note**: This project uses Qt5 for cross-platform GUI support (works on both X11 and Wayland). We do not use GTK.

## Architecture

The system is organized into well-defined subsystems with clean interfaces:

```
┌─────────────────────────────────────────────┐
│           C++ Tracker Server                │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐   │
│  │  Vision  │→│Estimation│→│  Motor   │   │
│  └──────────┘ └──────────┘ └──────────┘   │
└─────────────────────────────────────────────┘
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for detailed system design.

## Features

### Modular Design
- **Factory Pattern**: Easily swap vision detectors, estimators, and motor controllers
- **Clean Interfaces**: Well-defined abstractions for each subsystem
- **Extensible**: Add new models without changing core logic

### Vision Systems
- **Color-based detector**: HSV segmentation for colored balls
- **YOLO detector**: Deep learning object detection (YOLOv8 ready)
- Custom detectors via interface implementation

### State Estimation
- **Kalman Filter (CV)**: Constant velocity model for linear motion
- **Kalman Filter (CA)**: Constant acceleration model for ballistic trajectories
- **IMM Estimator**: Adaptive multi-model tracking
- 3D distance estimation from apparent object size

### Motor Control
- **SimpleFOC Controller**: Real gimbal control via serial
- **Mock Controller**: Testing without hardware
- PID control and angle limiting

## Directory Structure

```
├── docs/                        # Documentation
│   ├── ARCHITECTURE.md          # System architecture
│   ├── GETTING_STARTED.md       # Getting started guide
│   ├── IMPLEMENTATION_STATUS.md # Implementation status
│   ├── PROJECT_SUMMARY.md       # Project summary
│   ├── QUICK_REFERENCE.md       # Quick reference
│   └── TEST_GUIDE.md            # Testing guide
├── README.md                    # This file
├── server/                       # C++ server
│   ├── CMakeLists.txt           # Build configuration
│   ├── include/                 # Header files
│   │   ├── interfaces/          # Abstract interfaces
│   │   ├── vision/              # Vision detectors
│   │   ├── estimation/          # State estimators
│   │   ├── motor/               # Motor controllers
│   │   ├── factories/           # Factory classes
│   │   └── core/                # Server core
│   └── src/                     # Implementation files
│       ├── main.cpp             # Entry point
│       ├── vision/
│       ├── estimation/
│       ├── motor/
│       ├── factories/
│       └── core/
├── python/                      # Python utilities
│   ├── setup.sh                # Python setup script (Linux/Mac)
│   ├── setup.bat               # Python setup script (Windows)
│   ├── requirements.txt        # Python dependencies
│   ├── legacy-control-scripts/ # Legacy control scripts
│   ├── legacy-python-prototypes/ # Legacy prototypes
│   └── training/               # Model training scripts
│       ├── yolov8n.pt         # YOLOv8 nano weights
│       └── train_yolo.py      # Training script
├── embedded/                   # Embedded systems
│   └── gimbal-control/         # Arduino/SimpleFOC code
└── web/                        # Web interface
    └── client/                 # Web streaming client
```

## Building

### Prerequisites

- CMake 3.16+
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- OpenCV 4.x with Qt5 backend
- Qt5 libraries (qtbase5-dev, qtwayland5)

### Automated Build

Use the numbered scripts in order:

```bash
./1-install-dependencies.sh  # Install system dependencies
./2-rebuild-opencv-qt.sh     # Rebuild OpenCV with Qt backend
./3-build.sh                 # Build tracker server
./4-run.sh                   # Run tracker
```

### Manual Build (Linux/Mac)

```bash
cd server
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run

```bash
# Use default configuration (color detection, IMM estimator, mock motor)
./bin/tracker_server

# Specify components
./bin/tracker_server --vision color_based --estimator imm --motor mock

# Use real hardware
./bin/tracker_server --vision yolo --estimator kalman_cv --motor simplefoc

# List available options
./bin/tracker_server --help
```

## Configuration

### Vision Detector
```cpp
VisionConfig config;
config.model_type = "color_based";
config.confidence_threshold = 0.5f;
config.hue_min = 0;    // Orange ball
config.hue_max = 30;
```

### State Estimator
```cpp
EstimatorConfig config;
config.estimator_type = "imm";
config.fps = 30.0f;
config.process_noise_pos = 1.0f;
config.measurement_noise_pos = 10.0f;
```

### Motor Controller
```cpp
MotorConfig config;
config.controller_type = "simplefoc";
config.serial_port = "/dev/ttyUSB0";
config.baudrate = 115200;
config.pan_min_rad = -3.14f;
config.pan_max_rad = 3.14f;
```

## Adding New Components

### Custom Vision Detector

1. Create class implementing `IVisionDetector`:

```cpp
class MyDetector : public VisionDetectorBase {
public:
    bool initialize(const Config& config) override;
    std::vector<Detection> detect(...) override;
    std::string getType() const override { return "my_detector"; }
};
```

2. Register in factory:

```cpp
VisionFactory::registerCreator("my_detector", 
    [](const VisionConfig& cfg) {
        return std::make_shared<MyDetector>();
    });
```

3. Use it:

```bash
./tracker_server --vision my_detector
```

Same pattern applies for estimators and motor controllers!

## Python Components

### Model Training (Optional)

```bash
cd python/training
pip install -r requirements.txt
python train_yolo.py --data sports_ball.yaml
```

## Embedded System

The gimbal controller runs on Arduino with SimpleFOC library.

```bash
cd embedded/gimbal-control
# Upload to Arduino via Arduino IDE or platformio
```

## Testing

```bash
cd server/build
ctest
```

## Current Status

✅ **Completed**:
- Core architecture and interfaces
- Factory pattern implementation
- Mock motor controller (fully functional)
- Color-based vision detector (functional)
- Kalman CV estimator (functional)
- Server orchestration and main loop

🚧 **In Progress**:
- Kalman CA estimator (stub)
- IMM estimator (stub)
- YOLO detector (stub)
- SimpleFOC controller (partial - Linux only)
- Web streaming

📋 **Planned**:
- Unit tests
- Web interface
- Performance benchmarks
- Multi-object tracking

## Contributing

When adding new models:

1. Implement the appropriate interface (`IVisionDetector`, `IStateEstimator`, or `IMotorController`)
2. Register in the corresponding factory
3. Add configuration parameters to the config struct
4. Update documentation

## License

[Your License Here]

## References

- [SimpleFOC](https://simplefoc.com/) - Field Oriented Control library
- [OpenCV](https://opencv.org/) - Computer vision library
- [YOLOv8](https://github.com/ultralytics/ultralytics) - Object detection

---

**Need Help?** See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for detailed design documentation.