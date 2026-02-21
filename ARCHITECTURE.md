# System Architecture

## Overview
The Edge AI Multi-Sport Tracker consists of four main subsystems orchestrated by a C++ server:

```
┌───────────────────────────────────────────────────────┐
│                     C++ Server Core                   │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   Vision    │  │  Estimation  │  │    Motor     │  │
│  │   System    │→ │    System    │→ │  Controller  │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│         ↓                                             │      
│  ┌─────────────────────────────────────────────────┐  │
│  │           Web Streaming Service                 │  │
│  └─────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────┘
```

## Subsystems

### 1. Vision System
**Purpose**: Detect and track objects in video stream

**Interfaces**:
- `IVisionDetector`: Abstract interface for object detection
- Implementations: YOLODetector, ColorBasedDetector, etc.

**Responsibilities**:
- Capture frames from camera
- Detect objects (ball, person, etc.)
- Extract position, size, confidence

### 2. Estimation System
**Purpose**: State estimation and prediction for smooth tracking

**Interfaces**:
- `IStateEstimator`: Abstract interface for state estimation
- Implementations: KalmanFilterCV, KalmanFilterCA, IMMEstimator

**Responsibilities**:
- Predict next state
- Correct with measurements
- Output filtered position, velocity, acceleration
- 3D distance estimation

### 3. Motor Controller
**Purpose**: Control gimbal to track target

**Interfaces**:
- `IMotorController`: Abstract interface for motor control
- Implementations: SimpleFOCController, MockController

**Responsibilities**:
- Send commands to embedded system
- Receive feedback
- PID control for smooth tracking

### 4. Web Streaming Service
**Purpose**: Stream video and telemetry to web clients

**Responsibilities**:
- WebSocket server for real-time data
- Video streaming (MJPEG/WebRTC)
- Telemetry broadcasting

## Factory Pattern

Factories enable easy model swapping without changing core logic:

```cpp
// Example usage
auto vision = VisionFactory::create("yolo", config);
auto estimator = EstimatorFactory::create("imm", config);
auto motor = MotorControllerFactory::create("simplefoc", config);
```

## Directory Structure

```
server/
├── include/
│   ├── interfaces/        # Abstract interfaces
│   ├── vision/            # Vision system
│   ├── estimation/        # State estimation
│   ├── motor/             # Motor control
│   ├── streaming/         # Web streaming
│   ├── factories/         # Factory classes
│   └── core/              # Core server logic
├── src/                   # Implementation files
├── test/                  # Unit tests
└── CMakeLists.txt         # Build configuration

python/
└── training/              # CV model training scripts

embedded/
└── gimbal-control/        # Arduino/SimpleFOC code

web/
└── client/                # Web interface
```

## Data Flow

1. **Vision** captures frame and detects object → (x, y, radius)
2. **Estimator** filters measurement → (filtered x, y, z, velocities)
3. **Motor Controller** computes target angles → sends to embedded system
4. **Web Streamer** broadcasts frame + telemetry → web clients

## Configuration

All components are configurable via JSON/YAML:
- Vision models and parameters
- Estimator types and tuning
- Motor PID gains
- Network settings
