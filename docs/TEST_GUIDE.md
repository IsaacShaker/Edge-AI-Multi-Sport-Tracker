# Test Configuration Guide

## Component Matrix

### Vision Detectors
| Type | Status | Use Case |
|------|--------|----------|
| `color_based` | ✅ Working | Colored balls (basketball, tennis ball, etc.) |
| `yolo` | 🚧 Stub | Multi-object detection (future) |

### State Estimators  
| Type | Status | Use Case |
|------|--------|----------|
| `kalman_cv` | ✅ Working | Linear motion (constant velocity) |
| `kalman_ca` | ✅ Working | Ballistic motion (with gravity) |
| `imm` | ✅ Working | Adaptive (auto-switches between CV/CA) |

### Motor Controllers
| Type | Status | Use Case |
|------|--------|----------|
| `mock` | ✅ Working | Testing without hardware |
| `simplefoc` | 🚧 Partial | Real gimbal (Linux only) |

## Test Scenarios

### 1. Basic Test (Recommended)
```bash
./bin/tracker_server --estimator imm --motor mock
```
- Uses adaptive IMM estimator
- No hardware required
- Shows visualization

### 2. Linear Motion Test
```bash
./bin/tracker_server --estimator kalman_cv --motor mock
```
- Best for balls rolling on ground
- Constant velocity assumption

### 3. Ballistic Motion Test
```bash
./bin/tracker_server --estimator kalman_ca --motor mock
```
- Best for thrown/flying balls
- Includes gravity

### 4. No Visualization (Headless)
```bash
./bin/tracker_server --estimator imm --motor mock --no-viz
```
- For servers without display
- Monitor via console output

### 5. Different Camera
```bash
./bin/tracker_server --estimator imm --motor mock --camera 1
```
- Use /dev/video1 instead of /dev/video0

## Color Calibration

For different colored balls, edit the HSV ranges in:
`server/src/vision/color_based_detector.cpp`

Or change in VisionConfig initialization in main.cpp:

| Ball | Hue Min | Hue Max | Notes |
|------|---------|---------|-------|
| Orange (basketball) | 0 | 30 | Default |
| Yellow (tennis) | 20 | 40 | Yellow-green |
| Blue | 100 | 130 | Sky/ocean blue |
| Red | 160 | 180 | Wrap-around |
| Green | 40 | 80 | Grass green |

## Expected Output

### Successful Start:
```
=== Configuration ===
Vision:    color_based
Estimator: imm
Motor:     mock
=====================

=== Initializing Tracker Server ===
Creating vision detector: color_based
Creating state estimator: imm
[IMMEstimator] Initialized with 2 models
  Model 0: kalman_cv (prob=0.5)
  Model 1: kalman_ca (prob=0.5)
Creating motor controller: mock
[MockController] Initialized
[MockController] Connecting...
[MockController] Connected (simulated)
=== Tracker Server Initialized ===
Starting tracker server...
Tracker loop started
```

### Runtime Statistics (every 2 seconds):
```
FPS: 29.8 | Frames: 150 | Detections: 42 | Avg Conf: 0.87
[IMM] Model probabilities: kalman_cv=0.65 kalman_ca=0.35
```

## Troubleshooting

### No Camera
```
Failed to open camera 0
```
**Fix**: Check camera connection or try `--camera 1`

### No Detections
```
FPS: 30.0 | Frames: 300 | Detections: 0 | Avg Conf: 0.00
```
**Fix**: 
- Adjust lighting
- Verify ball color matches HSV range
- Lower confidence threshold

### Build Errors
```
cmake: command not found
```
**Fix**: Install dependencies:
```bash
sudo apt install cmake libopencv-dev build-essential
```

## Performance Tips

1. **HD Camera**: Reduce resolution for higher FPS
   - Edit `input_width` and `input_height` in main.cpp

2. **CPU Usage**: Lower target FPS
   - Edit `target_fps` in ServerConfig

3. **Better Tracking**: Tune Kalman filter parameters
   - Adjust process/measurement noise in EstimatorConfig

4. **Distance Estimation**: Calibrate focal length
   - See `estimateDistance()` in kalman filters

## Next Steps

Once basic testing works:

1. **Record Test Data**: Save frames for offline testing
2. **Tune Parameters**: Optimize for your specific sport/ball
3. **Add Real Motor**: Connect SimpleFOC gimbal
4. **Add YOLO**: For robust multi-object tracking
5. **Add Web Interface**: Stream to browser

## Quick Commands Reference

```bash
# Help
./bin/tracker_server --help

# Default (IMM + Mock + Color + Viz)
./bin/tracker_server

# Headless
./bin/tracker_server --no-viz

# CV only
./bin/tracker_server --estimator kalman_cv

# CA only
./bin/tracker_server --estimator kalman_ca

# IMM (Recommended)
./bin/tracker_server --estimator imm

# Different camera
./bin/tracker_server --camera 1
```
