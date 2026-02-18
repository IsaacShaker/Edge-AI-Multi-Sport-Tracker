# Python Training Tools

This directory contains Python scripts for training computer vision models.

## Purpose

While the main tracker server is implemented in C++, Python is used for:
- Training custom YOLO models
- Dataset preparation and augmentation
- Model evaluation and benchmarking
- Experimentation with new detection algorithms

## Structure

```
training/
├── requirements.txt      # Python dependencies
├── train_yolo.py         # YOLO training script
├── prepare_dataset.py    # Dataset preparation
└── evaluate_model.py     # Model evaluation
```

## Legacy Prototypes

The `../legacy-python-prototypes/` directory contains the original Python prototypes:
- Ball tracking with Kalman filters
- IMM estimators
- Distance estimation

These served as the reference implementation for the C++ server.

## Getting Started

### Install Dependencies

```bash
pip install -r requirements.txt
```

### Train a YOLO Model

```bash
python train_yolo.py --data sports_ball.yaml --epochs 100
```

### Export for C++

After training, export the model:

```bash
python export_model.py --weights runs/train/exp/weights/best.pt --format onnx
```

The ONNX model can be loaded by the C++ YOLO detector.

## Using Pre-trained Models

You can use pre-trained YOLOv8 models:

1. Download: `yolo task=detect mode=predict model=yolov8n.pt`
2. Export: `yolo task=detect mode=export model=yolov8n.pt format=onnx`
3. Configure in C++:

```cpp
config.vision.model_path = "path/to/yolov8n.onnx";
config.vision.model_type = "yolo";
```

## Dataset Format

Use YOLO format:
```
dataset/
├── images/
│   ├── train/
│   └── val/
└── labels/
    ├── train/
    └── val/
```

## References

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [YOLO Dataset Format](https://docs.ultralytics.com/datasets/)
