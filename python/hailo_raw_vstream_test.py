#!/usr/bin/env python3
"""
hailo_raw_vstream_test.py — mirrors the C++ HailoDetector pipeline exactly
---------------------------------------------------------------------------
Uses raw InputVStream / OutputVStream (NOT InferVStreams) with the same
parameters as the C++ server:
  • Input  : UINT8, HWC RGB, [0, 255]   (no normalisation — HEF bakes it in)
  • Output : FLOAT32, NMS layout

Runs one frame and dumps every non-zero output slot so you can see what the
NPU is actually returning before the C++ decoder touches it.

Usage (on Pi):
    python3 hailo_raw_vstream_test.py --hef ~/Edge-AI-Multi-Sport-Tracker/server/build/models/yolov8n.hef
    python3 hailo_raw_vstream_test.py --hef yolov8n.hef --image test.jpg
    python3 hailo_raw_vstream_test.py --hef yolov8n.hef --frames 30 --no-display

Requirements:
    pip install numpy opencv-python
    hailort Python wheel already installed via hailort 4.23.0 package
"""

import sys
import time
import argparse
import threading
from pathlib import Path

import cv2
import numpy as np

# ── COCO class names ─────────────────────────────────────────────────────────

COCO = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
    "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove",
    "skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
    "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli",
    "carrot","hot dog","pizza","donut","cake","chair","couch","potted plant",
    "bed","dining table","toilet","tv","laptop","mouse","remote","keyboard",
    "cell phone","microwave","oven","toaster","sink","refrigerator","book",
    "clock","vase","scissors","teddy bear","hair drier","toothbrush",
]

CONF = 0.05   # intentionally low — raise once detections appear
INPUT_W, INPUT_H = 640, 640


# ── Preprocessing ─────────────────────────────────────────────────────────────

def preprocess(bgr_frame: np.ndarray) -> np.ndarray:
    """BGR → 640×640 RGB uint8 — matches C++ prepareInput() in base class."""
    rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
    return cv2.resize(rgb, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)


# ── NMS output decoder ────────────────────────────────────────────────────────

def _dump_raw(output_name: str, buf, label: str = ""):
    """Print the raw structure of one InferVStreams output for debugging."""
    tag = f"[{label}] " if label else ""
    if isinstance(buf, np.ndarray):
        non_zero = np.count_nonzero(buf)
        print(f"  {tag}'{output_name}': ndarray shape={buf.shape} dtype={buf.dtype}"
              f"  non-zero={non_zero}  max={buf.max():.4f}")
        print(f"    first 12 values: {buf.flatten()[:12]}")
    elif isinstance(buf, (list, tuple)):
        print(f"  {tag}'{output_name}': {type(buf).__name__}[{len(buf)}]")
        non_empty = [(i, x) for i, x in enumerate(buf)
                     if isinstance(x, np.ndarray) and x.size > 0]
        print(f"    {len(non_empty)} non-empty class slots")
        for cls_id, arr in non_empty[:10]:
            lbl = COCO[cls_id] if cls_id < len(COCO) else str(cls_id)
            print(f"    class {cls_id:3d} ({lbl:<20}): shape={arr.shape}"
                  f"  sample={np.round(arr.flatten()[:6], 4)}")
        if len(non_empty) > 10:
            print(f"    ... ({len(non_empty)-10} more non-empty classes)")
    else:
        print(f"  {tag}'{output_name}': unexpected type {type(buf).__name__}  val={buf}")


def decode_output(buf, frame_w: int, frame_h: int, dump: bool = False):
    """
    Decode whatever InferVStreams hands back for a YOLOv8 NMS HEF.

    InferVStreams wraps the raw HAILO_NMS buffer into a convenient
    list[80] of per-class arrays, where each array has shape [N, 5]:
        [y_min, x_min, y_max, x_max, score]  — coords normalised [0,1]

    Falls back to treating buf as a flat ndarray (raw VStream layout) if
    it doesn't look like the list-of-80 format.
    """
    dets = []

    # ── Format A: list[num_classes] of per-class arrays ───────────────────────
    if isinstance(buf, (list, tuple)) and len(buf) == len(COCO):
        for cls_id, class_arr in enumerate(buf):
            if not isinstance(class_arr, np.ndarray) or class_arr.size == 0:
                continue
            rows = np.atleast_2d(class_arr)
            for row in rows:
                if len(row) < 5:
                    continue
                y_min, x_min, y_max, x_max, score = (
                    float(row[0]), float(row[1]),
                    float(row[2]), float(row[3]), float(row[4])
                )
                if score < CONF:
                    continue
                x1, y1 = int(x_min * frame_w), int(y_min * frame_h)
                x2, y2 = int(x_max * frame_w), int(y_max * frame_h)
                dets.append((x1, y1, x2, y2, score, cls_id))
                if dump:
                    lbl = COCO[cls_id]
                    print(f"    {lbl:<20} score={score:.3f}"
                          f"  ({x1},{y1})-({x2},{y2})")
        return dets

    # ── Format B: flat ndarray — HAILO_NMS packed layout ────────────────────
    flat = buf.flatten() if isinstance(buf, np.ndarray) else np.array([])
    if flat.size == 0:
        if dump:
            print("  [decode] empty / unrecognised output structure")
        return dets

    num_classes = len(COCO)
    max_bboxes  = (flat.size // num_classes - 1) // 5
    if max_bboxes <= 0:
        if dump:
            print(f"  [decode] flat size {flat.size} doesn't fit NMS layout — "
                  f"first 20: {flat[:20]}")
        return dets

    stride = 1 + max_bboxes * 5
    if dump:
        print(f"  [decode flat] size={flat.size}  max_bboxes={max_bboxes}"
              f"  stride={stride}  non-zero={np.count_nonzero(flat)}")
    for c in range(num_classes):
        base  = c * stride
        # Count is a float32 integer value (e.g. 2.0 for two detections).
        count = int(flat[base])
        if count <= 0:
            continue
        if dump:
            lbl = COCO[c] if c < len(COCO) else str(c)
            print(f"    class {c:3d} ({lbl}): count={count}")
        for b in range(min(count, max_bboxes)):
            off = base + 1 + b * 5
            y_min, x_min, y_max, x_max, score = flat[off:off+5]
            if score < CONF:
                continue
            x1, y1 = int(x_min * frame_w), int(y_min * frame_h)
            x2, y2 = int(x_max * frame_w), int(y_max * frame_h)
            dets.append((x1, y1, x2, y2, float(score), c))
            if dump:
                lbl = COCO[c] if c < len(COCO) else str(c)
                print(f"      det[{b}] {lbl}  score={score:.3f}"
                      f"  ({x1},{y1})-({x2},{y2})")
    return dets


# ── Camera helpers ────────────────────────────────────────────────────────────

def open_camera():
    """picamera2 → OpenCV V4L2 fallback."""
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cfg = cam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"AfMode": 2},
        )
        cam.configure(cfg)
        cam.start()

        class PiCap:
            def read(self_):
                f = cam.capture_array("main")
                # picamera2 RGB888 → we need BGR for cv2 functions
                return (f is not None), cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
            def release(self_): cam.stop(); cam.close()
            width  = 640
            height = 480

        f = PiCap()
        ret, frame = f.read()
        if ret:
            print(f"Camera: picamera2 ({frame.shape[1]}x{frame.shape[0]})")
            return f
        f.release()
    except Exception as e:
        print(f"  picamera2 unavailable: {e}")

    for dev in range(4):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        ret, frame = cap.read()
        if ret:
            print(f"Camera: /dev/video{dev} V4L2")
            cap.width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            cap.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return cap
        cap.release()
    return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global CONF
    ap = argparse.ArgumentParser(description="Hailo raw VStream debug test")
    ap.add_argument("--hef",        required=True, help="Path to .hef model file")
    ap.add_argument("--image",      default=None,  help="Run on a single image instead of camera")
    ap.add_argument("--frames",     type=int, default=0,
                    help="Stop after N frames (0 = run forever)")
    ap.add_argument("--no-display", action="store_true",
                    help="Headless mode — print stats to terminal only")
    ap.add_argument("--conf",       type=float, default=CONF,
                    help=f"Confidence threshold (default {CONF})")
    args = ap.parse_args()

    CONF = args.conf

    hef_path = Path(args.hef)
    if not hef_path.exists():
        print(f"[error] HEF not found: {hef_path}")
        sys.exit(1)

    # ── Import Hailo platform ─────────────────────────────────────────────────
    try:
        from hailo_platform import (
            HEF, VDevice, HailoStreamInterface,
            ConfigureParams, InputVStreamParams, OutputVStreamParams,
            FormatType,
        )
    except ImportError as e:
        print(f"[error] hailo_platform not importable: {e}")
        sys.exit(1)

    # ── Load HEF ─────────────────────────────────────────────────────────────
    print(f"\nLoading HEF: {hef_path}")
    hef = HEF(str(hef_path))
    in_infos  = hef.get_input_vstream_infos()
    out_infos = hef.get_output_vstream_infos()
    print(f"  Inputs  : {[i.name for i in in_infos]}")
    print(f"  Outputs : {[o.name for o in out_infos]}")

    input_name  = in_infos[0].name
    output_name = out_infos[0].name

    # ── Configure device ──────────────────────────────────────────────────────
    print("Opening Hailo device ...")
    with VDevice() as device:
        cfg_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        net_groups = device.configure(hef, cfg_params)
        ng = net_groups[0]

        # Mirror the C++ HailoDetector params exactly:
        #   input  → UINT8  (raw [0,255] bytes; HEF bakes normalisation)
        #   output → FLOAT32
        in_params  = InputVStreamParams.make_from_network_group(
            ng, quantized=False, format_type=FormatType.UINT8
        )
        out_params = OutputVStreamParams.make_from_network_group(
            ng, quantized=False, format_type=FormatType.FLOAT32
        )

        from hailo_platform import InferVStreams

        with ng.activate(ng.create_params()):
            with InferVStreams(ng, in_params, out_params) as pipeline:

                # ── Single image mode ─────────────────────────────────────────
                if args.image:
                    frame = cv2.imread(args.image)
                    if frame is None:
                        print(f"[error] Cannot read: {args.image}")
                        sys.exit(1)
                    orig_h, orig_w = frame.shape[:2]
                    inp = preprocess(frame)

                    print(f"\nRunning single-image inference on {args.image} ...")
                    print(f"  Input  : shape={inp.shape}  dtype={inp.dtype}"
                          f"  range=[{inp.min()},{inp.max()}]")

                    t0 = time.perf_counter()
                    raw = pipeline.infer({input_name: np.expand_dims(inp, 0)})
                    t1 = time.perf_counter()
                    print(f"  Infer  : {(t1-t0)*1000:.1f} ms\n  Output structure:")
                    _dump_raw(output_name, raw[output_name])

                    dets = decode_output(raw[output_name], orig_w, orig_h, dump=True)
                    print(f"\n  Detections found: {len(dets)}")
                    for (x1,y1,x2,y2,score,cls_id) in dets:
                        lbl = COCO[cls_id] if cls_id < len(COCO) else str(cls_id)
                        print(f"    {lbl:<20} {score:.3f}  ({x1},{y1})-({x2},{y2})")
                    return

                # ── Live camera mode ──────────────────────────────────────────
                print("Opening camera ...")
                cap = open_camera()
                if cap is None:
                    print("[error] No camera found. Use --image for a static test.")
                    sys.exit(1)

                frame_count = 0
                total_dets  = 0
                t_start     = time.time()
                last_status = t_start
                dump_frames = {0, 1}   # dump raw output on first 2 frames

                print("\nRunning — Ctrl+C to stop\n")
                try:
                    while args.frames == 0 or frame_count < args.frames:
                        ret, frame = cap.read()
                        if not ret:
                            break
                        orig_h, orig_w = frame.shape[:2]
                        inp = preprocess(frame)

                        t0  = time.perf_counter()
                        raw = pipeline.infer({input_name: np.expand_dims(inp, 0)})
                        t1  = time.perf_counter()
                        infer_ms = (t1 - t0) * 1000

                        do_dump = frame_count in dump_frames
                        if do_dump:
                            print(f"\n--- Frame {frame_count} raw output ---")
                            _dump_raw(output_name, raw[output_name],
                                      label=f"frame{frame_count}")

                        dets = decode_output(raw[output_name], orig_w, orig_h,
                                             dump=do_dump)

                        total_dets  += len(dets)
                        frame_count += 1
                        now          = time.time()
                        fps          = frame_count / (now - t_start)

                        if now - last_status >= 1.0:
                            last_status = now
                            print(f"Frame {frame_count:5d} | FPS {fps:5.1f}"
                                  f" | Infer {infer_ms:6.1f}ms"
                                  f" | Dets this frame: {len(dets)}"
                                  f" | Total: {total_dets}")
                            for (x1,y1,x2,y2,score,cls_id) in dets:
                                lbl = COCO[cls_id] if cls_id < len(COCO) else str(cls_id)
                                print(f"    {lbl:<20} {score:.3f}  ({x1},{y1})-({x2},{y2})")

                except KeyboardInterrupt:
                    print("\nStopped by user.")
                finally:
                    cap.release()

                elapsed = time.time() - t_start
                print(f"\nDone. {frame_count} frames in {elapsed:.1f}s"
                      f" = {frame_count/elapsed:.1f} fps avg")
                print(f"Total detections: {total_dets}")


if __name__ == "__main__":
    main()
