#!/usr/bin/env python3
"""
hailo_yolo_test.py — Hailo AI hardware + YOLOv8n detection test
--------------------------------------------------------------
Tests the Hailo AI accelerator on Raspberry Pi AI Kit.
Runs YOLOv8n inference on live camera or image and reports FPS.

Usage:
    python3 hailo_yolo_test.py --download              # auto-detect chip + download HEF
    python3 hailo_yolo_test.py --hef yolov8n.hef       # specify HEF manually
    python3 hailo_yolo_test.py --frames 200            # run N frames then exit
    python3 hailo_yolo_test.py --image test.jpg        # single image test
    python3 hailo_yolo_test.py --save output.jpg       # save last annotated frame

Requirements (on Pi):
    pip3 install hailo_platform  (already installed via hailort 4.23.0)
    pip3 install numpy opencv-python
"""

import sys
import time
import threading
import argparse
import subprocess
import urllib.request
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# COCO class names (80 classes)
# ---------------------------------------------------------------------------
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]

# ---------------------------------------------------------------------------
# HEF download URLs from Hailo Model Zoo v2.14.0
# ---------------------------------------------------------------------------
HEF_URLS = {
    "hailo8l": "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8l/yolov8n.hef",
    "hailo8":  "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/yolov8n.hef",
}

INPUT_W = 640
INPUT_H = 640
CONF_THRESHOLD = 0.05   # intentionally low for first-run diagnostics — raise once detections appear
IOU_THRESHOLD  = 0.45


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def detect_chip_type() -> str:
    """Return 'hailo8l' or 'hailo8' by querying hailortcli."""
    try:
        result = subprocess.run(
            ["hailortcli", "fw-control", "identify"],
            capture_output=True, text=True, timeout=6
        )
        out = result.stdout + result.stderr
        if "Hailo-8L" in out or "hailo8l" in out.lower() or "h8l" in out.lower():
            return "hailo8l"
        if "Hailo-8" in out:
            return "hailo8"
    except Exception as e:
        print(f"[warn] hailortcli query failed: {e}")
    # Raspberry Pi AI Kit ships with Hailo8L by default
    return "hailo8l"


def download_hef(chip: str, dest: Path) -> None:
    url = HEF_URLS[chip]
    print(f"Downloading YOLOv8n HEF for {chip} ...")
    print(f"  URL : {url}")
    print(f"  Dest: {dest}")

    def progress(block_num, block_size, total_size):
        downloaded = block_num * block_size
        if total_size > 0:
            pct = min(100, downloaded * 100 // total_size)
            bar = "#" * (pct // 5) + "-" * (20 - pct // 5)
            print(f"\r  [{bar}] {pct}%  ({downloaded // 1024} KB)", end="", flush=True)

    urllib.request.urlretrieve(url, dest, reporthook=progress)
    print(f"\nDownload complete ({dest.stat().st_size // 1024} KB)")


def preprocess(frame: np.ndarray) -> np.ndarray:
    """RGB frame (from picamera2 RGB888) → 640×640 RGB uint8, ready for Hailo."""
    return cv2.resize(frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)


def decode_yolov8_output(raw_outputs: dict, orig_w: int, orig_h: int,
                         debug: bool = False):
    """
    Decode Hailo YOLOv8n NMS postprocess output.

    Hailo hailort 4.x yolov8_nms_postprocess can return:
      - list[80]:  one ndarray per COCO class, shape [N,5] = y1,x1,y2,x2,score (normalised)
      - list[1]:   single ndarray of all detections, shape [N,6] = y1,x1,y2,x2,score,cls_id
      - ndarray:   shape [N,6] directly
    Coordinates are normalised to [0, 1] relative to 640×640 input.
    """
    detections = []

    for name, tensor in raw_outputs.items():
        if debug:
            print(f"  [debug] output '{name}': type={type(tensor).__name__}", end="")
            if isinstance(tensor, (list, tuple)):
                print(f"  len={len(tensor)}")
                for i, t in enumerate(tensor[:5]):  # show first 5 slots
                    if isinstance(t, np.ndarray):
                        print(f"    [{i}] shape={t.shape}  size={t.size}  "
                              f"min={t.min() if t.size else 'n/a'}  "
                              f"max={t.max() if t.size else 'n/a'}  "
                              f"sample={list(t.flat[:6])}")
                    else:
                        print(f"    [{i}] type={type(t).__name__}  val={t}")
                if len(tensor) > 5:
                    print(f"    ... ({len(tensor)-5} more slots)")
            elif isinstance(tensor, np.ndarray):
                print(f"  shape={tensor.shape}  min={tensor.min():.4f}  max={tensor.max():.4f}")
                print(f"    sample: {list(tensor.flat[:8])}")
            else:
                print(f"  val={tensor}")

        items = list(tensor) if isinstance(tensor, (list, tuple)) else [tensor]

        # Case A: list[80] — one array per class
        if len(items) == len(COCO_CLASSES):
            for cls_id, class_dets in enumerate(items):
                if not isinstance(class_dets, np.ndarray) or class_dets.size == 0:
                    continue
                class_dets = np.atleast_2d(class_dets)
                for row in class_dets:
                    if len(row) < 5:
                        continue
                    y1, x1, y2, x2, score = row[0], row[1], row[2], row[3], row[4]
                    if score < CONF_THRESHOLD:
                        continue
                    detections.append((int(x1*orig_w), int(y1*orig_h),
                                       int(x2*orig_w), int(y2*orig_h),
                                       float(score), cls_id))

        # Case B: list[1] or flat ndarray — all detections with class_id column
        else:
            for arr in items:
                if not isinstance(arr, np.ndarray) or arr.size == 0:
                    continue
                arr = np.atleast_2d(arr)
                for row in arr:
                    if len(row) < 6:
                        continue
                    y1, x1, y2, x2, score, cls_id = (
                        row[0], row[1], row[2], row[3], row[4], int(row[5])
                    )
                    if score < CONF_THRESHOLD:
                        continue
                    detections.append((int(x1*orig_w), int(y1*orig_h),
                                       int(x2*orig_w), int(y2*orig_h),
                                       float(score), cls_id))
        break

    return detections


# Per-class colour palette (BGR) — cycles through 20 distinct colours
_PALETTE = [
    (0,255,0),(255,128,0),(0,0,255),(255,0,0),(0,255,128),
    (128,0,255),(128,255,0),(255,0,128),(0,128,128),(0,128,255),
    (255,255,0),(0,255,255),(255,0,255),(64,255,128),(255,64,128),
    (128,255,64),(64,128,255),(255,128,64),(128,64,255),(64,255,255),
]

def _class_color(cls_id: int):
    return _PALETTE[cls_id % len(_PALETTE)]


def draw_detections(frame: np.ndarray, detections: list) -> np.ndarray:
    """Draw bounding boxes and labels on frame (in-place copy)."""
    out = frame.copy()
    for (x1, y1, x2, y2, score, cls_id) in detections:
        label = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
        color = _class_color(cls_id)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        text = f"{label} {score:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        lx, ly = x1, max(y1 - 4, th + 4)
        cv2.rectangle(out, (lx, ly - th - 4), (lx + tw + 4, ly + 2), color, -1)
        cv2.putText(out, text, (lx + 2, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def draw_hud(frame: np.ndarray, fps: float, infer_ms: float,
             frame_count: int, det_count: int, detections: list) -> np.ndarray:
    """Render a semi-transparent stats bar at the top of the frame."""
    out = frame.copy()
    h, w = out.shape[:2]
    bar_h = 52

    # Dark overlay
    overlay = out.copy()
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.65, out, 0.35, 0, out)

    # Left column — FPS / infer
    cv2.putText(out, f"FPS: {fps:5.1f}", (10, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 0), 1, cv2.LINE_AA)
    cv2.putText(out, f"Infer: {infer_ms:.1f}ms", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 230), 1, cv2.LINE_AA)

    # Centre — model tag
    tag = "YOLOv8n | Hailo"
    (tw, _), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.putText(out, tag, (w // 2 - tw // 2, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

    # Right column — detection tally
    det_str = f"Dets: {det_count}"
    (dw, _), _ = cv2.getTextSize(det_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.putText(out, det_str, (w - dw - 10, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 160, 0), 1, cv2.LINE_AA)
    frame_str = f"Frame: {frame_count}"
    (fw, _), _ = cv2.getTextSize(frame_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.putText(out, frame_str, (w - fw - 10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)

    # Bottom hint
    cv2.putText(out, "Press Q or ESC to quit", (10, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 120, 120), 1, cv2.LINE_AA)

    return out


class Picamera2Capture:
    """Thin wrapper around picamera2 so the rest of the code can call .read()."""

    def __init__(self, width: int = 640, height: int = 480):
        from picamera2 import Picamera2
        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},  # native RGB
            controls={"AfMode": 2},
        )
        self._cam.configure(cfg)
        self._cam.start()
        self.width  = width
        self.height = height

    def read(self):
        frame = self._cam.capture_array("main")  # BGR numpy array
        return (frame is not None), frame

    def release(self):
        self._cam.stop()
        self._cam.close()

    def isOpened(self):
        return True


def open_camera(camera_index: int):
    """Open Pi camera via picamera2 (libcamera), fallback to OpenCV V4L2."""

    # --- picamera2 (preferred on Pi OS with libcamera stack) ---
    try:
        cap = Picamera2Capture(width=640, height=480)
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"Camera: picamera2 / libcamera ({frame.shape[1]}x{frame.shape[0]})")
            return cap
        cap.release()
        print("  picamera2 opened but read failed")
    except Exception as e:
        print(f"  picamera2 unavailable: {e}")

    # --- OpenCV V4L2 fallback ---
    for dev in range(max(camera_index, 0), max(camera_index, 0) + 4):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"Camera: /dev/video{dev} V4L2 ({frame.shape[1]}x{frame.shape[0]})")
            return cap
        cap.release()

    return None


# ---------------------------------------------------------------------------
# Tkinter display window (runs in its own thread)
# ---------------------------------------------------------------------------

class DisplayWindow:
    """Lightweight tkinter window that shows annotated frames.
    Runs the Tk mainloop in this same thread — call update() from another thread.
    """
    def __init__(self, title: str = "Hailo YOLOv8n", width: int = 960, height: int = 720):
        import tkinter as tk
        from PIL import Image, ImageTk  # python3-pil.imagetk
        self._tk   = tk
        self._Image    = Image
        self._ImageTk  = ImageTk
        self._root = tk.Tk()
        self._root.title(title)
        self._root.resizable(True, True)
        self._root.configure(bg="black")
        self._label = tk.Label(self._root, bg="black")
        self._label.pack(fill=tk.BOTH, expand=True)
        self._root.bind("<KeyPress-q>", lambda _: self._quit())
        self._root.bind("<KeyPress-Q>", lambda _: self._quit())
        self._root.bind("<Escape>",     lambda _: self._quit())
        self._root.protocol("WM_DELETE_WINDOW", self._quit)
        self._running = True
        self._win_w = width
        self._win_h = height
        self._root.geometry(f"{width}x{height}")
        self._lock  = threading.Lock()
        self._next  = None   # next BGR frame to display

    def update(self, bgr_frame: np.ndarray):
        """Thread-safe: queue a new frame for display."""
        with self._lock:
            self._next = bgr_frame

    def pump(self):
        """Process one Tk event cycle + show queued frame. Call from main thread."""
        if not self._running:
            return False
        with self._lock:
            frame = self._next
            self._next = None
        if frame is not None:
            h, w = frame.shape[:2]
            # Scale to fit window while keeping aspect ratio
            scale = min(self._win_w / w, self._win_h / h)
            nw, nh = int(w * scale), int(h * scale)
            # frame is already RGB from picamera2 RGB888 — pass straight to PIL
            img = self._Image.fromarray(frame).resize((nw, nh), self._Image.LANCZOS)
            photo = self._ImageTk.PhotoImage(image=img)
            self._label.configure(image=photo)
            self._label.image = photo  # keep reference
        try:
            self._root.update()
        except Exception:
            self._running = False
        return self._running

    def _quit(self):
        self._running = False

    def close(self):
        try:
            self._root.destroy()
        except Exception:
            pass

    @property
    def running(self):
        return self._running


# ---------------------------------------------------------------------------
# Main inference loop
# ---------------------------------------------------------------------------

def run_test(hef_path: Path, args):
    try:
        from hailo_platform import (
            HEF, VDevice, HailoStreamInterface,
            ConfigureParams, InputVStreamParams, OutputVStreamParams,
            FormatType, InferVStreams,
        )
    except ImportError as e:
        print(f"[error] hailo_platform not available: {e}")
        print("Install hailort: https://github.com/hailo-ai/hailort/releases")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  Hailo YOLOv8n Hardware Test")
    print(f"  HEF : {hef_path}")
    print(f"  hailort version: 4.23.0 (hailo_platform)")
    print(f"{'='*60}\n")

    # ---- Load HEF ----
    print("[1/4] Loading HEF model ...")
    hef = HEF(str(hef_path))
    input_infos  = hef.get_input_vstream_infos()
    output_infos = hef.get_output_vstream_infos()
    print(f"      Inputs  : {[i.name for i in input_infos]}")
    print(f"      Outputs : {[o.name for o in output_infos]}")

    # ---- Configure device ----
    print("[2/4] Opening Hailo device ...")
    with VDevice() as target:
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        network_groups = target.configure(hef, configure_params)
        network_group  = network_groups[0]
        network_params = network_group.create_params()

        input_vstream_params = InputVStreamParams.make_from_network_group(
            network_group, quantized=False, format_type=FormatType.UINT8
        )
        output_vstream_params = OutputVStreamParams.make_from_network_group(
            network_group, quantized=False, format_type=FormatType.FLOAT32
        )

        input_name = input_infos[0].name

        # ---- Single image test ----
        if args.image:
            frame = cv2.imread(args.image)
            if frame is None:
                print(f"[error] Cannot read image: {args.image}")
                sys.exit(1)
            orig_h, orig_w = frame.shape[:2]
            inp = preprocess(frame)

            print("[3/4] Running single-image inference ...")
            with network_group.activate(network_params):
                with InferVStreams(network_group, input_vstream_params, output_vstream_params) as pipeline:
                    t0 = time.perf_counter()
                    raw_out = pipeline.infer({input_name: np.expand_dims(inp, 0)})
                    t1 = time.perf_counter()

            print(f"      Inference time: {(t1-t0)*1000:.1f} ms")
            print("      Raw output shapes:")
            for k, v in raw_out.items():
                if isinstance(v, (list, tuple)):
                    non_empty = sum(1 for x in v if isinstance(x, np.ndarray) and x.size > 0)
                    print(f"        '{k}': list[{len(v)}] ({non_empty} non-empty classes)")
                else:
                    print(f"        '{k}': shape={v.shape}  dtype={v.dtype}")

            dets = decode_yolov8_output(raw_out, orig_w, orig_h)
            print(f"\n[4/4] Detections ({len(dets)} found):")
            for (x1, y1, x2, y2, score, cls_id) in dets:
                label = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
                print(f"      {label:<20} conf={score:.3f}  box=({x1},{y1},{x2},{y2})")

            if args.save:
                ann = draw_detections(frame, dets)
                cv2.imwrite(args.save, ann)
                print(f"\nSaved annotated image: {args.save}")
            return

        # ---- Live camera test ----
        print("[3/4] Opening camera ...")
        cap = open_camera(args.camera)
        if cap is None:
            print("[error] No camera found. Use --image for a static image test.")
            sys.exit(1)

        display = not args.no_display
        win = None
        if display:
            try:
                win = DisplayWindow("Hailo YOLOv8n — Press Q to quit", 960, 720)
                print("[4/4] Running live inference. Press Q or ESC to stop.\n")
            except Exception as e:
                print(f"  [warn] Display unavailable ({e}), running headless.")
                display = False
        if not display:
            print("[4/4] Running live inference. Ctrl+C to stop.\n")

        frame_count  = 0
        det_count    = 0
        t_start      = time.time()
        last_print   = t_start
        cur_dets     = []
        infer_ms     = 0.0
        fps          = 0.0
        last_frame   = None

        with network_group.activate(network_params):
            with InferVStreams(network_group, input_vstream_params, output_vstream_params) as pipeline:
                try:
                    while args.frames == 0 or frame_count < args.frames:
                        ret, frame = cap.read()
                        if not ret:
                            print("\n[warn] Camera read failed")
                            break

                        orig_h, orig_w = frame.shape[:2]
                        inp = preprocess(frame)

                        t0 = time.perf_counter()
                        raw_out = pipeline.infer({input_name: np.expand_dims(inp, 0)})
                        t1 = time.perf_counter()
                        infer_ms = (t1 - t0) * 1000

                        # Debug dump on frames 0 and 1
                        if frame_count <= 1:
                            cur_dets_debug = decode_yolov8_output(
                                raw_out, orig_w, orig_h, debug=True
                            )
                        if frame_count == 0:
                            print("  Output shapes:")
                            for k, v in raw_out.items():
                                if isinstance(v, (list, tuple)):
                                    non_empty = [(i, x) for i, x in enumerate(v)
                                                 if isinstance(x, np.ndarray) and x.size > 0]
                                    print(f"    '{k}': list[{len(v)}] "
                                          f"({len(non_empty)} non-empty classes)")
                                    for cls_i, arr in non_empty[:3]:
                                        lbl = COCO_CLASSES[cls_i] if cls_i < len(COCO_CLASSES) else str(cls_i)
                                        print(f"      class {cls_i} ({lbl}): shape={arr.shape}  "
                                              f"sample={np.round(arr[0], 4)}")
                                else:
                                    print(f"    '{k}': shape={v.shape}  dtype={v.dtype}")
                                    if v.size > 0:
                                        print(f"      sample row: {np.squeeze(v).flat[:6]}")
                            print()

                        cur_dets = decode_yolov8_output(raw_out, orig_w, orig_h)
                        if cur_dets:
                            det_count += len(cur_dets)

                        frame_count += 1
                        now = time.time()
                        fps = frame_count / (now - t_start)

                        # --- Display window ---
                        if display and win is not None:
                            vis = draw_detections(frame, cur_dets)
                            vis = draw_hud(vis, fps, infer_ms,
                                          frame_count, det_count, cur_dets)
                            last_frame = vis
                            win.update(vis)
                            if not win.pump():
                                print("\nQuit by user.")
                                break

                        # Console status every second
                        if now - last_print >= 1.0:
                            print(
                                f"Frame {frame_count:5d} | FPS: {fps:5.1f} | "
                                f"Infer: {infer_ms:6.1f}ms | Dets total: {det_count}"
                            )
                            if cur_dets:
                                for (x1, y1, x2, y2, score, cls_id) in cur_dets[:3]:
                                    lbl = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
                                    print(f"  └─ {lbl} {score:.2f}")
                            last_print = now

                except KeyboardInterrupt:
                    print("\nStopped by user.")

        if display and win is not None:
            win.close()
        cap.release()
        elapsed = time.time() - t_start
        avg_fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"\n{'='*60}")
        print(f"  Results:")
        print(f"    Frames processed : {frame_count}")
        print(f"    Total detections : {det_count}")
        print(f"    Elapsed time     : {elapsed:.1f}s")
        print(f"    Average FPS      : {avg_fps:.1f}")
        print(f"{'='*60}")

        if args.save and last_frame is not None:
            cv2.imwrite(args.save, last_frame)
            print(f"Saved annotated frame: {args.save}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Hailo AI + YOLOv8n hardware test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--hef", type=str, default=None,
        help="Path to yolov8n.hef (default: auto-detect in script directory)",
    )
    parser.add_argument(
        "--download", action="store_true",
        help="Download HEF from Hailo Model Zoo if not found locally",
    )
    parser.add_argument(
        "--chip", type=str, choices=["hailo8l", "hailo8"], default=None,
        help="Force chip type for download (default: auto-detect via hailortcli)",
    )
    parser.add_argument(
        "--frames", type=int, default=100,
        help="Number of frames to process (0 = run until Ctrl+C, default: 100)",
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="V4L2 camera index fallback (default: 0)",
    )
    parser.add_argument(
        "--image", type=str, default=None,
        help="Run on a single image file instead of live camera",
    )
    parser.add_argument(
        "--save", type=str, default=None,
        help="Save annotated output frame/image to this path",
    )
    parser.add_argument(
        "--no-display", action="store_true",
        help="Disable live window (headless / SSH mode)",
    )
    args = parser.parse_args()

    # Resolve HEF path
    script_dir = Path(__file__).resolve().parent
    hef_path = Path(args.hef) if args.hef else script_dir / "yolov8n.hef"

    if not hef_path.exists():
        if args.download:
            chip = args.chip or detect_chip_type()
            print(f"Chip type: {chip}")
            download_hef(chip, hef_path)
        else:
            print(f"[error] HEF not found: {hef_path}")
            print(f"  Run with --download to fetch from Hailo Model Zoo.")
            print(f"  Or compile your own: https://github.com/hailo-ai/hailo_model_zoo")
            sys.exit(1)

    run_test(hef_path, args)


if __name__ == "__main__":
    main()
