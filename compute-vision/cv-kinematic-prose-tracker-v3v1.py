"""
Ball Tracking — YOLOv8n + IMM Kalman + 3D Depth
=================================================
Uses a pretrained YOLOv8 nano model as the primary detector.
Falls back to HSV color detection when YOLO doesn't find a ball.

  pip install ultralytics opencv-python numpy

The YOLOv8n weights (~6MB) auto-download on first run.

Terminal output per frame:
  frame  x  y  r_filt  depth_m  conf%  src  model  ms

  src = YOLO | HSV | coast
  conf% = YOLO confidence (or HSV circularity score mapped to %)

Press 'q' to quit.
"""

from collections import deque
from ultralytics import YOLO
import numpy as np
import argparse
import cv2
import time
import math
import sys


# ═══════════════════════════════════════════════
#  Kalman Model (CV / CA with radius channel)
# ═══════════════════════════════════════════════

class KalmanModel:
    def __init__(self, model_type="CV", fps=30,
                 gravity_m_s2=9.81, px_per_meter=500,
                 q_pos=1.0, q_vel=5.0, q_accel=2.0, r_meas=10.0,
                 q_r=0.5, q_dr=1.0, r_r=4.0):
        self.model_type = model_type
        if model_type == "CV":
            self._build_cv(q_pos, q_vel, r_meas, q_r, q_dr, r_r)
        else:
            self._build_ca(fps, gravity_m_s2, px_per_meter,
                           q_pos, q_vel, q_accel, r_meas, q_r, q_dr, r_r)
        self.initialized = False

    # ── CV: 6 states [x y vx vy r dr] ──
    def _build_cv(self, qp, qv, rm, qr, qdr, rr):
        self.kf = cv2.KalmanFilter(6, 3)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1,0,dt,0,0,0],[0,1,0,dt,0,0],[0,0,1,0,0,0],
            [0,0,0,1,0,0],[0,0,0,0,1,dt],[0,0,0,0,0,1]], np.float32)
        H = np.zeros((3,6), np.float32)
        H[0,0]=H[1,1]=H[2,4]=1
        self.kf.measurementMatrix = H
        self.kf.processNoiseCov = np.diag(
            [qp,qp,qv,qv,qr,qdr]).astype(np.float32)
        self.kf.measurementNoiseCov = np.diag(
            [rm,rm,rr]).astype(np.float32)
        self.kf.errorCovPost = np.eye(6, dtype=np.float32)*100
        self.gravity_px = 0.0

    # ── CA: 8 states [x y vx vy ax ay r dr] ──
    def _build_ca(self, fps, g, ppm, qp, qv, qa, rm, qr, qdr, rr):
        self.kf = cv2.KalmanFilter(8, 3)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1,0,dt,0,.5*dt*dt,0,0,0],[0,1,0,dt,0,.5*dt*dt,0,0],
            [0,0,1,0,dt,0,0,0],[0,0,0,1,0,dt,0,0],
            [0,0,0,0,1,0,0,0],[0,0,0,0,0,1,0,0],
            [0,0,0,0,0,0,1,dt],[0,0,0,0,0,0,0,1]], np.float32)
        H = np.zeros((3,8), np.float32)
        H[0,0]=H[1,1]=H[2,6]=1
        self.kf.measurementMatrix = H
        self.kf.processNoiseCov = np.diag(
            [qp,qp,qv,qv,qa,qa,qr,qdr]).astype(np.float32)
        self.kf.measurementNoiseCov = np.diag(
            [rm,rm,rr]).astype(np.float32)
        self.kf.errorCovPost = np.eye(8, dtype=np.float32)*100
        self.gravity_px = g * ppm / (fps**2)

    def init_state(self, x, y, r=20.0):
        if self.model_type == "CV":
            self.kf.statePost = np.array(
                [x,y,0,0,r,0], np.float32).reshape(6,1)
            self.kf.errorCovPost = np.eye(6, dtype=np.float32)*100
        else:
            self.kf.statePost = np.array(
                [x,y,0,0,0,self.gravity_px,r,0], np.float32).reshape(8,1)
            self.kf.errorCovPost = np.eye(8, dtype=np.float32)*100
        self.initialized = True

    def predict(self): return self.kf.predict()
    def correct(self, z): self.kf.correct(z)

    @property
    def pos(self):
        s = self.kf.statePost.flatten()
        return float(s[0]), float(s[1])

    @property
    def vel(self):
        s = self.kf.statePost.flatten()
        return float(s[2]), float(s[3])

    @property
    def filtered_radius(self):
        i = 4 if self.model_type == "CV" else 6
        return max(1.0, float(self.kf.statePost.flatten()[i]))

    @property
    def radius_vel(self):
        i = 5 if self.model_type == "CV" else 7
        return float(self.kf.statePost.flatten()[i])

    def future_position(self, n):
        s = self.kf.statePost.flatten()
        x,y,vx,vy = s[0],s[1],s[2],s[3]
        ax,ay = (s[4],s[5]) if self.model_type=="CA" else (0,0)
        ri = 4 if self.model_type=="CV" else 6
        return (float(x+vx*n+.5*ax*n*n),
                float(y+vy*n+.5*ay*n*n),
                float(max(1.0, s[ri]+s[ri+1]*n)))

    def innovation(self, z):
        return z - self.kf.measurementMatrix @ self.kf.statePre

    def innovation_cov(self):
        H = self.kf.measurementMatrix
        return H @ self.kf.errorCovPre @ H.T + self.kf.measurementNoiseCov


# ═══════════════════════════════════════════════
#  IMM Estimator
# ═══════════════════════════════════════════════

class IMMEstimator:
    def __init__(self, models, tpm=None, probs=None, fw=600, fh=450):
        self.models = models
        self.N = len(models)
        self.fw, self.fh = fw, fh
        if tpm is None:
            s = 0.95; sw = (1-s)/(self.N-1)
            self.tpm = np.full((self.N,self.N), sw); np.fill_diagonal(self.tpm, s)
        else:
            self.tpm = np.array(tpm, np.float64)
        self.mu = np.array(probs, np.float64) if probs else np.ones(self.N)/self.N
        self.initialized = False
        self.frames_since_seen = 0
        self.max_coast = 15

    def init_state(self, x, y, r=20.0):
        for m in self.models: m.init_state(x, y, r)
        self.initialized = True
        self.frames_since_seen = 0

    def predict(self):
        if not self.initialized: return None
        for m in self.models: m.predict()
        return self._blend()

    def correct(self, x, y, r):
        if not self.initialized:
            self.init_state(x, y, r); return
        z = np.array([[np.float32(x)],[np.float32(y)],[np.float32(r)]])
        likes = np.zeros(self.N)
        for i, m in enumerate(self.models):
            inn = m.innovation(z).astype(np.float64)
            S = m.innovation_cov().astype(np.float64)
            det = max(np.linalg.det(S), 1e-30)
            maha = float((inn.T @ np.linalg.inv(S) @ inn)[0,0])
            likes[i] = math.exp(-0.5*maha) / math.sqrt((2*math.pi)**3 * det)
        c = self.tpm.T @ self.mu
        self.mu = likes * c
        s = self.mu.sum()
        self.mu = self.mu/s if s > 1e-30 else np.ones(self.N)/self.N
        px, py = self._blend()
        if math.hypot(px-float(z[0,0]), py-float(z[1,0])) > max(self.fw,self.fh)*0.5:
            self.init_state(float(z[0,0]), float(z[1,0]), float(z[2,0])); return
        for m in self.models: m.correct(z)
        self.frames_since_seen = 0

    def coast(self): self.frames_since_seen += 1

    @property
    def is_tracking(self):
        return self.initialized and self.frames_since_seen < self.max_coast

    def _blend(self):
        x=y=0.0
        for i,m in enumerate(self.models):
            mx,my = m.pos; x+=self.mu[i]*mx; y+=self.mu[i]*my
        mg=50
        return max(-mg,min(self.fw+mg,x)), max(-mg,min(self.fh+mg,y))

    @property
    def pos(self): return self._blend()

    @property
    def filtered_radius(self):
        return max(1.0, sum(self.mu[i]*m.filtered_radius for i,m in enumerate(self.models)))

    @property
    def radius_vel(self):
        return sum(self.mu[i]*m.radius_vel for i,m in enumerate(self.models))

    def future_position(self, n):
        fx=fy=fr=0.0
        for i,m in enumerate(self.models):
            mx,my,mr = m.future_position(n)
            fx+=self.mu[i]*mx; fy+=self.mu[i]*my; fr+=self.mu[i]*mr
        return fx, fy, max(1.0, fr)

    @property
    def active_model(self):
        return self.models[int(np.argmax(self.mu))].model_type


# ═══════════════════════════════════════════════
#  Depth Estimator
# ═══════════════════════════════════════════════

class DepthEstimator:
    def __init__(self, ball_mm=65.0, focal_px=None, fw=600):
        self.focal = focal_px if focal_px else fw * 1.2
        self.K = ball_mm * self.focal
        self.calibrated = False

    def calibrate(self, dist_mm, r_px):
        self.focal = (dist_mm * 2.0 * r_px) / (self.K / self.focal)
        self.K = (self.K / self.focal) * self.focal  # recalc
        # simpler: recalc from scratch
        d_img = 2.0 * r_px
        ball_mm = self.K / self.focal
        self.focal = (dist_mm * d_img) / ball_mm
        self.K = ball_mm * self.focal
        self.calibrated = True
        print(f"  [Cal] f={self.focal:.1f}px")

    def estimate_m(self, filt_r):
        return self.K / (2.0 * max(filt_r, 1.0)) / 1000.0


# ═══════════════════════════════════════════════
#  YOLO Detector (primary)
# ═══════════════════════════════════════════════

class YOLODetector:
    """
    Wraps ultralytics YOLOv8n for 'sports ball' detection.
    COCO class 32 = 'sports ball'.
    Also accepts: frisbee(29), baseball bat(34) etc. — we only want 32.

    You can widen TARGET_CLASSES to track other objects.
    """

    TARGET_CLASSES = {32}          # sports ball
    EXTRA_BALL_LIKE = {29, 33}     # frisbee, kite — optional

    def __init__(self, model_size="yolov8n.pt", conf=0.25,
                 iou_nms=0.45, imgsz=640, verbose=False,
                 accept_extra=False):
        print(f"  Loading YOLO model '{model_size}' …")
        self.model = YOLO(model_size, verbose=verbose)
        self.conf = conf
        self.iou = iou_nms
        self.imgsz = imgsz
        self.classes = list(self.TARGET_CLASSES)
        if accept_extra:
            self.classes += list(self.EXTRA_BALL_LIKE)
        # Warm up
        dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
        self.model.predict(dummy, imgsz=imgsz, conf=conf, verbose=False)
        print(f"  YOLO ready — tracking COCO classes {self.classes}")

    def detect(self, frame, hint=None, tracking=False):
        """
        Returns: (cx, cy), radius, confidence  — or None, 0, 0.0
        The bounding box is converted to center + equivalent radius
        so it plugs straight into the Kalman filter.
        """
        results = self.model.predict(
            frame, imgsz=self.imgsz, conf=self.conf,
            iou=self.iou, classes=self.classes,
            verbose=False, stream=False,
        )
        if not results or len(results[0].boxes) == 0:
            return None, 0, 0.0

        boxes = results[0].boxes
        best_idx = 0
        best_score = -1.0

        for i in range(len(boxes)):
            conf_i = float(boxes.conf[i])
            score = conf_i

            # Proximity bonus if Kalman is tracking
            if tracking and hint is not None:
                x1,y1,x2,y2 = boxes.xyxy[i].cpu().numpy()
                cx = (x1+x2)/2; cy = (y1+y2)/2
                d = math.hypot(cx-hint[0], cy-hint[1])
                score *= (1.0 + 1.5 * math.exp(-0.5*(d/120)**2))

            if score > best_score:
                best_score = score
                best_idx = i

        x1,y1,x2,y2 = boxes.xyxy[best_idx].cpu().numpy().astype(float)
        conf = float(boxes.conf[best_idx])
        cx = (x1+x2)/2.0
        cy = (y1+y2)/2.0
        w = x2-x1; h = y2-y1
        radius = max(w, h) / 2.0  # equivalent radius from bbox

        return (int(cx), int(cy)), radius, conf


# ═══════════════════════════════════════════════
#  HSV Fallback Detector (for small / distant)
# ═══════════════════════════════════════════════

class HSVFallbackDetector:
    def __init__(self, lower=(29,86,6), upper=(64,255,255),
                 min_r=3, max_r=300):
        self.lower = np.array(lower, np.uint8)
        self.upper = np.array(upper, np.uint8)
        self.min_r = min_r
        self.max_r = max_r
        self.recent_r = deque(maxlen=30)
        self.kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))

    def detect(self, frame, hint=None, tracking=False):
        blurred = cv2.GaussianBlur(frame, (7,7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        cv2.erode(mask, self.kern, dst=mask, iterations=2)
        cv2.dilate(mask, self.kern, dst=mask, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, 0, 0.0

        best = None; best_score = -1
        for c in cnts:
            area = cv2.contourArea(c)
            if area < math.pi * self.min_r**2: continue
            (_,_), rad = cv2.minEnclosingCircle(c)
            if rad < self.min_r or rad > self.max_r: continue
            peri = cv2.arcLength(c, True)
            if peri == 0: continue
            circ = (4*math.pi*area)/(peri*peri)
            if circ < 0.35: continue
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            score = area * circ
            if tracking and hint:
                d = math.hypot(cx-hint[0], cy-hint[1])
                score *= (1.0 + 2.0*math.exp(-0.5*(d/150)**2))
            if score > best_score:
                best_score = score
                best = (cx, cy, rad, circ)

        if best is None:
            return None, 0, 0.0

        cx, cy, rad, circ = best
        self.recent_r.append(rad)
        # Map circularity (0.35–1.0) → pseudo-confidence (35%–100%)
        pseudo_conf = min(1.0, circ)
        return (cx, cy), rad, pseudo_conf


# ═══════════════════════════════════════════════
#  Main Loop
# ═══════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to video file")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--px-per-meter", type=float, default=500)
    ap.add_argument("--lookahead", type=int, default=5)
    ap.add_argument("--stay-prob", type=float, default=0.95)
    ap.add_argument("--ball-diameter", type=float, default=65.0,
                    help="real ball diameter mm (tennis=65, soccer=220)")
    ap.add_argument("--focal-length", type=float, default=None)
    ap.add_argument("--calibrate-dist", type=float, default=None)
    ap.add_argument("--yolo-model", type=str, default="yolov8n.pt",
                    help="YOLO model name (auto-downloads)")
    ap.add_argument("--yolo-conf", type=float, default=0.25,
                    help="YOLO confidence threshold")
    ap.add_argument("--yolo-imgsz", type=int, default=640,
                    help="YOLO inference resolution")
    ap.add_argument("--no-hsv-fallback", action="store_true",
                    help="disable HSV fallback detector")
    ap.add_argument("--no-gui", action="store_true",
                    help="headless — terminal only")
    args = ap.parse_args()

    W = args.width

    # ── Video source ──
    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(W*0.75))
    time.sleep(1.0)

    ok, probe = cap.read()
    if not ok:
        print("Cannot open video source"); return
    h0, w0 = probe.shape[:2]
    scale = W / w0
    H = int(h0 * scale)

    # ── Detectors ──
    yolo_det = YOLODetector(
        model_size=args.yolo_model,
        conf=args.yolo_conf,
        imgsz=args.yolo_imgsz,
    )
    hsv_det = None if args.no_hsv_fallback else HSVFallbackDetector()

    # ── Kalman / IMM ──
    cv_m = KalmanModel("CV", args.fps, q_pos=2, q_vel=10, r_meas=10)
    ca_m = KalmanModel("CA", args.fps, px_per_meter=args.px_per_meter,
                       q_pos=1, q_vel=5, q_accel=2, r_meas=10)
    s = args.stay_prob
    imm = IMMEstimator([cv_m, ca_m], tpm=[[s,1-s],[1-s,s]],
                       probs=[0.7,0.3], fw=W, fh=H)
    depth = DepthEstimator(args.ball_diameter, args.focal_length, W)

    # ── Terminal header ──
    hdr = (f"{'frm':>5}  {'x':>6}  {'y':>6}  {'r':>6}  "
           f"{'depth':>7}  {'conf':>5}  {'src':>5}  {'mdl':>3}  {'ms':>6}")
    print(f"\n{hdr}")
    print("-" * len(hdr))
    sys.stdout.flush()

    show = not args.no_gui
    frame_idx = 0
    yolo_hits = 0
    hsv_hits = 0
    coast_frames = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.resize(frame, (W, H), interpolation=cv2.INTER_LINEAR)

        t0 = time.perf_counter()

        hint = imm.pos if imm.is_tracking else None
        trk = imm.is_tracking

        # ── Primary: YOLO ──
        center, radius, conf = yolo_det.detect(frame, hint=hint, tracking=trk)
        src = "YOLO"

        # ── Fallback: HSV (if YOLO missed) ──
        if center is None and hsv_det is not None:
            center, radius, conf = hsv_det.detect(frame, hint=hint, tracking=trk)
            if center is not None:
                src = "HSV"

        # ── Kalman update ──
        if center is not None:
            imm.predict()
            imm.correct(center[0], center[1], radius)
            if src == "YOLO": yolo_hits += 1
            else: hsv_hits += 1
        else:
            imm.predict()
            imm.coast()
            src = "coast"
            conf = 0.0
            coast_frames += 1

        # ── Gather state ──
        kx = ky = None
        filt_r = None
        depth_m = None
        model = "---"
        if imm.is_tracking:
            kx, ky = imm.pos
            filt_r = imm.filtered_radius
            depth_m = depth.estimate_m(filt_r)
            model = imm.active_model

        dt_ms = (time.perf_counter() - t0) * 1000

        # ── Stream to terminal ──
        x_s = f"{kx:6.1f}" if kx is not None else "   ---"
        y_s = f"{ky:6.1f}" if ky is not None else "   ---"
        r_s = f"{filt_r:6.1f}" if filt_r is not None else "   ---"
        d_s = f"{depth_m:7.3f}" if depth_m is not None else "    ---"
        c_s = f"{conf*100:5.1f}" if conf > 0 else "  ---"
        print(f"{frame_idx:5d}  {x_s}  {y_s}  {r_s}  "
              f"{d_s}  {c_s}  {src:>5}  {model:>3}  {dt_ms:6.1f}")

        # ── GUI drawing ──
        if show:
            if imm.is_tracking and kx is not None:
                ri = int(filt_r)
                ix, iy = int(kx), int(ky)
                x1 = max(0, ix-ri); y1 = max(0, iy-ri)
                x2 = min(W-1, ix+ri); y2 = min(H-1, iy+ri)

                # Color by source: green=YOLO, yellow=HSV, cyan=coast
                if src == "YOLO":
                    box_col = (0, 255, 0)
                elif src == "HSV":
                    box_col = (0, 255, 255)
                else:
                    box_col = (255, 255, 0)

                cv2.rectangle(frame, (x1,y1), (x2,y2), box_col, 2)
                cv2.drawMarker(frame, (ix,iy), (0,0,255),
                               cv2.MARKER_CROSS, 14, 1)

                # Label: confidence + depth
                if conf > 0:
                    lbl = f"{conf*100:.0f}%"
                else:
                    lbl = "coast"
                if depth_m is not None:
                    lbl += f" {depth_m:.2f}m"
                cv2.putText(frame, lbl, (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_col, 1)

                # Source tag
                cv2.putText(frame, src, (x2+4, y1+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_col, 1)

                # Model indicator
                cv2.putText(frame, f"[{model}]", (x2+4, y1+30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200,200,200), 1)

            # Future prediction box (magenta)
            if imm.is_tracking:
                fx, fy, fr = imm.future_position(args.lookahead)
                fi = int(fr)
                cv2.rectangle(frame,
                    (int(fx)-fi, int(fy)-fi),
                    (int(fx)+fi, int(fy)+fi),
                    (255, 0, 255), 1)

            # Raw detection marker (small dot)
            if center is not None:
                cv2.circle(frame, center, 3, (255, 255, 255), -1)

            cv2.imshow("Ball Tracker — YOLOv8n + IMM", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("c") and args.calibrate_dist and imm.is_tracking:
                depth.calibrate(args.calibrate_dist, imm.filtered_radius)

        frame_idx += 1

    # ── Summary ──
    cap.release()
    if show:
        cv2.destroyAllWindows()

    total = max(frame_idx, 1)
    print(f"\n{'='*50}")
    print(f"  SUMMARY — {frame_idx} frames")
    print(f"{'='*50}")
    print(f"  YOLO detections  : {yolo_hits:5d}  ({100*yolo_hits/total:.1f}%)")
    print(f"  HSV  fallbacks   : {hsv_hits:5d}  ({100*hsv_hits/total:.1f}%)")
    print(f"  Coast (no det)   : {coast_frames:5d}  ({100*coast_frames/total:.1f}%)")
    tracked = yolo_hits + hsv_hits
    print(f"  Total tracked    : {tracked:5d}  ({100*tracked/total:.1f}%)")
    print(f"{'='*50}\n")


if __name__ == "__main__":
    main()