"""
Ball Tracking — Bounding Box + Terminal X,Y Stream
===================================================
Based on the IMM Kalman tracker with 3D depth.

Changes from original:
  - Draws axis-aligned bounding box instead of circle overlay
  - Streams x, y, radius, depth to terminal each frame
  - Performance: direct cv2.resize, smaller blur for small targets,
    stdout line-buffering, skips HUD text when headless (--no-gui)

Press 'q' to quit.
"""

from collections import deque
import numpy as np
import argparse
import cv2
import time
import math
import sys

# ── Kalman Model (unchanged core, trimmed docstrings) ──

class KalmanModel:
    def __init__(self, model_type="CV", fps=30,
                 gravity_m_s2=9.81, px_per_meter=500,
                 q_pos=1.0, q_vel=5.0, q_accel=2.0, r_meas=10.0,
                 q_radius=0.5, q_radius_vel=1.0, r_radius=4.0):
        self.model_type = model_type
        self.q_radius = q_radius
        self.q_radius_vel = q_radius_vel
        self.r_radius = r_radius
        if model_type == "CV":
            self._init_cv(q_pos, q_vel, r_meas)
        else:
            self._init_ca(fps, gravity_m_s2, px_per_meter,
                          q_pos, q_vel, q_accel, r_meas)
        self.initialized = False
        self.n = self.kf.statePre.shape[0]

    def _init_cv(self, q_pos, q_vel, r_meas):
        self.kf = cv2.KalmanFilter(6, 3)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1,0,dt,0, 0, 0],
            [0,1,0,dt, 0, 0],
            [0,0,1, 0, 0, 0],
            [0,0,0, 1, 0, 0],
            [0,0,0, 0, 1,dt],
            [0,0,0, 0, 0, 1],
        ], np.float32)
        self.kf.measurementMatrix = np.zeros((3,6), np.float32)
        self.kf.measurementMatrix[0,0] = 1
        self.kf.measurementMatrix[1,1] = 1
        self.kf.measurementMatrix[2,4] = 1
        self.kf.processNoiseCov = np.diag([
            q_pos, q_pos, q_vel, q_vel,
            self.q_radius, self.q_radius_vel]).astype(np.float32)
        self.kf.measurementNoiseCov = np.diag([
            r_meas, r_meas, self.r_radius]).astype(np.float32)
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100
        self.gravity_px = 0.0

    def _init_ca(self, fps, g, ppm, q_pos, q_vel, q_accel, r_meas):
        self.kf = cv2.KalmanFilter(8, 3)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1,0,dt,0, .5*dt*dt,0,       0, 0],
            [0,1,0,dt, 0,       .5*dt*dt, 0, 0],
            [0,0,1, 0, dt,      0,        0, 0],
            [0,0,0, 1, 0,       dt,       0, 0],
            [0,0,0, 0, 1,       0,        0, 0],
            [0,0,0, 0, 0,       1,        0, 0],
            [0,0,0, 0, 0,       0,        1,dt],
            [0,0,0, 0, 0,       0,        0, 1],
        ], np.float32)
        self.kf.measurementMatrix = np.zeros((3,8), np.float32)
        self.kf.measurementMatrix[0,0] = 1
        self.kf.measurementMatrix[1,1] = 1
        self.kf.measurementMatrix[2,6] = 1
        self.kf.processNoiseCov = np.diag([
            q_pos, q_pos, q_vel, q_vel, q_accel, q_accel,
            self.q_radius, self.q_radius_vel]).astype(np.float32)
        self.kf.measurementNoiseCov = np.diag([
            r_meas, r_meas, self.r_radius]).astype(np.float32)
        self.kf.errorCovPost = np.eye(8, dtype=np.float32) * 100
        self.gravity_px = g * ppm / (fps ** 2)

    def init_state(self, x, y, r=20.0):
        if self.model_type == "CV":
            self.kf.statePost = np.array(
                [x,y,0,0,r,0], np.float32).reshape(6,1)
            self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100
        else:
            self.kf.statePost = np.array(
                [x,y,0,0,0,self.gravity_px,r,0], np.float32).reshape(8,1)
            self.kf.errorCovPost = np.eye(8, dtype=np.float32) * 100
        self.initialized = True

    def predict(self):
        return self.kf.predict()

    def correct(self, z):
        self.kf.correct(z)

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
        s = self.kf.statePost.flatten()
        idx = 4 if self.model_type == "CV" else 6
        return max(1.0, float(s[idx]))

    @property
    def radius_vel(self):
        s = self.kf.statePost.flatten()
        idx = 5 if self.model_type == "CV" else 7
        return float(s[idx])

    def future_position(self, n):
        s = self.kf.statePost.flatten()
        x,y,vx,vy = s[0],s[1],s[2],s[3]
        ax,ay = (s[4],s[5]) if self.model_type=="CA" else (0,0)
        r_idx = 4 if self.model_type=="CV" else 6
        fx = x + vx*n + .5*ax*n*n
        fy = y + vy*n + .5*ay*n*n
        fr = max(1.0, s[r_idx] + s[r_idx+1]*n)
        return float(fx), float(fy), float(fr)

    def innovation(self, z):
        return z - self.kf.measurementMatrix @ self.kf.statePre

    def innovation_cov(self):
        H = self.kf.measurementMatrix
        return H @ self.kf.errorCovPre @ H.T + self.kf.measurementNoiseCov


# ── Depth Estimator ──

class DepthEstimator:
    def __init__(self, ball_diameter_mm=65.0, focal_length_px=None, frame_width=600):
        self.ball_diameter_mm = ball_diameter_mm
        self.focal_length = focal_length_px if focal_length_px else frame_width * 1.2
        self.K = self.ball_diameter_mm * self.focal_length
        self.calibrated = False

    def calibrate(self, known_dist_mm, obs_r_px):
        d_img = 2.0 * obs_r_px
        self.focal_length = (known_dist_mm * d_img) / self.ball_diameter_mm
        self.K = self.ball_diameter_mm * self.focal_length
        self.calibrated = True
        print(f"  [Cal] f={self.focal_length:.1f}px  K={self.K:.1f}")

    def estimate_m(self, filt_r):
        return self.K / (2.0 * max(filt_r, 1.0)) / 1000.0


# ── IMM Estimator ──

class IMMEstimator:
    def __init__(self, models, tpm=None, initial_probs=None, fw=600, fh=450):
        self.models = models
        self.N = len(models)
        self.fw, self.fh = fw, fh
        if tpm is None:
            stay = 0.95
            sw = (1-stay)/(self.N-1)
            self.tpm = np.full((self.N,self.N), sw)
            np.fill_diagonal(self.tpm, stay)
        else:
            self.tpm = np.array(tpm, np.float64)
        self.mu = np.array(initial_probs, np.float64) if initial_probs else np.ones(self.N)/self.N
        self.initialized = False
        self.frames_since_seen = 0
        self.max_coast = 15

    def init_state(self, x, y, r=20.0):
        for m in self.models:
            m.init_state(x, y, r)
        self.initialized = True
        self.frames_since_seen = 0

    def predict(self):
        if not self.initialized:
            return None
        for m in self.models:
            m.predict()
        return self._blend()

    def correct(self, x, y, r):
        if not self.initialized:
            self.init_state(x, y, r)
            return
        z = np.array([[np.float32(x)],[np.float32(y)],[np.float32(r)]])
        likes = np.zeros(self.N)
        for i, m in enumerate(self.models):
            innov = m.innovation(z).astype(np.float64)
            S = m.innovation_cov().astype(np.float64)
            det = max(np.linalg.det(S), 1e-30)
            maha = float((innov.T @ np.linalg.inv(S) @ innov)[0,0])
            likes[i] = math.exp(-0.5*maha) / math.sqrt((2*math.pi)**3 * det)
        c = self.tpm.T @ self.mu
        self.mu = likes * c
        s = self.mu.sum()
        self.mu = self.mu/s if s > 1e-30 else np.ones(self.N)/self.N

        px, py = self._blend()
        if math.hypot(px-float(z[0,0]), py-float(z[1,0])) > max(self.fw,self.fh)*0.5:
            self.init_state(float(z[0,0]), float(z[1,0]), float(z[2,0]))
            return
        for m in self.models:
            m.correct(z)
        self.frames_since_seen = 0

    def coast(self):
        self.frames_since_seen += 1

    @property
    def is_tracking(self):
        return self.initialized and self.frames_since_seen < self.max_coast

    def _blend(self):
        x = y = 0.0
        for i, m in enumerate(self.models):
            mx, my = m.pos
            x += self.mu[i]*mx; y += self.mu[i]*my
        mg = 50
        return max(-mg, min(self.fw+mg, x)), max(-mg, min(self.fh+mg, y))

    @property
    def pos(self):
        return self._blend()

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


# ── Detector (perf-tuned) ──

class BallDetector:
    def __init__(self, lower, upper, min_r=3, max_r=300):
        self.lower = np.array(lower, np.uint8)
        self.upper = np.array(upper, np.uint8)
        self.min_r = min_r
        self.max_r = max_r
        self.recent_r = deque(maxlen=30)
        self.min_circ = 0.35
        # Pre-allocate small morphology kernel
        self.kern_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        self.kern_med = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

    @property
    def adaptive_min(self):
        if len(self.recent_r) < 5:
            return self.min_r
        return max(self.min_r, float(np.median(self.recent_r)) * 0.3)

    def detect(self, frame, hint=None, tracking=False):
        # Adaptive blur size: smaller kernel when tracking small targets
        k = 5 if self.adaptive_min < 10 else 7
        blurred = cv2.GaussianBlur(frame, (k, k), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        if self.adaptive_min < 8:
            cv2.erode(mask, self.kern_small, dst=mask, iterations=1)
            cv2.dilate(mask, self.kern_small, dst=mask, iterations=2)
        else:
            cv2.erode(mask, self.kern_med, dst=mask, iterations=2)
            cv2.dilate(mask, self.kern_med, dst=mask, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, 0

        best = None
        best_score = -1
        min_area = math.pi * self.min_r ** 2

        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            (bx, by), rad = cv2.minEnclosingCircle(c)
            if rad < self.min_r or rad > self.max_r:
                continue
            peri = cv2.arcLength(c, True)
            if peri == 0:
                continue
            circ = (4*math.pi*area) / (peri*peri)
            if circ < self.min_circ:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            score = area * circ
            if tracking and hint is not None:
                d = math.hypot(cx-hint[0], cy-hint[1])
                score *= (1.0 + 2.0 * math.exp(-0.5*(d/150)**2))
            if score > best_score:
                best_score = score
                best = (cx, cy, rad)

        if best is None:
            return None, 0

        cx, cy, rad = best
        if len(self.recent_r) > 10:
            med = float(np.median(self.recent_r))
            if rad > med*4.0 and rad > 30:
                return None, 0

        self.recent_r.append(rad)
        return (cx, cy), rad


# ── Main Loop ──

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to video file")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=600)
    ap.add_argument("--px-per-meter", type=float, default=500)
    ap.add_argument("--lookahead", type=int, default=5)
    ap.add_argument("--stay-prob", type=float, default=0.95)
    ap.add_argument("--min-radius", type=float, default=3)
    ap.add_argument("--ball-diameter", type=float, default=65.0)
    ap.add_argument("--focal-length", type=float, default=None)
    ap.add_argument("--calibrate-dist", type=float, default=None)
    ap.add_argument("--no-gui", action="store_true",
                    help="headless mode — terminal output only")
    args = ap.parse_args()

    W = args.width
    color_lo = (29, 86, 6)
    color_hi = (64, 255, 255)

    # Open video source
    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(W * 0.75))
    time.sleep(1.0)

    # Read one frame to get actual dimensions
    ok, probe = cap.read()
    if not ok:
        print("Cannot open video source"); return
    h0, w0 = probe.shape[:2]
    scale = W / w0
    H = int(h0 * scale)

    # Build tracker
    cv_m = KalmanModel("CV", args.fps, q_pos=2, q_vel=10, r_meas=10)
    ca_m = KalmanModel("CA", args.fps, px_per_meter=args.px_per_meter,
                       q_pos=1, q_vel=5, q_accel=2, r_meas=10)
    s = args.stay_prob
    imm = IMMEstimator([cv_m, ca_m], tpm=[[s,1-s],[1-s,s]],
                       initial_probs=[0.7,0.3], fw=W, fh=H)
    det = BallDetector(color_lo, color_hi, min_r=args.min_radius)
    depth = DepthEstimator(args.ball_diameter, args.focal_length, W)

    # Terminal header
    print(f"\n{'frame':>6}  {'x':>7}  {'y':>7}  {'r_filt':>7}  "
          f"{'depth_m':>8}  {'model':>5}  {'ms':>6}")
    print("-" * 60)
    sys.stdout.flush()

    frame_idx = 0
    show_gui = not args.no_gui

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        # Resize with cv2 directly (faster than imutils)
        frame = cv2.resize(frame, (W, H), interpolation=cv2.INTER_LINEAR)

        t0 = time.perf_counter()

        hint = imm.pos if imm.is_tracking else None
        center, radius = det.detect(frame, hint=hint, tracking=imm.is_tracking)

        if center is not None:
            imm.predict()
            imm.correct(center[0], center[1], radius)
        else:
            imm.predict()
            imm.coast()

        # Gather tracking state
        depth_m = None
        filt_r = None
        kx = ky = None
        model = "---"
        if imm.is_tracking:
            kx, ky = imm.pos
            filt_r = imm.filtered_radius
            depth_m = depth.estimate_m(filt_r)
            model = imm.active_model

        dt_ms = (time.perf_counter() - t0) * 1000

        # ── Stream to terminal ──
        x_s = f"{kx:7.1f}" if kx is not None else "    ---"
        y_s = f"{ky:7.1f}" if ky is not None else "    ---"
        r_s = f"{filt_r:7.1f}" if filt_r is not None else "    ---"
        d_s = f"{depth_m:8.3f}" if depth_m is not None else "     ---"
        print(f"{frame_idx:6d}  {x_s}  {y_s}  {r_s}  {d_s}  {model:>5}  {dt_ms:6.2f}")

        # ── Draw bounding box + info ──
        if show_gui:
            if imm.is_tracking and kx is not None:
                r_int = int(filt_r)
                ix, iy = int(kx), int(ky)
                # Bounding box from filtered radius
                x1 = max(0, ix - r_int)
                y1 = max(0, iy - r_int)
                x2 = min(W-1, ix + r_int)
                y2 = min(H-1, iy + r_int)
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0, 255, 0), 2)
                # Cross-hair at center
                cv2.drawMarker(frame, (ix, iy), (0,0,255),
                               cv2.MARKER_CROSS, 12, 1)
                # Label
                label = f"({ix},{iy}) {depth_m:.2f}m" if depth_m else f"({ix},{iy})"
                cv2.putText(frame, label, (x1, y1-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
                # Model indicator
                cv2.putText(frame, f"[{model}]", (x2+4, y1+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,200,0), 1)

            # Raw detection circle (yellow, thin)
            if center is not None:
                cv2.circle(frame, center, int(radius), (0,255,255), 1)

            # Future prediction
            if imm.is_tracking:
                fx, fy, fr = imm.future_position(args.lookahead)
                fri = int(fr)
                cv2.rectangle(frame,
                    (int(fx)-fri, int(fy)-fri),
                    (int(fx)+fri, int(fy)+fri),
                    (255,0,255), 1)

            cv2.imshow("Ball Tracker — BBox + Stream", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("c") and args.calibrate_dist and imm.is_tracking:
                depth.calibrate(args.calibrate_dist, imm.filtered_radius)

        frame_idx += 1

    cap.release()
    if show_gui:
        cv2.destroyAllWindows()
    print(f"\nDone — {frame_idx} frames processed.")


if __name__ == "__main__":
    main()