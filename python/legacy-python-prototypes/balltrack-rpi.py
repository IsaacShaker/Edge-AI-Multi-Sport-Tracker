"""
Ball Tracking with OpenCV + Adaptive IMM Kalman Filter
======================================================
Adapted for Raspberry Pi 5 + IMX219 (MIPI CSI-2 camera0)
Raspberry Pi OS Bookworm

Uses Picamera2 (libcamera) for camera access.

Setup:
  sudo apt install -y python3-picamera2 python3-opencv python3-numpy

Press 'q' to quit — benchmark summary prints to the console.
"""

from collections import deque
import numpy as np
import argparse
import cv2
import time
import math

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False
    print("[WARN] picamera2 not found — falling back to cv2.VideoCapture")


# ──────────────────────────────────────────────
#  Camera wrapper for RPi5 + IMX219
# ──────────────────────────────────────────────

class RPiCamera:
    """Wraps Picamera2 for RPi5 + IMX219 via MIPI CSI."""

    def __init__(self, width=640, height=480, fps=30, camera_id=0):
        if not HAS_PICAMERA2:
            raise RuntimeError("picamera2 is required for MIPI camera access")

        self.picam2 = Picamera2(camera_num=camera_id)

        # Let Picamera2 negotiate the pixel format with the ISP
        # — don't force RGB888 which can silently fail on some setups
        config = self.picam2.create_video_configuration(
            main={"size": (width, height)},
            controls={"FrameDurationLimits": (int(1e6 // fps), int(1e6 // fps))},
            buffer_count=4,
        )
        self.picam2.configure(config)

        # Log what format was actually selected
        actual_fmt = self.picam2.camera_configuration()["main"]["format"]
        actual_size = self.picam2.camera_configuration()["main"]["size"]
        print(f"[INFO] Picamera2 configured: {actual_fmt} @ {actual_size}")
        self._format = actual_fmt

        self.picam2.start()
        # Give AE/AWB time to converge
        time.sleep(2.0)

        # Grab a test frame to verify we're getting real data
        test = self.picam2.capture_array("main")
        print(f"[INFO] Test frame: shape={test.shape}, dtype={test.dtype}, "
              f"min={test.min()}, max={test.max()}")
        if test.max() == 0:
            print("[WARN] Test frame is all zeros — camera may not be delivering data!")

    def read(self):
        frame = self.picam2.capture_array("main")

        # Handle whatever pixel format Picamera2 actually gave us
        if frame.ndim == 2:
            # Grayscale
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[2] == 4:
            # XRGB8888 / XBGR8888 — drop alpha channel
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif frame.shape[2] == 3:
            # Check if the negotiated format is RGB (not BGR)
            # Picamera2 default without explicit format tends to be XBGR8888
            # but if it's RGB, we need to convert
            if "RGB" in self._format and "BGR" not in self._format:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return True, frame

    def release(self):
        self.picam2.stop()
        self.picam2.close()


class FallbackCamera:
    """Falls back to cv2.VideoCapture (e.g. v4l2 /dev/video0)."""

    def __init__(self, src=0, width=640, height=480):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        time.sleep(2.0)

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


# ──────────────────────────────────────────────
#  Utility: replaces imutils.grab_contours
# ──────────────────────────────────────────────

def grab_contours(cnts):
    if len(cnts) == 2:
        return cnts[0]
    elif len(cnts) == 3:
        return cnts[1]
    raise Exception("Contour tuple has unexpected length")


# ──────────────────────────────────────────────
#  Single Kalman model wrapper
# ──────────────────────────────────────────────

class KalmanModel:
    def __init__(self, model_type="CV", fps=30,
                 gravity_m_s2=9.81, px_per_meter=500,
                 q_pos=1.0, q_vel=5.0, q_accel=2.0, r=10.0):
        self.model_type = model_type
        if model_type == "CV":
            self._init_cv(q_pos, q_vel, r)
        else:
            self._init_ca(fps, gravity_m_s2, px_per_meter,
                          q_pos, q_vel, q_accel, r)
        self.initialized = False
        self.n = self.kf.statePre.shape[0]

    def _init_cv(self, q_pos, q_vel, r):
        self.kf = cv2.KalmanFilter(4, 2)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.zeros((2, 4), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1
        self.kf.measurementMatrix[1, 1] = 1
        self.kf.processNoiseCov = np.diag(
            [q_pos, q_pos, q_vel, q_vel]
        ).astype(np.float32)
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * r
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 100
        self.gravity_px = 0.0

    def _init_ca(self, fps, g, ppm, q_pos, q_vel, q_accel, r):
        self.kf = cv2.KalmanFilter(6, 2)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  0.5*dt**2, 0        ],
            [0, 1, 0,  dt, 0,         0.5*dt**2 ],
            [0, 0, 1,  0,  dt,        0         ],
            [0, 0, 0,  1,  0,         dt        ],
            [0, 0, 0,  0,  1,         0         ],
            [0, 0, 0,  0,  0,         1         ],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.zeros((2, 6), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1
        self.kf.measurementMatrix[1, 1] = 1
        self.kf.processNoiseCov = np.diag(
            [q_pos, q_pos, q_vel, q_vel, q_accel, q_accel]
        ).astype(np.float32)
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * r
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100
        self.gravity_px = g * ppm / (fps ** 2)

    def init_state(self, x, y):
        if self.model_type == "CV":
            self.kf.statePost = np.array(
                [x, y, 0, 0], dtype=np.float32).reshape(4, 1)
        else:
            self.kf.statePost = np.array(
                [x, y, 0, 0, 0, self.gravity_px], dtype=np.float32).reshape(6, 1)
        self.kf.errorCovPost = np.eye(self.n, dtype=np.float32) * 100
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

    def future_position(self, n_frames):
        s = self.kf.statePost.flatten()
        x, y, vx, vy = s[0], s[1], s[2], s[3]
        dt = n_frames
        if self.model_type == "CA":
            ax, ay = s[4], s[5]
        else:
            ax, ay = 0.0, 0.0
        fx = x + vx * dt + 0.5 * ax * dt ** 2
        fy = y + vy * dt + 0.5 * ay * dt ** 2
        return float(fx), float(fy)

    def innovation(self, z):
        predicted_z = self.kf.measurementMatrix @ self.kf.statePre
        return z - predicted_z

    def innovation_cov(self):
        H = self.kf.measurementMatrix
        P = self.kf.errorCovPre
        R = self.kf.measurementNoiseCov
        return H @ P @ H.T + R


# ──────────────────────────────────────────────
#  Interacting Multiple Model (IMM) Estimator
# ──────────────────────────────────────────────

class IMMEstimator:
    def __init__(self, models, tpm=None, initial_probs=None,
                 frame_w=640, frame_h=480):
        self.models = models
        self.N = len(models)
        self.frame_w = frame_w
        self.frame_h = frame_h

        if tpm is None:
            stay = 0.95
            switch = (1 - stay) / (self.N - 1)
            self.tpm = np.full((self.N, self.N), switch)
            np.fill_diagonal(self.tpm, stay)
        else:
            self.tpm = np.array(tpm, dtype=np.float64)

        if initial_probs is None:
            self.mu = np.ones(self.N) / self.N
        else:
            self.mu = np.array(initial_probs, dtype=np.float64)

        self.initialized = False
        self.frames_since_seen = 0
        self.max_coast_frames = 15

    def init_state(self, x, y):
        for m in self.models:
            m.init_state(x, y)
        self.initialized = True
        self.frames_since_seen = 0

    def predict(self):
        if not self.initialized:
            return None
        for m in self.models:
            m.predict()
        return self._blended_pos()

    def correct(self, x, y):
        if not self.initialized:
            self.init_state(x, y)
            return

        z = np.array([[np.float32(x)], [np.float32(y)]])

        likelihoods = np.zeros(self.N)
        for i, m in enumerate(self.models):
            innov = m.innovation(z)
            S = m.innovation_cov()
            S64 = S.astype(np.float64)
            det = max(np.linalg.det(S64), 1e-30)
            inv_S = np.linalg.inv(S64)
            innov64 = innov.astype(np.float64)
            maha = float((innov64.T @ inv_S @ innov64)[0, 0])
            likelihoods[i] = math.exp(-0.5 * maha) / math.sqrt(
                (2 * math.pi) ** 2 * det)

        c = self.tpm.T @ self.mu
        self.mu = likelihoods * c
        total = self.mu.sum()
        if total > 1e-30:
            self.mu /= total
        else:
            self.mu = np.ones(self.N) / self.N

        px, py = self._blended_pos()
        dist = math.hypot(px - float(z[0][0]), py - float(z[1][0]))
        if dist > max(self.frame_w, self.frame_h) * 0.5:
            self.init_state(float(z[0][0]), float(z[1][0]))
            return

        for m in self.models:
            m.correct(z)
        self.frames_since_seen = 0

    def coast(self):
        self.frames_since_seen += 1

    @property
    def is_tracking(self):
        return self.initialized and self.frames_since_seen < self.max_coast_frames

    def _clamp(self, x, y):
        margin = 50
        x = max(-margin, min(self.frame_w + margin, x))
        y = max(-margin, min(self.frame_h + margin, y))
        return x, y

    def _blended_pos(self):
        x, y = 0.0, 0.0
        for i, m in enumerate(self.models):
            mx, my = m.pos
            x += self.mu[i] * mx
            y += self.mu[i] * my
        return self._clamp(x, y)

    @property
    def pos(self):
        return self._blended_pos()

    @property
    def state_info(self):
        best_i = int(np.argmax(self.mu))
        m = self.models[best_i]
        vx, vy = m.vel
        return {
            "x": self.pos[0], "y": self.pos[1],
            "vx": vx, "vy": vy,
            "model": m.model_type,
            "prob_cv": self.mu[0],
            "prob_ca": self.mu[1],
        }

    def future_position(self, n_frames):
        fx, fy = 0.0, 0.0
        for i, m in enumerate(self.models):
            mx, my = m.future_position(n_frames)
            fx += self.mu[i] * mx
            fy += self.mu[i] * my
        return fx, fy


# ──────────────────────────────────────────────
#  Adaptive Ball Detector
# ──────────────────────────────────────────────

class AdaptiveBallDetector:
    def __init__(self, color_lower, color_upper,
                 min_radius_abs=3, max_radius=300):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.min_radius_abs = min_radius_abs
        self.max_radius = max_radius
        self.recent_radii = deque(maxlen=30)
        self.min_circularity = 0.35

    @property
    def adaptive_min_radius(self):
        if len(self.recent_radii) < 5:
            return self.min_radius_abs
        median_r = float(np.median(self.recent_radii))
        return max(self.min_radius_abs, median_r * 0.3)

    def detect(self, frame, kalman_pos=None, kalman_tracking=False):
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        min_r = self.adaptive_min_radius
        if min_r < 8:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
        else:
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = grab_contours(cnts)

        if len(cnts) == 0:
            return None, 0, mask

        candidates = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < math.pi * self.min_radius_abs ** 2:
                continue

            ((bx, by), radius) = cv2.minEnclosingCircle(c)
            if radius < self.min_radius_abs or radius > self.max_radius:
                continue

            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = (4 * math.pi * area) / (perimeter ** 2)

            if circularity < self.min_circularity:
                continue

            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            score = area * circularity

            if kalman_tracking and kalman_pos is not None:
                dist = math.hypot(cx - kalman_pos[0], cy - kalman_pos[1])
                proximity_bonus = math.exp(-0.5 * (dist / 150) ** 2)
                score *= (1.0 + 2.0 * proximity_bonus)

            candidates.append({
                "center": (cx, cy),
                "radius": radius,
                "bx": bx, "by": by,
                "score": score,
                "circularity": circularity,
            })

        if not candidates:
            return None, 0, mask

        best = max(candidates, key=lambda c: c["score"])

        if len(self.recent_radii) > 10:
            median_r = float(np.median(self.recent_radii))
            if best["radius"] > median_r * 4.0 and best["radius"] > 30:
                return None, 0, mask

        self.recent_radii.append(best["radius"])
        return best["center"], best["radius"], mask


# ──────────────────────────────────────────────
#  Benchmark stats collector
# ──────────────────────────────────────────────

class BenchmarkStats:
    def __init__(self, lookahead):
        self.lookahead = lookahead
        self.raw_history = {}
        self.kalman_predictions = {}
        self.raw_deltas = []
        self.kalman_deltas = []
        self.prev_raw = None
        self.prev_kalman = None
        self.total_frames = 0
        self.raw_detected_frames = 0
        self.kalman_tracked_frames = 0
        self.times_raw = []
        self.times_kalman = []
        self.model_frames = {"CV": 0, "CA": 0}

    def record_frame(self, frame_idx, raw_center, kalman_pos,
                     kalman_future, is_kalman_tracking,
                     dt_raw, dt_kalman, active_model=None):
        self.total_frames += 1
        self.times_raw.append(dt_raw)
        self.times_kalman.append(dt_kalman)

        if raw_center is not None:
            self.raw_detected_frames += 1
            self.raw_history[frame_idx] = raw_center
        if is_kalman_tracking:
            self.kalman_tracked_frames += 1

        if kalman_future is not None:
            target_frame = frame_idx + self.lookahead
            self.kalman_predictions[target_frame] = kalman_future

        if raw_center is not None:
            if self.prev_raw is not None:
                dx = raw_center[0] - self.prev_raw[0]
                dy = raw_center[1] - self.prev_raw[1]
                self.raw_deltas.append(math.hypot(dx, dy))
            self.prev_raw = raw_center

        if kalman_pos is not None:
            if self.prev_kalman is not None:
                dx = kalman_pos[0] - self.prev_kalman[0]
                dy = kalman_pos[1] - self.prev_kalman[1]
                self.kalman_deltas.append(math.hypot(dx, dy))
            self.prev_kalman = kalman_pos

        if active_model is not None:
            self.model_frames[active_model] = self.model_frames.get(active_model, 0) + 1

    def report(self):
        sep = "=" * 60
        print(f"\n{sep}")
        print("  BENCHMARK: Raw Detection vs. Adaptive IMM Kalman (CV+CA)")
        print(sep)

        errors = []
        for target_frame, (px, py) in self.kalman_predictions.items():
            if target_frame in self.raw_history:
                tx, ty = self.raw_history[target_frame]
                errors.append(math.hypot(px - tx, py - ty))

        print(f"\n  1. PREDICTION ACCURACY  (look-ahead = {self.lookahead} frames)")
        if errors:
            arr = np.array(errors)
            print(f"     Scored predictions : {len(errors)}")
            print(f"     Mean error         : {arr.mean():.1f} px")
            print(f"     Median error       : {np.median(arr):.1f} px")
            print(f"     Std deviation      : {arr.std():.1f} px")
            print(f"     90th percentile    : {np.percentile(arr, 90):.1f} px")
            print(f"     Max error          : {arr.max():.1f} px")
        else:
            print("     (not enough data)")

        print(f"\n  2. SMOOTHNESS  (frame-to-frame displacement std dev)")
        if self.raw_deltas:
            raw_arr = np.array(self.raw_deltas)
            kal_arr = np.array(self.kalman_deltas) if self.kalman_deltas else np.array([0])
            raw_j = raw_arr.std()
            kal_j = kal_arr.std()
            print(f"     Raw detection jitter  : {raw_j:.2f} px  (mean step {raw_arr.mean():.1f} px)")
            print(f"     IMM Kalman jitter     : {kal_j:.2f} px  (mean step {kal_arr.mean():.1f} px)")
            if raw_j > 0:
                print(f"     Jitter reduction      : {(1 - kal_j / raw_j) * 100:.1f}%")

        print(f"\n  3. OCCLUSION BRIDGING")
        print(f"     Total frames           : {self.total_frames}")
        print(f"     Raw detected frames    : {self.raw_detected_frames}  "
              f"({100*self.raw_detected_frames/max(1,self.total_frames):.1f}%)")
        print(f"     IMM tracked frames     : {self.kalman_tracked_frames}  "
              f"({100*self.kalman_tracked_frames/max(1,self.total_frames):.1f}%)")
        extra = self.kalman_tracked_frames - self.raw_detected_frames
        print(f"     Extra frames from IMM  : {extra}")

        print(f"\n  4. PER-FRAME PROCESSING TIME")
        if self.times_raw:
            raw_ms = np.array(self.times_raw) * 1000
            kal_ms = np.array(self.times_kalman) * 1000
            print(f"     Raw detection only    : {raw_ms.mean():.2f} ms avg  "
                  f"({1000/raw_ms.mean():.0f} FPS capacity)")
            print(f"     Detection + IMM       : {kal_ms.mean():.2f} ms avg  "
                  f"({1000/kal_ms.mean():.0f} FPS capacity)")
            overhead = kal_ms.mean() - raw_ms.mean()
            print(f"     IMM overhead          : {overhead:.3f} ms/frame")

        print(f"\n  5. MODEL SELECTION BREAKDOWN")
        total_model = sum(self.model_frames.values())
        if total_model > 0:
            for name, count in self.model_frames.items():
                label = "Constant Velocity (hand-held)" if name == "CV" else "Constant Accel (throw)"
                print(f"     {label:36s}: {count:5d} frames  "
                      f"({100*count/total_model:.1f}%)")

        print(f"\n{sep}\n")


# ──────────────────────────────────────────────
#  Main tracking loop
# ──────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to video file (omit for RPi camera)")
    ap.add_argument("-b", "--buffer", type=int, default=64)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--px-per-meter", type=float, default=500)
    ap.add_argument("--lookahead", type=int, default=5)
    ap.add_argument("--stay-prob", type=float, default=0.95)
    ap.add_argument("--min-radius", type=float, default=3)
    ap.add_argument("--camera-id", type=int, default=0,
                    help="libcamera camera index (default 0)")
    ap.add_argument("--headless", action="store_true",
                    help="run without display (benchmark only)")
    ap.add_argument("--no-picamera2", action="store_true",
                    help="force cv2.VideoCapture instead of picamera2")
    args = vars(ap.parse_args())

    W = args["width"]
    H = args["height"]

    color_lower = (29, 86, 6)
    color_upper = (64, 255, 255)

    pts = deque(maxlen=args["buffer"])
    pred_pts = deque(maxlen=args["buffer"])

    # ── Open camera or video ──
    if args.get("video"):
        cam = cv2.VideoCapture(args["video"])
        use_video_file = True
    elif HAS_PICAMERA2 and not args["no_picamera2"]:
        print(f"[INFO] Opening IMX219 via Picamera2 (camera {args['camera_id']}, "
              f"{W}x{H} @ {args['fps']}fps)")
        cam = RPiCamera(width=W, height=H, fps=args["fps"],
                        camera_id=args["camera_id"])
        use_video_file = False
    else:
        print("[INFO] Using cv2.VideoCapture(0) fallback")
        cam = FallbackCamera(src=0, width=W, height=H)
        use_video_file = False

    # Build the two Kalman models
    cv_model = KalmanModel(
        model_type="CV", fps=args["fps"],
        q_pos=2.0, q_vel=10.0, r=10.0,
    )
    ca_model = KalmanModel(
        model_type="CA", fps=args["fps"],
        px_per_meter=args["px_per_meter"],
        q_pos=1.0, q_vel=5.0, q_accel=2.0, r=10.0,
    )

    stay = args["stay_prob"]
    switch = 1.0 - stay
    tpm = [[stay, switch],
           [switch, stay]]

    imm = IMMEstimator(
        models=[cv_model, ca_model],
        tpm=tpm,
        initial_probs=[0.7, 0.3],
        frame_w=W, frame_h=H,
    )

    detector = AdaptiveBallDetector(
        color_lower=color_lower,
        color_upper=color_upper,
        min_radius_abs=args["min_radius"],
    )

    bench = BenchmarkStats(lookahead=args["lookahead"])
    frame_idx = 0
    headless = args["headless"]

    print("[INFO] Tracking started — press 'q' to quit")

    while True:
        ret, frame = cam.read()
        if not ret or frame is None:
            break

        # For video files, resize to working width; live camera is already sized
        if use_video_file:
            h0, w0 = frame.shape[:2]
            if w0 != W:
                scale = W / w0
                frame = cv2.resize(frame, (W, int(h0 * scale)))

        # ── Detection ──
        t0 = time.perf_counter()

        kalman_hint = imm.pos if imm.is_tracking else None
        center, radius, mask = detector.detect(
            frame,
            kalman_pos=kalman_hint,
            kalman_tracking=imm.is_tracking,
        )

        t_raw = time.perf_counter()
        dt_raw = t_raw - t0

        # ── IMM update ──
        kalman_pos = None
        kalman_future = None

        if center is not None:
            if radius > detector.min_radius_abs:
                cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            imm.predict()
            imm.correct(center[0], center[1])
            kalman_pos = imm.pos
        else:
            pred = imm.predict()
            imm.coast()
            if imm.is_tracking and pred is not None:
                kalman_pos = pred

        if imm.is_tracking:
            kalman_future = imm.future_position(n_frames=args["lookahead"])

        t_kalman = time.perf_counter()
        dt_kalman = t_kalman - t0

        info_dict = imm.state_info if imm.initialized else None
        active_model = info_dict["model"] if info_dict else None

        bench.record_frame(
            frame_idx=frame_idx,
            raw_center=center,
            kalman_pos=kalman_pos,
            kalman_future=kalman_future,
            is_kalman_tracking=imm.is_tracking,
            dt_raw=dt_raw,
            dt_kalman=dt_kalman,
            active_model=active_model,
        )

        if not headless:
            # ── Draw IMM-filtered position ──
            if imm.is_tracking and kalman_pos is not None:
                px, py = int(kalman_pos[0]), int(kalman_pos[1])
                cv2.circle(frame, (px, py), 8, (255, 0, 0), 2)

                if kalman_future is not None:
                    fx, fy = int(kalman_future[0]), int(kalman_future[1])
                    cv2.circle(frame, (fx, fy), 10, (255, 0, 255), 2)
                    cv2.line(frame, (px, py), (fx, fy), (255, 0, 255), 1)

                pred_pts.appendleft((px, py))

            # ── Trails ──
            pts.appendleft(center)
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 255, 0), thickness)

            for i in range(1, len(pred_pts)):
                if pred_pts[i - 1] is None or pred_pts[i] is None:
                    continue
                cv2.line(frame, pred_pts[i - 1], pred_pts[i], (255, 0, 0), 2)

            # ── HUD ──
            if imm.initialized:
                si = imm.state_info
                cv_pct = si["prob_cv"] * 100
                ca_pct = si["prob_ca"] * 100

                bar_w, bar_h = 200, 16
                bar_x, bar_y = 10, 8
                cv_w = int(bar_w * si["prob_cv"])
                cv2.rectangle(frame, (bar_x, bar_y),
                              (bar_x + cv_w, bar_y + bar_h), (0, 200, 0), -1)
                cv2.rectangle(frame, (bar_x + cv_w, bar_y),
                              (bar_x + bar_w, bar_y + bar_h), (0, 0, 200), -1)
                cv2.rectangle(frame, (bar_x, bar_y),
                              (bar_x + bar_w, bar_y + bar_h), (255, 255, 255), 1)

                cv2.putText(frame, f"CV {cv_pct:.0f}%", (bar_x + 4, bar_y + 13),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(frame, f"CA {ca_pct:.0f}%",
                            (bar_x + bar_w - 60, bar_y + 13),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                info = (f"vel=({si['vx']:.1f},{si['vy']:.1f})  "
                        f"coast={imm.frames_since_seen}  "
                        f"minR={detector.adaptive_min_radius:.1f}")
                cv2.putText(frame, info, (10, 42),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            # Show FPS
            fps_text = f"FPS: {1.0/max(dt_kalman, 1e-6):.0f}"
            cv2.putText(frame, fps_text, (W - 120, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("Ball Tracker — RPi5 IMX219", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        else:
            pts.appendleft(center)
            if imm.is_tracking and kalman_pos is not None:
                pred_pts.appendleft((int(kalman_pos[0]), int(kalman_pos[1])))

        frame_idx += 1

    bench.report()
    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()