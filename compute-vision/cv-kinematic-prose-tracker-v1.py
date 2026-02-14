"""
Ball Tracking with OpenCV + Adaptive IMM Kalman Filter
======================================================
Runs two motion models in parallel via an Interacting Multiple Model
(IMM) estimator and blends them automatically:

  Model 1 — Constant Velocity  (CV):  good for hand-held / arbitrary motion
  Model 2 — Constant Acceleration (CA): good for throws (gravity model)

The IMM computes per-frame model probabilities so the tracker
seamlessly transitions between "someone holding/waving the ball"
and "the ball is in free flight."

Press 'q' to quit — benchmark summary prints to the console.
"""

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import math


# ──────────────────────────────────────────────
#  Single Kalman model wrapper
# ──────────────────────────────────────────────

class KalmanModel:
    """
    Wraps cv2.KalmanFilter with a named motion model.

    Constant Velocity (CV):  state = [x, y, vx, vy]         (4 states)
    Constant Acceleration (CA): state = [x, y, vx, vy, ax, ay] (6 states)
    """

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

    # ── Constant Velocity ──
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

    # ── Constant Acceleration ──
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
                [x, y, 0, 0], dtype=np.float32
            ).reshape(4, 1)
        else:
            self.kf.statePost = np.array(
                [x, y, 0, 0, 0, self.gravity_px], dtype=np.float32
            ).reshape(6, 1)
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
        """Measurement residual: z - H * x_predicted."""
        predicted_z = self.kf.measurementMatrix @ self.kf.statePre
        return z - predicted_z

    def innovation_cov(self):
        """S = H * P_pre * H^T + R"""
        H = self.kf.measurementMatrix
        P = self.kf.errorCovPre
        R = self.kf.measurementNoiseCov
        return H @ P @ H.T + R


# ──────────────────────────────────────────────
#  Interacting Multiple Model (IMM) Estimator
# ──────────────────────────────────────────────

class IMMEstimator:
    """
    Blends N Kalman models using Bayesian model-probability update.

    Transition probability matrix (TPM) controls how likely the system
    is to switch between models frame-to-frame.
    """

    def __init__(self, models, tpm=None, initial_probs=None,
                 frame_w=600, frame_h=450):
        self.models = models
        self.N = len(models)
        self.frame_w = frame_w
        self.frame_h = frame_h

        # Transition probability matrix: tpm[i][j] = P(switch to j | currently i)
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
        self.max_coast_frames = 8  # reduced from 15 to limit runaway predictions

    def init_state(self, x, y):
        for m in self.models:
            m.init_state(x, y)
        self.initialized = True
        self.frames_since_seen = 0

    def predict(self):
        """Predict step for all models."""
        if not self.initialized:
            return None
        for m in self.models:
            m.predict()
        # Return blended position
        return self._blended_pos()

    def correct(self, x, y):
        """Correct all models & update model probabilities."""
        if not self.initialized:
            self.init_state(x, y)
            return

        z = np.array([[np.float32(x)], [np.float32(y)]])

        # Compute likelihood of measurement under each model
        likelihoods = np.zeros(self.N)
        for i, m in enumerate(self.models):
            innov = m.innovation(z)
            S = m.innovation_cov()
            # Gaussian likelihood: N(innov; 0, S)
            S64 = S.astype(np.float64)
            det = max(np.linalg.det(S64), 1e-30)
            inv_S = np.linalg.inv(S64)
            innov64 = innov.astype(np.float64)
            maha = float((innov64.T @ inv_S @ innov64)[0, 0])
            likelihoods[i] = math.exp(-0.5 * maha) / math.sqrt(
                (2 * math.pi) ** 2 * det
            )

        # Bayesian update: prior from TPM, then weight by likelihood
        # c_j = sum_i( tpm[i,j] * mu[i] )  — predicted model prob
        c = self.tpm.T @ self.mu
        self.mu = likelihoods * c
        total = self.mu.sum()
        if total > 1e-30:
            self.mu /= total
        else:
            self.mu = np.ones(self.N) / self.N  # fallback to uniform

        # Check if prediction was wildly off — if so, re-init
        # to avoid a huge snap-back that poisons the stats
        px, py = self._blended_pos()
        dist = math.hypot(px - float(z[0][0]), py - float(z[1][0]))
        if dist > max(self.frame_w, self.frame_h) * 0.5:
            # Prediction was more than half a screen away — hard reset
            self.init_state(float(z[0][0]), float(z[1][0]))
            return

        # Correct each model with the measurement
        for m in self.models:
            m.correct(z)

        self.frames_since_seen = 0

    def coast(self):
        self.frames_since_seen += 1

    @property
    def is_tracking(self):
        return self.initialized and self.frames_since_seen < self.max_coast_frames

    def _clamp(self, x, y):
        """Keep positions within frame bounds + small margin."""
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
        """Blended future position across all models."""
        fx, fy = 0.0, 0.0
        for i, m in enumerate(self.models):
            mx, my = m.future_position(n_frames)
            fx += self.mu[i] * mx
            fy += self.mu[i] * my
        return fx, fy


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
        # Model selection tracking
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

        # 1. Prediction accuracy
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

        # 2. Smoothness
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

        # 3. Occlusion bridging
        print(f"\n  3. OCCLUSION BRIDGING")
        print(f"     Total frames           : {self.total_frames}")
        print(f"     Raw detected frames    : {self.raw_detected_frames}  "
              f"({100*self.raw_detected_frames/max(1,self.total_frames):.1f}%)")
        print(f"     IMM tracked frames     : {self.kalman_tracked_frames}  "
              f"({100*self.kalman_tracked_frames/max(1,self.total_frames):.1f}%)")
        extra = self.kalman_tracked_frames - self.raw_detected_frames
        print(f"     Extra frames from IMM  : {extra}")

        # 4. Processing time
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

        # 5. Model selection breakdown
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
    ap.add_argument("-v", "--video", help="path to video file (omit for webcam)")
    ap.add_argument("-b", "--buffer", type=int, default=64)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--px-per-meter", type=float, default=500)
    ap.add_argument("--lookahead", type=int, default=5)
    # Tuning knobs
    ap.add_argument("--stay-prob", type=float, default=0.95,
                    help="probability of staying in same model (0.5-0.99)")
    args = vars(ap.parse_args())

    color_lower = (29, 86, 6)
    color_upper = (64, 255, 255)

    pts = deque(maxlen=args["buffer"])
    pred_pts = deque(maxlen=args["buffer"])

    if not args.get("video", False):
        vs = VideoStream(src=0).start()
    else:
        vs = cv2.VideoCapture(args["video"])
    time.sleep(2.0)

    # Build the two models
    cv_model = KalmanModel(
        model_type="CV", fps=args["fps"],
        q_pos=2.0, q_vel=10.0, r=10.0,
    )
    ca_model = KalmanModel(
        model_type="CA", fps=args["fps"],
        px_per_meter=args["px_per_meter"],
        q_pos=1.0, q_vel=5.0, q_accel=2.0, r=10.0,
    )

    # Transition probability matrix
    stay = args["stay_prob"]
    switch = 1.0 - stay
    tpm = [[stay, switch],
           [switch, stay]]

    imm = IMMEstimator(
        models=[cv_model, ca_model],
        tpm=tpm,
        initial_probs=[0.7, 0.3],
        frame_w=600,
        frame_h=450,
    )

    bench = BenchmarkStats(lookahead=args["lookahead"])
    frame_idx = 0

    while True:
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        if frame is None:
            break

        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ── Detection ──
        t0 = time.perf_counter()

        mask = cv2.inRange(hsv, color_lower, color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        radius = 0

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((bx, by), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]),
                          int(M["m01"] / M["m00"]))
            if radius <= 10:
                center = None

        t_raw = time.perf_counter()
        dt_raw = t_raw - t0

        # ── IMM update ──
        kalman_pos = None
        kalman_future = None

        if center is not None:
            if radius > 10:
                cv2.circle(frame, (int(bx), int(by)), int(radius),
                           (0, 255, 255), 2)
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

        # Active model for benchmark
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

        # ── Draw IMM-filtered position ──
        if imm.is_tracking and kalman_pos is not None:
            px, py = int(kalman_pos[0]), int(kalman_pos[1])
            cv2.circle(frame, (px, py), 8, (255, 0, 0), 2)

            if kalman_future is not None:
                fx, fy = int(kalman_future[0]), int(kalman_future[1])
                cv2.circle(frame, (fx, fy), 10, (255, 0, 255), 2)
                cv2.line(frame, (px, py), (fx, fy), (255, 0, 255), 1)

            pred_pts.appendleft((px, py))

        # ── Measured trail (green) ──
        pts.appendleft(center)
        for i in range(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 255, 0), thickness)

        # ── Predicted trail (blue) ──
        for i in range(1, len(pred_pts)):
            if pred_pts[i - 1] is None or pred_pts[i] is None:
                continue
            cv2.line(frame, pred_pts[i - 1], pred_pts[i], (255, 0, 0), 2)

        # ── HUD with model probabilities ──
        if imm.initialized:
            si = imm.state_info
            # Color-coded model indicator
            cv_pct = si["prob_cv"] * 100
            ca_pct = si["prob_ca"] * 100

            # Bar showing CV vs CA blend
            bar_w = 200
            bar_h = 16
            bar_x, bar_y = 10, 8
            cv_w = int(bar_w * si["prob_cv"])
            # CV = green portion, CA = red portion
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
                    f"coast={imm.frames_since_seen}")
            cv2.putText(frame, info, (10, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.imshow("Ball Tracker — Adaptive IMM (CV+CA)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        frame_idx += 1

    bench.report()

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()