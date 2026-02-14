"""
Ball Tracking with OpenCV + Kalman Filter (Projectile Motion Model)
with built-in performance benchmarking vs. raw detection.

Based on the PyImageSearch ball tracking tutorial, enhanced with a
kinematic Kalman filter that models parabolic (thrown-ball) motion.

Benchmarking methodology:
  We compare two "trackers" on every frame:
    1. Raw   — the HSV contour centroid (baseline, like the tutorial)
    2. Kalman — the kinematic-filter estimate

  Because we don't have ground-truth, we use a *delayed-truth* approach:
    - At frame t, the Kalman filter predicts where the ball will be
      at frame t+L (the look-ahead).
    - At frame t+L, we record the raw detection as "truth".
    - We then score the prediction error vs. that truth.

  This lets us measure:
    • Prediction accuracy  (Kalman look-ahead vs. actual future position)
    • Smoothness           (jitter / frame-to-frame velocity noise)
    • Occlusion bridging   (how many dropout frames each tracker covers)
    • Per-frame latency    (processing time with and without the filter)

Usage:
  python ball_tracker_kalman.py                     # webcam
  python ball_tracker_kalman.py -v ball_video.mp4   # video file

Press 'q' to quit — a benchmark summary prints to the console.
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
#  Kalman Filter with constant-acceleration
#  (gravity) kinematic model
# ──────────────────────────────────────────────

class BallKalmanFilter:
    """
    State vector:  [x, y, vx, vy, ax, ay]^T
    """

    def __init__(self, fps=30, gravity_m_s2=9.81, px_per_meter=500):
        self.kf = cv2.KalmanFilter(6, 2)
        dt = 1.0

        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  0.5*dt**2, 0         ],
            [0, 1, 0,  dt, 0,         0.5*dt**2  ],
            [0, 0, 1,  0,  dt,        0          ],
            [0, 0, 0,  1,  0,         dt         ],
            [0, 0, 0,  0,  1,         0          ],
            [0, 0, 0,  0,  0,         1          ],
        ], dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((2, 6), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1
        self.kf.measurementMatrix[1, 1] = 1

        q_pos   = 1.0
        q_vel   = 5.0
        q_accel = 2.0
        self.kf.processNoiseCov = np.diag([
            q_pos, q_pos, q_vel, q_vel, q_accel, q_accel
        ]).astype(np.float32)

        r = 10.0
        self.kf.measurementNoiseCov = np.array([
            [r, 0],
            [0, r],
        ], dtype=np.float32)

        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100

        self.gravity_px = gravity_m_s2 * px_per_meter / (fps ** 2)
        self.initialized = False
        self.frames_since_seen = 0
        self.max_coast_frames = 15

    def init_state(self, x, y):
        self.kf.statePost = np.array(
            [x, y, 0, 0, 0, self.gravity_px], dtype=np.float32
        ).reshape(6, 1)
        self.initialized = True
        self.frames_since_seen = 0

    def predict(self):
        if not self.initialized:
            return None
        pred = self.kf.predict()
        return float(pred[0][0]), float(pred[1][0])

    def correct(self, x, y):
        if not self.initialized:
            self.init_state(x, y)
            return
        meas = np.array([[np.float32(x)], [np.float32(y)]])
        self.kf.correct(meas)
        self.frames_since_seen = 0

    def coast(self):
        self.frames_since_seen += 1

    @property
    def is_tracking(self):
        return self.initialized and self.frames_since_seen < self.max_coast_frames

    @property
    def state(self):
        s = self.kf.statePost.flatten()
        return {
            "x": s[0], "y": s[1],
            "vx": s[2], "vy": s[3],
            "ax": s[4], "ay": s[5],
        }

    def future_position(self, n_frames=3):
        s = self.kf.statePost.flatten()
        x, y, vx, vy, ax, ay = s
        dt = n_frames
        fx = x + vx * dt + 0.5 * ax * dt ** 2
        fy = y + vy * dt + 0.5 * ay * dt ** 2
        return float(fx), float(fy)


# ──────────────────────────────────────────────
#  Benchmark stats collector
# ──────────────────────────────────────────────

class BenchmarkStats:
    """Accumulates per-frame metrics for raw vs. Kalman tracking."""

    def __init__(self, lookahead):
        self.lookahead = lookahead

        # Ring buffer of raw detections indexed by frame number,
        # used as delayed ground truth for prediction scoring.
        self.raw_history = {}

        # Prediction accuracy: (frame_predicted_for, predicted_pos)
        self.kalman_predictions = {}  # frame -> (px, py)

        # Smoothness: consecutive position deltas
        self.raw_deltas = []
        self.kalman_deltas = []
        self.prev_raw = None
        self.prev_kalman = None

        # Occlusion bridging
        self.total_frames = 0
        self.raw_detected_frames = 0
        self.kalman_tracked_frames = 0

        # Per-frame processing time (seconds)
        self.times_raw = []       # detection-only time
        self.times_kalman = []    # detection + kalman time

    def record_frame(self, frame_idx, raw_center, kalman_pos,
                     kalman_future, is_kalman_tracking,
                     dt_raw, dt_kalman):
        self.total_frames += 1

        # ── Timing ──
        self.times_raw.append(dt_raw)
        self.times_kalman.append(dt_kalman)

        # ── Detection / tracking counts ──
        if raw_center is not None:
            self.raw_detected_frames += 1
            self.raw_history[frame_idx] = raw_center
        if is_kalman_tracking:
            self.kalman_tracked_frames += 1

        # ── Prediction accuracy (delayed truth) ──
        if kalman_future is not None:
            target_frame = frame_idx + self.lookahead
            self.kalman_predictions[target_frame] = kalman_future

        # Score any past predictions whose target frame is now
        # (handled in report, since we need all data first)

        # ── Smoothness (frame-to-frame jitter) ──
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

    def report(self):
        """Print a formatted comparison summary."""

        sep = "=" * 60
        print(f"\n{sep}")
        print("  BENCHMARK REPORT: Raw Detection vs. Kalman Kinematic")
        print(sep)

        # ── 1. Prediction accuracy ──
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
            print("     (not enough overlapping data to score)")

        # ── 2. Smoothness ──
        print(f"\n  2. SMOOTHNESS  (frame-to-frame displacement std dev)")
        if self.raw_deltas:
            raw_arr = np.array(self.raw_deltas)
            kal_arr = np.array(self.kalman_deltas) if self.kalman_deltas else np.array([0])

            raw_jitter = raw_arr.std()
            kal_jitter = kal_arr.std()

            print(f"     Raw detection jitter  : {raw_jitter:.2f} px  (mean step {raw_arr.mean():.1f} px)")
            print(f"     Kalman filter jitter  : {kal_jitter:.2f} px  (mean step {kal_arr.mean():.1f} px)")
            if raw_jitter > 0:
                reduction = (1 - kal_jitter / raw_jitter) * 100
                print(f"     Jitter reduction      : {reduction:.1f}%")
        else:
            print("     (not enough data)")

        # ── 3. Occlusion bridging ──
        print(f"\n  3. OCCLUSION BRIDGING")
        print(f"     Total frames           : {self.total_frames}")
        print(f"     Raw detected frames    : {self.raw_detected_frames}  "
              f"({100*self.raw_detected_frames/max(1,self.total_frames):.1f}%)")
        print(f"     Kalman tracked frames  : {self.kalman_tracked_frames}  "
              f"({100*self.kalman_tracked_frames/max(1,self.total_frames):.1f}%)")
        extra = self.kalman_tracked_frames - self.raw_detected_frames
        print(f"     Extra frames from Kalman: {extra}")

        # ── 4. Processing latency ──
        print(f"\n  4. PER-FRAME PROCESSING TIME")
        if self.times_raw:
            raw_ms = np.array(self.times_raw) * 1000
            kal_ms = np.array(self.times_kalman) * 1000
            print(f"     Raw detection only    : {raw_ms.mean():.2f} ms avg  "
                  f"({1000/raw_ms.mean():.0f} FPS capacity)")
            print(f"     Detection + Kalman    : {kal_ms.mean():.2f} ms avg  "
                  f"({1000/kal_ms.mean():.0f} FPS capacity)")
            overhead = kal_ms.mean() - raw_ms.mean()
            print(f"     Kalman overhead       : {overhead:.3f} ms/frame")

        print(f"\n{sep}\n")


# ──────────────────────────────────────────────
#  Main tracking loop
# ──────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to video file (omit for webcam)")
    ap.add_argument("-b", "--buffer", type=int, default=64, help="trail deque length")
    ap.add_argument("--fps", type=int, default=30, help="approx FPS of source")
    ap.add_argument("--px-per-meter", type=float, default=500,
                    help="rough scale factor for gravity prior")
    ap.add_argument("--lookahead", type=int, default=5,
                    help="frames to predict ahead for the ghost marker")
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

    kf = BallKalmanFilter(fps=args["fps"], px_per_meter=args["px_per_meter"])
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

        # ── Detection (timed separately) ──
        t0 = time.perf_counter()

        mask = cv2.inRange(hsv, color_lower, color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

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

        # ── Kalman update (timed as detection + kalman) ──
        kalman_pos = None
        kalman_future = None

        if center is not None:
            if center is not None and radius > 10:
                cv2.circle(frame, (int(bx), int(by)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            kf.predict()
            kf.correct(center[0], center[1])
            kalman_pos = (kf.state["x"], kf.state["y"])
        else:
            pred = kf.predict()
            kf.coast()
            if kf.is_tracking and pred is not None:
                kalman_pos = pred

        if kf.is_tracking:
            kalman_future = kf.future_position(n_frames=args["lookahead"])

        t_kalman = time.perf_counter()
        dt_kalman = t_kalman - t0

        # ── Record benchmark data ──
        bench.record_frame(
            frame_idx=frame_idx,
            raw_center=center,
            kalman_pos=kalman_pos,
            kalman_future=kalman_future,
            is_kalman_tracking=kf.is_tracking,
            dt_raw=dt_raw,
            dt_kalman=dt_kalman,
        )

        # ── Draw Kalman-filtered position ──
        if kf.is_tracking and kalman_pos is not None:
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

        # ── HUD ──
        if kf.is_tracking:
            st = kf.state
            info = (f"vel=({st['vx']:.1f},{st['vy']:.1f})  "
                    f"acc=({st['ax']:.1f},{st['ay']:.1f})  "
                    f"coast={kf.frames_since_seen}")
            cv2.putText(frame, info, (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Ball Tracker + Kalman", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        frame_idx += 1

    # ── Print benchmark report on exit ──
    bench.report()

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()