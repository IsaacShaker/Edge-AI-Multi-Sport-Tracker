"""
Ball Tracking with OpenCV + Kalman Filter (Projectile Motion Model)

Based on the PyImageSearch ball tracking tutorial, enhanced with a
kinematic Kalman filter that models parabolic (thrown-ball) motion.

Benefits over the baseline:
  - Predicts ball position *ahead* of the current frame, reducing
    perceived tracking latency.
  - Gracefully handles short occlusions / detection dropouts by
    coasting on the kinematic prediction.
  - Smooths noisy detections for a steadier track.

Usage:
  python ball_tracker_kalman.py                     # webcam
  python ball_tracker_kalman.py -v ball_video.mp4   # video file
"""

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time


# ──────────────────────────────────────────────
#  Kalman Filter with constant-acceleration
#  (gravity) kinematic model
# ──────────────────────────────────────────────

class BallKalmanFilter:
    """
    State vector:  [x, y, vx, vy, ax, ay]^T
    
    We model the ball as undergoing constant acceleration.
    For a thrown ball, ax ≈ 0 and ay ≈ gravity (pixels/frame^2).
    The filter will *learn* the actual accelerations from
    measurements, but we seed ay with a gravity prior.
    """

    def __init__(self, fps=30, gravity_m_s2=9.81, px_per_meter=500):
        self.kf = cv2.KalmanFilter(6, 2)  # 6 state dims, 2 measurement dims

        dt = 1.0  # one frame step (we fold real dt into units)

        # -- Transition matrix F (constant-acceleration kinematics) --
        # x'  = x + vx*dt + 0.5*ax*dt^2
        # y'  = y + vy*dt + 0.5*ay*dt^2
        # vx' = vx + ax*dt
        # vy' = vy + ay*dt
        # ax' = ax
        # ay' = ay
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  0.5*dt**2, 0         ],
            [0, 1, 0,  dt, 0,         0.5*dt**2  ],
            [0, 0, 1,  0,  dt,        0          ],
            [0, 0, 0,  1,  0,         dt         ],
            [0, 0, 0,  0,  1,         0          ],
            [0, 0, 0,  0,  0,         1          ],
        ], dtype=np.float32)

        # -- Measurement matrix H  (we only observe x, y) --
        self.kf.measurementMatrix = np.zeros((2, 6), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1  # measure x
        self.kf.measurementMatrix[1, 1] = 1  # measure y

        # -- Process noise Q --
        # Tuning knob: larger → trusts measurements more, reacts faster.
        # Smaller → smoother track, slower to react to sudden changes.
        q_pos   = 1.0
        q_vel   = 5.0
        q_accel = 2.0
        self.kf.processNoiseCov = np.diag([
            q_pos, q_pos, q_vel, q_vel, q_accel, q_accel
        ]).astype(np.float32)

        # -- Measurement noise R --
        # Typical HSV-contour centroid noise is a few pixels.
        r = 10.0
        self.kf.measurementNoiseCov = np.array([
            [r, 0],
            [0, r],
        ], dtype=np.float32)

        # -- Initial covariance --
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100

        # Convert gravity to pixels/frame^2 for the prior
        self.gravity_px = gravity_m_s2 * px_per_meter / (fps ** 2)

        self.initialized = False
        self.frames_since_seen = 0
        self.max_coast_frames = 15  # predict without measurement for up to N frames

    def init_state(self, x, y):
        """Seed the filter with the first detection."""
        self.kf.statePost = np.array(
            [x, y, 0, 0, 0, self.gravity_px], dtype=np.float32
        ).reshape(6, 1)
        self.initialized = True
        self.frames_since_seen = 0

    def predict(self):
        """Advance the state by one frame (call every frame)."""
        if not self.initialized:
            return None
        pred = self.kf.predict()
        return float(pred[0][0]), float(pred[1][0])

    def correct(self, x, y):
        """Incorporate a new measurement (detection)."""
        if not self.initialized:
            self.init_state(x, y)
            return
        meas = np.array([[np.float32(x)], [np.float32(y)]])
        self.kf.correct(meas)
        self.frames_since_seen = 0

    def coast(self):
        """No detection this frame — rely on prediction only."""
        self.frames_since_seen += 1

    @property
    def is_tracking(self):
        return self.initialized and self.frames_since_seen < self.max_coast_frames

    @property
    def state(self):
        """Return the full current state estimate."""
        s = self.kf.statePost.flatten()
        return {
            "x": s[0], "y": s[1],
            "vx": s[2], "vy": s[3],
            "ax": s[4], "ay": s[5],
        }

    def future_position(self, n_frames=3):
        """
        Extrapolate the *current* state n_frames into the future
        without modifying filter state. Useful for look-ahead display
        or pre-positioning a robot/camera.
        """
        s = self.kf.statePost.flatten()
        x, y, vx, vy, ax, ay = s
        dt = n_frames
        fx = x + vx * dt + 0.5 * ax * dt ** 2
        fy = y + vy * dt + 0.5 * ay * dt ** 2
        return float(fx), float(fy)


# ──────────────────────────────────────────────
#  Main tracking loop
# ──────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to video file (omit for webcam)")
    ap.add_argument("-b", "--buffer", type=int, default=64, help="trail deque length")
    ap.add_argument("--fps", type=int, default=30, help="approx FPS of source")
    ap.add_argument("--px-per-meter", type=float, default=500,
                    help="rough scale factor for gravity prior (pixels per real-world meter)")
    ap.add_argument("--lookahead", type=int, default=5,
                    help="frames to predict ahead for the ghost marker")
    args = vars(ap.parse_args())

    # HSV bounds for the green ball — adjust for your ball color!
    color_lower = (29, 86, 6)
    color_upper = (64, 255, 255)

    pts = deque(maxlen=args["buffer"])       # measured trail
    pred_pts = deque(maxlen=args["buffer"])   # predicted trail

    # Start video stream
    if not args.get("video", False):
        vs = VideoStream(src=0).start()
    else:
        vs = cv2.VideoCapture(args["video"])
    time.sleep(2.0)

    kf = BallKalmanFilter(
        fps=args["fps"],
        px_per_meter=args["px_per_meter"],
    )

    while True:
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        if frame is None:
            break

        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ── Detection (same as original tutorial) ──
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

            if radius > 10:
                # Draw raw detection
                cv2.circle(frame, (int(bx), int(by)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # ── Kalman correct ──
                kf.predict()                # must predict before correct
                kf.correct(center[0], center[1])
            else:
                center = None

        # If no detection this frame, coast on the kinematic model
        if center is None:
            pred = kf.predict()
            kf.coast()
        else:
            pred = (kf.state["x"], kf.state["y"])

        # ── Draw Kalman-filtered position ──
        if kf.is_tracking and pred is not None:
            px, py = int(pred[0]), int(pred[1])
            cv2.circle(frame, (px, py), 8, (255, 0, 0), 2)  # blue = filtered

            # ── Draw look-ahead "ghost" marker ──
            fx, fy = kf.future_position(n_frames=args["lookahead"])
            fx, fy = int(fx), int(fy)
            cv2.circle(frame, (fx, fy), 10, (255, 0, 255), 2)  # magenta = predicted future
            cv2.line(frame, (px, py), (fx, fy), (255, 0, 255), 1)

            pred_pts.appendleft((px, py))

        # ── Measured trail (green) ──
        pts.appendleft(center)
        for i in range(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 255, 0), thickness)

        # ── Predicted trail (blue, thinner) ──
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

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()