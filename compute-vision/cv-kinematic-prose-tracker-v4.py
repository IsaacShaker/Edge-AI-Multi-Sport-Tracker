"""
Ball Tracking with OpenCV + Adaptive IMM Kalman Filter + 3D Distance
=====================================================================
Extends the tracker with depth estimation by filtering apparent ball
radius as a third measurement alongside (x, y).

  State vectors now include radius and its rate of change:
    CV:  [x, y, vx, vy, r, dr]           (6 states, 3 measurements)
    CA:  [x, y, vx, vy, ax, ay, r, dr]   (8 states, 3 measurements)

  Distance estimation uses the pinhole model:
    Z = (real_diameter × focal_length) / (2 × filtered_radius)

  The Kalman-filtered radius is much smoother than raw detection radius,
  giving stable depth even when the ball flickers at range.

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
#  Single Kalman model wrapper (now with radius)
# ──────────────────────────────────────────────

class KalmanModel:
    """
    CV:  state = [x, y, vx, vy, r, dr]                 (6 states, 3 meas)
    CA:  state = [x, y, vx, vy, ax, ay, r, dr]         (8 states, 3 meas)

    The radius channel lets the Kalman filter smooth apparent size and
    predict it forward, which feeds directly into distance estimation.
    """

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

    # ── Constant Velocity: 6 states ──
    def _init_cv(self, q_pos, q_vel, r_meas):
        # States: [x, y, vx, vy, r, dr]
        self.kf = cv2.KalmanFilter(6, 3)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  0,  0],
            [0, 1, 0,  dt, 0,  0],
            [0, 0, 1,  0,  0,  0],
            [0, 0, 0,  1,  0,  0],
            [0, 0, 0,  0,  1, dt],
            [0, 0, 0,  0,  0,  1],
        ], dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((3, 6), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1  # x
        self.kf.measurementMatrix[1, 1] = 1  # y
        self.kf.measurementMatrix[2, 4] = 1  # r

        self.kf.processNoiseCov = np.diag([
            q_pos, q_pos, q_vel, q_vel,
            self.q_radius, self.q_radius_vel,
        ]).astype(np.float32)

        self.kf.measurementNoiseCov = np.diag([
            r_meas, r_meas, self.r_radius,
        ]).astype(np.float32)

        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100
        self.gravity_px = 0.0

    # ── Constant Acceleration: 8 states ──
    def _init_ca(self, fps, g, ppm, q_pos, q_vel, q_accel, r_meas):
        # States: [x, y, vx, vy, ax, ay, r, dr]
        self.kf = cv2.KalmanFilter(8, 3)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  0.5*dt**2, 0,         0,  0],
            [0, 1, 0,  dt, 0,         0.5*dt**2,  0,  0],
            [0, 0, 1,  0,  dt,        0,          0,  0],
            [0, 0, 0,  1,  0,         dt,         0,  0],
            [0, 0, 0,  0,  1,         0,          0,  0],
            [0, 0, 0,  0,  0,         1,          0,  0],
            [0, 0, 0,  0,  0,         0,          1, dt],
            [0, 0, 0,  0,  0,         0,          0,  1],
        ], dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((3, 8), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1  # x
        self.kf.measurementMatrix[1, 1] = 1  # y
        self.kf.measurementMatrix[2, 6] = 1  # r

        self.kf.processNoiseCov = np.diag([
            q_pos, q_pos, q_vel, q_vel, q_accel, q_accel,
            self.q_radius, self.q_radius_vel,
        ]).astype(np.float32)

        self.kf.measurementNoiseCov = np.diag([
            r_meas, r_meas, self.r_radius,
        ]).astype(np.float32)

        self.kf.errorCovPost = np.eye(8, dtype=np.float32) * 100
        self.gravity_px = g * ppm / (fps ** 2)

    def init_state(self, x, y, r=20.0):
        if self.model_type == "CV":
            self.kf.statePost = np.array(
                [x, y, 0, 0, r, 0], dtype=np.float32).reshape(6, 1)
            self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 100
        else:
            self.kf.statePost = np.array(
                [x, y, 0, 0, 0, self.gravity_px, r, 0],
                dtype=np.float32).reshape(8, 1)
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
        r_idx = 4 if self.model_type == "CV" else 6
        return max(1.0, float(s[r_idx]))

    @property
    def radius_vel(self):
        s = self.kf.statePost.flatten()
        dr_idx = 5 if self.model_type == "CV" else 7
        return float(s[dr_idx])

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
        # Future radius
        r_idx = 4 if self.model_type == "CV" else 6
        dr_idx = r_idx + 1
        fr = max(1.0, s[r_idx] + s[dr_idx] * dt)
        return float(fx), float(fy), float(fr)

    def innovation(self, z):
        predicted_z = self.kf.measurementMatrix @ self.kf.statePre
        return z - predicted_z

    def innovation_cov(self):
        H = self.kf.measurementMatrix
        P = self.kf.errorCovPre
        R = self.kf.measurementNoiseCov
        return H @ P @ H.T + R


# ──────────────────────────────────────────────
#  Depth Estimator (pinhole camera model)
# ──────────────────────────────────────────────

class DepthEstimator:
    """
    Uses the pinhole model:  Z = (D_real * f) / d_image

    Where:
      D_real  = real ball diameter (mm)
      f       = focal length (px)
      d_image = apparent diameter in image (px) = 2 * filtered_radius

    Focal length is estimated from frame width if not provided:
      f ≈ frame_width * 1.2  (reasonable for ~60° FOV webcams)

    Call calibrate() with a known distance to get precise results.
    """

    def __init__(self, ball_diameter_mm=65.0, focal_length_px=None,
                 frame_width=600):
        self.ball_diameter_mm = ball_diameter_mm
        if focal_length_px is not None:
            self.focal_length = focal_length_px
        else:
            # Reasonable default: ~60° horizontal FOV
            self.focal_length = frame_width * 1.2
        self.calibrated = False
        self.K = self.ball_diameter_mm * self.focal_length  # D * f constant

    def calibrate(self, known_distance_mm, observed_radius_px):
        """
        One-shot calibration: hold ball at known distance, press 'c'.
        Solves: f = (known_distance * 2 * observed_radius) / D_real
        """
        d_image = 2.0 * observed_radius_px
        self.focal_length = (known_distance_mm * d_image) / self.ball_diameter_mm
        self.K = self.ball_diameter_mm * self.focal_length
        self.calibrated = True
        print(f"  [Calibrated] focal_length = {self.focal_length:.1f} px, "
              f"K = {self.K:.1f}")

    def estimate(self, filtered_radius):
        """Returns estimated distance in mm."""
        d_image = 2.0 * max(filtered_radius, 1.0)
        return self.K / d_image

    def estimate_m(self, filtered_radius):
        """Returns estimated distance in meters."""
        return self.estimate(filtered_radius) / 1000.0


# ──────────────────────────────────────────────
#  Interacting Multiple Model (IMM) Estimator
# ──────────────────────────────────────────────

class IMMEstimator:
    def __init__(self, models, tpm=None, initial_probs=None,
                 frame_w=600, frame_h=450):
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
        return self._blended_pos()

    def correct(self, x, y, r):
        """Correct all models with (x, y, radius) measurement."""
        if not self.initialized:
            self.init_state(x, y, r)
            return

        z = np.array([[np.float32(x)], [np.float32(y)], [np.float32(r)]])

        likelihoods = np.zeros(self.N)
        for i, m in enumerate(self.models):
            innov = m.innovation(z)
            S = m.innovation_cov()
            S64 = S.astype(np.float64)
            det = max(np.linalg.det(S64), 1e-30)
            inv_S = np.linalg.inv(S64)
            innov64 = innov.astype(np.float64)
            n_meas = z.shape[0]
            maha = float((innov64.T @ inv_S @ innov64)[0, 0])
            likelihoods[i] = math.exp(-0.5 * maha) / math.sqrt(
                (2 * math.pi) ** n_meas * det)

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
            self.init_state(float(z[0][0]), float(z[1][0]), float(z[2][0]))
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
    def filtered_radius(self):
        """Blended filtered radius across models."""
        r = 0.0
        for i, m in enumerate(self.models):
            r += self.mu[i] * m.filtered_radius
        return max(1.0, r)

    @property
    def radius_vel(self):
        """Blended radius velocity (dr/dt) — positive = approaching."""
        dr = 0.0
        for i, m in enumerate(self.models):
            dr += self.mu[i] * m.radius_vel
        return dr

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
            "filtered_r": self.filtered_radius,
            "dr": self.radius_vel,
        }

    def future_position(self, n_frames):
        """Blended future (x, y, radius) across all models."""
        fx, fy, fr = 0.0, 0.0, 0.0
        for i, m in enumerate(self.models):
            mx, my, mr = m.future_position(n_frames)
            fx += self.mu[i] * mx
            fy += self.mu[i] * my
            fr += self.mu[i] * mr
        return fx, fy, max(1.0, fr)


# ──────────────────────────────────────────────
#  Adaptive Ball Detector
# ──────────────────────────────────────────────

class AdaptiveBallDetector:
    """
    Robust ball detector that handles harsh / uneven lighting by
    combining multiple strategies:

      1. CLAHE pre-processing to normalize local brightness
      2. Primary HSV mask with luminance-adaptive saturation floor
      3. Secondary LAB mask (b-channel) to catch washed-out yellows
      4. Adaptive color tracking — drifts thresholds toward recent hits
      5. Circularity + proximity scoring (unchanged from before)
    """

    def __init__(self, color_lower=(20, 40, 40), color_upper=(65, 255, 255),
                 min_radius_abs=3, max_radius=300,
                 clahe_clip=3.0, clahe_grid=8,
                 adapt_rate=0.02):
        # Mutable copies — will drift with adaptive tracking
        self.hsv_lower = np.array(list(color_lower), dtype=np.float32)
        self.hsv_upper = np.array(list(color_upper), dtype=np.float32)
        # Fixed safety bounds — adaptive tracking can't drift beyond these
        self.hsv_lower_floor = np.array([15, 20, 20], dtype=np.float32)
        self.hsv_upper_ceil  = np.array([80, 255, 255], dtype=np.float32)

        # LAB b-channel range for yellow (b > 128 is yellow-ish in LAB)
        self.lab_b_lower = 145
        self.lab_b_upper = 210
        # LAB L limits — reject very dark or near-white unless b is strong
        self.lab_l_lower = 60

        self.min_radius_abs = min_radius_abs
        self.max_radius = max_radius
        self.recent_radii = deque(maxlen=30)
        self.min_circularity = 0.35
        self.adapt_rate = adapt_rate  # how fast thresholds drift per frame

        # CLAHE for local contrast normalization
        self.clahe = cv2.createCLAHE(
            clipLimit=clahe_clip,
            tileGridSize=(clahe_grid, clahe_grid))

        # Rolling color samples for adaptive thresholds
        self.recent_hsv_means = deque(maxlen=60)

    @property
    def adaptive_min_radius(self):
        if len(self.recent_radii) < 5:
            return self.min_radius_abs
        median_r = float(np.median(self.recent_radii))
        return max(self.min_radius_abs, median_r * 0.3)

    def _preprocess(self, frame):
        """Apply CLAHE to the L-channel to even out lighting."""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l_eq = self.clahe.apply(l)
        lab_eq = cv2.merge([l_eq, a, b])
        corrected = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)
        return corrected, lab_eq

    def _hsv_mask(self, corrected_bgr):
        """
        Primary HSV mask with luminance-adaptive saturation floor.
        In bright areas (V > 200), saturation can drop to near zero
        on specular highlights, so we relax the S lower bound.
        """
        blurred = cv2.GaussianBlur(corrected_bgr, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv)

        # Base mask with current adaptive thresholds
        lower = self.hsv_lower.astype(np.uint8)
        upper = self.hsv_upper.astype(np.uint8)
        mask_base = cv2.inRange(hsv, lower, upper)

        # Highlight recovery mask: where V is very high, allow much lower S
        # This catches the washed-out top of the ball under direct light
        highlight_s_floor = 25  # much lower than normal ~86
        lower_bright = np.array([lower[0], highlight_s_floor, 200], dtype=np.uint8)
        upper_bright = np.array([upper[0], upper[1], 255], dtype=np.uint8)
        mask_bright = cv2.inRange(hsv, lower_bright, upper_bright)

        mask = cv2.bitwise_or(mask_base, mask_bright)
        return mask, hsv

    def _lab_mask(self, lab_img):
        """
        Secondary LAB mask — the b-channel encodes yellow-blue and is
        more robust to brightness variation than HSV saturation.
        """
        l, a, b = cv2.split(lab_img)

        # Yellow region in b-channel
        mask_b = cv2.inRange(b, self.lab_b_lower, self.lab_b_upper)
        # Reject very dark pixels (shadows, not ball)
        mask_l = cv2.inRange(l, self.lab_l_lower, 255)
        mask = cv2.bitwise_and(mask_b, mask_l)
        return mask

    def _update_adaptive_thresholds(self, hsv_img, center, radius):
        """
        Sample the HSV values inside the detected ball and slowly
        drift thresholds toward the actual observed color.
        """
        if center is None or radius < 3:
            return

        cx, cy = center
        r_sample = max(2, int(radius * 0.5))  # inner 50% to avoid edges
        h, w = hsv_img.shape[:2]

        # Build a small circular ROI mask
        y_min = max(0, cy - r_sample)
        y_max = min(h, cy + r_sample)
        x_min = max(0, cx - r_sample)
        x_max = min(w, cx + r_sample)

        roi = hsv_img[y_min:y_max, x_min:x_max]
        if roi.size == 0:
            return

        # Circular mask within ROI
        roi_h, roi_w = roi.shape[:2]
        Y, X = np.ogrid[:roi_h, :roi_w]
        cy_local, cx_local = roi_h // 2, roi_w // 2
        circ_mask = ((X - cx_local)**2 + (Y - cy_local)**2) <= r_sample**2

        pixels = roi[circ_mask]
        if len(pixels) < 5:
            return

        mean_hsv = pixels.mean(axis=0)
        self.recent_hsv_means.append(mean_hsv)

        if len(self.recent_hsv_means) < 10:
            return

        # Compute rolling stats
        arr = np.array(self.recent_hsv_means)
        mu = arr.mean(axis=0)
        sigma = arr.std(axis=0)

        # Target thresholds: mean ± 2.5σ, but clamped to safety bounds
        target_lower = np.clip(mu - 2.5 * sigma,
                               self.hsv_lower_floor, self.hsv_upper_ceil)
        target_upper = np.clip(mu + 2.5 * sigma,
                               self.hsv_lower_floor, self.hsv_upper_ceil)

        # Drift slowly toward targets
        a = self.adapt_rate
        self.hsv_lower = np.clip(
            self.hsv_lower * (1 - a) + target_lower * a,
            self.hsv_lower_floor, self.hsv_upper_ceil)
        self.hsv_upper = np.clip(
            self.hsv_upper * (1 - a) + target_upper * a,
            self.hsv_lower_floor, self.hsv_upper_ceil)

    def detect(self, frame, kalman_pos=None, kalman_tracking=False):
        # Step 1: CLAHE preprocessing
        corrected, lab_img = self._preprocess(frame)

        # Step 2: Dual-mask — HSV (primary) + LAB (rescue)
        mask_hsv, hsv_img = self._hsv_mask(corrected)
        mask_lab = self._lab_mask(lab_img)
        mask = cv2.bitwise_or(mask_hsv, mask_lab)

        # Step 3: Morphology (adaptive as before)
        min_r = self.adaptive_min_radius
        if min_r < 8:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
        else:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

        # Step 4: Find and score candidates
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

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
                "center": (cx, cy), "radius": radius,
                "bx": bx, "by": by, "score": score,
            })

        if not candidates:
            return None, 0, mask

        best = max(candidates, key=lambda c: c["score"])

        if len(self.recent_radii) > 10:
            median_r = float(np.median(self.recent_radii))
            if best["radius"] > median_r * 4.0 and best["radius"] > 30:
                return None, 0, mask

        self.recent_radii.append(best["radius"])

        # Step 5: Update adaptive thresholds from this detection
        self._update_adaptive_thresholds(hsv_img, best["center"], best["radius"])

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
        # Depth stats
        self.depth_samples = []
        self.raw_radius_samples = []
        self.filtered_radius_samples = []

    def record_frame(self, frame_idx, raw_center, kalman_pos,
                     kalman_future, is_kalman_tracking,
                     dt_raw, dt_kalman, active_model=None,
                     raw_radius=None, filtered_radius=None,
                     depth_m=None):
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
            self.kalman_predictions[target_frame] = kalman_future[:2]

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

        if raw_radius is not None:
            self.raw_radius_samples.append(raw_radius)
        if filtered_radius is not None:
            self.filtered_radius_samples.append(filtered_radius)
        if depth_m is not None:
            self.depth_samples.append(depth_m)

    def report(self):
        sep = "=" * 60
        print(f"\n{sep}")
        print("  BENCHMARK: Adaptive IMM Kalman (CV+CA) with 3D Depth")
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
            print(f"     90th percentile    : {np.percentile(arr, 90):.1f} px")
        else:
            print("     (not enough data)")

        print(f"\n  2. SMOOTHNESS  (frame-to-frame displacement std dev)")
        if self.raw_deltas:
            raw_arr = np.array(self.raw_deltas)
            kal_arr = np.array(self.kalman_deltas) if self.kalman_deltas else np.array([0])
            raw_j, kal_j = raw_arr.std(), kal_arr.std()
            print(f"     Raw jitter            : {raw_j:.2f} px")
            print(f"     IMM Kalman jitter     : {kal_j:.2f} px")
            if raw_j > 0:
                print(f"     Jitter reduction      : {(1 - kal_j / raw_j) * 100:.1f}%")

        print(f"\n  3. RADIUS SMOOTHING")
        if self.raw_radius_samples and self.filtered_radius_samples:
            raw_r = np.array(self.raw_radius_samples)
            filt_r = np.array(self.filtered_radius_samples)
            # Frame-to-frame jitter
            raw_r_jitter = np.diff(raw_r).std() if len(raw_r) > 1 else 0
            filt_r_jitter = np.diff(filt_r).std() if len(filt_r) > 1 else 0
            print(f"     Raw radius jitter     : {raw_r_jitter:.2f} px")
            print(f"     Filtered radius jitter: {filt_r_jitter:.2f} px")
            if raw_r_jitter > 0:
                print(f"     Radius jitter reduction: {(1 - filt_r_jitter / raw_r_jitter) * 100:.1f}%")

        print(f"\n  4. DEPTH ESTIMATION")
        if self.depth_samples:
            d = np.array(self.depth_samples)
            print(f"     Samples               : {len(d)}")
            print(f"     Range                 : {d.min():.2f} – {d.max():.2f} m")
            print(f"     Mean depth            : {d.mean():.2f} m")
            print(f"     Depth jitter (σ Δd)   : {np.diff(d).std():.3f} m")
        else:
            print("     (no depth data)")

        print(f"\n  5. OCCLUSION BRIDGING")
        print(f"     Total frames           : {self.total_frames}")
        print(f"     Raw detected frames    : {self.raw_detected_frames}  "
              f"({100*self.raw_detected_frames/max(1,self.total_frames):.1f}%)")
        print(f"     IMM tracked frames     : {self.kalman_tracked_frames}  "
              f"({100*self.kalman_tracked_frames/max(1,self.total_frames):.1f}%)")

        print(f"\n  6. PER-FRAME PROCESSING TIME")
        if self.times_raw:
            raw_ms = np.array(self.times_raw) * 1000
            kal_ms = np.array(self.times_kalman) * 1000
            print(f"     Detection only        : {raw_ms.mean():.2f} ms  "
                  f"({1000/raw_ms.mean():.0f} FPS)")
            print(f"     Detection + IMM+Depth : {kal_ms.mean():.2f} ms  "
                  f"({1000/kal_ms.mean():.0f} FPS)")

        print(f"\n  7. MODEL SELECTION")
        total_model = sum(self.model_frames.values())
        if total_model > 0:
            for name, count in self.model_frames.items():
                label = "Constant Velocity" if name == "CV" else "Constant Accel"
                print(f"     {label:24s}: {count:5d} frames  "
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
    ap.add_argument("--stay-prob", type=float, default=0.95)
    ap.add_argument("--min-radius", type=float, default=3)
    # Depth estimation params
    ap.add_argument("--ball-diameter", type=float, default=65.0,
                    help="real ball diameter in mm (tennis=65, soccer=220)")
    ap.add_argument("--focal-length", type=float, default=None,
                    help="camera focal length in px (auto-estimated if omitted)")
    ap.add_argument("--calibrate-dist", type=float, default=None,
                    help="known distance in mm for one-shot calibration (press 'c')")
    args = vars(ap.parse_args())

    # Wider initial range — the adaptive tracker will tighten these
    color_lower = (20, 40, 40)
    color_upper = (70, 255, 255)

    pts = deque(maxlen=args["buffer"])
    pred_pts = deque(maxlen=args["buffer"])

    if not args.get("video", False):
        vs = VideoStream(src=0).start()
    else:
        vs = cv2.VideoCapture(args["video"])
    time.sleep(2.0)

    # Build models — now with radius noise params
    cv_model = KalmanModel(
        model_type="CV", fps=args["fps"],
        q_pos=2.0, q_vel=10.0, r_meas=10.0,
        q_radius=0.5, q_radius_vel=1.0, r_radius=4.0,
    )
    ca_model = KalmanModel(
        model_type="CA", fps=args["fps"],
        px_per_meter=args["px_per_meter"],
        q_pos=1.0, q_vel=5.0, q_accel=2.0, r_meas=10.0,
        q_radius=0.5, q_radius_vel=1.0, r_radius=4.0,
    )

    stay = args["stay_prob"]
    switch = 1.0 - stay
    tpm = [[stay, switch], [switch, stay]]

    imm = IMMEstimator(
        models=[cv_model, ca_model], tpm=tpm,
        initial_probs=[0.7, 0.3], frame_w=600, frame_h=450,
    )

    detector = AdaptiveBallDetector(
        color_lower=color_lower, color_upper=color_upper,
        min_radius_abs=args["min_radius"],
        clahe_clip=3.0, clahe_grid=8,
        adapt_rate=0.02,
    )

    depth_est = DepthEstimator(
        ball_diameter_mm=args["ball_diameter"],
        focal_length_px=args["focal_length"],
        frame_width=600,
    )

    bench = BenchmarkStats(lookahead=args["lookahead"])
    frame_idx = 0
    calibrate_dist = args.get("calibrate_dist")

    print("\n  Controls: 'q' = quit, 'c' = calibrate depth (hold ball at known distance)")
    if calibrate_dist:
        print(f"  Calibration distance set to {calibrate_dist:.0f} mm — press 'c' when ready\n")

    while True:
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        if frame is None:
            break

        frame = imutils.resize(frame, width=600)

        # ── Detection ──
        t0 = time.perf_counter()

        kalman_hint = imm.pos if imm.is_tracking else None
        center, radius, mask = detector.detect(
            frame, kalman_pos=kalman_hint, kalman_tracking=imm.is_tracking)

        t_raw = time.perf_counter()
        dt_raw = t_raw - t0

        # ── IMM update (now passes radius) ──
        kalman_pos = None
        kalman_future = None
        raw_radius_for_bench = radius if center is not None else None

        if center is not None:
            if radius > detector.min_radius_abs:
                cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            imm.predict()
            imm.correct(center[0], center[1], radius)
            kalman_pos = imm.pos
        else:
            pred = imm.predict()
            imm.coast()
            if imm.is_tracking and pred is not None:
                kalman_pos = pred

        if imm.is_tracking:
            kalman_future = imm.future_position(n_frames=args["lookahead"])

        # ── Depth from filtered radius ──
        depth_m = None
        filt_r = None
        if imm.is_tracking:
            filt_r = imm.filtered_radius
            depth_m = depth_est.estimate_m(filt_r)

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
            dt_raw=dt_raw, dt_kalman=dt_kalman,
            active_model=active_model,
            raw_radius=raw_radius_for_bench,
            filtered_radius=filt_r,
            depth_m=depth_m,
        )

        # ── Draw IMM-filtered position + future ──
        if imm.is_tracking and kalman_pos is not None:
            px, py = int(kalman_pos[0]), int(kalman_pos[1])
            # Draw filtered radius circle (blue) — visually shows smoothing
            cv2.circle(frame, (px, py), int(imm.filtered_radius), (255, 0, 0), 2)

            if kalman_future is not None:
                fx, fy = int(kalman_future[0]), int(kalman_future[1])
                future_r = int(kalman_future[2]) if len(kalman_future) >= 3 else 10
                cv2.circle(frame, (fx, fy), future_r, (255, 0, 255), 2)
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
        if imm.initialized:
            si = imm.state_info
            cv_pct, ca_pct = si["prob_cv"] * 100, si["prob_ca"] * 100

            # Model probability bar
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

            # Velocity + coast + radius info
            info_line1 = (f"vel=({si['vx']:.1f},{si['vy']:.1f})  "
                          f"coast={imm.frames_since_seen}  "
                          f"minR={detector.adaptive_min_radius:.1f}")
            cv2.putText(frame, info_line1, (10, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

            # Depth + radius line
            r_filt = si["filtered_r"]
            dr = si["dr"]
            direction = "CLOSER" if dr > 0.3 else ("FARTHER" if dr < -0.3 else "STEADY")
            depth_str = f"{depth_m:.2f}m" if depth_m is not None else "---"
            cal_str = "CAL" if depth_est.calibrated else "est"
            info_line2 = (f"r={r_filt:.1f}px  dr={dr:+.2f}  "
                          f"depth={depth_str} ({cal_str})  [{direction}]")
            cv2.putText(frame, info_line2, (10, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

            # Depth bar visualization (right side)
            if depth_m is not None:
                db_x = 560
                db_h = 200
                db_y = 80
                db_w = 25
                # Map 0.3m–5m to full bar
                d_min, d_max = 0.3, 5.0
                d_clamped = max(d_min, min(d_max, depth_m))
                fill_frac = (d_clamped - d_min) / (d_max - d_min)
                fill_h = int(db_h * fill_frac)
                # Closer = green, farther = red
                r_comp = int(255 * fill_frac)
                g_comp = int(255 * (1 - fill_frac))
                cv2.rectangle(frame, (db_x, db_y),
                              (db_x + db_w, db_y + db_h), (80, 80, 80), -1)
                cv2.rectangle(frame, (db_x, db_y + db_h - fill_h),
                              (db_x + db_w, db_y + db_h), (0, g_comp, r_comp), -1)
                cv2.rectangle(frame, (db_x, db_y),
                              (db_x + db_w, db_y + db_h), (255, 255, 255), 1)
                cv2.putText(frame, f"{depth_m:.1f}m", (db_x - 8, db_y + db_h + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(frame, "DEPTH", (db_x - 5, db_y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow("Ball Tracker — IMM + 3D Depth", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("c"):
            # One-shot calibration
            if imm.is_tracking and calibrate_dist is not None:
                depth_est.calibrate(calibrate_dist, imm.filtered_radius)
            elif calibrate_dist is None:
                print("  [!] Set --calibrate-dist <mm> to enable calibration")

        frame_idx += 1

    bench.report()

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()