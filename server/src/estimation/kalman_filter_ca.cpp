// Full implementation of Kalman Filter with Constant Acceleration model
#include "../include/estimation/kalman_filter_ca.h"
#include <cmath>

namespace tracker {

bool KalmanFilterCA::initialize(const Config& config) {
    config_ = static_cast<const EstimatorConfig&>(config);
    initialized_ = false;
    
#ifdef ENABLE_OPENCV
    // Initialize 8-state Kalman filter: [x, y, vx, vy, ax, ay, r, dr]
    kf_ = cv::KalmanFilter(8, 3, 0, CV_32F);
    
    // Transition matrix (constant acceleration model)
    float dt = 1.0f / config_.fps;
    float dt2 = 0.5f * dt * dt;
    
    kf_.transitionMatrix = (cv::Mat_<float>(8, 8) <<
        1, 0, dt, 0,  dt2, 0,   0,  0,
        0, 1, 0,  dt, 0,   dt2, 0,  0,
        0, 0, 1,  0,  dt,  0,   0,  0,
        0, 0, 0,  1,  0,   dt,  0,  0,
        0, 0, 0,  0,  1,   0,   0,  0,
        0, 0, 0,  0,  0,   1,   0,  0,
        0, 0, 0,  0,  0,   0,   1, dt,
        0, 0, 0,  0,  0,   0,   0,  1
    );
    
    // Measurement matrix: we measure [x, y, r]
    kf_.measurementMatrix = cv::Mat::zeros(3, 8, CV_32F);
    kf_.measurementMatrix.at<float>(0, 0) = 1.0f;  // x
    kf_.measurementMatrix.at<float>(1, 1) = 1.0f;  // y
    kf_.measurementMatrix.at<float>(2, 6) = 1.0f;  // r
    
    // Process noise covariance
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1.0));
    kf_.processNoiseCov.at<float>(0, 0) = config_.process_noise_pos;
    kf_.processNoiseCov.at<float>(1, 1) = config_.process_noise_pos;
    kf_.processNoiseCov.at<float>(2, 2) = config_.process_noise_vel;
    kf_.processNoiseCov.at<float>(3, 3) = config_.process_noise_vel;
    kf_.processNoiseCov.at<float>(4, 4) = config_.process_noise_accel;
    kf_.processNoiseCov.at<float>(5, 5) = config_.process_noise_accel;
    kf_.processNoiseCov.at<float>(6, 6) = 0.5f;  // radius process noise
    kf_.processNoiseCov.at<float>(7, 7) = 1.0f;  // radius velocity process noise
    
    // Measurement noise covariance
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1.0));
    kf_.measurementNoiseCov.at<float>(0, 0) = config_.measurement_noise_pos;
    kf_.measurementNoiseCov.at<float>(1, 1) = config_.measurement_noise_pos;
    kf_.measurementNoiseCov.at<float>(2, 2) = config_.measurement_noise_radius;
    
    // Error covariance
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(100.0));
    
    // Calculate gravity in pixels per frame^2
    gravity_px_ = config_.gravity_m_s2 * config_.px_per_meter / (config_.fps * config_.fps);
    
    return true;
#else
    return false;
#endif
}

void KalmanFilterCA::initializeState(const Detection& detection) {
#ifdef ENABLE_OPENCV
    kf_.statePost.at<float>(0) = detection.center.x;
    kf_.statePost.at<float>(1) = detection.center.y;
    kf_.statePost.at<float>(2) = 0;  // vx
    kf_.statePost.at<float>(3) = 0;  // vy
    kf_.statePost.at<float>(4) = 0;  // ax
    kf_.statePost.at<float>(5) = gravity_px_;  // ay (gravity)
    kf_.statePost.at<float>(6) = detection.radius;
    kf_.statePost.at<float>(7) = 0;  // dr
    
    initialized_ = true;
    current_state_ = kalmanToEstimatedState();
#endif
}

EstimatedState KalmanFilterCA::predict(float dt) {
#ifdef ENABLE_OPENCV
    if (!initialized_) {
        return EstimatedState();
    }
    
    kf_.predict();
    current_state_ = kalmanToEstimatedState();
    return current_state_;
#else
    return EstimatedState();
#endif
}

EstimatedState KalmanFilterCA::update(const Detection& detection) {
#ifdef ENABLE_OPENCV
    if (!initialized_) {
        initializeState(detection);
        return current_state_;
    }
    
    cv::Mat measurement = (cv::Mat_<float>(3, 1) << 
        detection.center.x,
        detection.center.y,
        detection.radius
    );
    
    kf_.correct(measurement);
    current_state_ = kalmanToEstimatedState();
    return current_state_;
#else
    return EstimatedState();
#endif
}

EstimatedState KalmanFilterCA::getState() const {
    return current_state_;
}

void KalmanFilterCA::reset() {
    initialized_ = false;
    current_state_ = EstimatedState();
}

#ifdef ENABLE_OPENCV
EstimatedState KalmanFilterCA::kalmanToEstimatedState() const {
    EstimatedState state;
    state.position.x = kf_.statePost.at<float>(0);
    state.position.y = kf_.statePost.at<float>(1);
    state.velocity.vx = kf_.statePost.at<float>(2);
    state.velocity.vy = kf_.statePost.at<float>(3);
    state.acceleration.ax = kf_.statePost.at<float>(4);
    state.acceleration.ay = kf_.statePost.at<float>(5);
    
    float filtered_radius = kf_.statePost.at<float>(6);
    state.position.z = estimateDistance(filtered_radius);
    
    state.confidence = 1.0f;  // TODO: Compute from covariance
    state.timestamp_ms = 0;   // TODO: Add proper timestamp
    
    return state;
}

float KalmanFilterCA::estimateDistance(float filtered_radius) const {
    if (filtered_radius < 1.0f) {
        return 0.0f;
    }
    
    float focal_length = config_.focal_length_px > 0 
        ? config_.focal_length_px 
        : config_.frame_width * 1.2f;
    
    float K = config_.ball_diameter_mm * focal_length;
    float d_image = 2.0f * filtered_radius;
    
    return K / d_image;  // Distance in mm
}
#endif

} // namespace tracker
