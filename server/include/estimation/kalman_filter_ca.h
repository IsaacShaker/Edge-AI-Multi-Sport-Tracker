#pragma once

#include "../interfaces/i_state_estimator.h"
#include "../factories/estimator_factory.h"
#include <opencv2/opencv.hpp>

namespace tracker {

/**
 * @brief Kalman Filter with Constant Acceleration model
 * 
 * State: [x, y, vx, vy, ax, ay, r, dr] (8 states)
 * Measurements: [x, y, r] (3 measurements)
 * 
 * Assumes constant acceleration (good for ballistic trajectories with gravity)
 */
class KalmanFilterCA : public IStateEstimator {
public:
    KalmanFilterCA() = default;
    ~KalmanFilterCA() override = default;
    
    bool initialize(const Config& config) override;
    void initializeState(const Detection& detection) override;
    EstimatedState predict(float dt) override;
    EstimatedState update(const Detection& detection) override;
    EstimatedState getState() const override;
    bool isInitialized() const override { return initialized_; }
    void reset() override;
    std::string getType() const override { return "kalman_ca"; }

private:
    EstimatorConfig config_;
    bool initialized_;
    EstimatedState current_state_;
    float gravity_px_;  // Gravity in pixels/frame^2
    
    cv::KalmanFilter kf_;
    
    EstimatedState kalmanToEstimatedState() const;
    float estimateDistance(float filtered_radius) const;
};

} // namespace tracker
