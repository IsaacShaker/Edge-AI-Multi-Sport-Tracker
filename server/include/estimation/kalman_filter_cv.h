#pragma once

#include "../interfaces/i_state_estimator.h"
#include "../factories/estimator_factory.h"
#include <opencv2/opencv.hpp>

namespace tracker {

/**
 * @brief Kalman Filter with Constant Velocity model
 * 
 * State: [x, y, vx, vy, r, dr] (6 states)
 * Measurements: [x, y, r] (3 measurements)
 * 
 * Assumes constant velocity motion (good for linear trajectories)
 */
class KalmanFilterCV : public IStateEstimator {
public:
    KalmanFilterCV() = default;
    ~KalmanFilterCV() override = default;
    
    bool initialize(const Config& config) override;
    void initializeState(const Detection& detection) override;
    EstimatedState predict(float dt) override;
    EstimatedState update(const Detection& detection) override;
    EstimatedState getState() const override;
    bool isInitialized() const override { return initialized_; }
    void reset() override;
    std::string getType() const override { return "kalman_cv"; }

private:
    EstimatorConfig config_;
    bool initialized_;
    EstimatedState current_state_;
    
    cv::KalmanFilter kf_;
    
    /**
     * @brief Convert Kalman state to EstimatedState
     */
    EstimatedState kalmanToEstimatedState() const;
    
    /**
     * @brief Estimate distance from filtered radius
     */
    float estimateDistance(float filtered_radius) const;
};

} // namespace tracker
