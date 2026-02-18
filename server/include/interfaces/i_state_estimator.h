#pragma once

#include "types.h"
#include <memory>

namespace tracker {

/**
 * @brief Abstract interface for state estimation systems
 * 
 * All estimators (Kalman, IMM, EKF, etc.) must implement this interface.
 * This allows swapping between different estimation algorithms.
 */
class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;
    
    /**
     * @brief Initialize the estimator with configuration
     * @param config Configuration parameters (process noise, measurement noise, etc.)
     * @return true if initialization successful
     */
    virtual bool initialize(const Config& config) = 0;
    
    /**
     * @brief Initialize state with first measurement
     * @param detection Initial detection
     */
    virtual void initializeState(const Detection& detection) = 0;
    
    /**
     * @brief Predict the next state (time update)
     * @param dt Time step in seconds
     * @return Predicted state
     */
    virtual EstimatedState predict(float dt) = 0;
    
    /**
     * @brief Update state with new measurement (measurement update)
     * @param detection New detection/measurement
     * @return Corrected/filtered state
     */
    virtual EstimatedState update(const Detection& detection) = 0;
    
    /**
     * @brief Get current estimated state
     * @return Current state estimate
     */
    virtual EstimatedState getState() const = 0;
    
    /**
     * @brief Check if estimator has been initialized with a measurement
     * @return true if initialized
     */
    virtual bool isInitialized() const = 0;
    
    /**
     * @brief Reset the estimator
     */
    virtual void reset() = 0;
    
    /**
     * @brief Get the estimator type/name
     * @return Estimator identifier string
     */
    virtual std::string getType() const = 0;
};

using StateEstimatorPtr = std::shared_ptr<IStateEstimator>;

} // namespace tracker
