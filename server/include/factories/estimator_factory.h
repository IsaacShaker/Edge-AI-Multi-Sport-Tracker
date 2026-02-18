#pragma once

#include "../interfaces/i_state_estimator.h"
#include <string>
#include <map>
#include <functional>

namespace tracker {

/**
 * @brief Configuration for state estimators
 */
struct EstimatorConfig : public Config {
    std::string estimator_type;      // "kalman_cv", "kalman_ca", "imm", etc.
    
    // Common parameters
    float fps;
    float process_noise_pos;
    float process_noise_vel;
    float process_noise_accel;
    float measurement_noise_pos;
    float measurement_noise_radius;
    
    // IMM specific
    std::vector<std::string> model_types;  // For IMM: ["cv", "ca"]
    std::vector<float> initial_probabilities;
    float transition_probability;           // Diagonal of transition matrix
    
    // Distance estimation
    float ball_diameter_mm;
    float focal_length_px;
    int frame_width;
    
    // Physics
    float gravity_m_s2;
    float px_per_meter;
    
    EstimatorConfig()
        : fps(30.0f),
          process_noise_pos(1.0f),
          process_noise_vel(5.0f),
          process_noise_accel(2.0f),
          measurement_noise_pos(10.0f),
          measurement_noise_radius(4.0f),
          transition_probability(0.95f),
          ball_diameter_mm(65.0f),
          focal_length_px(0.0f),
          frame_width(640),
          gravity_m_s2(9.81f),
          px_per_meter(500.0f) {}
};

/**
 * @brief Factory for creating state estimator instances
 * 
 * Uses factory pattern to create different estimators.
 * Supports registration of new estimator types at runtime.
 */
class EstimatorFactory {
public:
    /**
     * @brief Create a state estimator by type
     * @param type Estimator type ("kalman_cv", "kalman_ca", "imm", etc.)
     * @param config Configuration for the estimator
     * @return Shared pointer to the estimator, or nullptr if type unknown
     */
    static StateEstimatorPtr create(
        const std::string& type,
        const EstimatorConfig& config
    );
    
    /**
     * @brief Register a custom estimator creator function
     * @param type Type identifier
     * @param creator Function that creates and returns an estimator
     */
    static void registerCreator(
        const std::string& type,
        std::function<StateEstimatorPtr(const EstimatorConfig&)> creator
    );
    
    /**
     * @brief Get list of available estimator types
     * @return Vector of registered type names
     */
    static std::vector<std::string> getAvailableTypes();

private:
    static std::map<std::string,
                    std::function<StateEstimatorPtr(const EstimatorConfig&)>> creators_;
    static void registerBuiltinCreators();
};

} // namespace tracker
