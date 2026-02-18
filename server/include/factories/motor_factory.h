#pragma once

#include "../interfaces/i_motor_controller.h"
#include <string>
#include <map>
#include <functional>

namespace tracker {

/**
 * @brief Configuration for motor controllers
 */
struct MotorConfig : public Config {
    std::string controller_type;     // "simplefoc", "servo", "mock", etc.
    
    // Serial communication
    std::string serial_port;         // "/dev/ttyUSB0", "COM6", etc.
    int baudrate;
    
    // Motor limits
    float pan_min_rad;
    float pan_max_rad;
    float tilt_min_rad;
    float tilt_max_rad;
    
    // PID gains (if applicable)
    float kp_pan, ki_pan, kd_pan;
    float kp_tilt, ki_tilt, kd_tilt;
    
    // Slew rate limits (rad/s)
    float max_pan_velocity;
    float max_tilt_velocity;
    
    MotorConfig()
        : baudrate(115200),
          pan_min_rad(-3.14f),
          pan_max_rad(3.14f),
          tilt_min_rad(-1.57f),
          tilt_max_rad(1.57f),
          kp_pan(1.0f), ki_pan(0.0f), kd_pan(0.0f),
          kp_tilt(1.0f), ki_tilt(0.0f), kd_tilt(0.0f),
          max_pan_velocity(2.0f),
          max_tilt_velocity(2.0f) {}
};

/**
 * @brief Factory for creating motor controller instances
 * 
 * Uses factory pattern to create different motor controllers.
 * Supports registration of new controller types at runtime.
 */
class MotorFactory {
public:
    /**
     * @brief Create a motor controller by type
     * @param type Controller type ("simplefoc", "servo", "mock", etc.)
     * @param config Configuration for the controller
     * @return Shared pointer to the controller, or nullptr if type unknown
     */
    static MotorControllerPtr create(
        const std::string& type,
        const MotorConfig& config
    );
    
    /**
     * @brief Register a custom controller creator function
     * @param type Type identifier
     * @param creator Function that creates and returns a controller
     */
    static void registerCreator(
        const std::string& type,
        std::function<MotorControllerPtr(const MotorConfig&)> creator
    );
    
    /**
     * @brief Get list of available controller types
     * @return Vector of registered type names
     */
    static std::vector<std::string> getAvailableTypes();

private:
    static std::map<std::string,
                    std::function<MotorControllerPtr(const MotorConfig&)>> creators_;
    static void registerBuiltinCreators();
};

} // namespace tracker
