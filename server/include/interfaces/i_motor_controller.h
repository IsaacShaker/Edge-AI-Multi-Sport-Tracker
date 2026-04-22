#pragma once

#include "types.h"
#include <memory>
#include <string>

namespace tracker {

/**
 * @brief Motor status information
 */
struct MotorStatus {
    GimbalAngles current_angles;
    GimbalAngles target_angles;
    bool is_connected;
    bool is_moving;
    std::string error_message;
    
    MotorStatus() : is_connected(false), is_moving(false) {}
};

/**
 * @brief Abstract interface for motor/gimbal control
 * 
 * All motor controllers must implement this interface.
 * This allows swapping between different hardware (SimpleFOC, servo, mock, etc.)
 */
class IMotorController {
public:
    virtual ~IMotorController() = default;
    
    /**
     * @brief Initialize the motor controller
     * @param config Configuration (serial port, baudrate, PID gains, etc.)
     * @return true if initialization successful
     */
    virtual bool initialize(const Config& config) = 0;
    
    /**
     * @brief Connect to the motor controller/embedded system
     * @return true if connection successful
     */
    virtual bool connect() = 0;
    
    /**
     * @brief Disconnect from the motor controller
     */
    virtual void disconnect() = 0;
    
    /**
     * @brief Enable motors from the motor controller
     * @return true if motors enabled successfully
     */
    virtual bool enableMotors() = 0;

    /**
     * @brief Disable motors from the motor controller
     * @return true if motors disabled successfully
     */
    virtual bool disableMotors() = 0;

    virtual void  pollPositionLoop() = 0;
    virtual float getPanRad()  const = 0;
    virtual float getTiltRad() const = 0;

    /**
     * @brief Set target gimbal angles
     * @param angles Target pan/tilt angles in radians
     * @return true if command sent successfully
     */
    virtual bool setTargetAngles(const GimbalAngles& angles) = 0;
    
    /**
     * @brief Get current motor status
     * @return Current status information
     */
    virtual MotorStatus getStatus() const = 0;
    
    /**
     * @brief Stop all motors
     * @return true if stop command successful
     */
    virtual bool stop() = 0;
    
    /**
     * @brief Home/calibrate the gimbal
     * @return true if homing successful
     */
    virtual bool home() = 0;
    
    /**
     * @brief Check if controller is ready
     * @return true if ready to receive commands
     */
    virtual bool isReady() const = 0;
    
    /**
     * @brief Get the controller type/name
     * @return Controller identifier string
     */
    virtual std::string getType() const = 0;
};

using MotorControllerPtr = std::shared_ptr<IMotorController>;

} // namespace tracker
