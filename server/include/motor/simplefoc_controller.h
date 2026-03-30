#pragma once

#include "../interfaces/i_motor_controller.h"
#include "../factories/motor_factory.h"
#include <string>
#include <mutex>
#include <atomic>

namespace tracker {

/**
 * @brief SimpleFOC motor controller via serial communication
 * 
 * Communicates with Arduino/SimpleFOC gimbal system.
 * Sends angle commands and receives status updates.
 */
class SimpleFOCController : public IMotorController {
public:
    SimpleFOCController() = default;
    ~SimpleFOCController() override;
    
    bool initialize(const Config& config) override;
    bool connect() override;
    void disconnect() override;
    bool enableMotors() override;
    bool disableMotors() override;
    bool setTargetAngles(const GimbalAngles& angles) override;
    MotorStatus getStatus() const override;
    bool stop() override;
    bool home() override;
    bool isReady() const override;
    std::string getType() const override { return "simplefoc"; }

private:
    MotorConfig config_;
    
    // Serial communication
    int serial_fd_;  // File descriptor (Unix) or handle (Windows)
    std::mutex serial_mutex_;
    std::atomic<bool> connected_;
    
    MotorStatus status_;
    mutable std::mutex status_mutex_;
    
    /**
     * @brief Open serial port
     */
    bool openSerial();
    
    /**
     * @brief Close serial port
     */
    void closeSerial();
    
    /**
     * @brief Send a command to the embedded system
     * @param command Command string (e.g., "B3.14")
     * @return true if sent successfully
     */
    bool sendCommand(const std::string& command);
    
    /**
     * @brief Read response from embedded system (non-blocking)
     */
    std::string readResponse();
    
    /**
     * @brief Clamp angle to valid range
     */
    float clampAngle(float angle, float min_angle, float max_angle) const;
};

} // namespace tracker
