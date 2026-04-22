#pragma once

#include "../interfaces/i_motor_controller.h"
#include "../factories/motor_factory.h"

namespace tracker {

/**
 * @brief Mock motor controller for testing without hardware
 * 
 * Simulates motor movement and provides realistic feedback
 * without requiring actual hardware connection.
 */
class MockController : public IMotorController {
public:
    MockController() = default;
    ~MockController() override = default;
    
    bool initialize(const Config& config) override;
    bool connect() override;
    void disconnect() override;
    bool enableMotors() override;
    bool disableMotors() override;
    bool setTargetAngles(const GimbalAngles& angles) override;
    MotorStatus getStatus() const override;
    bool stop() override;
    bool home() override;
    bool isReady() const override { return connected_; }
    std::string getType() const override { return "mock"; }

    void  pollPositionLoop() override;
    float getPanRad()  const override;
    float getTiltRad() const override;

private:
    MotorConfig config_;
    bool connected_;
    MotorStatus status_;
    
    /**
     * @brief Simulate gradual movement toward target
     */
    void simulateMovement();
};

} // namespace tracker
