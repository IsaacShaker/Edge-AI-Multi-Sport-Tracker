#include "../include/motor/mock_controller.h"
#include <iostream>
#include <cmath>

namespace tracker {

bool MockController::initialize(const Config& config) {
    config_ = static_cast<const MotorConfig&>(config);
    connected_ = false;
    
    status_.current_angles = GimbalAngles(0.0f, 0.0f);
    status_.target_angles = GimbalAngles(0.0f, 0.0f);
    status_.is_connected = false;
    status_.is_moving = false;
    
    std::cout << "[MockController] Initialized" << std::endl;
    return true;
}

bool MockController::connect() {
    std::cout << "[MockController] Connecting..." << std::endl;
    connected_ = true;
    status_.is_connected = true;
    std::cout << "[MockController] Connected (simulated)" << std::endl;
    return true;
}

void MockController::disconnect() {
    std::cout << "[MockController] Disconnecting..." << std::endl;
    connected_ = false;
    status_.is_connected = false;
}

bool MockController::setTargetAngles(const GimbalAngles& angles) {
    if (!connected_) {
        status_.error_message = "Not connected";
        return false;
    }
    
    // Clamp angles to valid range
    float pan = std::max(config_.pan_min_rad, 
                        std::min(config_.pan_max_rad, angles.pan));
    float tilt = std::max(config_.tilt_min_rad,
                         std::min(config_.tilt_max_rad, angles.tilt));
    
    status_.target_angles = GimbalAngles(pan, tilt);
    
    // Simulate immediate movement for mock
    simulateMovement();
    
    std::cout << "[MockController] Target set: pan=" << pan 
              << " rad, tilt=" << tilt << " rad" << std::endl;
    
    return true;
}

MotorStatus MockController::getStatus() const {
    return status_;
}

bool MockController::stop() {
    if (!connected_) {
        return false;
    }
    
    // Set target to current position
    status_.target_angles = status_.current_angles;
    status_.is_moving = false;
    
    std::cout << "[MockController] Stopped" << std::endl;
    return true;
}

bool MockController::home() {
    if (!connected_) {
        return false;
    }
    
    status_.target_angles = GimbalAngles(0.0f, 0.0f);
    status_.current_angles = GimbalAngles(0.0f, 0.0f);
    status_.is_moving = false;
    
    std::cout << "[MockController] Homed to (0, 0)" << std::endl;
    return true;
}

void MockController::simulateMovement() {
    // Simple simulation: snap to target with some smoothing
    const float alpha = 0.3f;  // Smoothing factor
    
    float pan_diff = status_.target_angles.pan - status_.current_angles.pan;
    float tilt_diff = status_.target_angles.tilt - status_.current_angles.tilt;
    
    status_.current_angles.pan += alpha * pan_diff;
    status_.current_angles.tilt += alpha * tilt_diff;
    
    // Check if still moving
    const float threshold = 0.01f;  // 0.01 radians
    status_.is_moving = (std::abs(pan_diff) > threshold || 
                        std::abs(tilt_diff) > threshold);
}

} // namespace tracker
