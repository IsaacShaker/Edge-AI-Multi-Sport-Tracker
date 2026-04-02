// Stub implementation - to be completed
#include "../include/motor/simplefoc_controller.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <thread>
#include <chrono>

namespace tracker {

SimpleFOCController::~SimpleFOCController() {
    if (connected_) {
        disconnect();
    }
}

bool SimpleFOCController::initialize(const Config& config) {
    config_ = static_cast<const MotorConfig&>(config);
    serial_fd_ = -1;
    connected_ = false;
    
    status_.current_angles = GimbalAngles(0.0f, 0.0f);
    status_.target_angles = GimbalAngles(0.0f, 0.0f);
    status_.is_connected = false;
    status_.is_moving = false;
    
    std::cout << "[SimpleFOCController] Initialized" << std::endl;
    return true;
}

bool SimpleFOCController::connect() {
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (connected_) {
            return true;
        }

        std::cout << "[SimpleFOCController] Connecting to " << config_.serial_port
                  << " at " << config_.baudrate << " baud..." << std::endl;

        if (!openSerial()) {
            std::cerr << "[SimpleFOCController] Failed to open serial port" << std::endl;
            return false;
        }

        connected_ = true;
        status_.is_connected = true;
    } // serial_mutex_ released before enableMotors() to avoid deadlock

    std::cout << "[SimpleFOCController] Connected" << std::endl;

    // Wait for the firmware to finish setup() + initFOC() for both motors.
    // initFOC() moves each motor through a calibration sequence and can take
    // 2-4 seconds total. We read from the serial port and wait for the "READY"
    // line the firmware prints when setup() is fully complete.
    // Fall back to a 5-second hard timeout if the sentinel never arrives.
    std::cout << "[SimpleFOCController] Waiting for firmware READY..." << std::endl;
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        char buf[256] = {};
        int  pos = 0;
        bool ready = false;

        // Put fd into blocking reads with a short timeout so we can check
        // the deadline without spinning 100% CPU.
        struct termios tty;
        tcgetattr(serial_fd_, &tty);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 2;  // 0.2 s read timeout
        tcsetattr(serial_fd_, TCSANOW, &tty);

        while (std::chrono::steady_clock::now() < deadline) {
            char c;
            ssize_t n = ::read(serial_fd_, &c, 1);
            if (n <= 0) continue;
            if (c == '\n' || c == '\r') {
                buf[pos] = '\0';
                if (std::string(buf).find("READY") != std::string::npos) {
                    ready = true;
                    break;
                }
                pos = 0;
            } else if (pos < static_cast<int>(sizeof(buf)) - 1) {
                buf[pos++] = c;
            }
        }

        if (ready) {
            std::cout << "[SimpleFOCController] Firmware ready." << std::endl;
        } else {
            std::cerr << "[SimpleFOCController] Timeout waiting for READY — firmware may still be initializing." << std::endl;
        }
    }

    if (!enableMotors()) {
        std::cerr << "[SimpleFOCController] Warning: failed to enable motors" << std::endl;
    }

    return true;
}

void SimpleFOCController::disconnect() {
    if (!connected_) {
        return;
    }

    // Send K0 before closing — must happen while connected_ is still true
    // and before acquiring serial_mutex_ to avoid deadlock with sendCommand().
    disableMotors();

    std::lock_guard<std::mutex> lock(serial_mutex_);
    closeSerial();
    connected_ = false;
    status_.is_connected = false;

    std::cout << "[SimpleFOCController] Disconnected" << std::endl;
}

bool SimpleFOCController::enableMotors() {
    if (!connected_) {
        status_.error_message = "Not connected";
        return false;
    }
    std::cout << "[SimpleFOCController] Enabling motors..." << std::endl;
    return sendCommand("K1");
}

bool SimpleFOCController::disableMotors() {
    if (!connected_) {
        status_.error_message = "Not connected";
        return false;
    }

    return sendCommand("K0");
}

bool SimpleFOCController::setTargetAngles(const GimbalAngles& angles) {
    if (!connected_) {
        status_.error_message = "Not connected";
        return false;
    }
    
    // Clamp angles
    float pan = clampAngle(angles.pan, config_.pan_min_rad, config_.pan_max_rad);
    float tilt = clampAngle(angles.tilt, config_.tilt_min_rad, config_.tilt_max_rad);
    
    status_.target_angles = GimbalAngles(pan, tilt);
    
    // Send command: "B<pan> T<tilt>"
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "B%.3f 1.0", pan);
    snprintf(cmd, sizeof(cmd), "T%.3f 1.0", tilt);
    
    if (!sendCommand(cmd)) {
        return false;
    }
    
    status_.is_moving = true;
    return true;
}

MotorStatus SimpleFOCController::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

bool SimpleFOCController::stop() {
    if (!connected_) {
        return false;
    }
    
    // Set target to current position
    return sendCommand("S0 0 1");
    return sendCommand("H");
}

bool SimpleFOCController::home() {
    if (!connected_) {
        return false;
    }
    
    status_.target_angles = GimbalAngles(0.0f, 0.0f);
    return setTargetAngles(status_.target_angles);
}

bool SimpleFOCController::isReady() const {
    return connected_;
}

bool SimpleFOCController::openSerial() {
#ifdef __linux__
    serial_fd_ = open(config_.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        return false;
    }

    // Drop DTR immediately to prevent the Teensy/Arduino from resetting when
    // the host opens the port. Without this, the firmware resets and the K1
    // command sent right after connect() arrives before the firmware is ready.
    int flags = 0;
    ioctl(serial_fd_, TIOCMGET, &flags);
    flags &= ~TIOCM_DTR;
    ioctl(serial_fd_, TIOCMSET, &flags);

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // Set baud rate
    speed_t baud = B115200;  // TODO: Map config_.baudrate to actual baud constant
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // No flow control
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIOFLUSH);  // discard any buffered garbage

    return true;
#else
    // TODO: Windows implementation
    std::cerr << "[SimpleFOCController] Serial not implemented for this platform" << std::endl;
    return false;
#endif
}

void SimpleFOCController::closeSerial() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool SimpleFOCController::sendCommand(const std::string& command) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    if (serial_fd_ < 0) {
        return false;
    }
    
    std::string cmd = command + "\n";
    ssize_t written = write(serial_fd_, cmd.c_str(), cmd.length());
    
    return written == static_cast<ssize_t>(cmd.length());
}

std::string SimpleFOCController::readResponse() {
    // TODO: Implement non-blocking read
    return "";
}

float SimpleFOCController::clampAngle(float angle, float min_angle, float max_angle) const {
    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}

} // namespace tracker
