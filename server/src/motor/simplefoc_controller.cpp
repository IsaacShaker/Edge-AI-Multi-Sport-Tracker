// simplefoc_controller.cpp
#include "../include/motor/simplefoc_controller.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <regex>

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
    status_.target_angles  = GimbalAngles(0.0f, 0.0f);
    status_.is_connected   = false;
    status_.is_moving      = false;

    actual_pan_rad_  = 0.0f;
    actual_tilt_rad_ = 0.0f;

    std::cout << "[SimpleFOCController] Initialized" << std::endl;
    return true;
}

bool SimpleFOCController::connect() {
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (connected_) return true;

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

    // Wait for firmware READY sentinel (up to 8 s)
    std::cout << "[SimpleFOCController] Waiting for firmware READY..." << std::endl;
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        char buf[256] = {};
        int  pos   = 0;
        bool ready = false;

        struct termios tty;
        tcgetattr(serial_fd_, &tty);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 2;   // 0.2 s read timeout
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

        if (ready)
            std::cout << "[SimpleFOCController] Firmware ready." << std::endl;
        else
            std::cerr << "[SimpleFOCController] Timeout waiting for READY — "
                         "firmware may still be initializing." << std::endl;
    }

    if (!enableMotors())
        std::cerr << "[SimpleFOCController] Warning: failed to enable motors" << std::endl;

    float pan = 0.0f;
    float tilt = 0.0f;
    queryPositionOnce(pan, tilt);
    std::cout << "[SimpleFOCController] Initial position: pan=" << pan 
              << " rad, tilt=" << tilt << " rad" << std::endl;

    GimbalAngles initial_angles(5.5f, 0.0f);
    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        status_.current_angles = initial_angles;
    }
    setTargetAngles(initial_angles);

    // ── Start background position-polling thread ──────────────────────────────
    // Sends "Y" at ~20 Hz and parses the response so getPanRad()/getTiltRad()
    // always return fresh values without blocking the control loop.
    poll_running_ = true;
    poll_thread_  = std::thread(&SimpleFOCController::pollPositionLoop, this);

    return true;
}

void SimpleFOCController::disconnect() {
    if (!connected_) return;

    // Stop polling thread first so it doesn't race on serial_fd_
    poll_running_ = false;
    if (poll_thread_.joinable())
        poll_thread_.join();

    disableMotors();

    std::lock_guard<std::mutex> lock(serial_mutex_);
    closeSerial();
    connected_           = false;
    status_.is_connected = false;

    std::cout << "[SimpleFOCController] Disconnected" << std::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Background position-polling loop  (~20 Hz)
//
// Firmware response to "Y":
//   "Top Position (Rads): 1.234 | Bottom Position (Rads): 5.678"
//   top    → tilt axis
//   bottom → pan axis
// ─────────────────────────────────────────────────────────────────────────────
void SimpleFOCController::pollPositionLoop() {
    while (poll_running_) {
        float pan = 0.0f;
        float tilt = 0.0f;

        if (queryPositionOnce(pan, tilt)) {
            {
                std::lock_guard<std::mutex> lk(pos_mutex_);
                actual_pan_rad_ = pan;
                actual_tilt_rad_ = tilt;
            }
            {
                std::lock_guard<std::mutex> slk(status_mutex_);
                status_.current_angles = GimbalAngles(pan, tilt);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public accessors — return latest polled values, never block
// ─────────────────────────────────────────────────────────────────────────────
float SimpleFOCController::getPanRad() const {
    std::lock_guard<std::mutex> lk(pos_mutex_);
    return actual_pan_rad_;
}

float SimpleFOCController::getTiltRad() const {
    std::lock_guard<std::mutex> lk(pos_mutex_);
    return actual_tilt_rad_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Remainder of implementation (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
bool SimpleFOCController::enableMotors() {
    if (!connected_) { status_.error_message = "Not connected"; return false; }
    std::cout << "[SimpleFOCController] Enabling motors..." << std::endl;
    return sendCommand("K1");
}

bool SimpleFOCController::disableMotors() {
    if (!connected_) { status_.error_message = "Not connected"; return false; }
    return sendCommand("K0");
}

bool SimpleFOCController::setTargetAngles(const GimbalAngles& angles) {
    if (!connected_) { status_.error_message = "Not connected"; return false; }

    status_.target_angles = GimbalAngles(angles.pan, angles.tilt);

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "M%.4f %.4f", angles.pan, angles.tilt);
    if (!sendCommand(cmd)) return false;

    // No longer doing optimistic position update —
    // pollPositionLoop() keeps actual_pan/tilt_rad_ current with real values.
    status_.is_moving = true;
    return true;
}

MotorStatus SimpleFOCController::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

bool SimpleFOCController::stop() {
    if (!connected_) return false;
    GimbalAngles cur;
    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        cur = status_.current_angles;
    }
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "M%.4f %.4f", cur.pan, cur.tilt);
    return sendCommand(cmd);
}

bool SimpleFOCController::home() {
    if (!connected_) return false;
    status_.target_angles = GimbalAngles(0.0f, 0.0f);
    return setTargetAngles(status_.target_angles);
}

bool SimpleFOCController::isReady() const {
    return connected_;
}

// bool SimpleFOCController::openSerial() {
// #ifdef __linux__
//     serial_fd_ = open(config_.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
//     if (serial_fd_ < 0) return false;

//     // Drop DTR to prevent Teensy/Arduino reset on port open
//     int flags = 0;
//     ioctl(serial_fd_, TIOCMGET, &flags);
//     flags &= ~TIOCM_DTR;
//     ioctl(serial_fd_, TIOCMSET, &flags);

//     struct termios options;
//     tcgetattr(serial_fd_, &options);

//     cfsetispeed(&options, B115200);
//     cfsetospeed(&options, B115200);

//     options.c_cflag &= ~PARENB;
//     options.c_cflag &= ~CSTOPB;
//     options.c_cflag &= ~CSIZE;
//     options.c_cflag |= CS8;
//     options.c_cflag &= ~CRTSCTS;
//     options.c_cflag |= CREAD | CLOCAL;
//     options.c_iflag &= ~(IXON | IXOFF | IXANY);
//     options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//     options.c_oflag &= ~OPOST;

//     tcsetattr(serial_fd_, TCSANOW, &options);
//     tcflush(serial_fd_, TCIOFLUSH);

//     return true;
// #else
//     std::cerr << "[SimpleFOCController] Serial not implemented for this platform" << std::endl;
//     return false;
// #endif
// }

bool SimpleFOCController::openSerial() {
#ifdef __linux__
    serial_fd_ = open(config_.serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
        std::perror("[SimpleFOCController] open");
        return false;
    }

    struct termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
        std::perror("[SimpleFOCController] tcgetattr");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    cfmakeraw(&options);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= (CLOCAL | CREAD);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;   // 1.0 second

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        std::perror("[SimpleFOCController] tcsetattr");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    tcflush(serial_fd_, TCIFLUSH);
    return true;
#else
    std::cerr << "[SimpleFOCController] Serial not implemented for this platform" << std::endl;
    return false;
#endif
}

bool SimpleFOCController::queryPositionOnce(float& pan, float& tilt) {
    std::lock_guard<std::mutex> lk(serial_mutex_);
    if (serial_fd_ < 0) return false;

    tcflush(serial_fd_, TCIFLUSH);

    ssize_t wn = ::write(serial_fd_, "Y\n", 2);
    if (wn != 2) {
        std::cerr << "[SimpleFOCController] Failed to write Y\n";
        return false;
    }

    tcdrain(serial_fd_);

    std::string line;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);

    while (std::chrono::steady_clock::now() < deadline) {
        char buf[64];
        ssize_t rn = ::read(serial_fd_, buf, sizeof(buf));

        if (rn > 0) {
            line.append(buf, buf + rn);

            // stop once we have a full line
            auto newline_pos = line.find('\n');
            if (newline_pos != std::string::npos) {
                line.resize(newline_pos);

                // trim trailing carriage return
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                // std::cout << "[SimpleFOCController] raw RX: " << line << std::endl;

                float tilt_rx = 0.0f;
                float pan_rx = 0.0f;

                if (sscanf(line.c_str(), "POS,%f,%f", &tilt_rx, &pan_rx) == 2) {
                    tilt = tilt_rx;
                    pan = pan_rx;
                    return true;
                } else {
                    std::cerr << "[SimpleFOCController] Failed to parse: " << line << std::endl;
                    return false;
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cerr << "[SimpleFOCController] No complete response to Y. Partial buffer: " << line << std::endl;
    return false;
}

void SimpleFOCController::closeSerial() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool SimpleFOCController::sendCommand(const std::string& command) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ < 0) return false;
    std::string cmd = command + "\n";
    ssize_t written = write(serial_fd_, cmd.c_str(), cmd.length());
    return written == static_cast<ssize_t>(cmd.length());
}

std::string SimpleFOCController::readResponse() {
    // Synchronous reads handled inside pollPositionLoop.
    return "";
}

float SimpleFOCController::clampAngle(float angle, float min_angle, float max_angle) const {
    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}

} // namespace tracker