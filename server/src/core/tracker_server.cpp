#include "../include/core/tracker_server.h"
#include "../include/factories/vision_factory.h"
#include "../include/factories/estimator_factory.h"
#include "../include/factories/motor_factory.h"
#include <iostream>

#ifdef ENABLE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace tracker {

TrackerServer::TrackerServer() 
    : running_(false) {
    stats_.fps = 0.0f;
    stats_.frames_processed = 0;
    stats_.detections_count = 0;
    stats_.avg_detection_confidence = 0.0f;
}

TrackerServer::~TrackerServer() {
    if (running_) {
        stop();
    }
}

bool TrackerServer::initialize(const ServerConfig& config) {
    config_ = config;
    
    std::cout << "=== Initializing Tracker Server ===" << std::endl;
    
    // Create vision detector
    std::cout << "Creating vision detector: " << config.vision.model_type << std::endl;
    vision_ = VisionFactory::create(config.vision.model_type, config.vision);
    if (!vision_) {
        std::cerr << "Failed to create vision detector" << std::endl;
        return false;
    }
    
    // Create state estimator
    std::cout << "Creating state estimator: " << config.estimator.estimator_type << std::endl;
    estimator_ = EstimatorFactory::create(config.estimator.estimator_type, config.estimator);
    if (!estimator_) {
        std::cerr << "Failed to create state estimator" << std::endl;
        return false;
    }
    
    // Create motor controller
    std::cout << "Creating motor controller: " << config.motor.controller_type << std::endl;
    motor_ = MotorFactory::create(config.motor.controller_type, config.motor);
    if (!motor_) {
        std::cerr << "Failed to create motor controller" << std::endl;
        return false;
    }
    
    // Connect motor controller
    if (!motor_->connect()) {
        std::cerr << "Failed to connect motor controller" << std::endl;
        return false;
    }
    
    std::cout << "=== Tracker Server Initialized ===" << std::endl;
    return true;
}

bool TrackerServer::start() {
    if (running_) {
        std::cerr << "Server already running" << std::endl;
        return false;
    }
    
    std::cout << "Starting tracker server..." << std::endl;
    running_ = true;
    tracker_thread_ = std::thread(&TrackerServer::trackerLoop, this);
    
    return true;
}

void TrackerServer::stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "Stopping tracker server..." << std::endl;
    running_ = false;
    
    if (tracker_thread_.joinable()) {
        tracker_thread_.join();
    }
    
    if (motor_) {
        motor_->disconnect();
    }
    
    std::cout << "Tracker server stopped" << std::endl;
}

TrackerServer::Stats TrackerServer::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void TrackerServer::trackerLoop() {
#ifdef ENABLE_OPENCV
    // Open camera
    cv::VideoCapture cap(config_.camera_device_id);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera " << config_.camera_device_id << std::endl;
        running_ = false;
        return;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config_.vision.input_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config_.vision.input_height);
    cap.set(cv::CAP_PROP_FPS, config_.target_fps);
    
    cv::Mat frame;
    last_frame_time_ = std::chrono::steady_clock::now();
    
    std::cout << "Tracker loop started" << std::endl;
    
    while (running_) {
        // Capture frame
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            continue;
        }
        
        // Process frame
        processFrame(frame.data, frame.cols, frame.rows);
        
        // Update FPS
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration<float>(now - last_frame_time_).count();
        last_frame_time_ = now;
        
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.fps = 1.0f / dt;
            stats_.frames_processed++;
        }
        
        // Optional visualization
        if (config_.enable_visualization) {
            cv::imshow("Tracker", frame);
            if (cv::waitKey(1) == 'q') {
                running_ = false;
            }
        }
    }
    
    cap.release();
    if (config_.enable_visualization) {
        cv::destroyAllWindows();
    }
#else
    std::cerr << "OpenCV not enabled, cannot run tracker loop" << std::endl;
    running_ = false;
#endif
}

void TrackerServer::processFrame(const void* frame_data, int width, int height) {
    // Detect objects
    auto detections = vision_->detect(frame_data, width, height);
    
    if (detections.empty()) {
        // No detection, predict only
        if (estimator_->isInitialized()) {
            float dt = 1.0f / config_.target_fps;
            auto state = estimator_->predict(dt);
            
            // Send predicted position to motors
            auto angles = computeGimbalAngles(state);
            motor_->setTargetAngles(angles);
        }
        return;
    }
    
    // Use first (best) detection
    const auto& detection = detections[0];
    
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.detections_count++;
        stats_.avg_detection_confidence = 
            (stats_.avg_detection_confidence * (stats_.detections_count - 1) + 
             detection.confidence) / stats_.detections_count;
    }
    
    // Update estimator
    if (!estimator_->isInitialized()) {
        estimator_->initializeState(detection);
    }
    
    auto state = estimator_->update(detection);
    
    // Compute gimbal angles and send to motors
    auto angles = computeGimbalAngles(state);
    motor_->setTargetAngles(angles);
}

GimbalAngles TrackerServer::computeGimbalAngles(const EstimatedState& state) {
    // Simple proportional control
    // Map pixel position to gimbal angles
    
    // Assuming (0,0) is center of frame
    // Positive X = right, positive Y = down
    
    // Map X position to pan angle
    float frame_center_x = config_.vision.input_width / 2.0f;
    float x_error = state.position.x - frame_center_x;
    float pan_angle = (x_error / frame_center_x) * 1.0f;  // Max 1 radian
    
    // Map Y position to tilt angle
    float frame_center_y = config_.vision.input_height / 2.0f;
    float y_error = state.position.y - frame_center_y;
    float tilt_angle = -(y_error / frame_center_y) * 0.5f;  // Max 0.5 radian
    
    return GimbalAngles(pan_angle, tilt_angle);
}

} // namespace tracker
