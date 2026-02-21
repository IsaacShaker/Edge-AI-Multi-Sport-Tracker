#include "../include/core/tracker_server.h"
#include "../include/factories/vision_factory.h"
#include "../include/factories/estimator_factory.h"
#include "../include/factories/motor_factory.h"
#include <iostream>
#include <opencv2/opencv.hpp>

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
    
    // Note: cv::imshow will create the window automatically on first call
    if (config_.enable_visualization) {
        std::cout << "Display enabled - press 'q' to quit" << std::endl;
    }
    
    std::cout << "Tracker loop started" << std::endl;
    
    while (running_) {
        // Capture frame
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            continue;
        }
        
        // Process frame
        if (config_.enable_visualization) {
            processFrameWithVisualization(frame);
            
            // Display frame
            try {
                cv::imshow("Edge AI Multi-Sport Tracker", frame);
                if (cv::waitKey(1) == 'q') {
                    running_ = false;
                }
            } catch (cv::Exception& e) {
                std::cerr << "\n\nERROR: Cannot display window!" << std::endl;
                std::cerr << "Details: " << e.what() << std::endl;
                std::cerr << "\nDisabling visualization. Tracker will continue without display." << std::endl;
                std::cerr << "\nPossible solutions:" << std::endl;
                std::cerr << "  1. Set QT_QPA_PLATFORM=xcb (for XWayland compatibility)" << std::endl;
                std::cerr << "  2. Verify Qt5 libraries are installed: qtbase5-dev qtwayland5" << std::endl;
                std::cerr << "  3. Use --no-viz flag to run without display\n" << std::endl;
                config_.enable_visualization = false;  // Disable to avoid repeated errors
            }
        } else {
            processFrame(frame.data, frame.cols, frame.rows);
        }
        
        // Update FPS
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration<float>(now - last_frame_time_).count();
        last_frame_time_ = now;
        
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.fps = 1.0f / dt;
            stats_.frames_processed++;
        }
    }
    
    cap.release();
    if (config_.enable_visualization) {
        cv::destroyAllWindows();
    }
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

void TrackerServer::processFrameWithVisualization(cv::Mat& frame) {
    // Detect objects
    auto detections = vision_->detect(frame.data, frame.cols, frame.rows);
    
    // Track if we have a valid state to draw
    EstimatedState current_state;
    bool has_state = false;
    
    if (detections.empty()) {
        // No detection, predict only
        if (estimator_->isInitialized()) {
            float dt = 1.0f / config_.target_fps;
            current_state = estimator_->predict(dt);
            has_state = true;
            
            // Send predicted position to motors
            auto angles = computeGimbalAngles(current_state);
            motor_->setTargetAngles(angles);
        }
    } else {
        // Use first (best) detection
        const auto& detection = detections[0];
        
        // Draw detection bounding box (GREEN)
        if (detection.has_bbox) {
            cv::Rect rect(
                static_cast<int>(detection.bbox.x),
                static_cast<int>(detection.bbox.y),
                static_cast<int>(detection.bbox.width),
                static_cast<int>(detection.bbox.height)
            );
            cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
        }
        
        // Draw detection center (GREEN circle)
        cv::circle(frame, 
                  cv::Point(static_cast<int>(detection.center.x), 
                           static_cast<int>(detection.center.y)), 
                  5, cv::Scalar(0, 255, 0), -1);
        
        // Draw radius circle if available
        if (detection.radius > 0) {
            cv::circle(frame,
                      cv::Point(static_cast<int>(detection.center.x),
                               static_cast<int>(detection.center.y)),
                      static_cast<int>(detection.radius),
                      cv::Scalar(0, 255, 0), 2);
        }
        
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
        
        current_state = estimator_->update(detection);
        has_state = true;
        
        // Compute gimbal angles and send to motors
        auto angles = computeGimbalAngles(current_state);
        motor_->setTargetAngles(angles);
    }
    
    // Draw predicted/estimated position (BLUE circle)
    if (has_state) {
        cv::circle(frame,
                  cv::Point(static_cast<int>(current_state.position.x),
                           static_cast<int>(current_state.position.y)),
                  8, cv::Scalar(255, 0, 0), -1);
        
        // Draw velocity vector if significant (BLUE arrow)
        float vel_magnitude = std::sqrt(current_state.velocity.vx * current_state.velocity.vx +
                                       current_state.velocity.vy * current_state.velocity.vy);
        if (vel_magnitude > 1.0f) {
            cv::Point start(static_cast<int>(current_state.position.x),
                          static_cast<int>(current_state.position.y));
            cv::Point end(static_cast<int>(current_state.position.x + current_state.velocity.vx * 0.1f),
                         static_cast<int>(current_state.position.y + current_state.velocity.vy * 0.1f));
            cv::arrowedLine(frame, start, end, cv::Scalar(255, 0, 0), 2);
        }
    }
    
    // Draw crosshair at center
    int center_x = frame.cols / 2;
    int center_y = frame.rows / 2;
    cv::line(frame, cv::Point(center_x - 20, center_y), cv::Point(center_x + 20, center_y), 
             cv::Scalar(255, 255, 255), 1);
    cv::line(frame, cv::Point(center_x, center_y - 20), cv::Point(center_x, center_y + 20), 
             cv::Scalar(255, 255, 255), 1);
    
    // Draw stats overlay
    std::string fps_text;
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        fps_text = "FPS: " + std::to_string(static_cast<int>(stats_.fps)) + 
                   " | Detections: " + std::to_string(stats_.detections_count) + 
                   "/" + std::to_string(stats_.frames_processed);
    }
    cv::putText(frame, fps_text, cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // Draw legend
    cv::putText(frame, "Green: Detection | Blue: Predicted", cv::Point(10, frame.rows - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}

} // namespace tracker
