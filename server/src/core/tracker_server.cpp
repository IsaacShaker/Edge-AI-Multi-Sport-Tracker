#include "../include/core/tracker_server.h"
#include "../include/factories/vision_factory.h"
#include "../include/factories/estimator_factory.h"
#include "../include/factories/motor_factory.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace tracker {

TrackerServer::TrackerServer() 
    : running_(false),
      lost_frames_count_(0),
      use_roi_(false),
      last_detection_size_(100.0f),
      has_last_prediction_(false),
      frame_count_(0) {  // Initial guess for object size
    stats_.fps = 0.0f;
    stats_.frames_processed = 0;
    stats_.detections_count = 0;
    stats_.avg_detection_confidence = 0.0f;
    search_roi_ = cv::Rect(0, 0, 0, 0);
}

TrackerServer::~TrackerServer() {
    if (running_) {
        stop();
    }
    if (prediction_log_.is_open()) {
        prediction_log_.close();
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
    
    // Initialize prediction error logging
    prediction_log_.open("prediction_log.csv");
    if (prediction_log_.is_open()) {
        prediction_log_ << "frame,predicted_x,predicted_y,actual_x,actual_y,velocity,error,timestamp_ms\n";
        std::cout << "Prediction logging enabled: prediction_log.csv" << std::endl;
    } else {
        std::cerr << "Warning: Could not open prediction log file" << std::endl;
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
    // Open camera.
    // If camera_source is a plain integer string ("0", "1", ...) open by
    // V4L2 index.  Otherwise treat it as a GStreamer pipeline string, which
    // is the correct approach for Raspberry Pi 5 with libcamera/Arducam.
    cv::VideoCapture cap;
    const std::string& src = config_.camera_source;
    bool isIndex = !src.empty() &&
        std::all_of(src.begin(), src.end(), ::isdigit);
    if (isIndex) {
        cap.open(std::stoi(src));
    } else {
        cap.open(src, cv::CAP_GSTREAMER);
    }
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera: " << src << std::endl;
        std::cerr << "  Integer index: try 0, 1, 2..." << std::endl;
        std::cerr << "  GStreamer pipeline: ensure OpenCV was built with GStreamer support" << std::endl;
        running_ = false;
        return;
    }

    // Only apply cap.set() for plain device indices.
    // For GStreamer pipelines the format/resolution/fps are already baked into
    // the pipeline string — calling cap.set() on a GStreamer backend triggers
    // an internal pipeline restart that kills libcamerasrc on RPi5/PiSP.
    if (isIndex) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  config_.vision.input_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, config_.vision.input_height);
        cap.set(cv::CAP_PROP_FPS,          config_.target_fps);
    }
    
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
    // Convert to cv::Mat
    cv::Mat frame(height, width, CV_8UC3, const_cast<void*>(frame_data));
    
    // Run detection and tracking
    auto result = detectAndTrack(frame);
    
    // Send commands to motor using Kalman-filtered current state
    // This is already the optimal blend of prediction + measurement
    if (result.has_state) {
        auto angles = computeGimbalAngles(result.current_state);
        motor_->setTargetAngles(angles);
    }
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

void TrackerServer::updateSearchROI(const EstimatedState& state, int frame_width, int frame_height) {
    // Calculate velocity magnitude
    float velocity_magnitude = std::sqrt(
        state.velocity.vx * state.velocity.vx + 
        state.velocity.vy * state.velocity.vy
    );
    
    // Velocity-adaptive ROI scaling
    // Slow objects: base scale (6x), Fast objects: up to max scale (12x)
    float velocity_factor = std::min(velocity_magnitude / VELOCITY_THRESHOLD, 1.0f);
    float roi_scale = ROI_BASE_SCALE + velocity_factor * (ROI_MAX_SCALE - ROI_BASE_SCALE);
    float roi_size = last_detection_size_ * roi_scale;
    
    // Predict where object will be in next frame (lead the target)
    float dt = 1.0f / config_.target_fps;
    float predicted_x = state.position.x + state.velocity.vx * dt;
    float predicted_y = state.position.y + state.velocity.vy * dt;
    
    // Center ROI on predicted position
    int roi_x = static_cast<int>(predicted_x - roi_size / 2);
    int roi_y = static_cast<int>(predicted_y - roi_size / 2);
    int roi_w = static_cast<int>(roi_size);
    int roi_h = static_cast<int>(roi_size);
    
    // Clamp to frame boundaries
    roi_x = std::max(0, std::min(roi_x, frame_width - roi_w));
    roi_y = std::max(0, std::min(roi_y, frame_height - roi_h));
    roi_w = std::min(roi_w, frame_width - roi_x);
    roi_h = std::min(roi_h, frame_height - roi_y);
    
    search_roi_ = cv::Rect(roi_x, roi_y, roi_w, roi_h);
    use_roi_ = true;
}

void TrackerServer::resetSearchROI() {
    use_roi_ = false;
    lost_frames_count_ = 0;
    search_roi_ = cv::Rect(0, 0, 0, 0);
}

TrackerServer::TrackingResult TrackerServer::detectAndTrack(cv::Mat& frame) {
    TrackingResult result;
    result.has_detection = false;
    result.has_state = false;
    result.has_prediction = false;
    result.roi_used = use_roi_ ? search_roi_ : cv::Rect(0, 0, frame.cols, frame.rows);
    
    // Prepare search region
    cv::Mat search_region;
    int roi_offset_x = 0;
    int roi_offset_y = 0;
    
    if (use_roi_ && search_roi_.width > 0 && search_roi_.height > 0) {
        // Clone the ROI to ensure contiguous memory
        search_region = frame(search_roi_).clone();
        roi_offset_x = search_roi_.x;
        roi_offset_y = search_roi_.y;
    } else {
        search_region = frame;
    }
    
    // Detect objects in search region
    auto detections = vision_->detect(search_region.data, search_region.cols, search_region.rows);
    
    // Adjust detection coordinates back to full frame
    for (auto& detection : detections) {
        detection.center.x += roi_offset_x;
        detection.center.y += roi_offset_y;
        if (detection.has_bbox) {
            detection.bbox.x += roi_offset_x;
            detection.bbox.y += roi_offset_y;
        }
    }
    
    if (detections.empty()) {
        lost_frames_count_++;
        
        // Early fallback: if using ROI and lost 2-4 frames, try full frame search
        // This catches fast-moving objects that escape the ROI
        if (use_roi_ && lost_frames_count_ >= FALLBACK_TRIGGER_FRAMES && lost_frames_count_ <= 4) {
            auto full_detections = vision_->detect(frame.data, frame.cols, frame.rows);
            
            if (!full_detections.empty()) {
                // Found it in full frame! Recover and continue tracking
                detections = full_detections;
                lost_frames_count_ = 0;
                result.roi_used = cv::Rect(0, 0, frame.cols, frame.rows);  // Mark as full frame
                std::cout << "[ROI Recovery] Object found in full-frame fallback search" << std::endl;
                goto process_detection;  // Jump to detection processing
            }
        }
        
        // After sustained loss, reset to full-frame search permanently
        if (lost_frames_count_ >= MAX_LOST_FRAMES) {
            resetSearchROI();
        }
        
        // No detection, predict only
        if (estimator_->isInitialized()) {
            float dt = 1.0f / config_.target_fps;
            result.current_state = estimator_->predict(dt);
            result.has_state = true;
            
            // Update ROI based on prediction (for next frame)
            if (use_roi_) {
                updateSearchROI(result.current_state, frame.cols, frame.rows);
            }
            
            // Predict future position for visualization only (150ms look-ahead)
            const float VIZ_LOOK_AHEAD = 0.15f;
            result.predicted_state = estimator_->predict(VIZ_LOOK_AHEAD);
            result.has_prediction = true;
        }
        return result;
    }
    
process_detection:
    
    // Reset lost frame counter on successful detection
    lost_frames_count_ = 0;
    
    // Use first (best) detection
    result.detection = detections[0];
    result.has_detection = true;
    
    // Update last detection size for ROI calculation
    if (result.detection.has_bbox) {
        last_detection_size_ = std::max(result.detection.bbox.width, result.detection.bbox.height);
    } else if (result.detection.radius > 0) {
        last_detection_size_ = result.detection.radius * 2.0f;
    }
    
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.detections_count++;
        stats_.avg_detection_confidence = 
            (stats_.avg_detection_confidence * (stats_.detections_count - 1) + 
             result.detection.confidence) / stats_.detections_count;
    }
    
    // Update estimator - Kalman filter internally:
    //   1. Predicts where ball should be based on previous state
    //   2. Corrects prediction with new measurement (detection)
    //   3. Returns optimal blend (Kalman gain determines weight)
    if (!estimator_->isInitialized()) {
        estimator_->initializeState(result.detection);
        result.current_state = estimator_->getState();
    } else {
        result.current_state = estimator_->update(result.detection);
    }
    result.has_state = true;
    
    // Log prediction error (compare last frame's prediction with current actual position)
    if (has_last_prediction_ && prediction_log_.is_open()) {
        // Calculate prediction error
        float error_x = result.current_state.position.x - last_prediction_.position.x;
        float error_y = result.current_state.position.y - last_prediction_.position.y;
        float error = std::sqrt(error_x * error_x + error_y * error_y);
        
        // Calculate velocity magnitude
        float velocity = std::sqrt(
            result.current_state.velocity.vx * result.current_state.velocity.vx +
            result.current_state.velocity.vy * result.current_state.velocity.vy
        );
        
        // Get timestamp
        auto now = std::chrono::steady_clock::now();
        auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        // Log: frame, predicted_x, predicted_y, actual_x, actual_y, velocity, error, timestamp
        prediction_log_ << frame_count_ << ","
                       << last_prediction_.position.x << ","
                       << last_prediction_.position.y << ","
                       << result.current_state.position.x << ","
                       << result.current_state.position.y << ","
                       << velocity << ","
                       << error << ","
                       << timestamp_ms << "\n";
        prediction_log_.flush();  // Ensure data is written
    }
    
    frame_count_++;
    
    // Store prediction for next frame (predict one frame ahead)
    float dt = 1.0f / config_.target_fps;
    last_prediction_ = estimator_->predict(dt);
    has_last_prediction_ = true;
    
    // Update ROI for next frame
    updateSearchROI(result.current_state, frame.cols, frame.rows);
    
    // Predict future position for visualization only (150ms look-ahead)
    const float VIZ_LOOK_AHEAD = 0.15f;
    result.predicted_state = estimator_->predict(VIZ_LOOK_AHEAD);
    result.has_prediction = true;
    
    return result;
}

void TrackerServer::processFrameWithVisualization(cv::Mat& frame) {
    // Run core detection and tracking logic
    auto result = detectAndTrack(frame);
    
    // Send motor commands using Kalman-filtered current state
    // This is already the optimal blend of prediction + measurement
    if (result.has_state) {
        auto angles = computeGimbalAngles(result.current_state);
        motor_->setTargetAngles(angles);
    }
    
    // === VISUALIZATION ===
    
    // Draw detection (GREEN)
    if (result.has_detection) {
        // Draw bounding box
        if (result.detection.has_bbox) {
            cv::Rect rect(
                static_cast<int>(result.detection.bbox.x),
                static_cast<int>(result.detection.bbox.y),
                static_cast<int>(result.detection.bbox.width),
                static_cast<int>(result.detection.bbox.height)
            );
            cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
        }
        
        // Draw raw detection center (GREEN circle)
        cv::circle(frame, 
                  cv::Point(static_cast<int>(result.detection.center.x), 
                           static_cast<int>(result.detection.center.y)), 
                  5, cv::Scalar(0, 255, 0), -1);
        
        // Draw radius circle if available
        if (result.detection.radius > 0) {
            cv::circle(frame,
                      cv::Point(static_cast<int>(result.detection.center.x),
                               static_cast<int>(result.detection.center.y)),
                      static_cast<int>(result.detection.radius),
                      cv::Scalar(0, 255, 0), 2);
        }
    }
    
    // Draw search ROI (YELLOW rectangle)
    if (result.roi_used.width > 0 && result.roi_used.height > 0 && use_roi_) {
        cv::rectangle(frame, result.roi_used, cv::Scalar(0, 255, 255), 2);
    }
    
    // Draw Kalman-filtered current state (BLUE circle) - WHERE GIMBAL AIMS
    // This is the optimal blend of prediction + measurement from Kalman filter
    if (result.has_state) {
        cv::circle(frame,
                  cv::Point(static_cast<int>(result.current_state.position.x),
                           static_cast<int>(result.current_state.position.y)),
                  10, cv::Scalar(255, 0, 0), -1);  // Solid BLUE circle - gimbal aim point
        
        // Draw velocity vector from current state (BLUE arrow)
        float vel_magnitude = std::sqrt(result.current_state.velocity.vx * result.current_state.velocity.vx +
                                       result.current_state.velocity.vy * result.current_state.velocity.vy);
        if (vel_magnitude > 1.0f) {
            cv::Point start(static_cast<int>(result.current_state.position.x),
                          static_cast<int>(result.current_state.position.y));
            cv::Point end(static_cast<int>(result.current_state.position.x + result.current_state.velocity.vx * 0.5f),
                         static_cast<int>(result.current_state.position.y + result.current_state.velocity.vy * 0.5f));
            cv::arrowedLine(frame, start, end, cv::Scalar(255, 0, 0), 2);
        }
    }
    
    // Draw predicted future position (RED circle) - visualization only, not used for control
    if (result.has_prediction) {
        cv::circle(frame,
                  cv::Point(static_cast<int>(result.predicted_state.position.x),
                           static_cast<int>(result.predicted_state.position.y)),
                  8, cv::Scalar(0, 0, 255), 2);  // Hollow RED circle - expected future
        
        // Draw trajectory line from current to predicted
        if (result.has_state) {
            cv::line(frame,
                    cv::Point(static_cast<int>(result.current_state.position.x),
                             static_cast<int>(result.current_state.position.y)),
                    cv::Point(static_cast<int>(result.predicted_state.position.x),
                             static_cast<int>(result.predicted_state.position.y)),
                    cv::Scalar(128, 128, 128), 1, cv::LINE_AA);  // Gray trajectory
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
    
    // Draw velocity info
    if (result.has_state) {
        float velocity_magnitude = std::sqrt(
            result.current_state.velocity.vx * result.current_state.velocity.vx +
            result.current_state.velocity.vy * result.current_state.velocity.vy
        );
        
        std::string vel_text = "Velocity: " + std::to_string(static_cast<int>(velocity_magnitude)) + " px/s";
        cv::putText(frame, vel_text,
                   cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    }
    
    // Draw ROI status
    std::string roi_status = use_roi_ ? "ROI: Active" : "ROI: Full Frame";
    if (lost_frames_count_ > 0) {
        roi_status += " | Lost: " + std::to_string(lost_frames_count_);
    }
    cv::putText(frame, roi_status,
               cv::Point(10, 90),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // Draw legend
    cv::putText(frame, "Green: Detection | Blue: Kalman Filtered (Gimbal Aim) | Red: Expected Future", 
               cv::Point(10, frame.rows - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}

} // namespace tracker
