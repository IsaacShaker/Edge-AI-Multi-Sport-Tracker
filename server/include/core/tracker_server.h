#pragma once

#include "../interfaces/i_vision_detector.h"
#include "../interfaces/i_state_estimator.h"
#include "../interfaces/i_motor_controller.h"
#include "../factories/vision_factory.h"
#include "../factories/estimator_factory.h"
#include "../factories/motor_factory.h"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

#ifdef ENABLE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace tracker {

/**
 * @brief Main tracker server configuration
 */
struct ServerConfig : public Config {
    // Subsystem configurations
    VisionConfig vision;
    EstimatorConfig estimator;
    MotorConfig motor;
    
    // Server parameters
    int camera_device_id;
    float target_fps;
    bool enable_visualization;
    bool enable_web_streaming;
    int web_port;
    
    ServerConfig()
        : camera_device_id(0),
          target_fps(30.0f),
          enable_visualization(true),
          enable_web_streaming(false),
          web_port(8080) {}
};

/**
 * @brief Main tracker server class
 * 
 * Orchestrates all subsystems:
 * - Captures video frames
 * - Detects objects with vision system
 * - Estimates state with Kalman/IMM filter
 * - Controls motors to track target
 * - Optionally streams to web clients
 */
class TrackerServer {
public:
    TrackerServer();
    ~TrackerServer();
    
    /**
     * @brief Initialize server with configuration
     */
    bool initialize(const ServerConfig& config);
    
    /**
     * @brief Start the tracking loop
     */
    bool start();
    
    /**
     * @brief Stop the tracking loop
     */
    void stop();
    
    /**
     * @brief Check if server is running
     */
    bool isRunning() const { return running_; }
    
    /**
     * @brief Get current statistics
     */
    struct Stats {
        float fps;
        int frames_processed;
        int detections_count;
        float avg_detection_confidence;
    };
    Stats getStats() const;

private:
    ServerConfig config_;
    
    // Subsystems
    VisionDetectorPtr vision_;
    StateEstimatorPtr estimator_;
    MotorControllerPtr motor_;
    
    // Control
    std::atomic<bool> running_;
    std::thread tracker_thread_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    std::chrono::steady_clock::time_point last_frame_time_;
    
    /**
     * @brief Main tracking loop (runs in separate thread)
     */
    void trackerLoop();
    
    /**
     * @brief Process a single frame
     */
    void processFrame(const void* frame_data, int width, int height);
    
#ifdef ENABLE_OPENCV
    /**
     * @brief Process a single frame with visualization
     */
    void processFrameWithVisualization(cv::Mat& frame);
#endif
    
    /**
     * @brief Compute gimbal angles from estimated state
     */
    GimbalAngles computeGimbalAngles(const EstimatedState& state);
};

} // namespace tracker
