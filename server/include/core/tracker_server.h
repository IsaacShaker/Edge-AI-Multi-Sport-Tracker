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
#include <opencv2/opencv.hpp>

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
    
    // ROI tracking parameters
    static constexpr float ROI_BASE_SCALE = 6.0f;      // Base ROI scale for slow objects
    static constexpr float ROI_MAX_SCALE = 12.0f;      // Max ROI scale for fast objects
    static constexpr float VELOCITY_THRESHOLD = 100.0f; // px/s threshold for max scale
    static constexpr int MAX_LOST_FRAMES = 10;         // Frames before full-frame search
    static constexpr int FALLBACK_TRIGGER_FRAMES = 2;  // Frames before trying full-frame fallback
    
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
     * @brief Disable ROI optimization (use full-frame search only)
     */
    void disableROI() { use_roi_ = false; }
    
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
    
    // ROI Tracking
    cv::Rect search_roi_;              // Current search region
    int lost_frames_count_;            // Consecutive frames without detection
    bool use_roi_;                     // Whether to use ROI search
    float last_detection_size_;        // Size of last successful detection
    
    /**
     * @brief Main tracking loop (runs in separate thread)
     */
    void trackerLoop();
    
    /**
     * @brief Update search ROI based on predicted position
     */
    void updateSearchROI(const EstimatedState& state, int frame_width, int frame_height);
    
    /**
     * @brief Reset to full-frame search
     */
    void resetSearchROI();
    
    /**
     * @brief Core detection and tracking results
     */
    struct TrackingResult {
        Detection detection;
        EstimatedState current_state;      // Kalman-filtered state (optimal blend of prediction + measurement)
        EstimatedState predicted_state;    // Future prediction for visualization only
        bool has_detection;
        bool has_state;
        bool has_prediction;                // Whether predicted_state is valid
        cv::Rect roi_used;
    };
    
    /**
     * @brief Core detection and tracking logic (shared by both process methods)
     */
    TrackingResult detectAndTrack(cv::Mat& frame);
    
    /**
     * @brief Process a single frame
     */
    void processFrame(const void* frame_data, int width, int height);
    
    /**
     * @brief Process a single frame with visualization
     */
    void processFrameWithVisualization(cv::Mat& frame);
    
    /**
     * @brief Compute gimbal angles from estimated state
     */
    GimbalAngles computeGimbalAngles(const EstimatedState& state);
};

} // namespace tracker
