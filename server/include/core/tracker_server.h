#pragma once

#include "../interfaces/i_vision_detector.h"
#include "../interfaces/i_state_estimator.h"
#include "../interfaces/i_motor_controller.h"
#include "../factories/vision_factory.h"
#include "../factories/estimator_factory.h"
#include "../factories/motor_factory.h"
#include "../streaming/stream_server.h"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <fstream>
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
    // Accepts either an integer device index ("0", "1") or a full
    // GStreamer pipeline string (used for libcamera on Raspberry Pi).
    std::string camera_source;
    float target_fps;
    bool enable_visualization;
    bool enable_web_streaming;
    int web_port;
    // When true, a ColorBasedDetector runs inline on every camera frame as a
    // fallback. Hailo/YOLO async results take priority when available.
    bool color_assist;
    // Debug: aim at the raw detection centre instead of the Kalman-filtered
    // position.  Removes smoothing — useful for calibrating motor response.
    bool use_raw_detection;
    
    ServerConfig()
        : camera_source("0"),
          target_fps(30.0f),
          enable_visualization(true),
          enable_web_streaming(false),
          web_port(8080),
          color_assist(false),
          use_raw_detection(false) {}
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
     * @brief Enable/disable active gimbal tracking from the web UI.
     * Motors will not move until this is set to true AND the ball has been
     * detected at least once.
     */
    void setTrackingActive(bool active) { tracking_active_ = active; }
    bool isTrackingActive() const { return tracking_active_; }

    /**
     * @brief Move the gimbal directly to a point in normalised image space.
     *        nx, ny are in [0, 1] (top-left origin, same as CSS).
     *        Bypasses tracking gate — intended for debug use only.
     */
    void moveGimbalToPixel(float nx, float ny);
    
    /**
     * @brief Check if server is running
     */
    bool isRunning() const { return running_; }
    
    /**
     * @brief Get current statistics
     */
    struct Stats {
        float fps;
        float inference_latency_ms;  // Avg time per detect() call (hardware speed)
        float inference_rate;        // Actual detect calls per wall-clock second
        int frames_processed;
        int detections_count;
        float avg_detection_confidence;
        float detection_rate;       // YOLO hit rate while actively tracking (0-1)
        float prediction_rmse;      // Kalman one-step prediction RMSE (pixels)
        float actual_pan_rad;       // Current pan angle in radians (for web UI)
        float actual_tilt_rad;      // Current tilt angle in radians (for web UI)
        float current_pan_rad;        // Current target pan angle in radians (for web UI)
        float current_tilt_rad;       // Current target tilt angle in radians (for web UI)
    };
    Stats getStats() const;

private:
    ServerConfig config_;
    
    // Subsystems
    VisionDetectorPtr vision_;           // Primary (hailo/yolo) — runs async
    VisionDetectorPtr secondary_vision_; // Color assist — runs inline, nullptr if disabled
    StateEstimatorPtr estimator_;
    MotorControllerPtr motor_;
    
    // Control
    std::atomic<bool> running_;
    // Gimbal tracking is off until the user enables it from the web UI.
    // Even when enabled, motors won't move until the ball has been seen once
    // (has_last_prediction_ guards that).
    std::atomic<bool> tracking_active_{false};
    std::thread tracker_thread_;

    // Async detection thread — decouples YOLO inference (~4fps) from capture (30fps)
    std::thread detect_thread_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat pending_detect_frame_;
    cv::Rect pending_detect_roi_;       // ROI snapshot passed to detect thread
    bool pending_detect_use_roi_{false};
    bool new_frame_ready_{false};
    std::mutex detections_mutex_;
    std::vector<Detection> async_detections_;
    bool fresh_detections_{false};
    // Timestamp of the frame that produced async_detections_; used to
    // compensate for async lag before feeding results to the Kalman filter.
    std::chrono::steady_clock::time_point async_detection_capture_time_;
    // Timestamp written by the camera thread when it submits a frame.
    std::chrono::steady_clock::time_point pending_detect_time_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    std::chrono::steady_clock::time_point last_frame_time_;
    
    // ROI Tracking
    cv::Rect search_roi_;              // Current search region
    int lost_frames_count_;            // Consecutive frames without detection
    bool use_roi_;                     // Whether to use ROI search
    float last_detection_size_;        // Size of last successful detection
    
    // Prediction Error Logging
    std::ofstream prediction_log_;
    // Motor command log — one row per setTargetAngles() call
    std::ofstream motor_log_;
    EstimatedState last_prediction_;
    bool has_last_prediction_;
    int frame_count_;

    // Prediction RMSE accumulators
    double prediction_error_sum_sq_;
    int prediction_error_count_;

    // Frames where estimator was active (ball had been seen before this frame)
    // Used as denominator for detection rate — excludes frames with no ball present
    int frames_while_tracking_;

    // Inference timing accumulators (updated in detectLoop, read in getStats)
    double inference_time_sum_ms_;
    int    inference_call_count_;
    std::chrono::steady_clock::time_point session_start_time_;

    // Incremental gimbal state — accumulated absolute target sent to firmware
    float actual_pan_rad_;
    float actual_tilt_rad_;
    float current_pan_rad_;
    float current_tilt_rad_;

    float integral_pan_ = 0.0f;
    float integral_tilt_ = 0.0f;

    float prev_ang_err_pan_ = 0.0f;
    float prev_ang_err_tilt_ = 0.0f;

    float deriv_pan_filt_ = 0.0f;
    float deriv_tilt_filt_ = 0.0f;

    std::chrono::steady_clock::time_point last_control_time_{};

    // Web streaming
    std::unique_ptr<StreamServer> stream_server_;
    
    /**
     * @brief Main tracking loop (runs in separate thread)
     */
    void trackerLoop();

    /**
     * @brief YOLO detection loop — runs in its own thread, feeds async_detections_
     */
    void detectLoop();
    
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
    GimbalAngles computeGimbalAngles(const EstimatedState& state, int frame_width, int frame_height);
};

} // namespace tracker
