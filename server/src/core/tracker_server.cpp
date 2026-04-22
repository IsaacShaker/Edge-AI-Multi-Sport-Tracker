#include "../include/core/tracker_server.h"
#include "../include/factories/vision_factory.h"
#include "../include/factories/estimator_factory.h"
#include "../include/factories/motor_factory.h"
#include <iostream>
#include <sstream>
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
      frame_count_(0),
      prediction_error_sum_sq_(0.0),
      prediction_error_count_(0),
      frames_while_tracking_(0),
      inference_time_sum_ms_(0.0),
      inference_call_count_(0),
      current_pan_rad_(0.0f),
      current_tilt_rad_(0.0f) {
    stats_.fps = 0.0f;
    stats_.inference_latency_ms = 0.0f;
    stats_.inference_rate = 0.0f;
    stats_.frames_processed = 0;
    stats_.detections_count = 0;
    stats_.avg_detection_confidence = 0.0f;
    stats_.detection_rate = 0.0f;
    stats_.prediction_rmse = 0.0f;
    frames_while_tracking_ = 0;
    search_roi_ = cv::Rect(0, 0, 0, 0);
}

TrackerServer::~TrackerServer() {
    if (running_) {
        stop();
    }
    if (prediction_log_.is_open()) {
        prediction_log_.close();
    }
    if (motor_log_.is_open()) {
        motor_log_.close();
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

    // Create color-assist detector if requested (and primary isn't already color)
    if (config_.color_assist && config_.vision.model_type != "color_based") {
        std::cout << "Creating color-assist detector (inline fallback)" << std::endl;
        secondary_vision_ = VisionFactory::create("color_based", config.vision);
        if (!secondary_vision_) {
            std::cerr << "Warning: Failed to create color-assist detector — continuing without it" << std::endl;
        }
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

    auto motor_status = motor_->getStatus();
    current_pan_rad_ = motor_status.current_angles.pan;
    current_tilt_rad_ = motor_status.current_angles.tilt;
    
    // Initialize prediction error logging
    prediction_log_.open("prediction_log.csv", std::ios::out | std::ios::trunc);
    if (prediction_log_.is_open()) {
        prediction_log_ << "frame,predicted_x,predicted_y,actual_x,actual_y,velocity,error,timestamp_ms\n";
        std::cout << "Prediction logging enabled: prediction_log.csv" << std::endl;
    } else {
        std::cerr << "Warning: Could not open prediction log file" << std::endl;
    }

    motor_log_.open("motor_log.csv", std::ios::out | std::ios::trunc);
    if (motor_log_.is_open()) {
        motor_log_ << "timestamp_ms,pan_rad,tilt_rad,pan_deg,tilt_deg,err_x_px,err_y_px,source\n";
        std::cout << "Motor command logging enabled: motor_log.csv" << std::endl;
    } else {
        std::cerr << "Warning: Could not open motor log file" << std::endl;
    }

    // Start web streaming server if enabled
    if (config_.enable_web_streaming) {
        stream_server_ = std::make_unique<StreamServer>();
        if (!stream_server_->start(config_.web_port)) {
            std::cerr << "Warning: Failed to start stream server on port "
                      << config_.web_port << std::endl;
            stream_server_.reset();
        } else {
            // Serialize runtime config once so the dashboard /config endpoint
            // can show it. Simple manual JSON — no library needed.
            auto esc = [](const std::string& s) {
                std::string out;
                for (char c : s) {
                    if (c == '"' || c == '\\') out += '\\';
                    out += c;
                }
                return out;
            };
            std::ostringstream j;
            j << "{"
              << "\"vision\":\""              << esc(config_.vision.model_type)        << "\","
              << "\"target_label\":\""        << esc(config_.vision.target_label)      << "\","
              << "\"confidence_threshold\":"  << config_.vision.confidence_threshold   << ","
              << "\"estimator\":\""           << esc(config_.estimator.estimator_type) << "\","
              << "\"motor\":\""               << esc(config_.motor.controller_type)    << "\","
              << "\"serial_port\":\""         << esc(config_.motor.serial_port)        << "\","
              << "\"fps\":"                   << config_.target_fps
              << "}";
            stream_server_->setConfig(j.str());
            // Register tracking toggle callback so the web UI can gate gimbal movement
            stream_server_->setTrackingCallback([this](bool en) {
                setTrackingActive(en);
                std::cout << "[Tracking] Active tracking " << (en ? "ENABLED" : "DISABLED")
                          << " by user via web UI" << std::endl;
            });
            // Register debug manual-target callback (used when tracking is off)
            stream_server_->setGimbalTargetCallback([this](float nx, float ny) {
                moveGimbalToPixel(nx, ny);
            });
        }
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
    session_start_time_ = std::chrono::steady_clock::now();
    detect_thread_  = std::thread(&TrackerServer::detectLoop,  this);
    tracker_thread_ = std::thread(&TrackerServer::trackerLoop, this);

    return true;
}

void TrackerServer::stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "Stopping tracker server..." << std::endl;
    running_ = false;

    // Wake detect thread so it can observe running_==false and exit
    {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        new_frame_ready_ = true;
    }
    frame_cv_.notify_one();

    if (detect_thread_.joinable()) {
        detect_thread_.join();
    }

    if (tracker_thread_.joinable()) {
        tracker_thread_.join();
    }

    if (motor_) {
        motor_->disconnect();
    }

    // ── Final metrics summary ──────────────────────────────────────────────
    {
        auto s = getStats();

        // Build summary string once — write to stdout, to the web /metrics
        // endpoint, and to metrics_summary.txt on disk (survives SSH disconnect).
        std::ostringstream summary;
        summary << "=== Session Metrics ===\n";
        summary << "  Detector          : " << config_.vision.model_type << "\n";
        summary << "  Frames processed  : " << s.frames_processed << "\n";
        summary << "  Display FPS (avg) : " << s.fps << "\n";
        summary << "  Inference rate    : " << s.inference_rate
                << " calls/s  (" << inference_call_count_ << " total calls)\n";
        summary << "  Avg latency/call  : " << s.inference_latency_ms << " ms\n";
        summary << "  Avg confidence    : " << s.avg_detection_confidence
                << "  (detected frames only)\n";
        if (s.prediction_rmse > 0.0f) {
            summary << "  Kalman pred RMSE  : " << s.prediction_rmse << " px\n";
        }
        summary << "======================\n";

        std::cout << "\n" << summary.str() << "\n";

        // Push to web endpoint before stopping stream server so a browser
        // request arriving right after Ctrl+C still gets the final data.
        if (stream_server_) {
            stream_server_->setMetrics(summary.str());
        }

        // Also persist to disk next to prediction_log.csv
        std::ofstream mf("metrics_summary.txt", std::ios::app);
        if (mf.is_open()) {
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            mf << "# run ended at steady_clock ms=" << now_ms << "\n";
            mf << summary.str() << "\n";
        }
    }

    if (stream_server_) {
        stream_server_->stop();
    }

    std::cout << "Tracker server stopped" << std::endl;
}

TrackerServer::Stats TrackerServer::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    Stats s = stats_;
    // Use frames_while_tracking_ as denominator: excludes frames before the
    // ball was ever seen so the rate reflects actual YOLO reliability.
    s.detection_rate  = (frames_while_tracking_ > 0)
        ? static_cast<float>(s.detections_count) / static_cast<float>(frames_while_tracking_)
        : 0.0f;
    s.prediction_rmse = (prediction_error_count_ > 0)
        ? static_cast<float>(std::sqrt(prediction_error_sum_sq_ / prediction_error_count_))
        : 0.0f;
    s.inference_latency_ms = (inference_call_count_ > 0)
        ? static_cast<float>(inference_time_sum_ms_ / inference_call_count_)
        : 0.0f;
    double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - session_start_time_).count();
    s.inference_rate = (elapsed_s > 0.0)
        ? static_cast<float>(inference_call_count_ / elapsed_s)
        : 0.0f;
    s.actual_pan_rad = actual_pan_rad_;
    s.actual_tilt_rad = actual_tilt_rad_;
    s.current_pan_rad = current_pan_rad_;
    s.current_tilt_rad = current_tilt_rad_;
    return s;
}

void TrackerServer::detectLoop() {
    while (running_) {
        cv::Mat frame;
        cv::Rect roi;
        bool use_roi = false;
        std::chrono::steady_clock::time_point capture_time;
        {
            std::unique_lock<std::mutex> lk(frame_mutex_);
            frame_cv_.wait(lk, [this]{ return new_frame_ready_ || !running_; });
            if (!running_) break;
            frame        = pending_detect_frame_;   // lightweight ref-counted copy
            roi          = pending_detect_roi_;
            use_roi      = pending_detect_use_roi_;
            capture_time = pending_detect_time_;
            new_frame_ready_ = false;
        }

        std::vector<Detection> dets;
        int roi_offset_x = 0;
        int roi_offset_y = 0;

        // Helper lambda: run one detect() call and accumulate wall-clock time.
        auto timedDetect = [&](const void* data, int w, int h) {
            auto t0 = std::chrono::steady_clock::now();
            auto result = vision_->detect(data, w, h);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            std::lock_guard<std::mutex> lk(stats_mutex_);
            inference_time_sum_ms_ += ms;
            inference_call_count_++;
            return result;
        };

        if (use_roi && roi.width > 0 && roi.height > 0) {
            // Detect on the cropped ROI — much faster than full frame
            cv::Mat crop = frame(roi).clone();
            dets = timedDetect(crop.data, crop.cols, crop.rows);

            // Offset coordinates back to full-frame space
            roi_offset_x = roi.x;
            roi_offset_y = roi.y;
            for (auto& d : dets) {
                d.center.x += roi_offset_x;
                d.center.y += roi_offset_y;
                if (d.has_bbox) {
                    d.bbox.x += roi_offset_x;
                    d.bbox.y += roi_offset_y;
                }
            }

            // Full-frame fallback: if ROI detect found nothing, try the whole frame
            // so fast-moving objects that escaped the ROI are still caught.
            if (dets.empty()) {
                dets = timedDetect(frame.data, frame.cols, frame.rows);
            }
        } else {
            dets = timedDetect(frame.data, frame.cols, frame.rows);
        }

        {
            std::lock_guard<std::mutex> lk(detections_mutex_);
            async_detections_             = std::move(dets);
            async_detection_capture_time_ = capture_time;
            fresh_detections_ = true;
        }
    }
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
        // Camera is mounted upside-down — flip vertically (flipCode=0 = around x-axis)
        cv::flip(frame, frame, 0);

        // Feed latest frame + current ROI snapshot to the async detection thread
        // (non-blocking — drops frames that arrive while YOLO is still busy).
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            pending_detect_frame_   = frame;
            pending_detect_roi_     = search_roi_;
            pending_detect_use_roi_ = use_roi_;
            pending_detect_time_    = std::chrono::steady_clock::now();
            new_frame_ready_ = true;
        }
        frame_cv_.notify_one();
        
        // Process frame — run full visualization pipeline when either the local
        // display or the web stream is active (both need the annotated frame).
        if (config_.enable_visualization || config_.enable_web_streaming) {
            processFrameWithVisualization(frame);

            if (config_.enable_visualization) {
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
                    config_.enable_visualization = false;
                }
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

        // Push live metrics to web endpoint every ~2 seconds
        if (stream_server_ && config_.enable_web_streaming) {
            static auto last_metrics_push = std::chrono::steady_clock::now();
            auto now2 = std::chrono::steady_clock::now();
            if (std::chrono::duration<float>(now2 - last_metrics_push).count() >= 2.0f) {
                last_metrics_push = now2;
                auto s = getStats();
                std::ostringstream m;
                m << "=== Live Metrics ===\n";
                m << "  Detector          : " << config_.vision.model_type << "\n";
                m << "  Frames processed  : " << s.frames_processed << "\n";
                m << "  Display FPS       : " << s.fps << "\n";
                m << "  Inference rate    : " << s.inference_rate
                  << " calls/s  (" << inference_call_count_ << " total calls)\n";
                m << "  Avg latency/call  : " << s.inference_latency_ms << " ms\n";
                m << "  Avg confidence    : " << s.avg_detection_confidence
                  << "  (detected frames only)\n";
                if (s.prediction_rmse > 0.0f) {
                    m << "  Kalman pred RMSE  : " << s.prediction_rmse << " px\n";
                }
                m << "====================\n";
                stream_server_->setMetrics(m.str());
            }
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
    
    // Only move the gimbal when the ball is actually detected this frame.
    // Kalman-predict-only frames (has_state but no detection) would chase the
    // velocity vector ahead of the ball — skip them.
    if (result.has_detection && tracking_active_ && has_last_prediction_) {
        // Debug mode: aim at the raw YOLO bbox centre instead of the
        // Kalman-smoothed position.  Removes lag/smoothing artifacts.
        EstimatedState aim_state = result.current_state;
        if (config_.use_raw_detection) {
            aim_state.position.x = result.detection.center.x;
            aim_state.position.y = result.detection.center.y;
        }
        auto angles = computeGimbalAngles(aim_state, frame.cols, frame.rows);
        motor_->setTargetAngles(angles);
        if (motor_log_.is_open()) {
            float ex = aim_state.position.x - (config_.vision.input_width  / 2.0f);
            float ey = aim_state.position.y - (config_.vision.input_height / 2.0f);
            auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            motor_log_ << ts << ","
                       << angles.pan  << "," << angles.tilt << ","
                       << angles.pan  * (180.0f / static_cast<float>(M_PI)) << ","
                       << angles.tilt * (180.0f / static_cast<float>(M_PI)) << ","
                       << ex << "," << ey << ",tracker\n";
            motor_log_.flush();
        }
    }
}

GimbalAngles TrackerServer::computeGimbalAngles(
    const EstimatedState& state,
    int frame_width,
    int frame_height)
{
    static constexpr float PAN_MAX     = 7.0f;
    static constexpr float PAN_MIN     = 4.0f;
    static constexpr float TILT_MAX    = 1.5f;
    static constexpr float TILT_MIN    = -0.75f;
    static constexpr float DEADBAND_PX = 2.0f;

    static constexpr float KP_PAN  = 0.40f;
    static constexpr float KI_PAN  = 0.03f;
    static constexpr float KD_PAN  = 0.04f;

    static constexpr float KP_TILT = 0.40f;
    static constexpr float KI_TILT = 0.02f;
    static constexpr float KD_TILT = 0.03f;

    static constexpr float I_PAN_MAX  = 0.25f;
    static constexpr float I_TILT_MAX = 0.20f;

    static constexpr float D_FILTER_ALPHA = 0.2f;  // 0..1, lower = smoother
    static constexpr float MAX_CORR_PAN   = 0.15f;
    static constexpr float MAX_CORR_TILT  = 0.12f;

    using clock = std::chrono::steady_clock;
    const auto now = clock::now();

    if (last_control_time_.time_since_epoch().count() == 0) {
        last_control_time_ = now;
    }

    float dt = std::chrono::duration<float>(now - last_control_time_).count();
    last_control_time_ = now;

    // protect against tiny/huge dt spikes
    dt = std::clamp(dt, 0.01f, 0.1f);

    const float actual_pan_rad  = motor_->getPanRad();
    const float actual_tilt_rad = motor_->getTiltRad();

    const float width      = static_cast<float>(frame_width);
    const float height     = static_cast<float>(frame_height);
    const float hfov_rad   = config_.vision.hfov_deg * (M_PI / 180.0f);
    const float vfov_rad   = config_.vision.vfov_deg * (M_PI / 180.0f);
    const float focal_x_px = (width  / 2.0f) / std::tan(hfov_rad / 2.0f);
    const float focal_y_px = (height / 2.0f) / std::tan(vfov_rad / 2.0f);

    float err_x = state.position.x - width  / 2.0f;
    float err_y = state.position.y - height / 2.0f;

    if (std::abs(err_x) < DEADBAND_PX) err_x = 0.0f;
    if (std::abs(err_y) < DEADBAND_PX) err_y = 0.0f;

    const float ang_err_pan  = -std::atan2(err_x, focal_x_px);
    const float ang_err_tilt = -std::atan2(err_y, focal_y_px);

    // reset integral when error changes sign
    if (ang_err_pan * integral_pan_ < 0.0f) {
        integral_pan_ = 0.0f;
    }
    if (ang_err_tilt * integral_tilt_ < 0.0f) {
        integral_tilt_ = 0.0f;
    }

    // only integrate when outside deadband
    if (ang_err_pan != 0.0f) {
        integral_pan_ += ang_err_pan * dt;
        integral_pan_ = std::clamp(integral_pan_, -I_PAN_MAX, I_PAN_MAX);
    }

    if (ang_err_tilt != 0.0f) {
        integral_tilt_ += ang_err_tilt * dt;
        integral_tilt_ = std::clamp(integral_tilt_, -I_TILT_MAX, I_TILT_MAX);
    }

    // raw derivative
    float d_pan_raw  = (ang_err_pan  - prev_ang_err_pan_)  / dt;
    float d_tilt_raw = (ang_err_tilt - prev_ang_err_tilt_) / dt;

    prev_ang_err_pan_  = ang_err_pan;
    prev_ang_err_tilt_ = ang_err_tilt;

    // low-pass filter derivative
    deriv_pan_filt_  = D_FILTER_ALPHA * d_pan_raw  + (1.0f - D_FILTER_ALPHA) * deriv_pan_filt_;
    deriv_tilt_filt_ = D_FILTER_ALPHA * d_tilt_raw + (1.0f - D_FILTER_ALPHA) * deriv_tilt_filt_;

    float corr_pan =
        KP_PAN * ang_err_pan +
        KI_PAN * integral_pan_ +
        KD_PAN * deriv_pan_filt_;

    float corr_tilt =
        KP_TILT * ang_err_tilt +
        KI_TILT * integral_tilt_ +
        KD_TILT * deriv_tilt_filt_;

    corr_pan  = std::clamp(corr_pan,  -MAX_CORR_PAN,  MAX_CORR_PAN);
    corr_tilt = std::clamp(corr_tilt, -MAX_CORR_TILT, MAX_CORR_TILT);

    float pan_rad  = std::clamp(actual_pan_rad  + corr_pan,  PAN_MIN,  PAN_MAX);
    float tilt_rad = std::clamp(actual_tilt_rad + corr_tilt, TILT_MIN, TILT_MAX);

    current_pan_rad_  = pan_rad;
    current_tilt_rad_ = tilt_rad;

    return GimbalAngles(pan_rad, tilt_rad);
}

void TrackerServer::moveGimbalToPixel(float nx, float ny) {
    // ── Manual-target tuning ──────────────────────────────────────────────
    // MANUAL_GAIN: fraction of the true angular error applied per click.
    //   1.0 = jump to exactly the clicked angle (may overshoot if motor
    //         steps aren't 1:1 with angle).  Reduce if you overshoot.
    static constexpr float MANUAL_GAIN_PAN  = 0.5f;
    static constexpr float MANUAL_GAIN_TILT = 0.5f;

    // ── Camera geometry (same as computeGimbalAngles) ─────────────────────
    const float hfov_rad = config_.vision.hfov_deg * (static_cast<float>(M_PI) / 180.0f);
    const float focal_px = (config_.vision.input_width / 2.0f) / std::tan(hfov_rad / 2.0f);

    float err_x = (nx - 0.5f) * static_cast<float>(config_.vision.input_width);
    float err_y = (ny - 0.5f) * static_cast<float>(config_.vision.input_height);

    // Full angular error to the clicked point
    float target_pan_rad  = -std::atan2(err_x, focal_px);
    float target_tilt_rad = -std::atan2(err_y, focal_px);

    // Step a fraction of the way from the current accumulator toward target
    float pan_rad  = current_pan_rad_  + MANUAL_GAIN_PAN  * (target_pan_rad  - current_pan_rad_);
    float tilt_rad = current_tilt_rad_ + MANUAL_GAIN_TILT * (target_tilt_rad - current_tilt_rad_);

    static constexpr float PAN_MAX  =  6.5f;
    static constexpr float PAN_MIN  = 4.5f;
    static constexpr float TILT_MAX =  1.0f;
    static constexpr float TILT_MIN = -1.0f;
    pan_rad  = std::clamp(pan_rad,  PAN_MIN,  PAN_MAX);
    tilt_rad = std::clamp(tilt_rad, TILT_MIN, TILT_MAX);

    // Update accumulated state so re-enabling tracking doesn't jump.
    current_pan_rad_  = pan_rad;
    current_tilt_rad_ = tilt_rad;

    GimbalAngles angles(pan_rad, tilt_rad);
    motor_->setTargetAngles(angles);
    std::cout << "[Debug] Manual gimbal target nx=" << nx << " ny=" << ny
              << " pan=" << pan_rad * (180.0f / static_cast<float>(M_PI)) << "\xc2\xb0"
              << " tilt=" << tilt_rad * (180.0f / static_cast<float>(M_PI)) << "\xc2\xb0\n";
    if (motor_log_.is_open()) {
        auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        motor_log_ << ts << ","
                   << pan_rad  << "," << tilt_rad << ","
                   << pan_rad  * (180.0f / static_cast<float>(M_PI)) << ","
                   << tilt_rad * (180.0f / static_cast<float>(M_PI)) << ","
                   << err_x << "," << err_y << ",manual\n";
        motor_log_.flush();
    }
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

    // Count frames where the estimator was already active (ball had been seen
    // at least once before this frame). Used as detection-rate denominator so
    // frames before the ball was ever put in front of the camera are excluded.
    if (has_last_prediction_) {
        frames_while_tracking_++;
    }

    // Pull the latest detections produced by the async detection thread.
    // We also capture the timestamp of the frame they came from so we can
    // compensate for async lag before feeding the position to Kalman.
    std::vector<Detection> async_dets;
    std::chrono::steady_clock::time_point async_capture_time;
    {
        std::lock_guard<std::mutex> lk(detections_mutex_);
        if (fresh_detections_) {
            async_dets        = async_detections_;
            async_capture_time = async_detection_capture_time_;
            fresh_detections_ = false;
        }
    }

    // ── Async lag compensation ────────────────────────────────────────────
    // Hailo results arrive 1-2 frames late (~8-33 ms).  Project each detected
    // position forward by lag × current Kalman velocity so the Kalman update
    // receives "where the ball is now" rather than "where it was when captured".
    if (!async_dets.empty() && estimator_->isInitialized()) {
        float lag_s = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - async_capture_time).count();
        // Sanity-cap: ignore compensation if lag is unreasonably large (> 500 ms)
        if (lag_s > 0.0f && lag_s < 0.5f) {
            auto state = estimator_->getState();
            for (auto& d : async_dets) {
                d.center.x += state.velocity.vx * lag_s;
                d.center.y += state.velocity.vy * lag_s;
                if (d.has_bbox) {
                    d.bbox.x += state.velocity.vx * lag_s;
                    d.bbox.y += state.velocity.vy * lag_s;
                }
            }
        }
    }

    // ── Detection source selection ────────────────────────────────────────
    // Hailo (async, lag-compensated) is primary.
    // Color (inline, zero-lag) is the per-frame fallback when Hailo is busy.
    std::vector<Detection> detections = std::move(async_dets);
    if (detections.empty() && secondary_vision_) {
        detections = secondary_vision_->detect(frame.data, frame.cols, frame.rows);
    }

    if (detections.empty()) {
        lost_frames_count_++;

        // After sustained loss, reset ROI state
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
        
        // Accumulate for RMSE
        prediction_error_sum_sq_ += static_cast<double>(error) * static_cast<double>(error);
        prediction_error_count_++;
        
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
    
    // Predict future position for visualization only (250ms look-ahead)
    const float VIZ_LOOK_AHEAD = 0.25f;
    result.predicted_state = estimator_->predict(VIZ_LOOK_AHEAD);
    result.has_prediction = true;
    
    return result;
}

void TrackerServer::processFrameWithVisualization(cv::Mat& frame) {
    // Run core detection and tracking logic
    auto result = detectAndTrack(frame);

    // Only move the gimbal when the ball is actually detected this frame.
    // Kalman-predict-only frames (has_state but no detection) would chase the
    // velocity vector ahead of the ball — skip them.
    GimbalAngles angles{0.0f, 0.0f};
    if (result.has_detection && tracking_active_ && has_last_prediction_) {
        EstimatedState aim_state = result.current_state;
        if (config_.use_raw_detection) {
            aim_state.position.x = result.detection.center.x;
            aim_state.position.y = result.detection.center.y;
        }
        angles = computeGimbalAngles(aim_state, frame.cols, frame.rows);
        motor_->setTargetAngles(angles);
        if (motor_log_.is_open()) {
            float ex = aim_state.position.x - (config_.vision.input_width  / 2.0f);
            float ey = aim_state.position.y - (config_.vision.input_height / 2.0f);
            auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            motor_log_ << ts << ","
                       << angles.pan  << "," << angles.tilt << ","
                       << angles.pan  * (180.0f / static_cast<float>(M_PI)) << ","
                       << angles.tilt * (180.0f / static_cast<float>(M_PI)) << ","
                       << ex << "," << ey << ",tracker\n";
            motor_log_.flush();
        }
    }
    
    // === VISUALIZATION ===
    
    // Draw detection (GREEN)
    if (result.has_detection) {
        // Draw bounding box
        if (result.detection.has_bbox) {
            // bbox.x/y is the center; cv::Rect expects the top-left corner
            cv::Rect rect(
                static_cast<int>(result.detection.bbox.x - result.detection.bbox.width  / 2.0f),
                static_cast<int>(result.detection.bbox.y - result.detection.bbox.height / 2.0f),
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
    std::string infer_text;
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        fps_text = "Display FPS: " + std::to_string(static_cast<int>(stats_.fps));
        double elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - session_start_time_).count();
        float actual_rate = (elapsed_s > 0.0)
            ? static_cast<float>(inference_call_count_ / elapsed_s) : 0.0f;
        float latency_ms = (inference_call_count_ > 0)
            ? static_cast<float>(inference_time_sum_ms_ / inference_call_count_) : 0.0f;
        infer_text = "Infer: " + std::to_string(static_cast<int>(actual_rate))
                   + "fps  " + std::to_string(latency_ms).substr(0, std::to_string(latency_ms).find('.') + 3)
                   + "ms/call  [" + config_.vision.model_type + "]";
    }
    cv::putText(frame, fps_text, cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, infer_text, cv::Point(10, 58),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // Draw velocity info
    if (result.has_state) {
        float velocity_magnitude = std::sqrt(
            result.current_state.velocity.vx * result.current_state.velocity.vx +
            result.current_state.velocity.vy * result.current_state.velocity.vy
        );
        
        std::string vel_text = "Velocity: " + std::to_string(static_cast<int>(velocity_magnitude)) + " px/s";
        cv::putText(frame, vel_text,
                   cv::Point(10, 88),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    }
    
    // Draw ROI status
    std::string roi_status = use_roi_ ? "ROI: Active" : "ROI: Full Frame";
    if (lost_frames_count_ > 0) {
        roi_status += " | Lost: " + std::to_string(lost_frames_count_);
    }
    cv::putText(frame, roi_status,
               cv::Point(10, 116),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // Draw legend
    cv::putText(frame, "Green: Detection | Blue: Kalman Filtered (Gimbal Aim) | Red: Expected Future", 
               cv::Point(10, frame.rows - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // Push annotated frame + telemetry to web stream if enabled
    if (stream_server_ && config_.enable_web_streaming) {
        TelemetryData telem;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            telem.fps         = stats_.fps;
            telem.frame_count = stats_.frames_processed;
        }
        telem.has_detection = result.has_detection;
        if (result.has_detection) {
            telem.confidence = result.detection.confidence;
        }
        if (result.has_state) {
            telem.pos_x = result.current_state.position.x;
            telem.pos_y = result.current_state.position.y;
            telem.vel_x = result.current_state.velocity.vx;
            telem.vel_y = result.current_state.velocity.vy;
        }
        telem.pan_deg  = angles.pan  * (180.0f / static_cast<float>(M_PI));
        telem.tilt_deg = angles.tilt * (180.0f / static_cast<float>(M_PI));
        telem.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        stream_server_->pushFrame(frame);
        stream_server_->pushTelemetry(telem);
    }
}

} // namespace tracker
