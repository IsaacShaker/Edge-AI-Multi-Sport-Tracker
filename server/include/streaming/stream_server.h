#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>

namespace tracker {

struct TelemetryData {
    float   pos_x         = 0.0f;
    float   pos_y         = 0.0f;
    float   vel_x         = 0.0f;
    float   vel_y         = 0.0f;
    float   pan_deg       = 0.0f;
    float   tilt_deg      = 0.0f;
    float   fps           = 0.0f;
    int     frame_count   = 0;
    bool    has_detection = false;
    float   confidence    = 0.0f;
    int64_t timestamp_ms  = 0;
};

// Lightweight HTTP server that serves:
//   GET /           → embedded web dashboard (HTML)
//   GET /stream     → MJPEG video stream (multipart/x-mixed-replace)
//   GET /telemetry  → Server-Sent Events (text/event-stream) with JSON telemetry
//
// Call pushFrame() and pushTelemetry() from the tracker thread.
// No external library dependencies — raw POSIX sockets + OpenCV JPEG encoding.
class StreamServer {
public:
    StreamServer();
    ~StreamServer();

    bool start(int port);
    void stop();
    bool isRunning() const { return running_; }

    // Called from the tracker thread after each processed frame.
    void pushFrame(const cv::Mat& frame, int jpeg_quality = 75);
    void pushTelemetry(const TelemetryData& data);

    // Called once after start() with a JSON string of the runtime config.
    // Served verbatim at GET /config so the dashboard can display settings.
    void setConfig(const std::string& json);

    // Called to update the live metrics text (plain text).
    // Served at GET /metrics as a downloadable file.
    void setMetrics(const std::string& text);

    // Register a callback invoked when the web UI toggles active tracking.
    // The TrackerServer passes a lambda that calls setTrackingActive().
    void setTrackingCallback(std::function<void(bool)> cb) { tracking_cb_ = std::move(cb); }

    // Register a callback invoked when the web UI sends a manual gimbal target.
    // nx, ny are normalised image coordinates in [0, 1].
    void setGimbalTargetCallback(std::function<void(float, float)> cb) { gimbal_target_cb_ = std::move(cb); }

private:
    std::atomic<bool> running_{false};
    int  port_{8080};
    int  server_fd_{-1};
    std::thread accept_thread_;

    // Shared frame buffer — latest JPEG only
    std::mutex              frame_mutex_;
    std::condition_variable frame_cv_;
    std::vector<uint8_t>    latest_jpeg_;
    uint64_t                frame_seq_{0};

    // Shared telemetry buffer — latest reading only
    std::mutex              telemetry_mutex_;
    std::condition_variable telemetry_cv_;
    TelemetryData           latest_telemetry_;
    uint64_t                telemetry_seq_{0};

    // Config JSON — set once at startup, served at /config
    std::string config_json_;

    // Metrics text — updated periodically, served at GET /metrics
    std::mutex  metrics_mutex_;
    std::string metrics_text_;

    // Callback invoked when the dashboard toggles tracking on/off
    std::function<void(bool)> tracking_cb_;

    // Callback invoked when the web UI sends a manual gimbal target (nx, ny in [0,1])
    std::function<void(float, float)> gimbal_target_cb_;

    // Recording state — video is written to disk in kClipDurationSecs-second
    // chunks so RAM usage stays flat even for hour-long recordings.
    static constexpr int kClipDurationSecs = 60;
    std::atomic<bool>                     recording_{false};
    std::mutex                            record_mutex_;
    cv::VideoWriter                       video_writer_;
    std::string                           record_path_;   // current clip path
    std::vector<std::string>              clip_paths_;    // completed clips
    int                                   clip_index_{0};
    std::chrono::steady_clock::time_point clip_start_time_;

    void acceptLoop();
    void handleClient(int fd);
    void serveStatic(int fd);
    void serveMjpeg(int fd);
    void serveSSE(int fd);
    void serveConfig(int fd);
    void serveMetrics(int fd);
    void serveCSV(int fd);
    void serveMotorLog(int fd);
    void serveTracking(int fd, bool enable);
    void serveGimbalTarget(int fd, float nx, float ny);
    void serveRecordStart(int fd);
    void serveRecordStop(int fd);
    void serveRecordDownload(int fd);
    void serve404(int fd);
    void startRecording();
    void stopRecording();
    static bool stitchClips(const std::vector<std::string>& clips,
                            const std::string& output);

    static bool        writeAll(int fd, const void* buf, size_t len);
    static std::string telemetryToJson(const TelemetryData& d);
    static const char* indexHtml();
};

} // namespace tracker
