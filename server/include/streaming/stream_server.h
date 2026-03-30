#pragma once

#include <atomic>
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

    void acceptLoop();
    void handleClient(int fd);
    void serveStatic(int fd);
    void serveMjpeg(int fd);
    void serveSSE(int fd);
    void serve404(int fd);

    static bool        writeAll(int fd, const void* buf, size_t len);
    static std::string telemetryToJson(const TelemetryData& d);
    static const char* indexHtml();
};

} // namespace tracker
