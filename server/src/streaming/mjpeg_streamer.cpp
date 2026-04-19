// StreamServer — serves an MJPEG video stream, Server-Sent Events telemetry,
// and an embedded web dashboard over plain HTTP sockets.
// No external library dependencies beyond POSIX sockets and OpenCV.

#include "streaming/stream_server.h"
#include "web_dashboard.h"  // auto-generated from web/index.html by CMake

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

namespace tracker {

// ── ctor / dtor ──────────────────────────────────────────────────────────────

StreamServer::StreamServer() = default;

StreamServer::~StreamServer() { stop(); }

// ── start / stop ─────────────────────────────────────────────────────────────

bool StreamServer::start(int port) {
    if (running_) return true;
    port_ = port;

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[StreamServer] socket() failed: " << strerror(errno) << "\n";
        return false;
    }

    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[StreamServer] bind() failed on port " << port_
                  << ": " << strerror(errno) << "\n";
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (::listen(server_fd_, 10) < 0) {
        std::cerr << "[StreamServer] listen() failed: " << strerror(errno) << "\n";
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_       = true;
    accept_thread_ = std::thread(&StreamServer::acceptLoop, this);

    std::cout << "[StreamServer] Listening on http://0.0.0.0:" << port_ << "\n";
    return true;
}

void StreamServer::stop() {
    if (!running_) return;
    running_ = false;

    // Unblock accept() by closing the server socket.
    if (server_fd_ >= 0) {
        ::shutdown(server_fd_, SHUT_RDWR);
        ::close(server_fd_);
        server_fd_ = -1;
    }

    // Wake any client threads blocked on condition variables.
    frame_cv_.notify_all();
    telemetry_cv_.notify_all();

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
}

// ── push methods (called from tracker thread) ────────────────────────────────

void StreamServer::pushFrame(const cv::Mat& frame, int jpeg_quality) {
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    if (!cv::imencode(".jpg", frame, jpeg, params)) return;

    {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        latest_jpeg_ = std::move(jpeg);
        ++frame_seq_;
    }
    frame_cv_.notify_all();
}

void StreamServer::pushTelemetry(const TelemetryData& data) {
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        latest_telemetry_ = data;
        ++telemetry_seq_;
    }
    telemetry_cv_.notify_all();
}

// ── accept loop ──────────────────────────────────────────────────────────────

void StreamServer::acceptLoop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t   addr_len = sizeof(client_addr);
        int client_fd = ::accept(server_fd_,
                                 reinterpret_cast<sockaddr*>(&client_addr),
                                 &addr_len);
        if (client_fd < 0) {
            if (!running_) break;
            continue;
        }
        // One detached thread per client (expected: 1-3 simultaneous clients).
        std::thread([this, client_fd]() {
            handleClient(client_fd);
            ::close(client_fd);
        }).detach();
    }
}

// ── request routing ──────────────────────────────────────────────────────────

void StreamServer::handleClient(int fd) {
    char buf[2048] = {};
    ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;

    // Extract the request path from "GET /path HTTP/1.x"
    std::string req(buf, static_cast<size_t>(n));
    std::string path = "/";
    auto sp1 = req.find(' ');
    if (sp1 != std::string::npos) {
        auto sp2 = req.find(' ', sp1 + 1);
        if (sp2 != std::string::npos)
            path = req.substr(sp1 + 1, sp2 - sp1 - 1);
    }
    // Strip query string
    auto q = path.find('?');
    if (q != std::string::npos) path = path.substr(0, q);

    if (path == "/" || path == "/index.html") {
        serveStatic(fd);
    } else if (path == "/stream") {
        serveMjpeg(fd);
    } else if (path == "/telemetry") {
        serveSSE(fd);
    } else if (path == "/config") {
        serveConfig(fd);
    } else {
        serve404(fd);
    }
}

// ── response helpers ─────────────────────────────────────────────────────────

bool StreamServer::writeAll(int fd, const void* buf, size_t len) {
    const char* p = static_cast<const char*>(buf);
    while (len > 0) {
        ssize_t written = ::write(fd, p, len);
        if (written <= 0) return false;
        p   += written;
        len -= static_cast<size_t>(written);
    }
    return true;
}

void StreamServer::serveStatic(int fd) {
    const char* html     = indexHtml();
    size_t      html_len = strlen(html);

    std::ostringstream hdr;
    hdr << "HTTP/1.1 200 OK\r\n"
        << "Content-Type: text/html; charset=utf-8\r\n"
        << "Content-Length: " << html_len << "\r\n"
        << "Connection: close\r\n"
        << "\r\n";
    std::string h = hdr.str();
    writeAll(fd, h.data(), h.size());
    writeAll(fd, html, html_len);
}

void StreamServer::serveMjpeg(int fd) {
    static const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: keep-alive\r\n"
        "\r\n";
    if (!writeAll(fd, hdr, strlen(hdr))) return;

    uint64_t last_seq = UINT64_MAX; // force immediate send of first frame
    while (running_) {
        std::vector<uint8_t> jpeg;
        {
            std::unique_lock<std::mutex> lk(frame_mutex_);
            frame_cv_.wait_for(lk, std::chrono::seconds(2),
                               [&] { return frame_seq_ != last_seq || !running_; });
            if (!running_) break;
            if (frame_seq_ == last_seq) continue; // timeout — nothing new yet
            last_seq = frame_seq_;
            jpeg     = latest_jpeg_;
        }

        std::ostringstream part;
        part << "--frame\r\n"
             << "Content-Type: image/jpeg\r\n"
             << "Content-Length: " << jpeg.size() << "\r\n"
             << "\r\n";
        std::string p = part.str();
        if (!writeAll(fd, p.data(), p.size()))       break;
        if (!writeAll(fd, jpeg.data(), jpeg.size())) break;
        if (!writeAll(fd, "\r\n", 2))                break;
    }
}

void StreamServer::serveSSE(int fd) {
    static const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: keep-alive\r\n"
        "\r\n";
    if (!writeAll(fd, hdr, strlen(hdr))) return;

    uint64_t last_seq = UINT64_MAX;
    while (running_) {
        TelemetryData data;
        {
            std::unique_lock<std::mutex> lk(telemetry_mutex_);
            telemetry_cv_.wait_for(lk, std::chrono::seconds(1),
                                   [&] { return telemetry_seq_ != last_seq || !running_; });
            if (!running_) break;
            if (telemetry_seq_ == last_seq) {
                // SSE keep-alive comment — prevents browser from closing the connection.
                if (!writeAll(fd, ": ping\n\n", 8)) break;
                continue;
            }
            last_seq = telemetry_seq_;
            data     = latest_telemetry_;
        }
        std::string msg = "data: " + telemetryToJson(data) + "\n\n";
        if (!writeAll(fd, msg.data(), msg.size())) break;
    }
}

void StreamServer::setConfig(const std::string& json) {
    config_json_ = json;
}

void StreamServer::serveConfig(int fd) {
    const std::string& body = config_json_.empty() ? "{}" : config_json_;
    std::ostringstream hdr;
    hdr << "HTTP/1.1 200 OK\r\n"
        << "Content-Type: application/json\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Connection: close\r\n"
        << "\r\n";
    std::string h = hdr.str();
    writeAll(fd, h.data(), h.size());
    writeAll(fd, body.data(), body.size());
}

void StreamServer::serve404(int fd) {
    static const char* body = "<h1>404 Not Found</h1>";
    std::ostringstream hdr;
    hdr << "HTTP/1.1 404 Not Found\r\n"
        << "Content-Type: text/html\r\n"
        << "Content-Length: " << strlen(body) << "\r\n"
        << "Connection: close\r\n\r\n";
    std::string h = hdr.str();
    writeAll(fd, h.data(), h.size());
    writeAll(fd, body, strlen(body));
}

// ── telemetry JSON ────────────────────────────────────────────────────────────

std::string StreamServer::telemetryToJson(const TelemetryData& d) {
    std::ostringstream j;
    j << "{"
      << "\"fps\":"           << static_cast<int>(d.fps)              << ","
      << "\"frame_count\":"   << d.frame_count                        << ","
      << "\"has_detection\":" << (d.has_detection ? "true" : "false") << ","
      << "\"confidence\":"    << d.confidence                         << ","
      << "\"pos_x\":"         << d.pos_x                             << ","
      << "\"pos_y\":"         << d.pos_y                             << ","
      << "\"vel_x\":"         << d.vel_x                             << ","
      << "\"vel_y\":"         << d.vel_y                             << ","
      << "\"pan_deg\":"       << d.pan_deg                           << ","
      << "\"tilt_deg\":"      << d.tilt_deg                          << ","
      << "\"timestamp_ms\":"  << d.timestamp_ms
      << "}";
    return j.str();
}

// ── embedded web dashboard ────────────────────────────────────────────────────

const char* StreamServer::indexHtml() {
    // HTML is compiled in from web/index.html via web_dashboard.h (auto-generated
    // by cmake/embed_html.cmake). Edit web/index.html to change the dashboard.
    return kIndexHtml();
}

} // namespace tracker
