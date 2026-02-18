#include "include/core/tracker_server.h"
#include "include/factories/vision_factory.h"
#include "include/factories/estimator_factory.h"
#include "include/factories/motor_factory.h"
#include <iostream>
#include <csignal>
#include <memory>

using namespace tracker;

// Global server instance for signal handling
std::unique_ptr<TrackerServer> g_server;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_server) {
        g_server->stop();
    }
}

void printUsage() {
    std::cout << "Edge AI Multi-Sport Tracker Server" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << "\nUsage: tracker_server [options]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  --vision <type>       Vision detector type (color_based, yolo)" << std::endl;
    std::cout << "  --estimator <type>    State estimator type (kalman_cv, kalman_ca, imm)" << std::endl;
    std::cout << "  --motor <type>        Motor controller type (simplefoc, mock)" << std::endl;
    std::cout << "  --camera <id>         Camera device ID (default: 0)" << std::endl;
    std::cout << "  --no-viz              Disable visualization" << std::endl;
    std::cout << "  --help                Show this help message" << std::endl;
}

void printAvailableModules() {
    std::cout << "\nAvailable Vision Detectors:" << std::endl;
    for (const auto& type : VisionFactory::getAvailableTypes()) {
        std::cout << "  - " << type << std::endl;
    }
    
    std::cout << "\nAvailable State Estimators:" << std::endl;
    for (const auto& type : EstimatorFactory::getAvailableTypes()) {
        std::cout << "  - " << type << std::endl;
    }
    
    std::cout << "\nAvailable Motor Controllers:" << std::endl;
    for (const auto& type : MotorFactory::getAvailableTypes()) {
        std::cout << "  - " << type << std::endl;
    }
}

int main(int argc, char** argv) {
    // Parse command line arguments
    ServerConfig config;
    
    // Default configuration
    config.vision.model_type = "color_based";
    config.estimator.estimator_type = "imm";
    config.motor.controller_type = "mock";
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h") {
            printUsage();
            printAvailableModules();
            return 0;
        }
        else if (arg == "--vision" && i + 1 < argc) {
            config.vision.model_type = argv[++i];
        }
        else if (arg == "--estimator" && i + 1 < argc) {
            config.estimator.estimator_type = argv[++i];
        }
        else if (arg == "--motor" && i + 1 < argc) {
            config.motor.controller_type = argv[++i];
        }
        else if (arg == "--camera" && i + 1 < argc) {
            config.camera_device_id = std::atoi(argv[++i]);
        }
        else if (arg == "--no-viz") {
            config.enable_visualization = false;
        }
        else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage();
            return 1;
        }
    }
    
    // Print configuration
    std::cout << "\n=== Configuration ===" << std::endl;
    std::cout << "Vision:    " << config.vision.model_type << std::endl;
    std::cout << "Estimator: " << config.estimator.estimator_type << std::endl;
    std::cout << "Motor:     " << config.motor.controller_type << std::endl;
    std::cout << "Camera:    " << config.camera_device_id << std::endl;
    std::cout << "=====================\n" << std::endl;
    
    // Create and initialize server
    g_server = std::make_unique<TrackerServer>();
    
    if (!g_server->initialize(config)) {
        std::cerr << "Failed to initialize tracker server" << std::endl;
        return 1;
    }
    
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Start server
    if (!g_server->start()) {
        std::cerr << "Failed to start tracker server" << std::endl;
        return 1;
    }
    
    // Wait for server to stop
    while (g_server->isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Print statistics every 2 seconds
        static auto last_stats_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - last_stats_time).count() > 2.0f) {
            auto stats = g_server->getStats();
            std::cout << "FPS: " << stats.fps 
                      << " | Frames: " << stats.frames_processed
                      << " | Detections: " << stats.detections_count
                      << " | Avg Conf: " << stats.avg_detection_confidence
                      << std::endl;
            last_stats_time = now;
        }
    }
    
    std::cout << "Server exited cleanly" << std::endl;
    return 0;
}
