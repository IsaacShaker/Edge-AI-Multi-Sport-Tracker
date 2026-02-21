#pragma once

#include "../interfaces/i_vision_detector.h"
#include <string>
#include <map>
#include <functional>

namespace tracker {

/**
 * @brief Configuration for vision detectors
 */
struct VisionConfig : public Config {
    std::string model_type;          // "yolo", "color_based", etc.
    std::string model_path;          // Path to model weights (for YOLO)
    float confidence_threshold;
    int input_width;
    int input_height;
    std::string target_label;        // "sports ball", "person", etc.
    
    // Color-based detection params
    int hue_min, hue_max;
    int sat_min, sat_max;
    int val_min, val_max;
    
    VisionConfig() 
        : confidence_threshold(0.5f),
          input_width(640),
          input_height(480),
          target_label("sports ball"),
          hue_min(25), hue_max(45),      // Yellow-green for tennis balls
          sat_min(80), sat_max(255),
          val_min(80), val_max(255) {}
};

/**
 * @brief Factory for creating vision detector instances
 * 
 * Uses factory pattern to create different vision detectors.
 * Supports registration of new detector types at runtime.
 */
class VisionFactory {
public:
    /**
     * @brief Create a vision detector by type
     * @param type Detector type ("yolo", "color_based", etc.)
     * @param config Configuration for the detector
     * @return Shared pointer to the detector, or nullptr if type unknown
     */
    static VisionDetectorPtr create(
        const std::string& type,
        const VisionConfig& config
    );
    
    /**
     * @brief Register a custom detector creator function
     * @param type Type identifier
     * @param creator Function that creates and returns a detector
     */
    static void registerCreator(
        const std::string& type,
        std::function<VisionDetectorPtr(const VisionConfig&)> creator
    );
    
    /**
     * @brief Get list of available detector types
     * @return Vector of registered type names
     */
    static std::vector<std::string> getAvailableTypes();

private:
    static std::map<std::string, 
                    std::function<VisionDetectorPtr(const VisionConfig&)>> creators_;
    static void registerBuiltinCreators();
};

} // namespace tracker
