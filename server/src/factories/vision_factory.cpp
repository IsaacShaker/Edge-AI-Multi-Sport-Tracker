#include "../include/factories/vision_factory.h"
#include "../include/vision/color_based_detector.h"
#include "../include/vision/yolo_detector.h"
#include <iostream>

namespace tracker {

// Static member initialization
std::map<std::string, std::function<VisionDetectorPtr(const VisionConfig&)>> 
    VisionFactory::creators_;

VisionDetectorPtr VisionFactory::create(
    const std::string& type,
    const VisionConfig& config
) {
    // Ensure built-in creators are registered
    if (creators_.empty()) {
        registerBuiltinCreators();
    }
    
    auto it = creators_.find(type);
    if (it == creators_.end()) {
        std::cerr << "Unknown vision detector type: " << type << std::endl;
        return nullptr;
    }
    
    return it->second(config);
}

void VisionFactory::registerCreator(
    const std::string& type,
    std::function<VisionDetectorPtr(const VisionConfig&)> creator
) {
    creators_[type] = creator;
}

std::vector<std::string> VisionFactory::getAvailableTypes() {
    if (creators_.empty()) {
        registerBuiltinCreators();
    }
    
    std::vector<std::string> types;
    types.reserve(creators_.size());
    for (const auto& pair : creators_) {
        types.push_back(pair.first);
    }
    return types;
}

void VisionFactory::registerBuiltinCreators() {
    // Register color-based detector
    registerCreator("color_based", [](const VisionConfig& config) -> VisionDetectorPtr {
        auto detector = std::make_shared<ColorBasedDetector>();
        if (detector->initialize(config)) {
            return detector;
        }
        return nullptr;
    });
    
    // Register YOLO detector
    registerCreator("yolo", [](const VisionConfig& config) -> VisionDetectorPtr {
        auto detector = std::make_shared<YOLODetector>();
        if (detector->initialize(config)) {
            return detector;
        }
        return nullptr;
    });
}

} // namespace tracker
