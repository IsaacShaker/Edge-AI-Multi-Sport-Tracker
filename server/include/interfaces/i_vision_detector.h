#pragma once

#include "types.h"
#include <memory>
#include <vector>

namespace tracker {

/**
 * @brief Abstract interface for vision/detection systems
 * 
 * All vision detectors must implement this interface.
 * This allows swapping between YOLO, traditional CV, or other detection methods.
 */
class IVisionDetector {
public:
    virtual ~IVisionDetector() = default;
    
    /**
     * @brief Initialize the detector with configuration
     * @param config Configuration parameters
     * @return true if initialization successful
     */
    virtual bool initialize(const Config& config) = 0;
    
    /**
     * @brief Process a frame and detect objects
     * @param frame_data Raw frame data (OpenCV Mat or similar)
     * @param width Frame width
     * @param height Frame height
     * @return Vector of detections
     */
    virtual std::vector<Detection> detect(
        const void* frame_data,
        int width,
        int height
    ) = 0;
    
    /**
     * @brief Get the detector type/name
     * @return Detector identifier string
     */
    virtual std::string getType() const = 0;
    
    /**
     * @brief Check if detector is ready
     * @return true if ready to process frames
     */
    virtual bool isReady() const = 0;
    
    /**
     * @brief Cleanup resources
     */
    virtual void shutdown() = 0;
};

using VisionDetectorPtr = std::shared_ptr<IVisionDetector>;

} // namespace tracker
