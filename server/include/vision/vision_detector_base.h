#pragma once

#include "../interfaces/i_vision_detector.h"
#include "../interfaces/types.h"

#ifdef ENABLE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace tracker {

/**
 * @brief Base class for vision detectors with common functionality
 */
class VisionDetectorBase : public IVisionDetector {
public:
    VisionDetectorBase() : ready_(false) {}
    virtual ~VisionDetectorBase() = default;
    
    bool isReady() const override { return ready_; }
    
protected:
    bool ready_;
    
#ifdef ENABLE_OPENCV
    /**
     * @brief Helper to convert void* frame data to cv::Mat
     */
    cv::Mat frameToMat(const void* frame_data, int width, int height) const;
#endif
};

} // namespace tracker
