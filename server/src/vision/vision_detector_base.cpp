// Stub implementation - to be completed
#include "../include/vision/vision_detector_base.h"

namespace tracker {

cv::Mat VisionDetectorBase::frameToMat(const void* frame_data, int width, int height) const {
    // Assume BGR format
    return cv::Mat(height, width, CV_8UC3, const_cast<void*>(frame_data));
}

} // namespace tracker
