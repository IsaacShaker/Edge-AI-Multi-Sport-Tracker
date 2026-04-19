#pragma once

#include "../interfaces/i_vision_detector.h"
#include "../interfaces/types.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

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
    
    /**
     * @brief Convert void* frame data to cv::Mat (BGR)
     */
    cv::Mat frameToMat(const void* frame_data, int width, int height) const;

    /**
     * @brief Resize frame to model input size and return raw RGB uint8 Mat.
     *
     * Suitable for Hailo HEFs (which bake normalization into the network) and
     * any other backend that wants [0,255] RGB bytes as input.
     */
    cv::Mat prepareInput(const void* frame_data, int width, int height,
                         int model_w, int model_h) const;

    /**
     * @brief Convert NMS result vectors into Detection objects.
     *
     * Shared by YOLODetector and HailoDetector so the output format is
     * identical regardless of which inference backend is used.
     *
     * @param boxes        Bounding boxes in full-frame pixel coords
     * @param confidences  Per-box confidence scores
     * @param class_ids    Per-box class indices
     * @param class_names  Class name table (indexed by class_id)
     * @param target_label Fallback label when class_id is out of range
     */
    std::vector<Detection> buildDetections(
        const std::vector<cv::Rect>& boxes,
        const std::vector<float>&    confidences,
        const std::vector<int>&      class_ids,
        const std::vector<std::string>& class_names,
        const std::string& target_label) const;
};

} // namespace tracker
