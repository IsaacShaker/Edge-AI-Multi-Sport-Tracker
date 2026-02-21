#pragma once

#include "vision_detector_base.h"
#include "../factories/vision_factory.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace tracker {

/**
 * @brief YOLO-based object detector
 * 
 * Uses YOLOv8 (or other YOLO versions) for robust object detection.
 * Can detect multiple object classes.
 */
class YOLODetector : public VisionDetectorBase {
public:
    YOLODetector() = default;
    ~YOLODetector() override = default;
    
    bool initialize(const Config& config) override;
    
    std::vector<Detection> detect(
        const void* frame_data,
        int width,
        int height
    ) override;
    
    std::string getType() const override { return "yolo"; }
    
    void shutdown() override;

private:
    VisionConfig config_;
    
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    
    /**
     * @brief Post-process YOLO network output
     */
    std::vector<Detection> postprocess(
        const std::vector<cv::Mat>& outputs,
        int frame_width,
        int frame_height
    );
};

} // namespace tracker
