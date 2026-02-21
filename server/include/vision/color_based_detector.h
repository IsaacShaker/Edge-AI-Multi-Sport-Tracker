#pragma once

#include "vision_detector_base.h"
#include "../factories/vision_factory.h"
#include <opencv2/opencv.hpp>

namespace tracker {

/**
 * @brief Color-based ball detector using HSV color segmentation
 * 
 * Simple but effective detector for colored balls (orange basketball, etc.)
 * Uses morphological operations and contour detection.
 */
class ColorBasedDetector : public VisionDetectorBase {
public:
    ColorBasedDetector() = default;
    ~ColorBasedDetector() override = default;
    
    bool initialize(const Config& config) override;
    
    std::vector<Detection> detect(
        const void* frame_data,
        int width,
        int height
    ) override;
    
    std::string getType() const override { return "color_based"; }
    
    void shutdown() override;

private:
    VisionConfig config_;
    
    cv::Scalar lower_bound_;
    cv::Scalar upper_bound_;
    
    /**
     * @brief Find the largest circular contour in a binary mask
     */
    bool findBallContour(const cv::Mat& mask, Detection& detection);
};

} // namespace tracker
