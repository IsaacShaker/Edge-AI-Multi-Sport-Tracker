#pragma once

#include "vision_detector_base.h"
#include "../factories/vision_factory.h"
#include <opencv2/opencv.hpp>

#include "hailo/hailort.hpp"
#include "hailo/infer_model.hpp"

namespace tracker {

/**
 * @brief YOLOv8 detector accelerated by Hailo-8 / Hailo-8L NPU.
 *
 * Uses the HailoRT high-level InferModel API (same as picamera2's Hailo class
 * and the official Hailo examples).  The low-level VStreams API was abandoned
 * because it consistently returns empty buffers on HailoRT 4.23.
 *
 * Output format: HAILO_FORMAT_ORDER_HAILO_NMS_BY_CLASS (format order 22).
 * Layout per class: [float32 bbox_count, {y_min,x_min,y_max,x_max,score}×N]
 * Coordinates are normalised to [0, 1].
 */
class HailoDetector : public VisionDetectorBase {
public:
    HailoDetector() = default;
    ~HailoDetector() override { shutdown(); }

    bool initialize(const Config& config) override;

    std::vector<Detection> detect(
        const void* frame_data,
        int width,
        int height
    ) override;

    std::string getType() const override { return "hailo"; }

    void shutdown() override;

private:
    std::unique_ptr<hailort::VDevice>        vdevice_;
    std::shared_ptr<hailort::InferModel>     infer_model_;
    hailort::ConfiguredInferModel            configured_model_;

    // Pre-allocated output buffer — must be page-aligned for Hailo DMA.
    // Uses mmap so the kernel guarantees PAGE_SIZE alignment, matching the
    // official Hailo Application Code Examples (hailo_infer.cpp).
    std::shared_ptr<uint8_t> output_buf_ptr_;
    size_t                   output_buf_size_{0};

    VisionConfig config_;

    int model_input_w_{640};
    int model_input_h_{640};
    int nms_num_classes_{80};
    int nms_max_bboxes_{100};

    static const std::vector<std::string> kCocoClassNames;
    int target_class_id_{-1};

    std::vector<Detection> postprocessNMS(
        int frame_width,
        int frame_height
    );
};

} // namespace tracker
