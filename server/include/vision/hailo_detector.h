#pragma once

#include "vision_detector_base.h"
#include "../factories/vision_factory.h"
#include <opencv2/opencv.hpp>

#include "hailo/hailort.h"
#include "hailo/hailort.hpp"

namespace tracker {

/**
 * @brief YOLOv8 detector accelerated by Hailo-8 / Hailo-8L NPU.
 *
 * Uses the HailoRT C++ API to run a pre-compiled .hef model file on the
 * Hailo NPU.
 *
 * Supports two HEF output formats automatically:
 *   NMS-embedded  — decoded detections by class (typical Hailo Model Zoo HEFs)
 *   Raw anchor    — three YOLO decode heads (80×80, 40×40, 20×20)
 *
 * Pre-processing (resize + normalize to [0,1]) runs on CPU before DMAing the
 * buffer to the NPU.  For raw-anchor HEFs, NMS also runs on CPU.
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
    std::unique_ptr<hailort::VDevice>                  vdevice_;
    std::shared_ptr<hailort::ConfiguredNetworkGroup>   network_group_;
    std::vector<hailort::InputVStream>                 input_streams_;
    std::vector<hailort::OutputVStream>                output_streams_;

    VisionConfig config_;

    // Model input dimensions (read from HEF at init time)
    int model_input_w_{640};
    int model_input_h_{640};

    // COCO class names — kept in sync with yolo_detector.cpp
    static const std::vector<std::string> kCocoClassNames;
    int target_class_id_{-1};

    // NMS output mode (set at initialize time by inspecting format.order)
    bool is_nms_output_{false};
    int  nms_num_classes_{80};
    int  nms_max_bboxes_{100};

    /**
     * @brief Decode Hailo NMS output (HAILO_FORMAT_ORDER_HAILO_NMS).
     *
     * Layout per class: [float32 count, y_min, x_min, y_max, x_max, score, ...]
     * Coordinates are normalised to [0, 1].
     */
    std::vector<Detection> postprocessNMS(
        const std::vector<float>& raw,
        int frame_width,
        int frame_height
    );

    /**
     * @brief Decode raw per-anchor output (three YOLO scale heads).
     */
    std::vector<Detection> postprocess(
        const std::vector<std::vector<float>>& raw_outputs,
        const std::vector<std::pair<int,int>>&  grid_sizes,
        int frame_width,
        int frame_height
    );
};

} // namespace tracker
