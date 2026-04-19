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
 * Expected HEF model: yolov8n_sports_ball (COCO, 640×640 input)
 * Output layers: the three YOLO decode heads (80×80, 40×40, 20×20).
 *
 * Pre-processing (resize + normalize to [0,1]) is done here in CPU before
 * DMAing the buffer to the NPU.  Post-processing (NMS) runs on CPU using
 * the same logic as YOLODetector so results are identical regardless of
 * which backend is active.
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

    /**
     * @brief Run NMS on raw per-anchor scores and boxes to build Detections.
     *
     * Each output vstream from a YOLOv8 HEF carries a flat float32 buffer
     * whose layout is [num_anchors, 4 + num_classes] (same as ONNX before
     * transpose).  This function decodes all three scale heads and merges them.
     */
    std::vector<Detection> postprocess(
        const std::vector<std::vector<float>>& raw_outputs,
        const std::vector<std::pair<int,int>>&  grid_sizes,
        int frame_width,
        int frame_height
    );
};

} // namespace tracker
