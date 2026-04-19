#pragma once

#include "vision_detector_base.h"
#include "../factories/vision_factory.h"
#include <opencv2/opencv.hpp>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <memory>

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

    Ort::Env                            env_{ORT_LOGGING_LEVEL_WARNING, "yolo"};
    Ort::SessionOptions                 session_options_;
    std::unique_ptr<Ort::Session>       session_;
    Ort::AllocatorWithDefaultOptions    allocator_;

    std::vector<std::string>            input_names_owned_;
    std::vector<std::string>            output_names_owned_;
    std::vector<const char*>            input_names_;
    std::vector<const char*>            output_names_;

    int64_t model_input_w_{640};
    int64_t model_input_h_{640};

    std::vector<std::string> class_names_;

    std::vector<Detection> postprocess(
        const float*  output_data,
        int64_t       num_features,
        int64_t       num_anchors,
        int           frame_width,
        int           frame_height
    );
};

} // namespace tracker
