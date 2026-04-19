#include "../include/vision/yolo_detector.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <numeric>

namespace tracker {

static const std::vector<std::string> kCocoClassNames = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
};

bool YOLODetector::initialize(const Config& config) {
    config_ = static_cast<const VisionConfig&>(config);

    if (config_.model_path.empty()) {
        std::cerr << "[YOLODetector] No model path specified" << std::endl;
        return false;
    }

    try {
        session_options_.SetIntraOpNumThreads(2);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        session_ = std::make_unique<Ort::Session>(
            env_, config_.model_path.c_str(), session_options_);

        // ── Input metadata ────────────────────────────────────────────────
        const size_t num_inputs = session_->GetInputCount();
        input_names_owned_.reserve(num_inputs);
        input_names_.reserve(num_inputs);
        for (size_t i = 0; i < num_inputs; ++i) {
            auto name = session_->GetInputNameAllocated(i, allocator_);
            input_names_owned_.push_back(name.get());
            input_names_.push_back(input_names_owned_.back().c_str());
        }

        // Read model input spatial size from the first input shape
        auto input_info = session_->GetInputTypeInfo(0);
        auto shape = input_info.GetTensorTypeAndShapeInfo().GetShape();
        // shape: [1, 3, H, W]  (-1 for dynamic dims)
        if (shape.size() == 4) {
            model_input_h_ = (shape[2] > 0) ? shape[2] : 640;
            model_input_w_ = (shape[3] > 0) ? shape[3] : 640;
        }

        // ── Output metadata ───────────────────────────────────────────────
        const size_t num_outputs = session_->GetOutputCount();
        output_names_owned_.reserve(num_outputs);
        output_names_.reserve(num_outputs);
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name = session_->GetOutputNameAllocated(i, allocator_);
            output_names_owned_.push_back(name.get());
            output_names_.push_back(output_names_owned_.back().c_str());
        }

    } catch (const Ort::Exception& e) {
        std::cerr << "[YOLODetector] Failed to load model '" << config_.model_path
                  << "': " << e.what() << std::endl;
        return false;
    }

    class_names_ = kCocoClassNames;

    std::cout << "[YOLODetector] Model loaded : " << config_.model_path << std::endl;
    std::cout << "[YOLODetector] Input size   : "
              << model_input_w_ << "x" << model_input_h_ << std::endl;
    std::cout << "[YOLODetector] Target label : '" << config_.target_label << "'" << std::endl;

    ready_ = true;
    return true;
}

std::vector<Detection> YOLODetector::detect(
    const void* frame_data,
    int width,
    int height
) {
    if (!ready_) return {};

    // ── Pre-process: resize, BGR→RGB, uint8→float32 [0,1], HWC→CHW ─────────
    cv::Mat frame(height, width, CV_8UC3, const_cast<void*>(frame_data));
    cv::Mat resized;
    cv::resize(frame, resized,
               cv::Size(static_cast<int>(model_input_w_),
                        static_cast<int>(model_input_h_)));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    // Build CHW float32 buffer
    const int64_t H = model_input_h_, W = model_input_w_, C = 3;
    std::vector<float> input_tensor(C * H * W);
    for (int c = 0; c < C; ++c)
        for (int r = 0; r < H; ++r)
            for (int col = 0; col < W; ++col)
                input_tensor[c * H * W + r * W + col] =
                    rgb.at<cv::Vec3b>(r, col)[c] / 255.0f;

    // ── Run inference ─────────────────────────────────────────────────────────
    std::array<int64_t, 4> input_shape{1, C, H, W};
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_ort = Ort::Value::CreateTensor<float>(
        mem_info,
        input_tensor.data(), input_tensor.size(),
        input_shape.data(), input_shape.size());

    std::vector<Ort::Value> outputs;
    try {
        outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_.data(),  &input_ort, 1,
            output_names_.data(), output_names_.size());
    } catch (const Ort::Exception& e) {
        std::cerr << "[YOLODetector] Inference error: " << e.what() << std::endl;
        return {};
    }

    if (outputs.empty()) return {};

    // ── Post-process ──────────────────────────────────────────────────────────
    // YOLOv8/YOLO11 ONNX output: [1, num_features, num_anchors]
    //   num_features = 4 + num_classes
    //   num_anchors  = 8400 for 640 input
    const auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    const int64_t num_features = out_shape[1];
    const int64_t num_anchors  = out_shape[2];
    const float*  out_data     = outputs[0].GetTensorMutableData<float>();

    return postprocess(out_data, num_features, num_anchors, width, height);
}

void YOLODetector::shutdown() {
    session_.reset();
    ready_ = false;
}

std::vector<Detection> YOLODetector::postprocess(
    const float*  data,        // [num_features, num_anchors] row-major
    int64_t       num_features,
    int64_t       num_anchors,
    int           frame_width,
    int           frame_height
) {
    const int64_t num_classes = num_features - 4;
    const float x_scale = static_cast<float>(frame_width)  / model_input_w_;
    const float y_scale = static_cast<float>(frame_height) / model_input_h_;

    // Resolve target class index (-1 = accept any)
    int target_class_id = -1;
    for (int i = 0; i < static_cast<int>(class_names_.size()); ++i) {
        if (class_names_[i] == config_.target_label) {
            target_class_id = i;
            break;
        }
    }

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      class_ids;

    // Output is stored as [num_features][num_anchors] — need to transpose access
    for (int64_t a = 0; a < num_anchors; ++a) {
        const float cx = data[0 * num_anchors + a];
        const float cy = data[1 * num_anchors + a];
        const float bw = data[2 * num_anchors + a];
        const float bh = data[3 * num_anchors + a];

        float max_score = 0.0f;
        int   best_class = -1;

        if (target_class_id >= 0 && target_class_id < num_classes) {
            max_score  = data[(4 + target_class_id) * num_anchors + a];
            best_class = target_class_id;
        } else {
            for (int64_t c = 0; c < num_classes; ++c) {
                float s = data[(4 + c) * num_anchors + a];
                if (s > max_score) { max_score = s; best_class = static_cast<int>(c); }
            }
        }

        if (max_score < config_.confidence_threshold) continue;

        const int x1 = static_cast<int>((cx - bw / 2.0f) * x_scale);
        const int y1 = static_cast<int>((cy - bh / 2.0f) * y_scale);
        boxes.emplace_back(x1, y1,
                           static_cast<int>(bw * x_scale),
                           static_cast<int>(bh * y_scale));
        confidences.push_back(max_score);
        class_ids.push_back(best_class);
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences,
                      config_.confidence_threshold, 0.45f,
                      nms_indices);

    // Gather NMS survivors into flat vectors then delegate to base helper
    std::vector<cv::Rect> nms_boxes;
    std::vector<float>    nms_confs;
    std::vector<int>      nms_ids;
    nms_boxes.reserve(nms_indices.size());
    nms_confs.reserve(nms_indices.size());
    nms_ids.reserve(nms_indices.size());
    for (int idx : nms_indices) {
        nms_boxes.push_back(boxes[idx]);
        nms_confs.push_back(confidences[idx]);
        nms_ids.push_back(class_ids[idx]);
    }
    return buildDetections(nms_boxes, nms_confs, nms_ids, class_names_, config_.target_label);
}

} // namespace tracker
