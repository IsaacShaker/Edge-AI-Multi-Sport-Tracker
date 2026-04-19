#include "../include/vision/hailo_detector.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <opencv2/opencv.hpp>

namespace tracker {

// Shared with yolo_detector.cpp — same COCO 80 class list
const std::vector<std::string> HailoDetector::kCocoClassNames = {
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

// ── initialize ────────────────────────────────────────────────────────────────

bool HailoDetector::initialize(const Config& config) {
    config_ = static_cast<const VisionConfig&>(config);

    // Resolve target class index for fast per-anchor filtering
    target_class_id_ = -1;
    for (int i = 0; i < static_cast<int>(kCocoClassNames.size()); ++i) {
        if (kCocoClassNames[i] == config_.target_label) {
            target_class_id_ = i;
            break;
        }
    }

    if (config_.model_path.empty()) {
        std::cerr << "[HailoDetector] No HEF model path specified" << std::endl;
        return false;
    }

    // ── Create virtual device (connects to the first Hailo-8/8L on the bus) ──
    auto vdevice_result = hailort::VDevice::create();
    if (!vdevice_result) {
        std::cerr << "[HailoDetector] Failed to create VDevice: "
                  << vdevice_result.status() << std::endl;
        return false;
    }
    vdevice_ = vdevice_result.release();

    // ── Load HEF ──────────────────────────────────────────────────────────────
    auto hef_result = hailort::Hef::create(config_.model_path);
    if (!hef_result) {
        std::cerr << "[HailoDetector] Failed to load HEF '" << config_.model_path
                  << "': " << hef_result.status() << std::endl;
        return false;
    }
    auto hef = hef_result.release();

    // ── Configure network group ────────────────────────────────────────────────
    auto configure_params = vdevice_->create_configure_params(hef);
    if (!configure_params) {
        std::cerr << "[HailoDetector] Failed to create configure params: "
                  << configure_params.status() << std::endl;
        return false;
    }

    auto network_groups_result = vdevice_->configure(hef, configure_params.value());
    if (!network_groups_result || network_groups_result->empty()) {
        std::cerr << "[HailoDetector] Failed to configure network group" << std::endl;
        return false;
    }
    network_group_ = network_groups_result.value()[0];

    // ── Create vstreams ────────────────────────────────────────────────────────
    auto input_params  = network_group_->make_input_vstream_params(
        false, HAILO_FORMAT_TYPE_FLOAT32,
        HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
    auto output_params = network_group_->make_output_vstream_params(
        false, HAILO_FORMAT_TYPE_FLOAT32,
        HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);

    if (!input_params || !output_params) {
        std::cerr << "[HailoDetector] Failed to create vstream params" << std::endl;
        return false;
    }

    auto input_streams_result  = hailort::VStreamsBuilder::create_input_vstreams(
        *network_group_, input_params.value());
    auto output_streams_result = hailort::VStreamsBuilder::create_output_vstreams(
        *network_group_, output_params.value());

    if (!input_streams_result || !output_streams_result) {
        std::cerr << "[HailoDetector] Failed to create vstreams" << std::endl;
        return false;
    }

    input_streams_  = input_streams_result.release();
    output_streams_ = output_streams_result.release();

    // Read actual input dimensions from HEF
    if (!input_streams_.empty()) {
        const auto& shape = input_streams_[0].get_info().shape;
        model_input_h_ = static_cast<int>(shape.height);
        model_input_w_ = static_cast<int>(shape.width);
    }

    // Detect output format: NMS-embedded vs raw anchor grids.
    // HAILO_FORMAT_ORDER_HAILO_NMS means the HEF already ran box decoding +
    // NMS and outputs detections organised by class.  This is the format used
    // by all standard Hailo Model Zoo detection HEFs (e.g. yolov8n.hef).
    if (!output_streams_.empty()) {
        const auto& info = output_streams_[0].get_info();
        if (info.format.order == HAILO_FORMAT_ORDER_HAILO_NMS) {
            is_nms_output_   = true;
            nms_num_classes_ = static_cast<int>(info.nms_shape.number_of_classes);
            nms_max_bboxes_  = static_cast<int>(info.nms_shape.max_bboxes_per_class);
            std::cout << "[HailoDetector] Output format : NMS (" << nms_num_classes_
                      << " classes, " << nms_max_bboxes_ << " max bboxes/class)" << std::endl;
        } else {
            std::cout << "[HailoDetector] Output format : raw anchor grids" << std::endl;
        }
    }

    std::cout << "[HailoDetector] HEF loaded: " << config_.model_path << std::endl;
    std::cout << "[HailoDetector] Input size : "
              << model_input_w_ << "x" << model_input_h_ << std::endl;
    std::cout << "[HailoDetector] Target     : '" << config_.target_label << "'" << std::endl;

    ready_ = true;
    return true;
}

// ── detect ────────────────────────────────────────────────────────────────────

std::vector<Detection> HailoDetector::detect(
    const void* frame_data,
    int width,
    int height
) {
    if (!ready_) return {};

    // ── Pre-process: resize to model input size, normalise to [0,1] float32 ──
    cv::Mat frame(height, width, CV_8UC3, const_cast<void*>(frame_data));
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(model_input_w_, model_input_h_));

    // Convert BGR uint8 → RGB float32 [0,1] in HWC layout
    cv::Mat rgb_f32;
    cv::cvtColor(resized, rgb_f32, cv::COLOR_BGR2RGB);
    rgb_f32.convertTo(rgb_f32, CV_32F, 1.0 / 255.0);

    // Flatten to a contiguous float buffer
    std::vector<float> input_data(
        reinterpret_cast<const float*>(rgb_f32.datastart),
        reinterpret_cast<const float*>(rgb_f32.dataend));

    // ── Write input to NPU ────────────────────────────────────────────────────
    auto write_status = input_streams_[0].write(
        hailort::MemoryView(input_data.data(),
                            input_data.size() * sizeof(float)));
    if (write_status != HAILO_SUCCESS) {
        std::cerr << "[HailoDetector] write failed: " << write_status << std::endl;
        return {};
    }

    // ── Read all output heads ─────────────────────────────────────────────────
    // Use get_frame_size() for buffer allocation — shape fields are not valid
    // for NMS-format outputs (they use nms_shape instead).
    std::vector<std::vector<float>> raw_outputs;
    std::vector<std::pair<int,int>>  grid_sizes;

    for (auto& out_stream : output_streams_) {
        const auto&  info        = out_stream.get_info();
        const size_t frame_bytes = out_stream.get_frame_size();
        std::vector<float> buf(frame_bytes / sizeof(float));

        auto read_status = out_stream.read(
            hailort::MemoryView(buf.data(), frame_bytes));
        if (read_status != HAILO_SUCCESS) {
            std::cerr << "[HailoDetector] read failed: " << read_status << std::endl;
            return {};
        }

        raw_outputs.push_back(std::move(buf));
        grid_sizes.emplace_back(
            static_cast<int>(info.shape.height),
            static_cast<int>(info.shape.width));
    }

    // Route to the correct postprocessor based on HEF output format
    if (is_nms_output_) {
        return postprocessNMS(raw_outputs[0], width, height);
    }
    return postprocess(raw_outputs, grid_sizes, width, height);
}

// ── shutdown ──────────────────────────────────────────────────────────────────

void HailoDetector::shutdown() {
    input_streams_.clear();
    output_streams_.clear();
    network_group_.reset();
    vdevice_.reset();
    ready_ = false;
}

// ── postprocessNMS ────────────────────────────────────────────────────────────
// Decodes HAILO_FORMAT_ORDER_HAILO_NMS output (NMS already performed in HEF).
//
// Buffer layout per class (float32):
//   [count]  [y_min, x_min, y_max, x_max, score] × max_bboxes
//
// Coordinates are normalised to [0, 1] relative to model input.

std::vector<Detection> HailoDetector::postprocessNMS(
    const std::vector<float>& raw,
    int frame_width,
    int frame_height
) {
    const int stride = 1 + nms_max_bboxes_ * 5; // float32 slots per class

    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    std::vector<Detection> detections;

    for (int c = 0; c < nms_num_classes_; ++c) {
        if (target_class_id_ >= 0 && c != target_class_id_) continue;

        const float* cls_ptr = raw.data() + c * stride;
        const int    count   = static_cast<int>(cls_ptr[0]);
        if (count <= 0) continue;

        const float* bbox = cls_ptr + 1;
        for (int b = 0; b < std::min(count, nms_max_bboxes_); ++b, bbox += 5) {
            const float score = bbox[4];
            if (score < config_.confidence_threshold) continue;

            // Hailo NMS order: y_min, x_min, y_max, x_max (normalised [0,1])
            const float x1 = bbox[1] * static_cast<float>(frame_width);
            const float y1 = bbox[0] * static_cast<float>(frame_height);
            const float x2 = bbox[3] * static_cast<float>(frame_width);
            const float y2 = bbox[2] * static_cast<float>(frame_height);

            Detection det;
            det.bbox.x       = (x1 + x2) / 2.0f;
            det.bbox.y       = (y1 + y2) / 2.0f;
            det.bbox.width   = x2 - x1;
            det.bbox.height  = y2 - y1;
            det.center.x     = det.bbox.x;
            det.center.y     = det.bbox.y;
            det.radius       = std::min(det.bbox.width, det.bbox.height) / 2.0f;
            det.confidence   = score;
            det.has_bbox     = true;
            det.timestamp_ms = now_ms;
            det.label        = (c >= 0 && c < static_cast<int>(kCocoClassNames.size()))
                               ? kCocoClassNames[c] : config_.target_label;
            detections.push_back(det);
        }
    }

    return detections;
}

// ── postprocess ───────────────────────────────────────────────────────────────
// Each Hailo output vstream for a YOLOv8 HEF has shape [grid_h, grid_w, 4+classes].
// We flatten all three scale heads, run NMS, and return Detections.

std::vector<Detection> HailoDetector::postprocess(
    const std::vector<std::vector<float>>& raw_outputs,
    const std::vector<std::pair<int,int>>&  grid_sizes,
    int frame_width,
    int frame_height
) {
    const float x_scale = static_cast<float>(frame_width)  / model_input_w_;
    const float y_scale = static_cast<float>(frame_height) / model_input_h_;
    const int   num_classes = static_cast<int>(kCocoClassNames.size()); // 80

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      class_ids;

    for (size_t head = 0; head < raw_outputs.size(); ++head) {
        const auto& buf    = raw_outputs[head];
        const int   grid_h = grid_sizes[head].first;
        const int   grid_w = grid_sizes[head].second;
        const int   stride_h = model_input_h_ / grid_h;
        const int   stride_w = model_input_w_ / grid_w;
        const int   num_feat  = 4 + num_classes; // per anchor

        const int num_anchors = grid_h * grid_w;
        if (static_cast<int>(buf.size()) != num_anchors * num_feat) continue;

        for (int i = 0; i < num_anchors; ++i) {
            const float* row = buf.data() + i * num_feat;

            // YOLOv8 decode: cx,cy are relative to grid cell, wh are absolute in model coords
            const int gy = i / grid_w;
            const int gx = i % grid_w;

            const float cx = (row[0] + gx) * stride_w;
            const float cy = (row[1] + gy) * stride_h;
            const float bw = row[2] * model_input_w_;
            const float bh = row[3] * model_input_h_;

            float max_score = 0.0f;
            int   best_class = -1;

            if (target_class_id_ >= 0) {
                max_score  = row[4 + target_class_id_];
                best_class = target_class_id_;
            } else {
                for (int c = 0; c < num_classes; ++c) {
                    if (row[4 + c] > max_score) {
                        max_score  = row[4 + c];
                        best_class = c;
                    }
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
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences,
                      config_.confidence_threshold, 0.45f,
                      nms_indices);

    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    std::vector<Detection> detections;
    detections.reserve(nms_indices.size());

    for (int idx : nms_indices) {
        const cv::Rect& box = boxes[idx];
        Detection det;
        det.bbox.x       = box.x + box.width  / 2.0f;
        det.bbox.y       = box.y + box.height / 2.0f;
        det.bbox.width   = static_cast<float>(box.width);
        det.bbox.height  = static_cast<float>(box.height);
        det.center.x     = det.bbox.x;
        det.center.y     = det.bbox.y;
        det.radius       = std::min(box.width, box.height) / 2.0f;
        det.confidence   = confidences[idx];
        det.has_bbox     = true;
        det.timestamp_ms = now_ms;
        const int cid    = class_ids[idx];
        det.label = (cid >= 0 && cid < static_cast<int>(kCocoClassNames.size()))
                    ? kCocoClassNames[cid]
                    : config_.target_label;
        detections.push_back(det);
    }

    return detections;
}

} // namespace tracker
