/**
 * hailo_detector.cpp
 *
 * Uses the HailoRT high-level InferModel API — the same API used by the
 * official picamera2 Hailo class and all Raspberry Pi AI Kit examples.
 *
 * The old VStreams API (VStreamsBuilder + InputVStream/OutputVStream) was
 * replaced because it consistently returns empty output buffers on HailoRT
 * 4.23 when the HEF uses the NMS_BY_CLASS output format.
 *
 * Flow mirrors picamera2/devices/hailo/hailo.py:
 *   VDevice::create_infer_model() → InferModel::configure()
 *   → ConfiguredInferModel::create_bindings() → run() per frame
 */

#include "../include/vision/hailo_detector.h"
#include <iostream>
#include <algorithm>
#include <sys/mman.h>
#include <cstring>
#include <opencv2/opencv.hpp>

// Mirrors page_aligned_alloc() from the official Hailo Application Code Examples
// (hailo_infer.cpp). HailoRT DMA requires output buffers to be PAGE_SIZE-aligned;
// using a plain std::vector causes silent write failures (all-zero output).
static std::shared_ptr<uint8_t> page_aligned_alloc(size_t size) {
    auto addr = mmap(nullptr, size, PROT_WRITE | PROT_READ,
                     MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
    if (MAP_FAILED == addr) throw std::bad_alloc();
    return std::shared_ptr<uint8_t>(
        reinterpret_cast<uint8_t*>(addr),
        [size](void* p){ munmap(p, size); }
    );
}

namespace tracker {

// ── COCO 80 class names ───────────────────────────────────────────────────────

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

    // Resolve target class index
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

    // ── Create VDevice (default params include round-robin scheduler) ──────────
    auto vdevice_result = hailort::VDevice::create();
    if (!vdevice_result) {
        std::cerr << "[HailoDetector] Failed to create VDevice: "
                  << vdevice_result.status() << std::endl;
        return false;
    }
    vdevice_ = vdevice_result.release();

    // ── create_infer_model (high-level API — mirrors picamera2 Hailo class) ───
    auto infer_model_result = vdevice_->create_infer_model(config_.model_path);
    if (!infer_model_result) {
        std::cerr << "[HailoDetector] Failed to create InferModel from '"
                  << config_.model_path << "': "
                  << infer_model_result.status() << std::endl;
        return false;
    }
    infer_model_ = infer_model_result.release();
    infer_model_->set_batch_size(1);

    // Read input shape from InferModel's input stream BEFORE configure()
    {
        auto in_shape = infer_model_->input()->shape();
        model_input_h_ = static_cast<int>(in_shape.height);
        model_input_w_ = static_cast<int>(in_shape.width);
    }

    // Read NMS shape and per-output buffer sizes BEFORE configure()
    {
        auto nms_shape_result = infer_model_->output()->get_nms_shape();
        if (nms_shape_result) {
            nms_num_classes_ = static_cast<int>(nms_shape_result->number_of_classes);
            nms_max_bboxes_  = static_cast<int>(nms_shape_result->max_bboxes_per_class);
        }
        output_buf_size_ = infer_model_->output()->get_frame_size();
    }
    output_buf_ptr_ = page_aligned_alloc(output_buf_size_);
    std::memset(output_buf_ptr_.get(), 0, output_buf_size_);

    // Match input format to what the HEF expects (usually UINT8)
    infer_model_->input()->set_format_type(infer_model_->input()->format().type);

    // Request FLOAT32 outputs — same as picamera2
    // output() returns a mutable Expected<InferStream> on the InferModel
    for (const auto& name : infer_model_->get_output_names()) {
        infer_model_->output(name)->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);
    }

    // ── configure() ───────────────────────────────────────────────────────────
    auto configured_result = infer_model_->configure();
    if (!configured_result) {
        std::cerr << "[HailoDetector] Failed to configure InferModel: "
                  << configured_result.status() << std::endl;
        return false;
    }
    configured_model_ = std::move(configured_result.value());

    std::cout << "[HailoDetector] HEF loaded   : " << config_.model_path << std::endl;
    std::cout << "[HailoDetector] Input size   : "
              << model_input_w_ << "x" << model_input_h_ << std::endl;
    std::cout << "[HailoDetector] NMS classes  : " << nms_num_classes_
              << "  max_bboxes/class: " << nms_max_bboxes_ << std::endl;
    std::cout << "[HailoDetector] Target label : '" << config_.target_label
              << "'  (class id " << target_class_id_ << ")" << std::endl;

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

    // Resize + BGR→RGB, raw uint8 (HEF bakes normalisation)
    cv::Mat rgb = prepareInput(frame_data, width, height, model_input_w_, model_input_h_);

    // Build bindings with our pre-allocated output buffer
    const auto& out_name = infer_model_->get_output_names()[0];
    std::map<std::string, hailort::MemoryView> buf_map;
    buf_map[out_name] = hailort::MemoryView(output_buf_ptr_.get(), output_buf_size_);

    auto bindings_result = configured_model_.create_bindings(buf_map);
    if (!bindings_result) {
        std::cerr << "[HailoDetector] create_bindings failed: "
                  << bindings_result.status() << std::endl;
        return {};
    }
    auto& bindings = bindings_result.value();

    // Set input buffer
    auto status = bindings.input()->set_buffer(
        hailort::MemoryView(rgb.data,
                            static_cast<size_t>(rgb.total() * rgb.elemSize())));
    if (status != HAILO_SUCCESS) {
        std::cerr << "[HailoDetector] set_buffer (input) failed: " << status << std::endl;
        return {};
    }

    // Synchronous inference (mirrors picamera2 Hailo.run())
    status = configured_model_.run(bindings, std::chrono::milliseconds(1000));
    if (status != HAILO_SUCCESS) {
        std::cerr << "[HailoDetector] run() failed: " << status << std::endl;
        return {};
    }

    return postprocessNMS(width, height);
}

// ── shutdown ──────────────────────────────────────────────────────────────────

void HailoDetector::shutdown() {
    if (ready_) {
        configured_model_.shutdown();
    }
    infer_model_.reset();
    vdevice_.reset();
    ready_ = false;
}

// ── postprocessNMS ────────────────────────────────────────────────────────────
// HAILO_FORMAT_ORDER_HAILO_NMS_BY_CLASS (format order 22) layout (FLOAT32):
//   Packed: for each class c in [0, num_classes):
//     [float32 bbox_count][hailo_bbox_float32_t × bbox_count]
//   Each hailo_bbox_float32_t = {y_min, x_min, y_max, x_max, score} (5 floats).
//   Layout is variable-stride/packed — NOT padded to max_bboxes per class.
//   This matches parse_nms_data() in the Hailo Application Code Examples utils.cpp.

std::vector<Detection> HailoDetector::postprocessNMS(
    int frame_width,
    int frame_height
) {
    const uint8_t* data   = output_buf_ptr_.get();
    size_t         offset = 0;

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      class_ids;

    for (int c = 0; c < nms_num_classes_; ++c) {
        // Read detection count as float32 (packed, variable-stride layout)
        float32_t count_f = 0.0f;
        std::memcpy(&count_f, data + offset, sizeof(float32_t));
        offset += sizeof(float32_t);
        const int count = static_cast<int>(count_f);

        for (int b = 0; b < count; ++b) {
            hailo_bbox_float32_t bbox{};
            std::memcpy(&bbox, data + offset, sizeof(hailo_bbox_float32_t));
            offset += sizeof(hailo_bbox_float32_t);

            if (target_class_id_ >= 0 && c != target_class_id_) continue;
            if (bbox.score < config_.confidence_threshold) continue;

            // NMS coordinates are normalised [0,1]: y_min, x_min, y_max, x_max
            const int x1 = static_cast<int>(bbox.x_min * frame_width);
            const int y1 = static_cast<int>(bbox.y_min * frame_height);
            const int x2 = static_cast<int>(bbox.x_max * frame_width);
            const int y2 = static_cast<int>(bbox.y_max * frame_height);
            boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
            confidences.push_back(bbox.score);
            class_ids.push_back(c);
        }
    }

    return buildDetections(boxes, confidences, class_ids, kCocoClassNames, config_.target_label);
}

} // namespace tracker
