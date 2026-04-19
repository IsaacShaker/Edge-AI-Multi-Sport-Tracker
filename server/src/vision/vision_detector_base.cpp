// Stub implementation - to be completed
#include "../include/vision/vision_detector_base.h"
#include <chrono>
#include <algorithm>

namespace tracker {

cv::Mat VisionDetectorBase::frameToMat(const void* frame_data, int width, int height) const {
    return cv::Mat(height, width, CV_8UC3, const_cast<void*>(frame_data));
}

cv::Mat VisionDetectorBase::prepareInput(const void* frame_data, int width, int height,
                                          int model_w, int model_h) const {
    cv::Mat frame(height, width, CV_8UC3, const_cast<void*>(frame_data));
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(model_w, model_h));
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    return rgb; // uint8, HWC, [0,255]
}

std::vector<Detection> VisionDetectorBase::buildDetections(
    const std::vector<cv::Rect>&    boxes,
    const std::vector<float>&       confidences,
    const std::vector<int>&         class_ids,
    const std::vector<std::string>& class_names,
    const std::string&              target_label) const
{
    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    std::vector<Detection> detections;
    detections.reserve(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) {
        const cv::Rect& box = boxes[i];
        Detection det;
        det.bbox.x       = box.x + box.width  / 2.0f;
        det.bbox.y       = box.y + box.height / 2.0f;
        det.bbox.width   = static_cast<float>(box.width);
        det.bbox.height  = static_cast<float>(box.height);
        det.center.x     = det.bbox.x;
        det.center.y     = det.bbox.y;
        det.radius       = std::min(box.width, box.height) / 2.0f;
        det.confidence   = confidences[i];
        det.has_bbox     = true;
        det.timestamp_ms = now_ms;
        const int cid    = class_ids[i];
        det.label = (cid >= 0 && cid < static_cast<int>(class_names.size()))
                    ? class_names[cid] : target_label;
        detections.push_back(det);
    }
    return detections;
}

} // namespace tracker
