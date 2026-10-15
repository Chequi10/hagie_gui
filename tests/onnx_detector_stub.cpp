// Test double: exercise the actual worker without a Python model or Qt.
#include "ai/onnx_tassel_detector.h"
#include <chrono>
#include <thread>
struct OnnxTasselDetector::Impl { std::string failure; };
OnnxTasselDetector::OnnxTasselDetector() : impl(std::make_unique<Impl>()) {}
OnnxTasselDetector::~OnnxTasselDetector() = default;
bool OnnxTasselDetector::initialize(const std::string& python, const std::string&,
                                   const std::string&, const std::atomic<bool>& running) {
    if (python == "block") {
        while (running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        impl->failure = "stopped";
        return false;
    }
    if (python == "fail") { impl->failure = "load failure"; return false; }
    return true;
}
bool OnnxTasselDetector::processFrame(std::size_t camera, const RgbFrameSource::Frame& frame,
                                     TasselDetector::Result& result, float confidence, float,
                                     const std::atomic<bool>& running) {
    if (!running.load()) return false;
    result = {};
    result.camera_index = camera; result.timestamp_ms = frame.timestamp_ms;
    result.image_width = frame.width; result.image_height = frame.height; result.valid = true;
    TasselDetector::Detection detection;
    detection.x = 1; detection.y = 1; detection.width = 1; detection.height = 1;
    detection.confidence = confidence;
    result.detections.push_back(detection);
    return true;
}
std::string OnnxTasselDetector::error() const { return impl->failure; }
double OnnxTasselDetector::inferenceMs() const { return 10; }
