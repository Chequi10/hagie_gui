#pragma once
#include <atomic>
#include <memory>
#include <string>
#include "ai/tassel_detector.h"

// CPU backend used by the same inference worker as TensorRT.
class OnnxTasselDetector {
public:
    OnnxTasselDetector();
    ~OnnxTasselDetector();
    bool initialize(const std::string& python, const std::string& helper,
                    const std::string& model, const std::atomic<bool>& running);
    bool processFrame(std::size_t camera, const RgbFrameSource::Frame& frame,
                      TasselDetector::Result& result, float confidence, float iou,
                      const std::atomic<bool>& running);
    std::string error() const;
    double inferenceMs() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
