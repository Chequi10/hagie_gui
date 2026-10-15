#include "ai/yolo_inference_worker.h"
#include "vision/video_rgb_frame_source.h"
#include <cassert>
#include <functional>
#include <chrono>
#include <thread>

static void waitUntil(const std::function<bool()>& condition) {
    const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!condition() && std::chrono::steady_clock::now() < limit)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(condition());
}
int main() {
    RgbCameraWorker cameras;
    auto cache = std::make_shared<VideoRgbFrameSource::Cache>();
    cameras.setFrameSource(0, std::make_unique<VideoRgbFrameSource>(0, cache));
    cameras.start();
    YoloInferenceWorker worker(cameras, nullptr);
    assert(worker.initializeOnnx("model", "python", "helper", 0, 0.1f, 0.45f));
    worker.setFrameInterval(2);
    assert(worker.start());
    RgbFrameSource::Frame frame;
    frame.width = 2; frame.height = 2; frame.data.assign(12, 42); frame.valid = true;
    auto publish = [&](std::uint64_t timestamp) {
        frame.timestamp_ms = timestamp; cache->publish(frame);
        waitUntil([&]() { return cache->isConsumed(); });
    };
    publish(100);
    YoloInferenceWorker::Result result;
    assert(!worker.getLatestResult(0, result));
    publish(101);
    assert(worker.getLatestResult(0, result));
    assert(result.rgbOnly && !result.pointCloudValid && result.pointCloud.empty());
    assert(result.rgbFrame.timestamp_ms == result.detectionResult.timestamp_ms);
    assert(result.detectionResult.timestamp_ms == 101 && result.rgbFrame.data == frame.data);
    assert(result.detectionResult.detections[0].confidence == 0.1f);
    assert(!result.detectionResult.detections[0].position_3d_valid);
    assert(!worker.getLatestResult(1, result));
    worker.setThresholds(0.6f, 0.3f);
    publish(102); publish(103);
    assert(worker.getLatestResult(0, result) && result.detectionResult.detections[0].confidence == 0.6f);
    worker.stop();
    assert(worker.initializeOnnx("model", "fail", "helper", 0, 0.25f, 0.45f));
    assert(!worker.getLatestResult(0, result));
    worker.start();
    waitUntil([&]() { return !worker.error().empty(); });
    worker.stop();
    assert(worker.initializeOnnx("model", "block", "helper", 0, 0.25f, 0.45f));
    worker.start();
    const auto start = std::chrono::steady_clock::now();
    worker.stop();
    assert(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
    cameras.stop();
}
