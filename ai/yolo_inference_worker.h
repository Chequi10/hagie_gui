#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <string>

#include "ai/tassel_detector.h"
#include "ai/tensorrt_tassel_detector.h"
#include "vision/rgb_camera_worker.h"
#include "vision/vision_3d_processor.h"

class Vision3DWorker;

class YoloInferenceWorker
{
public:

    static constexpr std::size_t CAMERA_COUNT =
        RgbCameraWorker::CAMERA_COUNT;

    struct Result
    {
        TasselDetector::Result detectionResult;
        RgbFrameSource::Frame rgbFrame;
        bool rgbOnly = false;
        double inferenceMs = 0;

        Vision3DProcessor::PointCloud pointCloud;

        std::uint64_t pointCloudTimestampMs =
            0;

        bool pointCloudValid =
            false;
    };    


    YoloInferenceWorker(
        RgbCameraWorker& rgbCameraWorker,
        Vision3DWorker *vision3DWorker
    );

    ~YoloInferenceWorker();


    bool initialize(
        const char* enginePath,
        TensorRtTasselDetector::ModelOutputFormat outputFormat =
            TensorRtTasselDetector::ModelOutputFormat::Auto,
        float confidenceThreshold = 0.25f,
        float nmsThreshold = 0.45f
    );


    bool initializeOnnx(const std::string& model, const std::string& python,
                        const std::string& helper, std::size_t camera,
                        float confidence, float nms);
    void setThresholds(float confidence, float nms);
    std::string error() const;

    bool start();

    void stop();

    bool isRunning() const;

    void setFrameInterval(
        std::size_t interval
    );


    bool getLatestResult(
        std::size_t cameraIndex,
        Result& result
    ) const;


private:

    void workerLoop();
    void resetResults();
    bool useOnnx = false;
    std::size_t onnxCamera = 0;
    std::string onnxModel, onnxPython, onnxHelper;
    std::string lastError;
    std::atomic<float> confidence {0.25f};
    std::atomic<float> nms {0.45f};


    RgbCameraWorker& rgbCameraWorker;

    Vision3DWorker *vision3DWorker =
    nullptr;

    TensorRtTasselDetector detector;


    std::atomic<bool> running {
        false
    };


    std::thread workerThread;


    mutable std::mutex resultMutex;


    std::array<
        Result,
        CAMERA_COUNT
    > latestResults;


    std::array<
        std::uint64_t,
        CAMERA_COUNT
    > lastProcessedTimestamp {};

    std::atomic<std::size_t> frameInterval {
        1
    };


    std::array<
        std::size_t,
        CAMERA_COUNT
    > frameCounters {};
};