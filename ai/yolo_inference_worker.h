#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>

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
        const char* enginePath
    );


    bool start();

    void stop();

    bool isRunning() const;


    bool getLatestResult(
        std::size_t cameraIndex,
        Result& result
    ) const;


private:

    void workerLoop();


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
};