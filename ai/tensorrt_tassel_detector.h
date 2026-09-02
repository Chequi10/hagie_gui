#pragma once

#include <cstddef>

#include "ai/tassel_detector.h"
#include "vision/rgb_frame_source.h"


#ifdef HAGIE_ENABLE_TENSORRT
#include <NvInfer.h>
#endif


class TensorRtTasselDetector
{
public:

    TensorRtTasselDetector();

    ~TensorRtTasselDetector();


    bool initialize(
        const char* enginePath
    );


    bool isInitialized() const;


    bool processFrame(
        std::size_t cameraIndex,
        const RgbFrameSource::Frame& frame,
        TasselDetector::Result& result
    );


private:

    bool initialized =
        false;


#ifdef HAGIE_ENABLE_TENSORRT

    class Logger :
        public nvinfer1::ILogger
    {
    public:

        void log(
            Severity severity,
            const char* message
        ) noexcept override;
    };


    Logger logger;


    nvinfer1::IRuntime* runtime =
        nullptr;

    nvinfer1::ICudaEngine* engine =
        nullptr;

    nvinfer1::IExecutionContext* context =
        nullptr;

#endif
};