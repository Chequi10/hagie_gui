#pragma once

#include <cstddef>

#include "ai/tassel_detector.h"
#include "vision/rgb_frame_source.h"


class TensorRtTasselDetector
{
public:

    TensorRtTasselDetector() = default;

    ~TensorRtTasselDetector() = default;


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
};