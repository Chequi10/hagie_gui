#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vision/rgb_frame_source.h"


class TasselDetector
{
public:

    struct Detection
    {
        float confidence =
            0.0f;

        int x =
            0;

        int y =
            0;

        int width =
            0;

        int height =
            0;
    };


    struct Result
    {
        std::size_t camera_index =
            0;

        std::uint64_t timestamp_ms =
            0;

        std::vector<Detection> detections;

        bool valid =
            false;
    };


    TasselDetector() = default;

    ~TasselDetector() = default;


    bool processFrame(
        std::size_t cameraIndex,
        const RgbFrameSource::Frame& frame,
        Result& result
    );
};
