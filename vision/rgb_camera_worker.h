#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>

#include "vision/rgb_frame_source.h"


class RgbCameraWorker
{
public:

    static constexpr std::size_t CAMERA_COUNT =
        7;


    RgbCameraWorker();

    ~RgbCameraWorker();


    bool setFrameSource(
        std::size_t cameraIndex,
        std::unique_ptr<RgbFrameSource> source
    );


    void clearFrameSource(
        std::size_t cameraIndex
    );


    bool start();

    void stop();

    bool isRunning() const;


    bool getFrame(
        std::size_t cameraIndex,
        RgbFrameSource::Frame& frame
    );


private:

    bool running;

    std::array<
        std::unique_ptr<RgbFrameSource>,
        CAMERA_COUNT
    > sources;
    mutable std::mutex mutex;
};