#pragma once

#include <atomic>
#include <cstddef>

#include "vision/rgb_frame_source.h"


class SimulatedRgbFrameSource
    : public RgbFrameSource
{
public:

    explicit SimulatedRgbFrameSource(
        std::size_t cameraIndex
    );


    bool start() override;

    void stop() override;

    bool isRunning() const override;


    bool getFrame(
        Frame& frame
    ) override;


    std::size_t getCameraIndex() const override;


private:

    std::size_t cameraIndex;

    std::atomic<bool> running;
};