#pragma once

#include <cstddef>

#include "vision/rgb_frame_source.h"
#include "vision/zed_gmsl_point_cloud_source.h"


class ZedGmslRgbFrameSource
    : public RgbFrameSource
{
public:

    ZedGmslRgbFrameSource(
        std::size_t cameraIndex,
        ZedGmslPointCloudSource::SharedRgbFramePtr sharedFrame
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

    ZedGmslPointCloudSource::SharedRgbFramePtr sharedFrame;

    bool running = false;
};