#include "vision/zed_gmsl_rgb_frame_source.h"

#include <mutex>


ZedGmslRgbFrameSource::ZedGmslRgbFrameSource(
    std::size_t cameraIndex,
    ZedGmslPointCloudSource::SharedRgbFramePtr sharedFrame)
    :
    cameraIndex(cameraIndex),
    sharedFrame(std::move(sharedFrame))
{
}


bool ZedGmslRgbFrameSource::start()
{
    running =
        sharedFrame != nullptr;

    return running;
}


void ZedGmslRgbFrameSource::stop()
{
    running =
        false;
}


bool ZedGmslRgbFrameSource::isRunning() const
{
    return running;
}


bool ZedGmslRgbFrameSource::getFrame(
    Frame& frame)
{
    if (!running ||
        sharedFrame == nullptr)
    {
        frame =
            Frame {};

        return false;
    }


    std::lock_guard<std::mutex> lock(
        sharedFrame->mutex
    );


    if (!sharedFrame->frame.valid)
    {
        frame =
            Frame {};

        return false;
    }


    frame =
        sharedFrame->frame;


    return true;
}


std::size_t
ZedGmslRgbFrameSource::getCameraIndex() const
{
    return cameraIndex;
}