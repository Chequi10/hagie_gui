#include "vision/rgb_camera_worker.h"


RgbCameraWorker::RgbCameraWorker()
    : running(false)
{
}


RgbCameraWorker::~RgbCameraWorker()
{
    stop();
}


bool RgbCameraWorker::setFrameSource(
    std::size_t cameraIndex,
    std::unique_ptr<RgbFrameSource> source)
{
    if (cameraIndex >= CAMERA_COUNT ||
        source == nullptr)
    {
        return false;
    }


    if (running)
    {
        source->start();
    }


    sources[cameraIndex] =
        std::move(source);


    return true;
}


void RgbCameraWorker::clearFrameSource(
    std::size_t cameraIndex)
{
    if (cameraIndex >= CAMERA_COUNT)
    {
        return;
    }


    if (sources[cameraIndex] != nullptr)
    {
        sources[cameraIndex]->stop();

        sources[cameraIndex].reset();
    }
}


bool RgbCameraWorker::start()
{
    if (running)
    {
        return true;
    }


    bool anySourceStarted =
        false;


    for (auto& source : sources)
    {
        if (source == nullptr)
        {
            continue;
        }


        if (source->start())
        {
            anySourceStarted =
                true;
        }
    }


    running =
        anySourceStarted;


    return running;
}


void RgbCameraWorker::stop()
{
    for (auto& source : sources)
    {
        if (source != nullptr)
        {
            source->stop();
        }
    }


    running =
        false;
}


bool RgbCameraWorker::isRunning() const
{
    return running;
}


bool RgbCameraWorker::getFrame(
    std::size_t cameraIndex,
    RgbFrameSource::Frame& frame)
{
    if (!running ||
        cameraIndex >= CAMERA_COUNT)
    {
        frame =
            RgbFrameSource::Frame {};

        return false;
    }


    if (sources[cameraIndex] == nullptr)
    {
        frame =
            RgbFrameSource::Frame {};

        return false;
    }


    return sources[cameraIndex]->getFrame(
        frame
    );
}