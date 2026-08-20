#ifndef SIMULATED_POINT_CLOUD_SOURCE_H
#define SIMULATED_POINT_CLOUD_SOURCE_H

#include <atomic>
#include <cstddef>

#include "vision/point_cloud_source.h"


class SimulatedPointCloudSource
    : public PointCloudSource
{
public:

    explicit SimulatedPointCloudSource(
        std::size_t cameraIndex
    );


    bool start() override;

    void stop() override;

    bool isRunning() const override;


    bool getPointCloud(
        Vision3DProcessor::PointCloud& cloud
    ) override;


    std::size_t getCameraIndex() const override;


private:

    std::size_t cameraIndex;

    std::atomic<bool> running;
};


#endif