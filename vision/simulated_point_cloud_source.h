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

    bool getCameraOrientation(
        CameraOrientation& orientation
    ) override;

    void setSimulatedOrientation(
        float rollDeg,
        float pitchDeg
    );


private:

    std::size_t cameraIndex;

    std::atomic<bool> running;

    float simulatedRollDeg = 0.0f;

    float simulatedPitchDeg = 0.0f;
};


#endif