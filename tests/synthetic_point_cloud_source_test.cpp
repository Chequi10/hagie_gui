#include "synthetic_point_cloud_source_test.h"


SyntheticPointCloudSource::SyntheticPointCloudSource(
    std::size_t cameraIndex)
    : cameraIndex(cameraIndex),
      running(false)
{
}


bool SyntheticPointCloudSource::start()
{
    running = true;

    return true;
}


void SyntheticPointCloudSource::stop()
{
    running = false;
}


bool SyntheticPointCloudSource::isRunning() const
{
    return running;
}


std::size_t
SyntheticPointCloudSource::getCameraIndex() const
{
    return cameraIndex;
}

void SyntheticPointCloudSource::setSimulatedOrientation(
    float rollDeg,
    float pitchDeg)
{
    simulatedRollDeg =
        rollDeg;

    simulatedPitchDeg =
        pitchDeg;
}


bool SyntheticPointCloudSource::getCameraOrientation(
    CameraOrientation& orientation)
{
    if (!running)
    {
        orientation =
            CameraOrientation {};

        return false;
    }

    orientation.roll_deg =
        simulatedRollDeg;

    orientation.pitch_deg =
        simulatedPitchDeg;

    orientation.valid =
        true;

    return true;
}


bool SyntheticPointCloudSource::getPointCloud(
    Vision3DProcessor::PointCloud& cloud)
{
    if (!running)
    {
        return false;
    }


    cloud.clear();


    /*
    * ========================================================
    * NUBES SINTÉTICAS POR CÁMARA
    * ========================================================
    *
    * Cámara 0 -> cuerpos 1 y 2
    * Cámara 1 -> cuerpos 2 y 3
    * Cámara 2 -> cuerpos 3 y 4
    * Cámara 3 -> cuerpos 4 y 5
    * Cámara 4 -> cuerpos 5 y 6
    *
    * Se utiliza solapamiento entre cámaras vecinas.
    */


    switch (cameraIndex)
    {
        case 0:
        {
            cloud.push_back(
                {-2.5f, 0.0f, 0.40f}
            );

            cloud.push_back(
                {-1.5f, 0.0f, 0.50f}
            );

            break;
        }


        case 1:
        {
            cloud.push_back(
                {-1.5f, 0.0f, 0.50f}
            );

            cloud.push_back(
                {-0.5f, 0.0f, 0.60f}
            );

            break;
        }


        case 2:
        {
            cloud.push_back(
                {-0.5f, 0.0f, 0.60f}
            );

            cloud.push_back(
                {0.5f, 0.0f, 0.70f}
            );

            break;
        }


        case 3:
        {
            cloud.push_back(
                {0.5f, 0.0f, 0.70f}
            );

            cloud.push_back(
                {1.5f, 0.0f, 0.80f}
            );

            break;
        }


        case 4:
        {
            cloud.push_back(
                {1.5f, 0.0f, 0.80f}
            );

            cloud.push_back(
                {2.5f, 0.0f, 0.90f}
            );

            break;
        }


        default:
        {
            return false;
        }
    }


    return true;
}