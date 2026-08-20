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
     * Se generan puntos simples y conocidos.
     *
     * Estas nubes sirven solamente para probar
     * la capa de adquisición y el futuro
     * Vision3DWorker.
     *
     * Cámara 0 -> cuerpos 1 y 2
     * Cámara 1 -> cuerpos 3 y 4
     * Cámara 2 -> cuerpos 5 y 6
     */


    switch (cameraIndex)
    {
        case 0:
        {
            /*
             * Cuerpo 1 -> 400 mm
             */
            cloud.push_back(
                {-2.5f, 0.0f, 0.40f}
            );


            /*
             * Cuerpo 2 -> 500 mm
             */
            cloud.push_back(
                {-1.5f, 0.0f, 0.50f}
            );


            break;
        }


        case 1:
        {
            /*
             * Cuerpo 3 -> 600 mm
             */
            cloud.push_back(
                {-0.5f, 0.0f, 0.60f}
            );


            /*
             * Cuerpo 4 -> 700 mm
             */
            cloud.push_back(
                {0.5f, 0.0f, 0.70f}
            );


            break;
        }


        case 2:
        {
            /*
             * Cuerpo 5 -> 800 mm
             */
            cloud.push_back(
                {1.5f, 0.0f, 0.80f}
            );


            /*
             * Cuerpo 6 -> 900 mm
             */
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