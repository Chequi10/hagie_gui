#include "vision/simulated_point_cloud_source.h"


SimulatedPointCloudSource::SimulatedPointCloudSource(
    std::size_t cameraIndex)
    :
    cameraIndex(cameraIndex),
    running(false)
{
}


bool SimulatedPointCloudSource::start()
{
    running.store(
        true
    );

    return true;
}


void SimulatedPointCloudSource::stop()
{
    running.store(
        false
    );
}


bool SimulatedPointCloudSource::isRunning() const
{
    return running.load();
}


std::size_t
SimulatedPointCloudSource::getCameraIndex() const
{
    return cameraIndex;
}


bool SimulatedPointCloudSource::getPointCloud(
    Vision3DProcessor::PointCloud& cloud)
{
    if (!running.load())
    {
        return false;
    }


    cloud.clear();


    /*
     * ========================================================
     * SIMULACIÓN DE NUBES DE PUNTOS
     * ========================================================
     *
     * Cámara 0 -> cuerpos 1 y 2
     * Cámara 1 -> cuerpos 3 y 4
     * Cámara 2 -> cuerpos 5 y 6
     *
     * Los valores coinciden con los utilizados
     * en las pruebas de Vision3DProcessor.
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