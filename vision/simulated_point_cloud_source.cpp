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

void SimulatedPointCloudSource::setSimulatedOrientation(
    float rollDeg,
    float pitchDeg)
{
    simulatedRollDeg =
        rollDeg;

    simulatedPitchDeg =
        pitchDeg;
}


bool SimulatedPointCloudSource::getCameraOrientation(
    CameraOrientation& orientation)
{
    if (!running.load())
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

bool SimulatedPointCloudSource::getPointCloud(
    Vision3DProcessor::PointCloud& cloud)
{
    if (!running.load())
    {
        return false;
    }


    cloud.clear();

        /*
     * Agrega una panoja simulada con:
     *
     * - posición física XYZ
     * - posición correspondiente en la imagen RGB
     *
     * Así simulamos la asociación RGB + profundidad
     * que entregará la ZED real.
     */
    auto addSimulatedTassel =
        [&cloud](
            float x,
            float y,
            float z,
            int imageX,
            int imageY)
        {
            Vision3DProcessor::Point3D point;

            point.x =
                x;

            point.y =
                y;

            point.z =
                z;

            point.image_x =
                imageX;

            point.image_y =
                imageY;

            point.image_coordinates_valid =
                true;


            cloud.push_back(
                point
            );
        };


        /*
     * ========================================================
     * SIMULACIÓN DE NUBES DE PUNTOS
     * ========================================================
     *
     * Cámara 0 -> cuerpos 1 y 2
     * Cámara 1 -> cuerpos 2 y 3
     * Cámara 2 -> cuerpos 3 y 4
     * Cámara 3 -> cuerpos 4 y 5
     * Cámara 4 -> cuerpos 5 y 6
     *
     * Se utiliza solapamiento entre cámaras para que
     * los cuerpos intermedios puedan ser observados
     * por más de una cámara.
     */


    switch (cameraIndex)
    {
        case 0:
        {
            /*
             * Cámara 0
             *
             * Detección 1 -> Cuerpo 1
             * Detección 2 -> Cuerpo 2
             */
            addSimulatedTassel(
                -2.5f,
                0.0f,
                0.40f,
                95,
                85
            );

            addSimulatedTassel(
                -1.5f,
                0.0f,
                0.50f,
                174,
                67
            );

            break;
        }


        case 1:
        {
            /*
             * La primera detección representa
             * la MISMA zona física C2 que también
             * observa la cámara 0.
             */
            addSimulatedTassel(
                -1.5f,
                0.0f,
                0.50f,
                95,
                85
            );

            addSimulatedTassel(
                -0.5f,
                0.0f,
                0.60f,
                174,
                67
            );

            break;
        }


        case 2:
        {
            addSimulatedTassel(
                -0.5f,
                0.0f,
                0.60f,
                95,
                85
            );

            addSimulatedTassel(
                0.5f,
                0.0f,
                0.70f,
                174,
                67
            );

            break;
        }


        case 3:
        {
            addSimulatedTassel(
                0.5f,
                0.0f,
                0.70f,
                95,
                85
            );

            addSimulatedTassel(
                1.5f,
                0.0f,
                0.80f,
                174,
                67
            );

            break;
        }


        case 4:
        {
            addSimulatedTassel(
                1.5f,
                0.0f,
                0.80f,
                95,
                85
            );

            addSimulatedTassel(
                2.5f,
                0.0f,
                0.90f,
                174,
                67
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
