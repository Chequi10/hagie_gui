#include "vision/zed_gmsl_point_cloud_source.h"

#include <cmath>


ZedGmslPointCloudSource::ZedGmslPointCloudSource(
    std::size_t cameraIndex,
    uint32_t serialNumber)
    :
    cameraIndex(cameraIndex),
    serialNumber(serialNumber)
{
}


ZedGmslPointCloudSource::~ZedGmslPointCloudSource()
{
    stop();
}


bool ZedGmslPointCloudSource::start()
{
#ifdef HAGIE_ENABLE_ZED_SDK

    /*
     * ========================================================
     * CONFIGURACIÓN ZED
     * ========================================================
     */

    sl::InitParameters initParameters;


    /*
     * Abrir la cámara física por número de serie.
     *
     * Esto evita depender del orden en que
     * Linux / ZED Link enumere las cámaras.
     */
    initParameters.input.setFromSerialNumber(
        serialNumber
    );


    /*
     * Configuración inicial.
     *
     * Más adelante podremos ajustar:
     *
     * - resolución;
     * - FPS;
     * - modo de profundidad;
     *
     * según rendimiento real en la AGX Orin.
     */
    initParameters.camera_resolution =
        sl::RESOLUTION::HD1200;


    initParameters.camera_fps =
        30;


    /*
     * Modo de profundidad inicial.
     *
     * Lo dejamos preparado para NEURAL.
     *
     * Si después necesitamos reducir carga
     * GPU podremos probar PERFORMANCE,
     * QUALITY u otro modo.
     */
    initParameters.depth_mode =
        sl::DEPTH_MODE::NEURAL;


    /*
     * Nuestro Vision3DProcessor trabaja
     * actualmente con coordenadas en metros.
     */
    initParameters.coordinate_units =
        sl::UNIT::METER;

        /*
        * Sistema de coordenadas elegido para Hagie:
        *
        * ZED X -> adelante
        * ZED Y -> lateral
        * ZED Z -> vertical
        */
        initParameters.coordinate_system =
            sl::COORDINATE_SYSTEM::
                RIGHT_HANDED_Z_UP_X_FORWARD;

                


    /*
     * Abrir la cámara.
     */
    sl::ERROR_CODE error =
        camera.open(
            initParameters
        );


    if (error != sl::ERROR_CODE::SUCCESS)
    {
        running.store(
            false
        );


        return false;
    }


    running.store(
        true
    );


    return true;

#else

    /*
     * ========================================================
     * BUILD SIN ZED SDK
     * ========================================================
     *
     * En la ThinkPad no existe hardware ni SDK ZED.
     *
     * La fuente permanece desactivada.
     */
    running.store(
        false
    );


    return false;

#endif
}


void ZedGmslPointCloudSource::stop()
{
#ifdef HAGIE_ENABLE_ZED_SDK

    /*
     * Cerrar la cámara si estaba abierta.
     */
    if (camera.isOpened())
    {
        camera.close();
    }

#endif


    running.store(
        false
    );
}


bool ZedGmslPointCloudSource::isRunning() const
{
    return running.load();
}


std::size_t
ZedGmslPointCloudSource::getCameraIndex() const
{
    return cameraIndex;
}


uint32_t
ZedGmslPointCloudSource::getSerialNumber() const
{
    return serialNumber;
}


bool ZedGmslPointCloudSource::getPointCloud(
    Vision3DProcessor::PointCloud& cloud)
{
    cloud.clear();


    if (!running.load())
    {
        return false;
    }


#ifdef HAGIE_ENABLE_ZED_SDK

    /*
     * ========================================================
     * CAPTURAR FRAME
     * ========================================================
     */

    sl::ERROR_CODE grabResult =
        camera.grab(
            runtimeParameters
        );


    if (grabResult != sl::ERROR_CODE::SUCCESS)
    {
        return false;
    }


    /*
     * ========================================================
     * OBTENER NUBE 3D
     * ========================================================
     *
     * XYZ:
     *
     * X, Y, Z en metros.
     */
    sl::ERROR_CODE measureResult =
        camera.retrieveMeasure(
            zedPointCloud,
            sl::MEASURE::XYZ
        );


    if (measureResult != sl::ERROR_CODE::SUCCESS)
    {
        return false;
    }


    /*
     * ========================================================
     * CONVERTIR A FORMATO GENÉRICO
     * ========================================================
     *
     * No copiamos necesariamente cada píxel.
     *
     * Submuestreamos para reducir:
     *
     * - memoria;
     * - CPU;
     * - ancho de banda interno;
     * - tiempo de Vision3DProcessor.
     *
     * Inicialmente tomamos un punto cada 4 píxeles
     * en X e Y.
     */
    static constexpr int SAMPLE_STEP =
        4;


    const int width =
        zedPointCloud.getWidth();


    const int height =
        zedPointCloud.getHeight();


    /*
     * Reservar memoria aproximada.
     */
    cloud.reserve(
        static_cast<std::size_t>(
            (width / SAMPLE_STEP)
            *
            (height / SAMPLE_STEP)
        )
    );


    for (int y = 0;
         y < height;
         y += SAMPLE_STEP)
    {
        for (int x = 0;
             x < width;
             x += SAMPLE_STEP)
        {
            sl::float4 point;


            sl::ERROR_CODE pointError =
                zedPointCloud.getValue(
                    x,
                    y,
                    &point
                );


            if (pointError !=
                sl::ERROR_CODE::SUCCESS)
            {
                continue;
            }


            /*
             * Descartar NaN / infinito.
             */
            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z))
            {
                continue;
            }


            /*
             * Convertir al formato común
             * utilizado por todo Hagie.
             */
            Vision3DProcessor::Point3D
                convertedPoint;


            /*
            * ========================================================
            * CONVERSIÓN ZED -> HAGIE
            * ========================================================
            *
            * La ZED se configura con:
            *
            * RIGHT_HANDED_Z_UP_X_FORWARD
            *
            * Por lo tanto:
            *
            * ZED X = longitudinal / hacia adelante
            * ZED Y = lateral
            * ZED Z = vertical
            *
            * El sistema interno Hagie espera:
            *
            * Hagie X = lateral
            * Hagie Y = longitudinal
            * Hagie Z = vertical
            */
            convertedPoint.x =
                point.y;


            convertedPoint.y =
                point.x;


            convertedPoint.z =
                point.z;


            cloud.push_back(
                convertedPoint
);
        }
    }


    /*
     * Una nube vacía se considera inválida.
     */
    if (cloud.empty())
    {
        return false;
    }


    return true;

#else

    /*
     * Sin ZED SDK nunca se produce
     * una nube real.
     */
    return false;

#endif
}