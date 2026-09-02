#include "vision/zed_gmsl_point_cloud_source.h"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <chrono>


ZedGmslPointCloudSource::ZedGmslPointCloudSource(
    std::size_t cameraIndex,
    uint32_t serialNumber)
    :
    cameraIndex(cameraIndex),
    serialNumber(serialNumber),
    sharedRgbFrame(
        std::make_shared<SharedRgbFrame>()
    )
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

bool ZedGmslPointCloudSource::getCameraOrientation(
    CameraOrientation& orientation)
{
    orientation =
        CameraOrientation {};


#ifdef HAGIE_ENABLE_ZED_SDK

    if (!running.load())
    {
        return false;
    }

    /*
     * La lectura real de la IMU ZED
     * se implementará usando el SDK
     * cuando compilemos en la Jetson
     * con HAGIE_ENABLE_ZED_SDK.
     *
     * Por ahora dejamos la interfaz
     * correctamente conectada.
     */

    return false;

#else

    /*
     * En la ThinkPad no hay ZED SDK
     * ni hardware real.
     */
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
    * TIMESTAMP DE LA CAPTURA
    * ========================================================
    *
    * Este timestamp corresponde al grab() común
    * del cual obtenemos:
    *
    * - imagen RGB;
    * - nube XYZ.
    *
    * Por lo tanto ambas representan exactamente
    * el mismo instante de adquisición.
    */
    const std::uint64_t captureTimestampMs =
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                std::chrono::steady_clock::now()
                    .time_since_epoch()
            ).count()
        );

    

    /*
    * ========================================================
    * OBTENER IMAGEN RGB
    * ========================================================
    */
    sl::ERROR_CODE imageResult =
        camera.retrieveImage(
            zedRgbImage,
            sl::VIEW::LEFT,
            sl::MEM::CPU
        );


    if (imageResult != sl::ERROR_CODE::SUCCESS)
    {
        return false;
    }
    /*
    * ========================================================
    * GUARDAR RGB SINCRONIZADO
    * ========================================================
    */


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

        RgbFrameSource::Frame rgbFrame;


    rgbFrame.width =
        static_cast<std::size_t>(
            zedRgbImage.getWidth()
        );

    rgbFrame.height =
        static_cast<std::size_t>(
            zedRgbImage.getHeight()
        );


    constexpr std::size_t RGB_CHANNELS =
        3;


    rgbFrame.data.resize(
        rgbFrame.width *
        rgbFrame.height *
        RGB_CHANNELS
    );


    /*
    * La imagen ZED se obtiene con 4 componentes
    * por píxel.
    *
    * Para el resto de Hagie guardamos solamente
    * tres componentes RGB.
    */
    for (std::size_t y = 0;
        y < rgbFrame.height;
        ++y)
    {
        for (std::size_t x = 0;
            x < rgbFrame.width;
            ++x)
        {
            sl::uchar4 pixel;


            if (zedRgbImage.getValue(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    &pixel
                ) != sl::ERROR_CODE::SUCCESS)
            {
                continue;
            }


            const std::size_t offset =
                (
                    y * rgbFrame.width +
                    x
                ) * RGB_CHANNELS;


            rgbFrame.data[offset + 0] =
                pixel[2];

            rgbFrame.data[offset + 1] =
                pixel[1];

            rgbFrame.data[offset + 2] =
                pixel[0];
        }
    }


    rgbFrame.timestamp_ms =
        captureTimestampMs;

    rgbFrame.valid =
        true;


    
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


            /*
            * Guardamos también el píxel exacto
            * de la imagen ZED que produjo este punto.
            */
            convertedPoint.image_x =
                x;

            convertedPoint.image_y =
                y;

            convertedPoint.image_coordinates_valid =
                true;


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

    lastPointCloudTimestampMs =
        captureTimestampMs;

    /*
    * Publicar solamente cuando el frame
    * está completamente construido.
    */
    std::lock_guard<std::mutex> lock(
        sharedRgbFrame->mutex
    );

    sharedRgbFrame->frame =
        std::move(rgbFrame);




    return true;

#else

    /*
     * Sin ZED SDK nunca se produce
     * una nube real.
     */
    return false;

#endif
}

bool ZedGmslPointCloudSource::
getLastPointCloudTimestampMs(
    std::uint64_t& timestampMs
) const
{
    if (lastPointCloudTimestampMs == 0)
    {
        timestampMs = 0;

        return false;
    }


    timestampMs =
        lastPointCloudTimestampMs;

    return true;
}

bool ZedGmslPointCloudSource::getLatestRgbFrame(
    RgbFrameSource::Frame& frame) const
{
    if (sharedRgbFrame == nullptr)
    {
        frame =
            RgbFrameSource::Frame {};

        return false;
    }


    std::lock_guard<std::mutex> lock(
        sharedRgbFrame->mutex
    );


    if (!sharedRgbFrame->frame.valid)
    {
        frame =
            RgbFrameSource::Frame {};

        return false;
    }


    frame =
        sharedRgbFrame->frame;


    return true;
}

ZedGmslPointCloudSource::SharedRgbFramePtr
ZedGmslPointCloudSource::getSharedRgbFrame() const
{
    return sharedRgbFrame;
}

std::vector<uint32_t>
ZedGmslPointCloudSource::detectConnectedSerialNumbers()
{
    std::vector<uint32_t> serialNumbers;


#ifdef HAGIE_ENABLE_ZED_SDK

    /*
     * ========================================================
     * HARDWARE REAL
     * ========================================================
     */
    auto devices =
        sl::Camera::getDeviceList();


    for (const auto& device : devices)
    {
        if (device.serial_number == 0)
        {
            continue;
        }


        serialNumbers.push_back(
            device.serial_number
        );
    }


#else

    /*
     * ========================================================
     * SIMULACIÓN PARA DESARROLLO
     * ========================================================
     *
     * Permite simular cámaras ZED conectadas mediante:
     *
     * HAGIE_ZED_FAKE_SERIALS=11111111,98765432,44444
     *
     * Si la variable no existe, devuelve lista vacía.
     */
    const char *fakeSerials =
        std::getenv(
            "HAGIE_ZED_FAKE_SERIALS"
        );


    if (fakeSerials != nullptr)
    {
        std::stringstream stream(
            fakeSerials
        );


        std::string item;


        while (std::getline(
            stream,
            item,
            ','
        ))
        {
            try
            {
                unsigned long value =
                    std::stoul(
                        item
                    );


                if (value != 0)
                {
                    serialNumbers.push_back(
                        static_cast<uint32_t>(
                            value
                        )
                    );
                }
            }
            catch (...)
            {
                /*
                 * Ignorar valores inválidos.
                 */
            }
        }
    }

#endif


    return serialNumbers;
}