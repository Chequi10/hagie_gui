#include "vision/zed_orientation_adapter.h"
#include <QDebug>
#include "vision/zed_gmsl_point_cloud_source.h"

#ifdef HAGIE_ENABLE_ZED_SDK
#include <sl/CameraOne.hpp>
#endif

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <iostream>


ZedGmslPointCloudSource::ZedGmslPointCloudSource(
    std::size_t cameraIndex,
    uint32_t serialNumber,
    int cameraFps,
    const std::string& cameraResolution,
    int cameraTimeoutMs,
    bool autoReconnect,
    int reconnectIntervalMs)
    :
    cameraIndex(
        cameraIndex
    ),
    serialNumber(
        serialNumber
    ),
    cameraFps(
        cameraFps
    ),
    cameraResolution(
        cameraResolution
    ),
    cameraTimeoutMs(
        cameraTimeoutMs
    ),
    autoReconnect(
        autoReconnect
    ),
    reconnectIntervalMs(
        reconnectIntervalMs
    ),
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
    if (cameraResolution == "HD1080")
        {
            initParameters.camera_resolution =
                sl::RESOLUTION::HD1080;
        }
        else
        {
            initParameters.camera_resolution =
                sl::RESOLUTION::HD720;
        }


    initParameters.camera_fps =
    cameraFps;


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
        sl::UNIT::MILLIMETER;

        /*
        * Sistema de coordenadas elegido para Hagie:
        *
        * ZED X -> adelante
        * ZED Y -> lateral
        * ZED Z -> vertical
        */
        initParameters.coordinate_system =
            sl::COORDINATE_SYSTEM::
                RIGHT_HANDED_Z_UP_X_FWD;

                


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

    const std::uint64_t nowMs =
    static_cast<std::uint64_t>(
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(
            std::chrono::steady_clock::now()
                .time_since_epoch()
        ).count()
    );


    lastSuccessfulGrabMs =
        nowMs;

    lastReconnectAttemptMs =
        nowMs;

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
    std::lock_guard<std::mutex> lock(orientationMutex);

    orientation = latestOrientation;

    return running.load() && orientation.valid;
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



bool ZedGmslPointCloudSource::getCameraIntrinsics(
    CameraIntrinsics& intrinsics) const
{
    intrinsics = CameraIntrinsics {};

#ifdef HAGIE_ENABLE_ZED_SDK

    if (!isRunning())
        return false;

    const auto info =
        camera.getCameraInformation();

    const auto& config =
        info.camera_configuration;

    const auto& calibration =
        config.calibration_parameters.left_cam;

    intrinsics.fx = calibration.fx;
    intrinsics.fy = calibration.fy;
    intrinsics.cx = calibration.cx;
    intrinsics.cy = calibration.cy;

    intrinsics.width =
        static_cast<int>(config.resolution.width);

    intrinsics.height =
        static_cast<int>(config.resolution.height);

    intrinsics.valid =
        intrinsics.fx > 0.0f &&
        intrinsics.fy > 0.0f &&
        intrinsics.width > 0 &&
        intrinsics.height > 0;

    return intrinsics.valid;

#else

    return false;

#endif
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
        if (!autoReconnect)
        {
            return false;
        }


        const std::uint64_t nowMs =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<
                    std::chrono::milliseconds
                >(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()
                ).count()
            );


        if (lastReconnectAttemptMs != 0 &&
            (nowMs - lastReconnectAttemptMs) <
                static_cast<std::uint64_t>(
                    reconnectIntervalMs
                ))
        {
            return false;
        }


        lastReconnectAttemptMs =
            nowMs;


        stop();


        if (!start())
        {
            return false;
        }
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
        const std::uint64_t nowMs =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<
                    std::chrono::milliseconds
                >(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()
                ).count()
            );


        const std::uint64_t elapsedMs =
            nowMs - lastSuccessfulGrabMs;


        if (elapsedMs >=
            static_cast<std::uint64_t>(
                cameraTimeoutMs
            ))
        {
            running.store(
                false
            );
        }


        return false;
    }

    /*
     * Leer IMU sincronizada con el frame ZED.
     */
    {
        CameraOrientation currentOrientation {};

        sl::SensorsData sensorsData;

        const sl::ERROR_CODE imuResult =
            camera.getSensorsData(
                sensorsData,
                sl::TIME_REFERENCE::IMAGE
            );

        if (imuResult == sl::ERROR_CODE::SUCCESS &&
            sensorsData.imu.is_available)
        {
            currentOrientation =
                ZedOrientationAdapter::fromRotation(
                    sensorsData.imu.pose.getRotationMatrix());

            static thread_local unsigned int imuCounter = 0;

            if (++imuCounter % 15 == 0)
            {
                qInfo() << "[HAGIE IMU REAL]"
                        << "Roll:" << currentOrientation.roll_deg
                        << "Pitch:" << currentOrientation.pitch_deg
                        << "Valida:" << currentOrientation.valid;
            }
        }

        std::lock_guard<std::mutex> lock(orientationMutex);
        latestOrientation = currentOrientation;
    }

    lastSuccessfulGrabMs =
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
    * RGB y XYZ deben compartir exactamente
    * el mismo sistema de coordenadas de imagen.
    */
    if (static_cast<std::size_t>(width) !=
            rgbFrame.width ||
        static_cast<std::size_t>(height) !=
            rgbFrame.height)
    {
        return false;
    }


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


    std::size_t sampledPoints = 0;
    std::size_t finitePoints = 0;

    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();

    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for (int y = 0;
         y < height;
         y += SAMPLE_STEP)
    {
        for (int x = 0;
             x < width;
             x += SAMPLE_STEP)
        {
            ++sampledPoints;

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


            ++finitePoints;

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
            * RIGHT_HANDED_Z_UP_X_FWD
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


            minX = std::min(minX, convertedPoint.x);
            minY = std::min(minY, convertedPoint.y);
            minZ = std::min(minZ, convertedPoint.z);

            maxX = std::max(maxX, convertedPoint.x);
            maxY = std::max(maxY, convertedPoint.y);
            maxZ = std::max(maxZ, convertedPoint.z);

            cloud.push_back(
                convertedPoint
);
        }
    }


    /*
     * DIAGNOSTICO DE NUBE 3D REAL
     * Mostrar cada 30 cuadros.
     */
    static std::uint64_t diagnosticFrame = 0;

    if ((++diagnosticFrame % 30) == 0)
    {
        std::cerr
            << "\\n[ZED 3D] Camara: "
            << cameraIndex
            << " | Serial: " << serialNumber
            << "\\nMuestreados: " << sampledPoints
            << " | Finitos: " << finitePoints
            << " | En nube: " << cloud.size()
            << std::endl;

        if (!cloud.empty())
        {
            /*
             * MEDICION DE UNA REGION CENTRAL
             * de la imagen RGB.
             */
            const int centerX = width / 2;
            const int centerY = height / 2;

            const int radiusX = width / 10;
            const int radiusY = height / 10;

            std::vector<float> centerXValues;
            std::vector<float> centerYValues;
            std::vector<float> centerZValues;

            for (const auto& point : cloud)
            {
                if (!point.image_coordinates_valid)
                    continue;

                if (std::abs(point.image_x - centerX) > radiusX ||
                    std::abs(point.image_y - centerY) > radiusY)
                    continue;

                centerXValues.push_back(point.x);
                centerYValues.push_back(point.y);
                centerZValues.push_back(point.z);
            }

            if (!centerYValues.empty())
            {
                auto median = [](std::vector<float>& values)
                {
                    std::sort(values.begin(), values.end());
                    return values[values.size() / 2];
                };

                float measuredX = median(centerXValues);
                float measuredY = median(centerYValues);
                float measuredZ = median(centerZValues);

                std::sort(centerZValues.begin(), centerZValues.end());

                const std::size_t p95Index =
                    static_cast<std::size_t>(
                        0.95f * (centerZValues.size() - 1)
                    );

                const float p95Z = centerZValues[p95Index];

                std::cerr
                    << "[REGION RGB 20%] Pixeles: "
                    << centerX << "," << centerY
                    << " | Puntos: " << centerYValues.size()
                    << " | X=" << measuredX
                    << " Y=" << measuredY
                    << " Z=" << measuredZ
                    << " P95_Z=" << p95Z
                    << " mm"
                    << std::endl;
            }
            else
            {
                std::cerr
                    << "[CENTRO RGB] Sin puntos validos"
                    << std::endl;
            }

            std::cerr
                << "X: " << minX << " a " << maxX
                << " mm | Y: " << minY << " a " << maxY
                << " mm | Z: " << minZ << " a " << maxZ
                << " mm"
                << std::endl;

            for (std::size_t i = 0;
                 i < std::min<std::size_t>(3, cloud.size());
                 ++i)
            {
                const auto& point = cloud[i];

                std::cerr
                    << "P" << i + 1
                    << ": X=" << point.x
                    << " Y=" << point.y
                    << " Z=" << point.z
                    << " mm"
                    << std::endl;
            }
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

    const auto stereoDevices =
        sl::Camera::getDeviceList();

    const auto monoDevices =
        sl::CameraOne::getDeviceList();

    for (const auto& device : stereoDevices)
    {
        if (device.serial_number != 0 &&
            std::find(serialNumbers.begin(),
                      serialNumbers.end(),
                      device.serial_number) == serialNumbers.end())
        {
            serialNumbers.push_back(device.serial_number);
        }
    }

    for (const auto& device : monoDevices)
    {
        if (device.serial_number != 0 &&
            std::find(serialNumbers.begin(),
                      serialNumbers.end(),
                      device.serial_number) == serialNumbers.end())
        {
            serialNumbers.push_back(device.serial_number);
        }
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