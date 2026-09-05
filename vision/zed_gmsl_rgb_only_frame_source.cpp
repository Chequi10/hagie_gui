#include "vision/zed_gmsl_rgb_only_frame_source.h"

#include <chrono>


ZedGmslRgbOnlyFrameSource::
ZedGmslRgbOnlyFrameSource(
    std::size_t cameraIndex,
    uint32_t serialNumber)
    :
    cameraIndex(cameraIndex),
    serialNumber(serialNumber)
{
}


ZedGmslRgbOnlyFrameSource::
~ZedGmslRgbOnlyFrameSource()
{
    stop();
}


bool
ZedGmslRgbOnlyFrameSource::start()
{
#ifdef HAGIE_ENABLE_ZED_SDK

    sl::InitParameters initParameters;


    /*
     * Abrir la ZED física por número de serie.
     */
    initParameters.input.setFromSerialNumber(
        serialNumber
    );


    /*
     * Misma resolución y FPS que las cámaras delanteras.
     */
    initParameters.camera_resolution =
        sl::RESOLUTION::HD1200;


    initParameters.camera_fps =
        30;


    /*
     * Cámara trasera usada solamente para RGB.
     * No necesitamos cálculo de profundidad.
     */
    initParameters.depth_mode =
        sl::DEPTH_MODE::NONE;


    sl::ERROR_CODE error =
        camera.open(
            initParameters
        );


    if (error != sl::ERROR_CODE::SUCCESS)
    {
        running.store(false);

        return false;
    }


    running.store(true);

    return true;

#else

    /*
     * ThinkPad / build sin ZED SDK.
     */
    running.store(false);

    return false;

#endif
}


void
ZedGmslRgbOnlyFrameSource::stop()
{
#ifdef HAGIE_ENABLE_ZED_SDK

    if (camera.isOpened())
    {
        camera.close();
    }

#endif

    running.store(false);
}


bool
ZedGmslRgbOnlyFrameSource::isRunning() const
{
    return running.load();
}


bool
ZedGmslRgbOnlyFrameSource::getFrame(
    Frame& frame)
{
    frame =
        Frame {};


    if (!running.load())
    {
        return false;
    }


#ifdef HAGIE_ENABLE_ZED_SDK

    /*
     * Capturar un nuevo frame.
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
     * Timestamp correspondiente a esta captura.
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
     * Obtener solamente la imagen RGB izquierda.
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


    frame.width =
        static_cast<std::size_t>(
            zedRgbImage.getWidth()
        );


    frame.height =
        static_cast<std::size_t>(
            zedRgbImage.getHeight()
        );


    constexpr std::size_t RGB_CHANNELS =
        3;


    frame.data.resize(
        frame.width *
        frame.height *
        RGB_CHANNELS
    );


    /*
     * ZED entrega 4 componentes por píxel.
     * Hagie utiliza RGB888:
     *
     * R G B R G B ...
     */
    for (std::size_t y = 0;
         y < frame.height;
         ++y)
    {
        for (std::size_t x = 0;
             x < frame.width;
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
                    y * frame.width +
                    x
                ) * RGB_CHANNELS;


            frame.data[offset + 0] =
                pixel[2];

            frame.data[offset + 1] =
                pixel[1];

            frame.data[offset + 2] =
                pixel[0];
        }
    }


    frame.timestamp_ms =
        captureTimestampMs;


    frame.valid =
        true;


    return true;

#else

    return false;

#endif
}


std::size_t
ZedGmslRgbOnlyFrameSource::
getCameraIndex() const
{
    return cameraIndex;
}


uint32_t
ZedGmslRgbOnlyFrameSource::
getSerialNumber() const
{
    return serialNumber;
}
