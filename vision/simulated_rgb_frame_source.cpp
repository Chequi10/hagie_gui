#include "vision/simulated_rgb_frame_source.h"

#include <chrono>


namespace
{

std::uint64_t getCurrentTimestampMs()
{
    const auto now =
        std::chrono::steady_clock::now()
            .time_since_epoch();


    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(now).count()
    );
}

}


SimulatedRgbFrameSource::SimulatedRgbFrameSource(
    std::size_t cameraIndex)
    : cameraIndex(cameraIndex),
      running(false)
{
}


bool SimulatedRgbFrameSource::start()
{
    startTimestampMs =
        getCurrentTimestampMs();

    running =
        true;

    return true;
}


void SimulatedRgbFrameSource::stop()
{
    running =
        false;
}


bool SimulatedRgbFrameSource::isRunning() const
{
    return running;
}


std::size_t
SimulatedRgbFrameSource::getCameraIndex() const
{
    return cameraIndex;
}


bool SimulatedRgbFrameSource::getFrame(
    Frame& frame)
{
    if (!running)
    {
        frame =
            Frame {};

        return false;
    }


    const std::uint64_t nowMs =
        getCurrentTimestampMs();


    /*
     * ========================================================
     * RETARDO SIMULADO DE CÁMARAS TRASERAS
     * ========================================================
     *
     * Cámaras físicas:
     *
     * 0..4 = frontales
     * 5..6 = traseras
     *
     * Las traseras comienzan a entregar imágenes
     * 2000 ms después para simular el avance de
     * la máquina desde adelante hacia atrás.
     */

    constexpr std::uint64_t REAR_CAMERA_DELAY_MS =
        2000;


    if (cameraIndex >= 5 &&
        nowMs < startTimestampMs +
                REAR_CAMERA_DELAY_MS)
    {
        frame =
            Frame {};

        return false;
    }


    constexpr std::size_t WIDTH =
        320;

    constexpr std::size_t HEIGHT =
        180;

    constexpr std::size_t CHANNELS =
        3;


    frame.width =
        WIDTH;

    frame.height =
        HEIGHT;

    frame.data.resize(
        WIDTH *
        HEIGHT *
        CHANNELS
    );


    /*
     * Cada cámara genera una imagen RGB
     * de un tono diferente.
     */
    const std::uint8_t baseValue =
        static_cast<std::uint8_t>(
            25 +
            (cameraIndex * 30)
        );


    for (std::size_t pixel = 0;
         pixel < WIDTH * HEIGHT;
         ++pixel)
    {
        const std::size_t offset =
            pixel * CHANNELS;

        frame.data[offset + 0] =
            baseValue;

        frame.data[offset + 1] =
            static_cast<std::uint8_t>(
                255 - baseValue
            );

        frame.data[offset + 2] =
            static_cast<std::uint8_t>(
                50 +
                (cameraIndex * 20)
            );
    }


    frame.timestamp_ms =
        nowMs;

    frame.valid =
        true;


    return true;
}