#include "vision/simulated_rgb_frame_source.h"

#include <chrono>


SimulatedRgbFrameSource::SimulatedRgbFrameSource(
    std::size_t cameraIndex)
    : cameraIndex(cameraIndex),
      running(false)
{
}


bool SimulatedRgbFrameSource::start()
{
    running = true;

    return true;
}


void SimulatedRgbFrameSource::stop()
{
    running = false;
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
     *
     * Por ahora sólo sirve para verificar
     * que las 7 fuentes funcionan de forma
     * independiente.
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


    const auto now =
        std::chrono::steady_clock::now()
            .time_since_epoch();

    frame.timestamp_ms =
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(now).count()
        );

    frame.valid =
        true;


    return true;
}