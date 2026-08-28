#include <iostream>
#include <memory>

#include "vision/rgb_camera_worker.h"
#include "vision/simulated_rgb_frame_source.h"


int main()
{
    std::cout
        << "=== TEST RGB CAMERA WORKER ==="
        << std::endl;


    RgbCameraWorker worker;


    /*
     * ========================================================
     * INSTALAR LAS 7 CÁMARAS RGB SIMULADAS
     * ========================================================
     */
    for (std::size_t camera = 0;
         camera < RgbCameraWorker::CAMERA_COUNT;
         ++camera)
    {
        auto source =
            std::make_unique<
                SimulatedRgbFrameSource
            >(camera);


        if (!worker.setFrameSource(
                camera,
                std::move(source)
            ))
        {
            std::cerr
                << "ERROR instalando cámara "
                << (camera + 1)
                << std::endl;

            return 1;
        }
    }


    /*
     * ========================================================
     * INICIAR
     * ========================================================
     */
    if (!worker.start())
    {
        std::cerr
            << "ERROR iniciando RgbCameraWorker"
            << std::endl;

        return 1;
    }


    bool testOk =
        true;


    /*
     * ========================================================
     * LEER UN FRAME DE CADA CÁMARA
     * ========================================================
     */
    for (std::size_t camera = 0;
         camera < RgbCameraWorker::CAMERA_COUNT;
         ++camera)
    {
        RgbFrameSource::Frame frame;


        const bool ok =
            worker.getFrame(
                camera,
                frame
            );


        std::cout
            << "Cámara "
            << (camera + 1)
            << ": ";


        if (!ok ||
            !frame.valid)
        {
            std::cout
                << "ERROR"
                << std::endl;

            testOk =
                false;

            continue;
        }


        std::cout
            << frame.width
            << "x"
            << frame.height
            << " RGB"
            << " | bytes = "
            << frame.data.size()
            << " | timestamp = "
            << frame.timestamp_ms
            << " ms"
            << std::endl;


        if (frame.width != 320 ||
            frame.height != 180 ||
            frame.data.size() !=
                320 * 180 * 3)
        {
            testOk =
                false;
        }
    }


    worker.stop();


    if (!testOk)
    {
        std::cerr
            << "TEST ERROR"
            << std::endl;

        return 1;
    }


    std::cout
        << "TEST OK"
        << std::endl;


    return 0;
}