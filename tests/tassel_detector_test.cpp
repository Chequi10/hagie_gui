#include <iostream>

#include "ai/tassel_detector.h"
#include "vision/simulated_rgb_frame_source.h"


int main()
{
    std::cout
        << "=== TEST TASSEL DETECTOR ==="
        << std::endl;


    TasselDetector detector;

    bool testOk =
        true;


    for (std::size_t camera = 0;
         camera < 7;
         ++camera)
    {
        SimulatedRgbFrameSource source(
            camera
        );

        if (!source.start())
        {
            std::cerr
                << "ERROR iniciando cámara "
                << (camera + 1)
                << std::endl;

            return 1;
        }


        RgbFrameSource::Frame frame;

        if (!source.getFrame(frame))
        {
            std::cerr
                << "ERROR obteniendo frame cámara "
                << (camera + 1)
                << std::endl;

            return 1;
        }


        TasselDetector::Result result;

        if (!detector.processFrame(
                camera,
                frame,
                result
            ))
        {
            std::cerr
                << "ERROR procesando cámara "
                << (camera + 1)
                << std::endl;

            testOk =
                false;

            continue;
        }


        const std::size_t expectedCount =
            camera < 5
                ? 2
                : 1;


        std::cout
            << "Cámara "
            << (camera + 1)
            << ": "
            << result.detections.size()
            << " panojas detectadas"
            << std::endl;


        if (result.detections.size() !=
            expectedCount)
        {
            std::cerr
                << "ERROR: esperadas "
                << expectedCount
                << std::endl;

            testOk =
                false;
        }


        source.stop();
    }


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