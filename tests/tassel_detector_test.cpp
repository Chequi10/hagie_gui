#include <iostream>
#include <thread>
#include <chrono>

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


    /*
     * ========================================================
     * TEST NORMAL DEL DETECTOR SIMULADO
     * ========================================================
     */
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


        /*
         * Las cámaras traseras 6 y 7
         * tienen un retardo simulado
         * de 2 segundos.
         */
        if (camera >= 5)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    2100
                )
            );
        }


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
            << std::endl
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


        for (std::size_t i = 0;
             i < result.detections.size();
             ++i)
        {
            const auto& detection =
                result.detections[i];


            std::cout
                << "  Panoja "
                << (i + 1)
                << " -> cuerpo "
                << (detection.body_index + 1)
                << std::endl;
        }


        /*
         * Verificación cámaras frontales.
         */
        if (camera < 5 &&
            result.detections.size() >= 2)
        {
            const std::size_t expectedBody1 =
                camera;

            const std::size_t expectedBody2 =
                camera + 1;


            if (result.detections[0].body_index !=
                expectedBody1)
            {
                testOk =
                    false;
            }


            if (result.detections[1].body_index !=
                expectedBody2)
            {
                testOk =
                    false;
            }
        }


        source.stop();
    }


    /*
     * ========================================================
     * TEST DIRECTO DE ZONAS TRASERAS
     * ========================================================
     *
     * Imagen simulada:
     * ancho = 300 píxeles.
     *
     * x = 50  -> primer tercio
     * x = 150 -> segundo tercio
     * x = 250 -> tercer tercio
     */
    constexpr std::size_t TEST_WIDTH =
        300;


    std::cout
        << std::endl
        << "=== TEST ZONAS TRASERAS ==="
        << std::endl;


    /*
     * Cámara trasera izquierda.
     * Debe cubrir cuerpos 1, 2 y 3.
     */
    const std::size_t leftBody1 =
        TasselDetector::bodyFromImagePosition(
            5,
            50,
            TEST_WIDTH
        );

    const std::size_t leftBody2 =
        TasselDetector::bodyFromImagePosition(
            5,
            150,
            TEST_WIDTH
        );

    const std::size_t leftBody3 =
        TasselDetector::bodyFromImagePosition(
            5,
            250,
            TEST_WIDTH
        );


    std::cout
        << "Cámara 6 izquierda -> cuerpo "
        << (leftBody1 + 1)
        << std::endl;

    std::cout
        << "Cámara 6 centro    -> cuerpo "
        << (leftBody2 + 1)
        << std::endl;

    std::cout
        << "Cámara 6 derecha   -> cuerpo "
        << (leftBody3 + 1)
        << std::endl;


    if (leftBody1 != 0 ||
        leftBody2 != 1 ||
        leftBody3 != 2)
    {
        std::cerr
            << "ERROR en zonas cámara 6"
            << std::endl;

        testOk =
            false;
    }


    /*
     * Cámara trasera derecha.
     * Debe cubrir cuerpos 4, 5 y 6.
     */
    const std::size_t rightBody1 =
        TasselDetector::bodyFromImagePosition(
            6,
            50,
            TEST_WIDTH
        );

    const std::size_t rightBody2 =
        TasselDetector::bodyFromImagePosition(
            6,
            150,
            TEST_WIDTH
        );

    const std::size_t rightBody3 =
        TasselDetector::bodyFromImagePosition(
            6,
            250,
            TEST_WIDTH
        );


    std::cout
        << "Cámara 7 izquierda -> cuerpo "
        << (rightBody1 + 1)
        << std::endl;

    std::cout
        << "Cámara 7 centro    -> cuerpo "
        << (rightBody2 + 1)
        << std::endl;

    std::cout
        << "Cámara 7 derecha   -> cuerpo "
        << (rightBody3 + 1)
        << std::endl;


    if (rightBody1 != 3 ||
        rightBody2 != 4 ||
        rightBody3 != 5)
    {
        std::cerr
            << "ERROR en zonas cámara 7"
            << std::endl;

        testOk =
            false;
    }


    /*
     * ========================================================
     * RESULTADO FINAL
     * ========================================================
     */
    if (!testOk)
    {
        std::cerr
            << std::endl
            << "TEST ERROR"
            << std::endl;

        return 1;
    }


    std::cout
        << std::endl
        << "TEST OK"
        << std::endl;


    return 0;
}