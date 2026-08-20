#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "core/hagie_state.h"
#include "vision/vision_3d_processor.h"
#include "vision/vision_3d_worker.h"
#include "vision/vision_height_source.h"

#include "tests/synthetic_point_cloud_source_test.h"


int main()
{
    std::cout
        << "=== TEST VISION 3D WORKER ==="
        << std::endl;


    /*
     * ========================================================
     * ESTADO CENTRAL
     * ========================================================
     */
    HagieState state;


    /*
     * ========================================================
     * FUENTE DE ALTURAS
     * ========================================================
     *
     * Para este test NO queremos que genere
     * alturas simuladas internamente.
     *
     * Los datos vendrán del Vision3DWorker.
     */
    VisionHeightSource heightSource(
        &state
    );

    heightSource.setSourceMode(
        VisionHeightSource::SourceMode::EXTERNAL
    );


    if (!heightSource.start())
    {
        std::cerr
            << "ERROR: no se pudo iniciar VisionHeightSource"
            << std::endl;

        return 1;
    }


    /*
     * ========================================================
     * PROCESADOR 3D
     * ========================================================
     */
    Vision3DProcessor processor;


    /*
     * ========================================================
     * CONFIGURACIÓN DE LAS 3 CÁMARAS
     * ========================================================
     *
     * Cámara 0 -> cuerpos 0 y 1
     * Cámara 1 -> cuerpos 2 y 3
     * Cámara 2 -> cuerpos 4 y 5
     */
    for (std::size_t camera = 0;
         camera < Vision3DProcessor::CAMERA_COUNT;
         ++camera)
    {
        Vision3DProcessor::CameraConfig config;


        config.enabled =
            true;


        /*
         * Primero deshabilitar todos los cuerpos.
         */
        for (std::size_t body = 0;
             body < HagieState::BODY_COUNT;
             ++body)
        {
            config.body_enabled[body] =
                false;
        }


        /*
         * Dos cuerpos por cámara.
         */
        std::size_t firstBody =
            camera * 2;


        config.body_enabled[firstBody] =
            true;

        config.body_enabled[firstBody + 1] =
            true;


        processor.setCameraConfig(
            camera,
            config
        );
    }


    /*
     * ========================================================
     * WORKER
     * ========================================================
     */
    Vision3DWorker worker(
        &state,
        &processor,
        &heightSource
    );


    /*
     * ========================================================
     * CÁMARAS SINTÉTICAS
     * ========================================================
     */
    for (std::size_t camera = 0;
         camera < Vision3DProcessor::CAMERA_COUNT;
         ++camera)
    {
        auto source =
            std::make_unique<
                SyntheticPointCloudSource
            >(
                camera
            );


        if (!worker.setPointCloudSource(
                camera,
                std::move(source)
            ))
        {
            std::cerr
                << "ERROR instalando fuente de cámara "
                << camera
                << std::endl;


            heightSource.stop();

            return 1;
        }
    }


    /*
     * ========================================================
     * INICIAR WORKER
     * ========================================================
     */
    if (!worker.start())
    {
        std::cerr
            << "ERROR: no se pudo iniciar Vision3DWorker"
            << std::endl;


        heightSource.stop();

        return 1;
    }


    /*
     * Dar tiempo para procesar varias iteraciones.
     */
    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            500
        )
    );


    /*
     * ========================================================
     * LEER RESULTADO DESDE HAGIESTATE
     * ========================================================
     */
    bool testOk =
        true;


    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        HagieState::BodyState bodyState =
            state.getBodyState(
                body
            );


        std::cout
            << "Cuerpo "
            << (body + 1)
            << ": altura = "
            << bodyState.vision_height_mm
            << " mm"
            << " | valid = "
            << (
                bodyState.vision_valid
                    ? "SI"
                    : "NO"
            )
            << std::endl;


        if (!bodyState.vision_valid)
        {
            testOk =
                false;
        }
    }


    /*
     * ========================================================
     * CIERRE
     * ========================================================
     */
    worker.stop();

    heightSource.stop();


    /*
     * ========================================================
     * RESULTADO
     * ========================================================
     */
    if (!testOk)
    {
        std::cerr
            << "TEST FALLIDO"
            << std::endl;

        return 1;
    }


    std::cout
        << "TEST OK"
        << std::endl;


    return 0;
}