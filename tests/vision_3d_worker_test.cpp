#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <cmath>

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
    SyntheticPointCloudSource *camera0Source =
        nullptr;


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


        /*
         * Cámara 0 con IMU simulada.
         *
         * Roll  = +3.5°
         * Pitch = -0.2°
         */
        if (camera == 0)
        {
            camera0Source =
                source.get();

            camera0Source
                ->setSimulatedOrientation(
                    3.5f,
                    -0.2f
                );
        }


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
     * IMU GENERAL SIMULADA DE LA HAGIE
     * ========================================================
     *
     * Roll  = +2.0°
     * Pitch = -1.0°
     *
     * Con la cámara 0:
     *
     * Error Roll:
     * 3.5 - 2.0 = +1.5°
     *
     * Error Pitch:
     * -0.2 - (-1.0) = +0.8°
     */
        /*
     * ========================================================
     * IMU GENERAL NEUTRA PARA TEST DE ALTURAS
     * ========================================================
     *
     * No queremos que la corrección de inclinación
     * modifique las alturas sintéticas originales.
     */
    HagieState::ImuState imuState;

    imuState.valid =
        true;

    imuState.roll_deg =
        0.0f;

    imuState.pitch_deg =
        0.0f;

    state.setImuState(
        imuState
    );


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


    bool testOk =
        true;


    /*
     * ========================================================
     * TEST 14 - ERROR DE MONTAJE POR IMU
     * ========================================================
    /*
     * Para este test solamente cambiamos la IMU
     * general de la Hagie.
     *
     * La cámara 0 ya está simulando:
     *
     * Roll  = +3.5°
     * Pitch = -0.2°
     */
    HagieState::ImuState mountingTestImu;

    mountingTestImu.valid =
        true;

    mountingTestImu.roll_deg =
        2.0f;

    mountingTestImu.pitch_deg =
        -1.0f;

    state.setImuState(
        mountingTestImu
    );   


    std::cout
        << std::endl
        << "========================================"
        << std::endl
        << "TEST 14 - ERROR DE MONTAJE POR IMU"
        << std::endl
        << "========================================"
        << std::endl;


    float rollOffsetDeg =
        0.0f;

    float pitchOffsetDeg =
        0.0f;


    bool mountingOffsetValid =
        worker.getCameraMountingOffset(
            0,
            rollOffsetDeg,
            pitchOffsetDeg
        );


    std::cout
        << "IMU Hagie: "
        << "Roll=2.00 deg "
        << "Pitch=-1.00 deg"
        << std::endl;


    std::cout
        << "IMU Camara 1: "
        << "Roll=3.50 deg "
        << "Pitch=-0.20 deg"
        << std::endl;


    if (!mountingOffsetValid)
    {
        std::cout
            << "Resultado: NO VALIDO"
            << std::endl;

        testOk =
            false;
    }
    else
    {
        std::cout
            << "Error montaje: "
            << "Roll="
            << rollOffsetDeg
            << " deg "
            << "Pitch="
            << pitchOffsetDeg
            << " deg"
            << std::endl;


        constexpr float TOLERANCE_DEG =
            0.01f;


        if (
            std::fabs(
                rollOffsetDeg - 1.5f
            ) > TOLERANCE_DEG
            ||
            std::fabs(
                pitchOffsetDeg - 0.8f
            ) > TOLERANCE_DEG
        )
        {
            std::cout
                << "TEST 14 FALLIDO"
                << std::endl;

            testOk =
                false;
        }
        else
        {
            std::cout
                << "TEST 14 OK"
                << std::endl;
        }
    }

    /*
     * Restaurar IMU neutra para no contaminar
     * el resto de la prueba.
     */
    state.setImuState(
        imuState
    );
    /*
     * ========================================================
     * LEER RESULTADO DESDE HAGIESTATE
     * ========================================================
     */
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
     * RESULTADO FINAL
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