#include "vision_height_source.h"


#include <chrono>


VisionHeightSource::VisionHeightSource(
    HagieState *state)
    : state(state),
      running(false)
{
}


VisionHeightSource::~VisionHeightSource()
{
    stop();
}


// ============================================================
// START
// ============================================================

bool VisionHeightSource::start()
{
    if (running)
    {
        return true;
    }


    running = true;


    /*
     * Informar que el módulo de visión
     * está funcionando.
     */
    if (state != nullptr)
    {
        HagieState::SystemState system =
            state->getSystemState();


        system.vision_running =
            true;


        state->setSystemState(
            system
        );
    }


    workerThread =
        std::thread(
            &VisionHeightSource::workerLoop,
            this
        );


    return true;
}


// ============================================================
// STOP
// ============================================================

void VisionHeightSource::stop()
{
    running = false;


    if (workerThread.joinable())
    {
        workerThread.join();
    }


    /*
     * Invalidar las mediciones de visión.
     */
    if (state != nullptr)
    {
        for (std::size_t body = 0;
             body < BODY_COUNT;
             ++body)
        {
            state->setBodyVisionValid(
                body,
                false
            );
        }


        HagieState::SystemState system =
            state->getSystemState();


        system.vision_running =
            false;


        state->setSystemState(
            system
        );
    }
}


// ============================================================
// ESTADO
// ============================================================

bool VisionHeightSource::isRunning() const
{
    return running;
}


// ============================================================
// OBTENER ÚLTIMO RESULTADO
// ============================================================

VisionHeightSource::VisionResult
VisionHeightSource::getResult() const
{
    std::lock_guard<std::mutex> lock(
        resultMutex
    );


    return result;
}


// ============================================================
// HILO PRINCIPAL
// ============================================================

void VisionHeightSource::workerLoop()
{
    using namespace std::chrono;


    while (running)
    {
        /*
         * Actualmente genera datos simulados.
         *
         * Después esto será reemplazado por:
         *
         * ZED
         *  ↓
         * nube de puntos
         *  ↓
         * zona correspondiente a cada cuerpo
         *  ↓
         * cálculo de altura
         */
        generateSimulatedResult();


        /*
         * Simulación a 10 Hz.
         */
        std::this_thread::sleep_for(
            milliseconds(100)
        );
    }
}


// ============================================================
// SIMULACIÓN
// ============================================================

void VisionHeightSource::generateSimulatedResult()
{
    /*
     * Alturas base simuladas.
     *
     * Estas representan ALTURA DE CULTIVO
     * detectada por visión.
     *
     * NO representan:
     *
     * - altura del cabezal;
     * - objetivo de la STM32.
     */
    static constexpr
    std::array<uint16_t, BODY_COUNT>
    BASE_HEIGHT_MM =
    {
        400,
        450,
        500,
        550,
        600,
        650
    };


    /*
     * Variación 0..40 mm.
     */
    uint16_t variation =
        static_cast<uint16_t>(
            simulationStep % 41
        );


    VisionResult newResult;


    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        if ((body % 2) == 0)
        {
            newResult
                .bodies[body]
                .height_mm =
                    BASE_HEIGHT_MM[body]
                    + variation;
        }
        else
        {
            newResult
                .bodies[body]
                .height_mm =
                    BASE_HEIGHT_MM[body]
                    - variation;
        }


        newResult
            .bodies[body]
            .valid =
                true;
    }


    /*
     * Actualizar secuencia.
     */
    {
        std::lock_guard<std::mutex> lock(
            resultMutex
        );


        newResult.sequence =
            result.sequence + 1;


        result =
            newResult;
    }


    /*
     * Publicar en estado central.
     */
    publishResult(
        newResult
    );


    simulationStep++;


    if (simulationStep >= 1000000U)
    {
        simulationStep = 0;
    }
}


// ============================================================
// PUBLICAR EN HAGIESTATE
// ============================================================

void VisionHeightSource::publishResult(
    const VisionResult& newResult)
{
    if (state == nullptr)
    {
        return;
    }


    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        state->setBodyVisionHeight(
            body,
            newResult
                .bodies[body]
                .height_mm,
            newResult
                .bodies[body]
                .valid
        );
    }
}