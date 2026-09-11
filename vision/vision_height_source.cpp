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
// SELECCIÓN DE FUENTE DE VISIÓN
// ============================================================

void VisionHeightSource::setSourceMode(
    SourceMode mode)
{
    sourceMode.store(
        mode
    );


    /*
     * Al cambiar de fuente invalidamos inmediatamente
     * las mediciones anteriores.
     *
     * Esto evita que AUTO VISIÓN pueda utilizar
     * accidentalmente un dato perteneciente a
     * la fuente anterior.
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
    }
}


VisionHeightSource::SourceMode
VisionHeightSource::getSourceMode() const
{
    return sourceMode.load();
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
// RECIBIR RESULTADO DE VISIÓN REAL
// ============================================================

void VisionHeightSource::submitResult(
    const VisionResult& newResult)
{
    VisionResult resultCopy =
        newResult;


    /*
     * Actualizar secuencia y almacenar
     * el último resultado recibido.
     */
    {
        std::lock_guard<std::mutex> lock(
            resultMutex
        );


        resultCopy.sequence =
            result.sequence + 1;


        result =
            resultCopy;
    }


    /*
     * Publicar en HagieState.
     *
     * Este es el mismo camino que después
     * utilizarán las cámaras reales.
     */
    publishResult(
        resultCopy
    );
}


// ============================================================
// HILO PRINCIPAL
// ============================================================

void VisionHeightSource::workerLoop()
{
    using namespace std::chrono;


    while (running)
    {
        SourceMode mode =
            sourceMode.load();


        if (mode == SourceMode::SIMULATION)
        {
            /*
            * Fuente simulada.
            */
            generateSimulatedResult();
        }
        else
        {
            /*
            * Fuente EXTERNAL.
            *
            * No generamos ningún dato aquí.
            *
            * Las cámaras 3D reales entregarán
            * los resultados mediante:
            *
            * submitResult(...)
            */
        }


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
     * Variación simulada 0..40 mm.
     */
    /*
    * ============================================================
    * PERFIL DE ALTURA SIMULADA POR ESCALONES
    * ============================================================
    *
    * workerLoop() corre cada 100 ms.
    *
    * 50 pasos  = 5 segundos.
    *
    * Esto permite observar claramente en el Trend:
    *
    * - cambio de objetivo;
    * - retardo del encoder;
    * - tiempo de establecimiento.
    */

    const uint32_t phase =
        simulationStep % 250U;


    /*
    * Offset común por etapa.
    */
    int32_t stepOffsetMm =
        0;


    if (phase < 50U)
    {
        /*
        * 0 a 5 s
        */
        stepOffsetMm =
            0;
    }
    else if (phase < 100U)
    {
        /*
        * 5 a 10 s
        */
        stepOffsetMm =
            100;
    }
    else if (phase < 150U)
    {
        /*
        * 10 a 15 s
        */
        stepOffsetMm =
            -50;
    }
    else if (phase < 200U)
    {
        /*
        * 15 a 20 s
        */
        stepOffsetMm =
            150;
    }
    else
    {
        /*
        * 20 a 25 s
        */
        stepOffsetMm =
            20;
    }


    /*
     * Timestamp monotónico actual.
     *
     * Se utiliza steady_clock porque no depende
     * de cambios en la hora del sistema.
     */
    uint64_t nowMs =
        static_cast<uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()
                ).count()
        );


    VisionResult newResult;


    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        /*
         * Cuerpos pares suben con la variación.
         * Cuerpos impares bajan.
         *
         * Esto solamente sirve para reconocer
         * visualmente que cada cuerpo recibe
         * información independiente.
         */
        /*
        * Cada cuerpo parte de una altura base distinta.
        *
        * Todos reciben escalones,
        * pero con una pequeña diferencia entre cuerpos
        * para comprobar que los históricos son independientes.
        */
        int32_t simulatedHeight =
            static_cast<int32_t>(
                BASE_HEIGHT_MM[body]
            )
            +
            stepOffsetMm;


        /*
        * Pequeña separación adicional entre respuestas
        * para que no sean idénticas.
        */
        if ((body % 2) == 0)
        {
            simulatedHeight +=
                static_cast<int32_t>(
                    body * 5
                );
        }
        else
        {
            simulatedHeight -=
                static_cast<int32_t>(
                    body * 5
                );
        }


        /*
        * Protección.
        */
        if (simulatedHeight < 0)
        {
            simulatedHeight =
                0;
        }


        newResult
            .bodies[body]
            .height_mm =
                static_cast<uint16_t>(
                    simulatedHeight
                );

        newResult
            .bodies[body]
            .valid =
                true;


        newResult
            .bodies[body]
            .timestamp_ms =
                nowMs;
    }


    /*
     * IMPORTANTE:
     *
     * La simulación también utiliza submitResult().
     *
     * De esta forma existe un único camino para:
     *
     * - simulación;
     * - cámaras reales.
     *
     * Más adelante solamente cambia quién genera
     * VisionResult.
     */
    submitResult(
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
                .valid,
            newResult
                .bodies[body]
                .timestamp_ms
        );
    }
}