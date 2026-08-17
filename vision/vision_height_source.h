#ifndef VISION_HEIGHT_SOURCE_H
#define VISION_HEIGHT_SOURCE_H


#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>


#include "core/hagie_state.h"


class VisionHeightSource
{
public:

    static constexpr std::size_t BODY_COUNT = 6;


    struct BodyVisionResult
    {
        uint16_t height_mm = 0;

        bool valid = false;
    };


    struct VisionResult
    {
        std::array<
            BodyVisionResult,
            BODY_COUNT
        > bodies {};

        uint64_t sequence = 0;
    };


    /*
     * Recibimos HagieState para publicar allí
     * los resultados de visión.
     */
    explicit VisionHeightSource(
        HagieState *state
    );


    ~VisionHeightSource();


    VisionHeightSource(
        const VisionHeightSource&
    ) = delete;


    VisionHeightSource& operator=(
        const VisionHeightSource&
    ) = delete;


    // ========================================================
    // CONTROL
    // ========================================================

    bool start();

    void stop();

    bool isRunning() const;


    // ========================================================
    // RESULTADO
    // ========================================================

    VisionResult getResult() const;


private:

    // ========================================================
    // HILO
    // ========================================================

    void workerLoop();


    // ========================================================
    // SIMULACIÓN
    // ========================================================

    void generateSimulatedResult();


    /*
     * Publicar el último resultado en HagieState.
     */
    void publishResult(
        const VisionResult& newResult
    );


    // ========================================================
    // ESTADO CENTRAL
    // ========================================================

    HagieState *state;


    // ========================================================
    // CONTROL DEL HILO
    // ========================================================

    std::atomic<bool> running;

    std::thread workerThread;


    // ========================================================
    // ÚLTIMO RESULTADO
    // ========================================================

    mutable std::mutex resultMutex;

    VisionResult result;


    // ========================================================
    // SIMULACIÓN
    // ========================================================

    uint32_t simulationStep = 0;
};


#endif