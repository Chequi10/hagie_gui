#ifndef STM32_WORKER_H
#define STM32_WORKER_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "core/hagie_state.h"
#include "stm32/stm32canbusif.h"

#include <condition_variable>
#include <deque>
#include <mutex>


class STM32Worker
{
public:
    STM32Worker(
        HagieState *state,
        const std::string& port,
        uint32_t baudrate = 115200
    );

    ~STM32Worker();

    /*
     * No permitimos copiar este objeto.
     *
     * STM32Worker posee un hilo y una interfaz serie,
     * por lo que debe existir una única instancia.
     */
    STM32Worker(const STM32Worker&) = delete;
    STM32Worker& operator=(const STM32Worker&) = delete;


    // ========================================================
    // Control del worker
    // ========================================================

    bool start();

    void stop();

    bool isRunning() const;


    // ========================================================
    // Comandos hacia STM32
    // ========================================================

    void setTargetHeight(
        uint8_t body,
        uint16_t height_mm
    );

    void setValveCommand(
        uint8_t body,
        int16_t command
    );

    void stopAllValves();

    void clearBodyFault(
        uint8_t body
    );

    // ========================================================
    // Configuración runtime STM32 - OPCODE 'K'
    // ========================================================

    void setBodyLimits(
        uint8_t body,
        uint16_t min_height_mm,
        uint16_t max_height_mm
    );

    void setMoveCommandThreshold(
        uint16_t threshold
    );

    void setMinBodyMovement(
        float movement_mm
    );

    void setNoMovementTimeout(
        uint32_t timeout_ms
    );

    void setTargetTimeout(
        uint32_t timeout_ms
    );

    void setEncoderDirection(
        uint8_t body,
        uint8_t direction
    );

    void setEncoderScale(
        uint8_t body,
        float mm_per_pulse
    );


private:
    // ========================================================
    // Hilo principal STM32
    // ========================================================

    void workerLoop();


    // ========================================================
    // Cola de comandos TX
    // ========================================================

    enum class CommandType
    {
        SET_TARGET_HEIGHT,
        SET_VALVE_COMMAND,
        STOP_ALL_VALVES,
        CLEAR_BODY_FAULT,

        // OPCODE 'K'
        SET_BODY_LIMITS,
        SET_MOVE_COMMAND_THRESHOLD,
        SET_MIN_BODY_MOVEMENT,
        SET_NO_MOVEMENT_TIMEOUT,
        SET_TARGET_TIMEOUT,
        SET_ENCODER_DIRECTION,
        SET_ENCODER_SCALE
    };

    struct Command
    {
        CommandType type;

        uint8_t body = 0;

        /*
        * Usado por comandos de válvula
        * y algunos valores simples.
        */
        int16_t value = 0;

        /*
        * Dos uint16_t para límites min/max.
        */
        uint16_t value_u16_1 = 0;
        uint16_t value_u16_2 = 0;

        /*
        * Timeouts.
        */
        uint32_t value_u32 = 0;

        /*
        * Movimiento mínimo y escala encoder.
        */
        float value_float = 0.0f;
    };

    void enqueueCommand(
        const Command& command
    );

    void processTxQueue();


    // ========================================================
    // Configuración de callbacks RX
    // ========================================================

    void configureCallbacks();


    // ========================================================
    // Estado compartido
    // ========================================================

    HagieState *state;


    // ========================================================
    // Puerto serie
    // ========================================================

    std::string port;
    uint32_t baudrate;


    // ========================================================
    // Interfaz existente STM32
    // ========================================================

    std::unique_ptr<stm32canbus_serialif> stm32;


    // ========================================================
    // Control del hilo
    // ========================================================

    std::atomic<bool> running;

    std::thread workerThread;
    std::mutex txMutex;

    std::deque<Command> txQueue;
};


#endif