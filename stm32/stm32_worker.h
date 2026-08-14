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
        CLEAR_BODY_FAULT
    };

    struct Command
    {
        CommandType type;

        uint8_t body = 0;
        int16_t value = 0;
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