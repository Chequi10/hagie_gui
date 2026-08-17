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
#include <array>
#include <chrono>


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

    enum class ConfigSyncStatus
    {
        PENDING,
        SYNCHRONIZED,
        ERROR
    };

    ConfigSyncStatus getConfigSyncStatus() const;

    void beginConfigurationSync();

   


private:
     // ========================================================
    // Copia de configuración para resincronizar al reconectar
    // ========================================================

    struct RuntimeConfiguration
    {
        bool valid = false;

        std::array<uint16_t, HagieState::BODY_COUNT>
            min_height_mm {};

        std::array<uint16_t, HagieState::BODY_COUNT>
            max_height_mm {};

        std::array<uint8_t, HagieState::BODY_COUNT>
            encoder_direction {};

        std::array<float, HagieState::BODY_COUNT>
            encoder_scale {};

        uint16_t move_command_threshold = 100;

        float min_body_movement_mm = 2.0f;

        uint32_t no_movement_timeout_ms = 1000;

        uint32_t target_timeout_ms = 1000;
    };


    RuntimeConfiguration runtimeConfig;


    /*
    * Reenvía la última configuración conocida
    * después de una reconexión STM32.
    */
    void enqueueRuntimeConfiguration();   
    
    struct ConfigAckState
    {
        bool received = false;

        uint8_t subcommand = 0;
        uint8_t body = 0xFF;
        uint8_t status = 0;

        uint32_t value1 = 0;
        uint32_t value2 = 0;
    };

    ConfigAckState lastConfigAck;

    mutable std::mutex configAckMutex;

    /*
    * ========================================================
    * Seguimiento de ACK de configuración
    * ========================================================
    *
    * Para considerar la configuración sincronizada deben
    * confirmarse:
    *
    * K01 x 6
    * K02
    * K03
    * K04
    * K05
    * K06 x 6
    * K07 x 6
    *
    * Total: 22 ACK únicos.
    */

    std::array<bool, HagieState::BODY_COUNT>
        configAckLimits {};

    std::array<bool, HagieState::BODY_COUNT>
        configAckDirection {};

    std::array<bool, HagieState::BODY_COUNT>
        configAckScale {};

    bool configAckMoveThreshold = false;
    bool configAckMinMovement = false;
    bool configAckNoMovementTimeout = false;
    bool configAckTargetTimeout = false;

    ConfigSyncStatus configSyncStatus =
        ConfigSyncStatus::PENDING;
  

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

    std::mutex configMutex;

    /*
    * ========================================================
    * Sincronización secuencial de configuración
    * ========================================================
    *
    * Se envía un solo K por vez.
    * El siguiente se envía únicamente cuando llega
    * el ACK correspondiente.
    *
    * Secuencia:
    *
    *  0..5   -> K01 cuerpos 0..5
    *  6      -> K02
    *  7      -> K03
    *  8      -> K04
    *  9      -> K05
    * 10..15  -> K06 cuerpos 0..5
    * 16..21  -> K07 cuerpos 0..5
    *
    * 22      -> terminada
    */

    static constexpr uint8_t CONFIG_SYNC_COMMAND_COUNT = 22;

    uint8_t configSyncStep = 0;

    bool configCommandPending = false;

    uint8_t configRetryCount = 0;

    std::chrono::steady_clock::time_point
        configCommandSentTime;


    /*
    * Enviar únicamente el comando correspondiente
    * al paso actual de sincronización.
    */
    void sendCurrentConfigurationCommand();


    /*
    * Procesar timeout / reintento de la
    * sincronización de configuración.
    */
    void processConfigurationSync();


    
};


#endif