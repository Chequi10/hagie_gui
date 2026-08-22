#include "stm32_worker.h"

#include <chrono>
#include <exception>
#include <iostream>
#include <utility>


STM32Worker::STM32Worker(
    HagieState *state,
    const std::string& port,
    uint32_t baudrate)
    : state(state),
      port(port),
      baudrate(baudrate),
      stm32(nullptr),
      running(false)
{
}


STM32Worker::~STM32Worker()
{
    stop();
}


// ============================================================
// START / STOP
// ============================================================

bool STM32Worker::start()
{
    if (running)
    {
        return true;
    }


    try
    {
        stm32 =
            std::make_unique<
                stm32canbus_serialif
            >(
                port,
                baudrate
            );


        configureCallbacks();


        stm32->start();


        running = true;


        workerThread =
            std::thread(
                &STM32Worker::workerLoop,
                this
            );


        /*
         * La interfaz fue creada correctamente.
         *
         * El estado real de comunicación seguirá
         * siendo actualizado por STM32.
         */
        notifyConnectionState(
            true
        );


        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "STM32Worker start error: "
            << e.what()
            << std::endl;


        stm32.reset();


        running = false;


        if (state != nullptr)
        {
            HagieState::SystemState system =
                state->getSystemState();


            system.stm32_connected =
                false;


            state->setSystemState(
                system
            );
        }


        notifyConnectionState(
            false
        );


        return false;
    }
}


void STM32Worker::stop()
{
    if (!running)
    {
        if (stm32)
        {
            stm32->stop();

            stm32.reset();
        }


        notifyConnectionState(
            false
        );


        return;
    }


    running = false;


    /*
     * Primero intentar dejar el sistema seguro.
     */
    if (stm32)
    {
        try
        {
            stm32->stop_all_valves();
        }
        catch (...)
        {
        }
    }


    if (workerThread.joinable())
    {
        workerThread.join();
    }


    if (stm32)
    {
        stm32->stop();

        stm32.reset();
    }


    if (state != nullptr)
    {
        HagieState::SystemState system =
            state->getSystemState();


        system.stm32_connected =
            false;


        state->setSystemState(
            system
        );
    }


    notifyConnectionState(
        false
    );
}


bool STM32Worker::isRunning() const
{
    return running;
}


// ============================================================
// CALLBACK DE CONEXIÓN
// ============================================================

void STM32Worker::setConnectionCallback(
    connection_callback callback)
{
    onConnection =
        std::move(callback);
}


void STM32Worker::notifyConnectionState(
    bool connected)
{
    /*
     * ========================================================
     * PÉRDIDA DE STM32
     * ========================================================
     *
     * Nunca conservar órdenes de movimiento pendientes.
     */
    if (!connected)
    {
        {
            std::lock_guard<std::mutex> lock(
                txMutex
            );

            txQueue.clear();
        }

        /*
         * La representación local también queda
         * inmediatamente en MANUAL.
         */
        if (state != nullptr)
        {
            for (std::size_t body = 0;
                 body < HagieState::BODY_COUNT;
                 ++body)
            {
                state->setBodyAutoMode(
                    body,
                    false
                );

                state->setBodyValveCommand(
                    body,
                    0
                );
            }
        }
    }


    /*
     * Avisar a MainWindow.
     *
     * Este callback puede ejecutarse desde el
     * hilo STM32. MainWindow lo pasará al hilo Qt.
     */
    if (onConnection)
    {
        onConnection(
            connected
        );
    }
}


// ============================================================
// COLA TX
// ============================================================

void STM32Worker::enqueueCommand(
    const Command& command)
{
    std::lock_guard<std::mutex> lock(
        txMutex
    );


    txQueue.push_back(
        command
    );
}


void STM32Worker::processTxQueue()
{
    if (!stm32)
    {
        return;
    }


    while (true)
    {
        Command command;


        {
            std::lock_guard<std::mutex> lock(
                txMutex
            );


            if (txQueue.empty())
            {
                break;
            }


            command =
                txQueue.front();


            txQueue.pop_front();
        }


        switch (command.type)
        {
            case CommandType::SET_TARGET_HEIGHT:
            {
                


                stm32->set_target_height(
                    command.body,
                    static_cast<uint16_t>(
                        command.value
                    )
                );


                break;
            }


            case CommandType::SET_VALVE_COMMAND:
            {
                stm32->set_valve_command(
                    command.body,
                    command.value
                );


                break;
            }


            case CommandType::STOP_ALL_VALVES:
            {
                stm32->stop_all_valves();


                break;
            }


            case CommandType::CLEAR_BODY_FAULT:
            {
                stm32->clear_body_fault(
                    command.body
                );


                break;
            }


            case CommandType::SET_BODY_LIMITS:
            {
                stm32->set_body_limits(
                    command.body,
                    command.value_u16_1,
                    command.value_u16_2
                );


                break;
            }


            case CommandType::SET_MOVE_COMMAND_THRESHOLD:
            {
                stm32->set_move_command_threshold(
                    command.value_u16_1
                );


                break;
            }


            case CommandType::SET_MIN_BODY_MOVEMENT:
            {
                stm32->set_min_body_movement(
                    command.value_float
                );


                break;
            }


            case CommandType::SET_NO_MOVEMENT_TIMEOUT:
            {
                stm32->set_no_movement_timeout(
                    command.value_u32
                );


                break;
            }


            case CommandType::SET_TARGET_TIMEOUT:
            {
                stm32->set_target_timeout(
                    command.value_u32
                );


                break;
            }


            case CommandType::SET_ENCODER_DIRECTION:
            {
                stm32->set_encoder_direction(
                    command.body,
                    static_cast<uint8_t>(
                        command.value
                    )
                );


                break;
            }


            case CommandType::SET_ENCODER_SCALE:
            {
                stm32->set_encoder_scale(
                    command.body,
                    command.value_float
                );


                break;
            }
        }
    }
}


// ============================================================
// COMANDOS HACIA STM32
// ============================================================

void STM32Worker::setTargetHeight(
    uint8_t body,
    uint16_t height_mm)
{
    if (!running)
    {
        return;
    }

    
    enqueueCommand(
        {
            CommandType::SET_TARGET_HEIGHT,
            body,
            static_cast<int16_t>(
                height_mm
            )
        }
    );


    if (state != nullptr &&
        body < HagieState::BODY_COUNT)
    {
        state->setBodyTarget(
            body,
            height_mm
        );


        state->setBodyAutoMode(
            body,
            true
        );
    }
}


void STM32Worker::setValveCommand(
    uint8_t body,
    int16_t command)
{
    if (!running)
    {
        return;
    }


    enqueueCommand(
        {
            CommandType::SET_VALVE_COMMAND,
            body,
            command
        }
    );


    if (state != nullptr &&
        body < HagieState::BODY_COUNT)
    {
        state->setBodyAutoMode(
            body,
            false
        );
    }
}


void STM32Worker::stopAllValves()
{
    if (!running)
    {
        return;
    }


    enqueueCommand(
        {
            CommandType::STOP_ALL_VALVES,
            0,
            0
        }
    );


    if (state != nullptr)
    {
        for (std::size_t body = 0;
             body < HagieState::BODY_COUNT;
             ++body)
        {
            state->setBodyAutoMode(
                body,
                false
            );
        }
    }
}


void STM32Worker::clearBodyFault(
    uint8_t body)
{
    if (!running)
    {
        return;
    }


    enqueueCommand(
        {
            CommandType::CLEAR_BODY_FAULT,
            body,
            0
        }
    );
}


// ============================================================
// CONFIGURACIÓN RUNTIME
// ============================================================

void STM32Worker::setBodyLimits(
    uint8_t body,
    uint16_t min_height_mm,
    uint16_t max_height_mm)
{
    if (body >=
        HagieState::BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.min_height_mm[body] =
        min_height_mm;


    runtimeConfig.max_height_mm[body] =
        max_height_mm;


    runtimeConfig.valid =
        true;
}


void STM32Worker::setMoveCommandThreshold(
    uint16_t threshold)
{
    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.move_command_threshold =
        threshold;


    runtimeConfig.valid =
        true;
}


void STM32Worker::setMinBodyMovement(
    float movement_mm)
{
    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.min_body_movement_mm =
        movement_mm;


    runtimeConfig.valid =
        true;
}


void STM32Worker::setNoMovementTimeout(
    uint32_t timeout_ms)
{
    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.no_movement_timeout_ms =
        timeout_ms;


    runtimeConfig.valid =
        true;
}


void STM32Worker::setTargetTimeout(
    uint32_t timeout_ms)
{
    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.target_timeout_ms =
        timeout_ms;


    runtimeConfig.valid =
        true;
}


void STM32Worker::setEncoderDirection(
    uint8_t body,
    uint8_t direction)
{
    if (body >=
        HagieState::BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.encoder_direction[body] =
        direction;


    runtimeConfig.valid =
        true;
}


void STM32Worker::setEncoderScale(
    uint8_t body,
    float mm_per_pulse)
{
    if (body >=
        HagieState::BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        configMutex
    );


    runtimeConfig.encoder_scale[body] =
        mm_per_pulse;


    runtimeConfig.valid =
        true;
}


// ============================================================
// SINCRONIZACIÓN CONFIGURACIÓN
// ============================================================

void STM32Worker::enqueueRuntimeConfiguration()
{
    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );


        if (!runtimeConfig.valid)
        {
            return;
        }
    }


    beginConfigurationSync();
}


void STM32Worker::beginConfigurationSync()
{
    {
        std::lock_guard<std::mutex> lock(
            configAckMutex
        );


        configAckLimits.fill(
            false
        );


        configAckDirection.fill(
            false
        );


        configAckScale.fill(
            false
        );


        configAckMoveThreshold =
            false;


        configAckMinMovement =
            false;


        configAckNoMovementTimeout =
            false;


        configAckTargetTimeout =
            false;


        configSyncStatus =
            ConfigSyncStatus::PENDING;
    }


    configSyncStep =
        0;


    configCommandPending =
        false;


    configRetryCount =
        0;


    sendCurrentConfigurationCommand();
}


STM32Worker::ConfigSyncStatus
STM32Worker::getConfigSyncStatus() const
{
    std::lock_guard<std::mutex> lock(
        configAckMutex
    );


    return configSyncStatus;
}


void STM32Worker::sendCurrentConfigurationCommand()
{
    RuntimeConfiguration configCopy;


    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );


        if (!runtimeConfig.valid)
        {
            std::lock_guard<std::mutex> ackLock(
                configAckMutex
            );


            configSyncStatus =
                ConfigSyncStatus::ERROR;


            return;
        }


        configCopy =
            runtimeConfig;
    }


    if (!stm32)
    {
        return;
    }


    if (configSyncStep >=
        CONFIG_SYNC_COMMAND_COUNT)
    {
        std::lock_guard<std::mutex> lock(
            configAckMutex
        );


        configSyncStatus =
            ConfigSyncStatus::SYNCHRONIZED;


        configCommandPending =
            false;


        return;
    }


    /*
     * 0..5 -> K01
     */
    if (configSyncStep <= 5)
    {
        uint8_t body =
            configSyncStep;


        stm32->set_body_limits(
            body,
            configCopy.min_height_mm[body],
            configCopy.max_height_mm[body]
        );
    }


    /*
     * 6 -> K02
     */
    else if (configSyncStep == 6)
    {
        stm32->set_move_command_threshold(
            configCopy.move_command_threshold
        );
    }


    /*
     * 7 -> K03
     */
    else if (configSyncStep == 7)
    {
        stm32->set_min_body_movement(
            configCopy.min_body_movement_mm
        );
    }


    /*
     * 8 -> K04
     */
    else if (configSyncStep == 8)
    {
        stm32->set_no_movement_timeout(
            configCopy.no_movement_timeout_ms
        );
    }


    /*
     * 9 -> K05
     */
    else if (configSyncStep == 9)
    {
        stm32->set_target_timeout(
            configCopy.target_timeout_ms
        );
    }


    /*
     * 10..15 -> K06
     */
    else if (configSyncStep <= 15)
    {
        uint8_t body =
            configSyncStep - 10;


        stm32->set_encoder_direction(
            body,
            configCopy.encoder_direction[body]
        );
    }


    /*
     * 16..21 -> K07
     */
    else
    {
        uint8_t body =
            configSyncStep - 16;


        stm32->set_encoder_scale(
            body,
            configCopy.encoder_scale[body]
        );
    }


    configCommandPending =
        true;


    configCommandSentTime =
        std::chrono::steady_clock::now();
}


void STM32Worker::processConfigurationSync()
{
    if (!configCommandPending)
    {
        return;
    }


    auto now =
        std::chrono::steady_clock::now();


    auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(
            now -
            configCommandSentTime
        ).count();


    if (elapsed < 250)
    {
        return;
    }


    /*
     * Timeout esperando ACK.
     */
    if (configRetryCount < 3)
    {
        configRetryCount++;


        configCommandPending =
            false;


        sendCurrentConfigurationCommand();


        return;
    }


    /*
     * Demasiados intentos.
     */
    {
        std::lock_guard<std::mutex> lock(
            configAckMutex
        );


        configSyncStatus =
            ConfigSyncStatus::ERROR;
    }


    configCommandPending =
        false;
}


// ============================================================
// CALLBACKS RX
// ============================================================

void STM32Worker::configureCallbacks()
{
    if (!stm32)
    {
        return;
    }


    // --------------------------------------------------------
    // ENCODERS
    // --------------------------------------------------------

    stm32->set_encoder_callback(
        [this](
            const stm32canbus_serialif::encoder_state&
                encoderState)
        {
            if (state == nullptr)
            {
                return;
            }


            for (std::size_t body = 0;
                 body < HagieState::BODY_COUNT;
                 ++body)
            {
                state->setBodyHeight(
                    body,
                    encoderState.height_mm[body]
                );
            }
        }
    );


    // --------------------------------------------------------
    // VÁLVULAS
    // --------------------------------------------------------

    stm32->set_valve_callback(
        [this](
            const stm32canbus_serialif::valve_state&
                valveState)
        {
            if (state == nullptr)
            {
                return;
            }


            for (std::size_t body = 0;
                 body < HagieState::BODY_COUNT;
                 ++body)
            {
                state->setBodyValveCommand(
                    body,
                    valveState.command[body]
                );
            }
        }
    );


    // --------------------------------------------------------
    // DIAGNÓSTICO
    // --------------------------------------------------------

    stm32->set_diagnostic_callback(
        [this](
            const stm32canbus_serialif::diagnostic_state&
                diagnostic)
        {
            if (state == nullptr)
            {
                return;
            }


            HagieState::SystemState system =
                state->getSystemState();


            system.stm32_connected =
                diagnostic.jetson_connection_ok;


            system.can_ok =
                (
                    (
                        diagnostic.system_faults
                        & 0x10U
                    ) == 0
                );


            system.system_faults =
                diagnostic.system_faults;


            system.axiomatic_modules =
                diagnostic.axiomatic_modules;


            state->setSystemState(
                system
            );


            for (std::size_t body = 0;
                 body < HagieState::BODY_COUNT;
                 ++body)
            {
                state->setBodyFaults(
                    body,
                    diagnostic.body_faults[body]
                );
            }
        }
    );


    // --------------------------------------------------------
    // ESTADO GENERAL STM32
    // --------------------------------------------------------

    stm32->set_stm32_state_callback(
        [this](
            const stm32canbus_serialif::stm32_state&
                stm32State)
        {
            if (state == nullptr)
            {
                return;
            }


            HagieState::SystemState system =
                state->getSystemState();


            system.stm32_connected =
                stm32State.jetson_connection_ok;


            system.stm32_uptime_ticks =
                stm32State.uptime_ticks;


            system.axiomatic_modules =
                stm32State.axiomatic_modules;


            state->setSystemState(
                system
            );
        }
    );


    // --------------------------------------------------------
    // IMU
    // --------------------------------------------------------

    stm32->set_imu_callback(
        [this](
            const stm32canbus_serialif::imu_state& imu)
        {
            if (state == nullptr)
            {
                return;
            }


            HagieState::ImuState imuState;


            imuState.valid =
                imu.valid;


            imuState.roll_deg =
                imu.roll_deg;


            imuState.pitch_deg =
                imu.pitch_deg;


            imuState.gravity_deg =
                imu.gravity_deg;


            imuState.gyro_roll_dps =
                imu.gyro_roll_dps;


            imuState.gyro_pitch_dps =
                imu.gyro_pitch_dps;


            imuState.gyro_yaw_dps =
                imu.gyro_yaw_dps;


            imuState.accel_x_mps2 =
                imu.accel_x_mps2;


            imuState.accel_y_mps2 =
                imu.accel_y_mps2;


            imuState.accel_z_mps2 =
                imu.accel_z_mps2;


            state->setImuState(
                imuState
            );


            HagieState::SystemState system =
                state->getSystemState();


            system.imu_valid =
                imu.valid;


            state->setSystemState(
                system
            );
        }
    );


    // --------------------------------------------------------
    // ACK CONFIGURACIÓN
    // --------------------------------------------------------

    stm32->set_config_ack_callback(
        [this](
            const stm32canbus_serialif::config_ack& ack)
        {
            std::cout
                << "CONFIG ACK"
                << " K="
                << static_cast<int>(
                    ack.subcommand
                )
                << " body="
                << static_cast<int>(
                    ack.body
                )
                << " status="
                << static_cast<int>(
                    ack.status
                )
                << " value1="
                << ack.value1
                << " value2="
                << ack.value2
                << std::endl;


            RuntimeConfiguration expected;


            {
                std::lock_guard<std::mutex> lock(
                    configMutex
                );


                expected =
                    runtimeConfig;
            }


            bool validAck =
                false;


            /*
             * STM32 rechazó el parámetro.
             */
            if (ack.status != 0)
            {
                std::lock_guard<std::mutex> lock(
                    configAckMutex
                );


                configSyncStatus =
                    ConfigSyncStatus::ERROR;


                configCommandPending =
                    false;


                return;
            }


            switch (configSyncStep)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                {
                    uint8_t body =
                        configSyncStep;


                    if (
                        ack.subcommand == 0x01 &&
                        ack.body == body &&
                        ack.value1 ==
                            expected
                                .min_height_mm[body] &&
                        ack.value2 ==
                            expected
                                .max_height_mm[body]
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                case 6:
                {
                    if (
                        ack.subcommand == 0x02 &&
                        ack.body == 0xFF &&
                        ack.value1 ==
                            expected
                                .move_command_threshold
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                case 7:
                {
                    uint32_t expectedValue =
                        static_cast<uint32_t>(
                            expected
                                .min_body_movement_mm
                            * 100.0f
                            + 0.5f
                        );


                    if (
                        ack.subcommand == 0x03 &&
                        ack.body == 0xFF &&
                        ack.value1 ==
                            expectedValue
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                case 8:
                {
                    if (
                        ack.subcommand == 0x04 &&
                        ack.body == 0xFF &&
                        ack.value1 ==
                            expected
                                .no_movement_timeout_ms
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                case 9:
                {
                    if (
                        ack.subcommand == 0x05 &&
                        ack.body == 0xFF &&
                        ack.value1 ==
                            expected
                                .target_timeout_ms
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                case 10:
                case 11:
                case 12:
                case 13:
                case 14:
                case 15:
                {
                    uint8_t body =
                        configSyncStep - 10;


                    if (
                        ack.subcommand == 0x06 &&
                        ack.body == body &&
                        ack.value1 ==
                            expected
                                .encoder_direction[body]
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                case 16:
                case 17:
                case 18:
                case 19:
                case 20:
                case 21:
                {
                    uint8_t body =
                        configSyncStep - 16;


                    uint32_t expectedValue =
                        static_cast<uint32_t>(
                            expected
                                .encoder_scale[body]
                            * 100000.0f
                            + 0.5f
                        );


                    if (
                        ack.subcommand == 0x07 &&
                        ack.body == body &&
                        ack.value1 ==
                            expectedValue
                    )
                    {
                        validAck =
                            true;
                    }


                    break;
                }


                default:
                {
                    return;
                }
            }


            /*
             * ACK incorrecto.
             */
            if (!validAck)
            {
                std::lock_guard<std::mutex> lock(
                    configAckMutex
                );


                configSyncStatus =
                    ConfigSyncStatus::ERROR;


                configCommandPending =
                    false;


                return;
            }


            /*
             * ACK correcto.
             */
            configCommandPending =
                false;


            configRetryCount =
                0;


            configSyncStep++;


            /*
             * Fin de los 22 comandos.
             */
            if (
                configSyncStep >=
                CONFIG_SYNC_COMMAND_COUNT
            )
            {
                std::lock_guard<std::mutex> lock(
                    configAckMutex
                );


                configSyncStatus =
                    ConfigSyncStatus::SYNCHRONIZED;


                return;
            }


            /*
             * Siguiente comando.
             */
            sendCurrentConfigurationCommand();
        }
    );
}


// ============================================================
// HILO STM32
// ============================================================

void STM32Worker::workerLoop()
{
    using namespace std::chrono;


    auto lastHeartbeat =
        steady_clock::now();


    while (running)
    {
        /*
         * ====================================================
         * Si no tenemos interfaz, intentar reconectar.
         * ====================================================
         */
        if (!stm32)
        {
            try
            {
                std::cout
                    << "Intentando reconectar STM32..."
                    << std::endl;


                stm32 =
                    std::make_unique<
                        stm32canbus_serialif
                    >(
                        port,
                        baudrate
                    );


                configureCallbacks();


                stm32->start();


                lastHeartbeat =
                    steady_clock::now();


                /*
                 * =================================================
                 * SEGURIDAD DE RECONEXIÓN
                 * =================================================
                 *
                 * Antes de:
                 *
                 * - avisar a GUI
                 * - sincronizar configuración
                 * - aceptar nuevas consignas
                 *
                 * mandar PARADA TOTAL directamente.
                 *
                 * De esta manera una STM32 que vuelve
                 * a aparecer nunca retoma un AUTO anterior.
                 */
                stm32->stop_all_valves();


                /*
                 * También reflejar localmente MANUAL.
                 */
                if (state != nullptr)
                {
                    for (std::size_t body = 0;
                         body < HagieState::BODY_COUNT;
                         ++body)
                    {
                        state->setBodyAutoMode(
                            body,
                            false
                        );

                        state->setBodyValveCommand(
                            body,
                            0
                        );
                    }
                }


                /*
                 * Ahora sí informar que volvió.
                 */
                notifyConnectionState(
                    true
                );


                /*
                 * Reenviar configuración runtime.
                 */
                enqueueRuntimeConfiguration();


                std::cout
                    << "STM32 reconectada."
                    << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr
                    << "STM32 aun no disponible: "
                    << e.what()
                    << std::endl;


                /*
                 * Si la interfaz llegó a crearse,
                 * cerrarla antes de destruirla.
                 */
                if (stm32)
                {
                    try
                    {
                        stm32->stop();
                    }
                    catch (...)
                    {
                    }
                }


                stm32.reset();


                notifyConnectionState(
                    false
                );


                std::this_thread::sleep_for(
                    seconds(1)
                );


                continue;
            }
        }


        /*
         * ====================================================
         * Comunicación normal
         * ====================================================
         */

        auto now =
            steady_clock::now();


        auto elapsed =
            duration_cast<
                milliseconds
            >(
                now -
                lastHeartbeat
            );


        /*
         * ----------------------------------------------------
         * HEARTBEAT
         * ----------------------------------------------------
         */
        if (elapsed.count() >= 100)
        {
            try
            {
                stm32->send_heartbeat();


                lastHeartbeat =
                    now;
            }
            catch (const std::exception& e)
            {
                std::cerr
                    << "STM32 desconectada durante heartbeat: "
                    << e.what()
                    << std::endl;


                /*
                 * Estado global.
                 */
                if (state != nullptr)
                {
                    HagieState::SystemState system =
                        state->getSystemState();


                    system.stm32_connected =
                        false;


                    system.can_ok =
                        false;


                    system.imu_valid =
                        false;


                    state->setSystemState(
                        system
                    );
                }


                /*
                 * Cancela AUTO y elimina órdenes
                 * pendientes antes de reconectar.
                 */
                notifyConnectionState(
                    false
                );


                /*
                 * Cerrar interfaz dañada.
                 */
                try
                {
                    stm32->stop();
                }
                catch (...)
                {
                }


                stm32.reset();


                std::this_thread::sleep_for(
                    seconds(1)
                );


                continue;
            }
        }


        /*
         * ----------------------------------------------------
         * COLA TX
         * ----------------------------------------------------
         */
        try
        {
            processTxQueue();


            processConfigurationSync();
        }
        catch (const std::exception& e)
        {
            std::cerr
                << "Error TX STM32: "
                << e.what()
                << std::endl;


            /*
             * Estado global.
             */
            if (state != nullptr)
            {
                HagieState::SystemState system =
                    state->getSystemState();


                system.stm32_connected =
                    false;


                system.can_ok =
                    false;


                system.imu_valid =
                    false;


                state->setSystemState(
                    system
                );
            }


            /*
             * Cancela AUTO y vacía la cola.
             */
            notifyConnectionState(
                false
            );


            /*
             * Cerrar interfaz.
             */
            try
            {
                stm32->stop();
            }
            catch (...)
            {
            }


            stm32.reset();


            std::this_thread::sleep_for(
                seconds(1)
            );


            continue;
        }


        /*
         * Ciclo aproximadamente 100 Hz.
         */
        std::this_thread::sleep_for(
            milliseconds(10)
        );
    }
}