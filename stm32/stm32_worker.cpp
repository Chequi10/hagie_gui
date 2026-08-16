#include "stm32_worker.h"

#include <chrono>
#include <exception>
#include <iostream>


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
            std::make_unique<stm32canbus_serialif>(
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

            system.stm32_connected = false;

            state->setSystemState(system);
        }

        return false;
    }
}


void STM32Worker::enqueueCommand(
    const Command& command)
{
    std::lock_guard<std::mutex> lock(txMutex);

    txQueue.push_back(command);
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
            std::lock_guard<std::mutex> lock(txMutex);

            if (txQueue.empty())
            {
                break;
            }

            command = txQueue.front();
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

void STM32Worker::stop()
{
    if (!running)
    {
        if (stm32)
        {
            stm32->stop();
            stm32.reset();
        }

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

        system.stm32_connected = false;

        state->setSystemState(system);
    }
}


bool STM32Worker::isRunning() const
{
    return running;
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




void STM32Worker::setBodyLimits(
    uint8_t body,
    uint16_t min_height_mm,
    uint16_t max_height_mm)
{
    if (body >= HagieState::BODY_COUNT)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.min_height_mm[body] =
            min_height_mm;

        runtimeConfig.max_height_mm[body] =
            max_height_mm;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_BODY_LIMITS;

    command.body =
        body;

    command.value_u16_1 =
        min_height_mm;

    command.value_u16_2 =
        max_height_mm;

    enqueueCommand(command);
}


void STM32Worker::setMoveCommandThreshold(
    uint16_t threshold)
{
    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.move_command_threshold =
            threshold;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_MOVE_COMMAND_THRESHOLD;

    command.value_u16_1 =
        threshold;

    enqueueCommand(command);
}


void STM32Worker::setMinBodyMovement(
    float movement_mm)
{
    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.min_body_movement_mm =
            movement_mm;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_MIN_BODY_MOVEMENT;

    command.value_float =
        movement_mm;

    enqueueCommand(command);
}

void STM32Worker::setNoMovementTimeout(
    uint32_t timeout_ms)
{
    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.no_movement_timeout_ms =
            timeout_ms;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_NO_MOVEMENT_TIMEOUT;

    command.value_u32 =
        timeout_ms;

    enqueueCommand(command);
}
void STM32Worker::setTargetTimeout(
    uint32_t timeout_ms)
{
    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.target_timeout_ms =
            timeout_ms;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_TARGET_TIMEOUT;

    command.value_u32 =
        timeout_ms;

    enqueueCommand(command);
}


void STM32Worker::setEncoderDirection(
    uint8_t body,
    uint8_t direction)
{
    if (body >= HagieState::BODY_COUNT)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.encoder_direction[body] =
            direction;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_ENCODER_DIRECTION;

    command.body =
        body;

    command.value =
        static_cast<int16_t>(
            direction
        );

    enqueueCommand(command);
}


void STM32Worker::setEncoderScale(
    uint8_t body,
    float mm_per_pulse)
{
    if (body >= HagieState::BODY_COUNT)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        runtimeConfig.encoder_scale[body] =
            mm_per_pulse;

        runtimeConfig.valid = true;
    }

    if (!running)
    {
        return;
    }

    Command command;

    command.type =
        CommandType::SET_ENCODER_SCALE;

    command.body =
        body;

    command.value_float =
        mm_per_pulse;

    enqueueCommand(command);
}

void STM32Worker::enqueueRuntimeConfiguration()
{
    RuntimeConfiguration configCopy;

    {
        std::lock_guard<std::mutex> lock(
            configMutex
        );

        if (!runtimeConfig.valid)
        {
            return;
        }

        configCopy =
            runtimeConfig;
    }


    /*
     * K 0x01
     * Límites por cuerpo.
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        Command command;

        command.type =
            CommandType::SET_BODY_LIMITS;

        command.body =
            static_cast<uint8_t>(body);

        command.value_u16_1 =
            configCopy.min_height_mm[body];

        command.value_u16_2 =
            configCopy.max_height_mm[body];

        enqueueCommand(command);
    }


    /*
     * K 0x02
     */
    {
        Command command;

        command.type =
            CommandType::SET_MOVE_COMMAND_THRESHOLD;

        command.value_u16_1 =
            configCopy.move_command_threshold;

        enqueueCommand(command);
    }


    /*
     * K 0x03
     */
    {
        Command command;

        command.type =
            CommandType::SET_MIN_BODY_MOVEMENT;

        command.value_float =
            configCopy.min_body_movement_mm;

        enqueueCommand(command);
    }


    /*
     * K 0x04
     */
    {
        Command command;

        command.type =
            CommandType::SET_NO_MOVEMENT_TIMEOUT;

        command.value_u32 =
            configCopy.no_movement_timeout_ms;

        enqueueCommand(command);
    }


    /*
     * K 0x05
     */
    {
        Command command;

        command.type =
            CommandType::SET_TARGET_TIMEOUT;

        command.value_u32 =
            configCopy.target_timeout_ms;

        enqueueCommand(command);
    }


    /*
     * K 0x06
     * Sentido encoder.
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        Command command;

        command.type =
            CommandType::SET_ENCODER_DIRECTION;

        command.body =
            static_cast<uint8_t>(body);

        command.value =
            static_cast<int16_t>(
                configCopy.encoder_direction[body]
            );

        enqueueCommand(command);
    }


    /*
     * K 0x07
     * Escala encoder.
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        Command command;

        command.type =
            CommandType::SET_ENCODER_SCALE;

        command.body =
            static_cast<uint8_t>(body);

        command.value_float =
            configCopy.encoder_scale[body];

        enqueueCommand(command);
    }
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
            const stm32canbus_serialif::encoder_state& encoderState)
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
            const stm32canbus_serialif::valve_state& valveState)
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
            const stm32canbus_serialif::diagnostic_state& diagnostic)
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
                ((diagnostic.system_faults & 0x10U) == 0);

            system.system_faults =
                diagnostic.system_faults;

            system.axiomatic_modules =
                diagnostic.axiomatic_modules;

            state->setSystemState(system);


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
            const stm32canbus_serialif::stm32_state& stm32State)
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

            state->setSystemState(system);
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
                    std::make_unique<stm32canbus_serialif>(
                        port,
                        baudrate
                    );

                configureCallbacks();

                stm32->start();

                lastHeartbeat =
                    steady_clock::now();

                /*
                * Reenviar configuración runtime
                * después de reconectar.
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

                stm32.reset();

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
            duration_cast<milliseconds>(
                now - lastHeartbeat
            );


        /*
         * ----------------------------------------------------
         * Heartbeat
         * ----------------------------------------------------
         *
         * Se transmite desde ESTE MISMO HILO.
         */
        if (elapsed.count() >= 100)
        {
            try
            {
                stm32->send_heartbeat();

                lastHeartbeat = now;
            }
            catch (const std::exception& e)
            {
                std::cerr
                    << "STM32 desconectada durante heartbeat: "
                    << e.what()
                    << std::endl;

                /*
                 * Actualizar estado visible por GUI.
                 */
                if (state != nullptr)
                {
                    HagieState::SystemState system =
                        state->getSystemState();

                    system.stm32_connected = false;
                    system.can_ok = false;
                    system.imu_valid = false;

                    state->setSystemState(system);
                }

                /*
                 * Cerrar la interfaz dañada.
                 */
                try
                {
                    stm32->stop();
                }
                catch (...)
                {
                }

                stm32.reset();

                /*
                 * Esperar antes de volver a intentar.
                 */
                std::this_thread::sleep_for(
                    seconds(1)
                );

                continue;
            }
        }


        /*
         * ----------------------------------------------------
         * Cola de comandos TX
         * ----------------------------------------------------
         *
         * Todos los comandos provenientes de:
         *
         * - GUI
         * - IA
         * - control
         * - tests
         *
         * terminan en txQueue.
         *
         * ESTE es el único hilo autorizado a transmitir
         * hacia la STM32.
         */
        try
        {
            processTxQueue();
        }
        catch (const std::exception& e)
        {
            std::cerr
                << "Error TX STM32: "
                << e.what()
                << std::endl;

            /*
             * Marcar comunicación caída.
             */
            if (state != nullptr)
            {
                HagieState::SystemState system =
                    state->getSystemState();

                system.stm32_connected = false;
                system.can_ok = false;
                system.imu_valid = false;

                state->setSystemState(system);
            }

            /*
             * Cerrar la interfaz actual.
             */
            try
            {
                stm32->stop();
            }
            catch (...)
            {
            }

            stm32.reset();

            /*
             * Esperar antes de intentar reconectar.
             */
            std::this_thread::sleep_for(
                seconds(1)
            );

            continue;
        }


        /*
         * Ciclo del worker:
         * aproximadamente 100 Hz.
         */
        std::this_thread::sleep_for(
            milliseconds(10)
        );
    }
}