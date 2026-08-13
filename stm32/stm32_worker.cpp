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
    if (!running || !stm32)
    {
        return;
    }

    stm32->set_target_height(
        body,
        height_mm
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
    if (!running || !stm32)
    {
        return;
    }

    stm32->set_valve_command(
        body,
        command
    );

    if (state != nullptr &&
        body < HagieState::BODY_COUNT)
    {
        /*
         * OPCODE 'B' coloca el cuerpo
         * en modo MANUAL en STM32.
         */
        state->setBodyAutoMode(
            body,
            false
        );
    }
}


void STM32Worker::stopAllValves()
{
    if (!running || !stm32)
    {
        return;
    }

    stm32->stop_all_valves();

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
    if (!running || !stm32)
    {
        return;
    }

    stm32->clear_body_fault(
        body
    );
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
        if (stm32)
        {
            auto now =
                steady_clock::now();

            auto elapsed =
                duration_cast<milliseconds>(
                    now - lastHeartbeat
                );

            /*
             * Heartbeat hacia STM32.
             *
             * Se mantiene separado de la GUI,
             * visión e IA.
             */
            if (elapsed.count() >= 100)
            {
                stm32->send_heartbeat();

                lastHeartbeat = now;
            }
        }

        std::this_thread::sleep_for(
            milliseconds(10)
        );
    }
}