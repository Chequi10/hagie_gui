#include "hagie_state.h"


HagieState::HagieState()
{
}


// ============================================================
// ESTADO DE CUERPOS
// ============================================================

void HagieState::setBodyHeight(
    std::size_t body,
    uint16_t height_mm)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    bodies[body].height_mm =
        height_mm;
}


void HagieState::setBodyTarget(
    std::size_t body,
    uint16_t target_mm)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    bodies[body].target_mm =
        target_mm;
}


void HagieState::setBodyValveCommand(
    std::size_t body,
    int16_t command)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    bodies[body].valve_command =
        command;
}


void HagieState::setBodyAutoMode(
    std::size_t body,
    bool auto_mode)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    bodies[body].auto_mode =
        auto_mode;
}


void HagieState::setBodyFaults(
    std::size_t body,
    uint32_t faults)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    bodies[body].faults =
        faults;
}


// ============================================================
// VISIÓN 3D
// ============================================================

void HagieState::setBodyVisionHeight(
    std::size_t body,
    uint16_t height_mm,
    bool valid,
    uint64_t timestamp_ms)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(
        stateMutex
    );

    bodies[body].vision_height_mm =
        height_mm;

    bodies[body].vision_valid =
        valid;

    bodies[body].vision_timestamp_ms =
        timestamp_ms;
}


void HagieState::setBodyVisionValid(
    std::size_t body,
    bool valid)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    bodies[body].vision_valid =
        valid;
}


// ============================================================
// OBTENER ESTADO DE CUERPO
// ============================================================

HagieState::BodyState
HagieState::getBodyState(
    std::size_t body) const
{
    if (body >= BODY_COUNT)
    {
        return BodyState {};
    }


    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    return bodies[body];
}


// ============================================================
// ESTADO GENERAL DEL SISTEMA
// ============================================================

void HagieState::setSystemState(
    const SystemState& state)
{
    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    systemState =
        state;
}


HagieState::SystemState
HagieState::getSystemState() const
{
    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    return systemState;
}


// ============================================================
// IMU
// ============================================================

void HagieState::setImuState(
    const ImuState& state)
{
    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    imuState =
        state;
}


HagieState::ImuState
HagieState::getImuState() const
{
    std::lock_guard<std::mutex> lock(
        stateMutex
    );


    return imuState;
}