#ifndef HAGIE_STATE_H
#define HAGIE_STATE_H

#include <array>
#include <cstdint>
#include <mutex>


class HagieState
{
public:
    static constexpr std::size_t BODY_COUNT = 6;

    struct BodyState
    {
        uint16_t height_mm = 0;
        uint16_t target_mm = 0;

        int16_t valve_command = 0;

        bool auto_mode = false;

        uint32_t faults = 0;
    };


    struct SystemState
    {
        bool stm32_connected = false;
        bool can_ok = false;
        bool imu_valid = false;
        bool ai_running = false;

        uint32_t system_faults = 0;

        uint32_t stm32_uptime_ticks = 0;

        uint8_t axiomatic_modules = 0;
    };


    struct ImuState
    {
        bool valid = false;

        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
        float gravity_deg = 0.0f;

        float gyro_roll_dps = 0.0f;
        float gyro_pitch_dps = 0.0f;
        float gyro_yaw_dps = 0.0f;

        float accel_x_mps2 = 0.0f;
        float accel_y_mps2 = 0.0f;
        float accel_z_mps2 = 0.0f;
    };


    HagieState();


    // ============================================================
    // Estado de cuerpos
    // ============================================================

    void setBodyHeight(
        std::size_t body,
        uint16_t height_mm
    );

    void setBodyTarget(
        std::size_t body,
        uint16_t target_mm
    );

    void setBodyValveCommand(
        std::size_t body,
        int16_t command
    );

    void setBodyAutoMode(
        std::size_t body,
        bool auto_mode
    );

    void setBodyFaults(
        std::size_t body,
        uint32_t faults
    );

    BodyState getBodyState(
        std::size_t body
    ) const;


    // ============================================================
    // Estado general
    // ============================================================

    void setSystemState(
        const SystemState& state
    );

    SystemState getSystemState() const;


    // ============================================================
    // IMU
    // ============================================================

    void setImuState(
        const ImuState& state
    );

    ImuState getImuState() const;


private:
    mutable std::mutex stateMutex;

    std::array<BodyState, BODY_COUNT> bodies {};

    SystemState systemState {};

    ImuState imuState {};
};


#endif