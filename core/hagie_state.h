#ifndef HAGIE_STATE_H
#define HAGIE_STATE_H


#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>


class HagieState
{
public:

    static constexpr std::size_t BODY_COUNT = 6;


    // ============================================================
    // Estado de cada cuerpo
    // ============================================================

    struct BodyState
    {
        /*
         * Altura real del cuerpo medida por encoder.
         */
        uint16_t height_mm = 0;


        /*
         * Objetivo actual enviado al control.
         */
        uint16_t target_mm = 0;


        /*
         * Comando actual de válvula.
         *
         * -1000 .. +1000
         */
        int16_t valve_command = 0;


        /*
         * true:
         * cuerpo trabajando en automático.
         *
         * false:
         * cuerpo en manual.
         */
        bool auto_mode = false;


        /*
         * Fallas individuales del cuerpo.
         */
        uint32_t faults = 0;


        /*
         * ========================================================
         * VISIÓN 3D
         * ========================================================
         *
         * Altura detectada por cámaras / nube de puntos.
         *
         * IMPORTANTE:
         *
         * Esto NO representa la posición del cabezal.
         * Es una medición producida por visión.
         */
        uint16_t vision_height_mm = 0;


        /*
         * Indica si la última medición de visión
         * para este cuerpo es válida.
         */
        bool vision_valid = false;

        /*
        * Timestamp de la última medición válida
        * entregada por visión 3D.
        *
        * Unidad: milisegundos de steady_clock.
        */
        uint64_t vision_timestamp_ms = 0;
    };


    // ============================================================
    // Estado general del sistema
    // ============================================================

    struct SystemState
    {
        bool stm32_connected = false;

        bool can_ok = false;

        bool imu_valid = false;

        bool ai_running = false;


        /*
         * Estado del módulo de visión.
         */
        bool vision_running = false;


        uint32_t system_faults = 0;


        uint32_t stm32_uptime_ticks = 0;


        uint8_t axiomatic_modules = 0;
    };


    // ============================================================
    // IMU
    // ============================================================

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


    // ============================================================
    // Constructor
    // ============================================================

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


    /*
     * ========================================================
     * VISIÓN
     * ========================================================
     */

    void setBodyVisionHeight(
        std::size_t body,
        uint16_t height_mm,
        bool valid,
        uint64_t timestamp_ms
    );


    void setBodyVisionValid(
        std::size_t body,
        bool valid
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

    /*
     * Un único mutex protege todo el estado compartido.
     *
     * GUI, STM32, visión e IA pueden ejecutarse
     * desde distintos hilos.
     */
    mutable std::mutex stateMutex;


    std::array<
        BodyState,
        BODY_COUNT
    > bodies {};


    SystemState systemState {};


    ImuState imuState {};
};


#endif