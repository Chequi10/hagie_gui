#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <boost/asio.hpp>

#include "protocol.h"


class stm32canbus_serialif :
    private protocol::packet_decoder,
    private protocol::packet_encoder
{
public:

    static constexpr std::size_t BODY_COUNT = 6;
    static constexpr std::size_t BUFSIZE = 64;


    // ============================================================
    // Estados recibidos desde STM32
    // ============================================================

    struct encoder_state
    {
        std::array<uint16_t, BODY_COUNT> height_mm {};
    };

    struct imu_state
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


    struct valve_state
    {
        std::array<int16_t, BODY_COUNT> command {};
    };


    struct diagnostic_state
    {
        uint32_t axiomatic_rx_dropped = 0;
        
        uint32_t uart_error_count = 0;

        uint32_t jetson_tx_queue_dropped = 0;

        uint32_t jetson_tx_dma_errors = 0;

        bool jetson_connection_ok = false;

        uint8_t axiomatic_modules = 0;

        uint32_t system_faults = 0;

        std::array<uint32_t, BODY_COUNT> body_faults {};
    };


    struct stm32_state
    {
        bool jetson_connection_ok = false;

        uint32_t uptime_ticks = 0;

        uint8_t encoder_count = 0;

        uint8_t body_count = 0;

        uint8_t axiomatic_modules = 0;
    };


    // ============================================================
    // Callbacks
    // ============================================================

    using encoder_callback =
        std::function<void(const encoder_state&)>;

    using valve_callback =
        std::function<void(const valve_state&)>;

    using diagnostic_callback =
        std::function<void(const diagnostic_state&)>;

    using stm32_state_callback =
        std::function<void(const stm32_state&)>;

    using imu_callback =
        std::function<void(const imu_state&)>;


    // ============================================================
    // Constructor / destructor
    // ============================================================

    stm32canbus_serialif(
        const std::string& dev_name,
        unsigned int baudrate
    );

    ~stm32canbus_serialif();


    // ============================================================
    // Control de comunicación
    // ============================================================

    void start();

    void stop();

    bool is_running() const;


    // ============================================================
    // Callbacks de recepción
    // ============================================================

    void set_encoder_callback(
        encoder_callback callback
    );

    void set_valve_callback(
        valve_callback callback
    );

    void set_diagnostic_callback(
        diagnostic_callback callback
    );

    void set_stm32_state_callback(
        stm32_state_callback callback
    );

    void set_imu_callback(
        imu_callback callback
    );


    // ============================================================
    // Comandos Jetson -> STM32
    // ============================================================

    // OPCODE 'A'
    void send_heartbeat();


    // OPCODE 'B'
    // body: 0..5
    // command: -1000 .. +1000
    void set_valve_command(
        uint8_t body,
        int16_t command
    );


    // OPCODE 'C'
    void stop_all_valves();


    // OPCODE 'D'
    // body: 0..5
    // height_mm: altura objetivo en milímetros
    void set_target_height(
        uint8_t body,
        uint16_t height_mm
    );


    // OPCODE 'J'
    // body: 0..5
    // Reconoce y limpia la falla NO_MOVEMENT
    // del cuerpo indicado.
    void clear_body_fault(
        uint8_t body
    );

    // ============================================================
    // OPCODE 'K' - Configuración runtime STM32
    // ============================================================

    // K 0x01
    // Límites mínimo y máximo de un cuerpo.
    void set_body_limits(
        uint8_t body,
        uint16_t min_height_mm,
        uint16_t max_height_mm
    );


    // K 0x02
    // Umbral mínimo de comando para considerar movimiento.
    void set_move_command_threshold(
        uint16_t threshold
    );


    // K 0x03
    // Movimiento mínimo esperado, expresado en mm.
    void set_min_body_movement(
        float movement_mm
    );


    // K 0x04
    // Timeout NO_MOVEMENT en milisegundos.
    void set_no_movement_timeout(
        uint32_t timeout_ms
    );


    // K 0x05
    // Timeout de actualización de consigna AUTO, en ms.
    void set_target_timeout(
        uint32_t timeout_ms
    );


    // K 0x06
    // direction:
    // 0 = normal
    // 1 = invertido
    void set_encoder_direction(
        uint8_t body,
        uint8_t direction
    );


    // K 0x07
    // Escala del encoder en mm/pulso.
    void set_encoder_scale(
        uint8_t body,
        float mm_per_pulse
    );

private:

    // ============================================================
    // Boost.Asio
    // ============================================================

    boost::asio::io_context io;

    boost::asio::serial_port port;

    std::array<uint8_t, BUFSIZE> rx_buffer {};

    std::thread comm_thread;

    std::atomic<bool> running {false};

  


    // ============================================================
    // Callbacks
    // ============================================================

    encoder_callback on_encoder;

    valve_callback on_valve;

    diagnostic_callback on_diagnostic;

    stm32_state_callback on_stm32_state;

    imu_callback on_imu;


    // ============================================================
    // Comunicación serie
    // ============================================================

    void run();

    void read_some();

    void read_handler(
        const boost::system::error_code& error,
        std::size_t bytes_transferred
    );


    // ============================================================
    // protocol::packet_decoder
    // ============================================================

    void handle_packet(
        const uint8_t* payload,
        std::size_t n
    ) override;

    void set_error(
        protocol::packet_decoder::error_code ec
    ) override;


    // ============================================================
    // protocol::packet_encoder
    // ============================================================

    void send_impl(
        const uint8_t* buf,
        uint8_t n
    ) override;
};