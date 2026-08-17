#include "stm32canbusif.h"

#include <algorithm>
#include <iostream>
#include <utility>


stm32canbus_serialif::stm32canbus_serialif(
    const std::string& dev_name,
    unsigned int baudrate)
    :
    io(),
    port(io)
{
    port.open(dev_name);

    port.set_option(
        boost::asio::serial_port_base::baud_rate(baudrate)
    );

    port.set_option(
        boost::asio::serial_port_base::character_size(8)
    );

    port.set_option(
        boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none
        )
    );

    port.set_option(
        boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one
        )
    );

    port.set_option(
        boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none
        )
    );
}


stm32canbus_serialif::~stm32canbus_serialif()
{
    stop();
}


// ============================================================
// Control
// ============================================================

void stm32canbus_serialif::start()
{
    if (running)
    {
        return;
    }

    running = true;

    comm_thread = std::thread(
        [this]()
        {
            run();
        }
    );
}


void stm32canbus_serialif::stop()
{
    if (!running)
    {
        return;
    }

    running = false;

    boost::system::error_code ec;

    port.cancel(ec);

    io.stop();

    if (comm_thread.joinable())
    {
        comm_thread.join();
    }

    if (port.is_open())
    {
        port.close(ec);
    }
}


bool stm32canbus_serialif::is_running() const
{
    return running;
}


// ============================================================
// Callbacks
// ============================================================

void stm32canbus_serialif::set_encoder_callback(
    encoder_callback callback)
{
    on_encoder = std::move(callback);
}


void stm32canbus_serialif::set_valve_callback(
    valve_callback callback)
{
    on_valve = std::move(callback);
}


void stm32canbus_serialif::set_diagnostic_callback(
    diagnostic_callback callback)
{
    on_diagnostic = std::move(callback);
}


void stm32canbus_serialif::set_stm32_state_callback(
    stm32_state_callback callback)
{
    on_stm32_state = std::move(callback);
}

void stm32canbus_serialif::set_imu_callback(
    imu_callback callback)
{
    on_imu = std::move(callback);
}

void stm32canbus_serialif::set_config_ack_callback(
    config_ack_callback callback)
{
    on_config_ack =
        std::move(callback);
        
}


// ============================================================
// Recepción serie
// ============================================================

void stm32canbus_serialif::run()
{
    read_some();

    io.run();
}


void stm32canbus_serialif::read_some()
{
    if (!running)
    {
        return;
    }

    port.async_read_some(
        boost::asio::buffer(rx_buffer),

        [this](
            const boost::system::error_code& error,
            std::size_t bytes_transferred)
        {
            read_handler(error, bytes_transferred);
        }
    );
}


void stm32canbus_serialif::read_handler(
    const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
    if (error)
    {
        if (running)
        {
            std::cerr
                << "Error puerto serie: "
                << error.message()
                << std::endl;
        }

        return;
    }

    for (std::size_t i = 0;
         i < bytes_transferred;
         ++i)
    {
        protocol::packet_decoder::feed(
            rx_buffer[i]
        );
    }

    read_some();
}


// ============================================================
// Decoder - STM32 -> Jetson
// ============================================================

void stm32canbus_serialif::handle_packet(
    const uint8_t* payload,
    std::size_t n)
{
    if (payload == nullptr || n == 0)
    {
        return;
    }

    const uint8_t opcode = payload[0];

    switch (opcode)
    {
        // ----------------------------------------------------
        // 'E' - Estado de los 6 encoders
        // ----------------------------------------------------

        case 'E':
        {
            if (n != 13)
            {
                return;
            }

            encoder_state state;

            for (std::size_t i = 0;
                 i < BODY_COUNT;
                 ++i)
            {
                const std::size_t index =
                    1 + (i * 2);

                state.height_mm[i] =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(
                            payload[index]
                        ) << 8) |
                        static_cast<uint16_t>(
                            payload[index + 1]
                        )
                    );
            }

            if (on_encoder)
            {
                on_encoder(state);
            }

            break;
        }


        // ----------------------------------------------------
        // 'F' - Estado de las 6 válvulas
        // ----------------------------------------------------

        case 'F':
        {
            if (n != 13)
            {
                return;
            }

            valve_state state;

            for (std::size_t i = 0;
                 i < BODY_COUNT;
                 ++i)
            {
                const std::size_t index =
                    1 + (i * 2);

                uint16_t raw =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(
                            payload[index]
                        ) << 8) |
                        static_cast<uint16_t>(
                            payload[index + 1]
                        )
                    );

                state.command[i] =
                    static_cast<int16_t>(raw);
            }

            if (on_valve)
            {
                on_valve(state);
            }

            break;
        }


        // ----------------------------------------------------
        // 'G' - Diagnóstico
        // ----------------------------------------------------

        case 'G':
        {
            if (n != 47)
            {
                return;
            }

            diagnostic_state state;

            state.axiomatic_rx_dropped =
                (static_cast<uint32_t>(payload[1]) << 24) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 8)  |
                 static_cast<uint32_t>(payload[4]);

            state.jetson_connection_ok =
                payload[5] != 0;

            state.axiomatic_modules =
                payload[6];
            state.uart_error_count =
                (static_cast<uint32_t>(payload[7]) << 24) |
                (static_cast<uint32_t>(payload[8]) << 16) |
                (static_cast<uint32_t>(payload[9]) << 8)  |
                static_cast<uint32_t>(payload[10]);    

                state.jetson_tx_queue_dropped =
                (static_cast<uint32_t>(payload[11]) << 24) |
                (static_cast<uint32_t>(payload[12]) << 16) |
                (static_cast<uint32_t>(payload[13]) << 8)  |
                static_cast<uint32_t>(payload[14]);

                state.jetson_tx_dma_errors =
                (static_cast<uint32_t>(payload[15]) << 24) |
                (static_cast<uint32_t>(payload[16]) << 16) |
                (static_cast<uint32_t>(payload[17]) << 8)  |
                static_cast<uint32_t>(payload[18]);

                state.system_faults =
                (static_cast<uint32_t>(payload[19]) << 24) |
                (static_cast<uint32_t>(payload[20]) << 16) |
                (static_cast<uint32_t>(payload[21]) << 8)  |
                static_cast<uint32_t>(payload[22]);


            for (std::size_t body = 0;
                body < BODY_COUNT;
                ++body)
            {
                std::size_t index =
                    23 + (body * 4);

                state.body_faults[body] =
                    (static_cast<uint32_t>(payload[index]) << 24) |
                    (static_cast<uint32_t>(payload[index + 1]) << 16) |
                    (static_cast<uint32_t>(payload[index + 2]) << 8) |
                    static_cast<uint32_t>(payload[index + 3]);
            }

            if (on_diagnostic)
            {
                on_diagnostic(state);
            }

            break;
        }


        // ----------------------------------------------------
        // 'H' - Estado general STM32
        // ----------------------------------------------------

        case 'H':
        {
            if (n != 9)
            {
                return;
            }

            stm32_state state;

            state.jetson_connection_ok =
                payload[1] != 0;

            state.uptime_ticks =
                (static_cast<uint32_t>(payload[2]) << 24) |
                (static_cast<uint32_t>(payload[3]) << 16) |
                (static_cast<uint32_t>(payload[4]) << 8)  |
                 static_cast<uint32_t>(payload[5]);

            state.encoder_count =
                payload[6];

            state.body_count =
                payload[7];

            state.axiomatic_modules =
                payload[8];

            if (on_stm32_state)
            {
                on_stm32_state(state);
            }

            break;
        }

        // ----------------------------------------------------
        // 'I' - Estado IMU
        // ----------------------------------------------------

        case 'I':
        {
            if (n != 20)
            {
                return;
            }

            imu_state state;

            state.valid =
                payload[1] != 0;

            auto read_int16_be =
                [payload](std::size_t index) -> int16_t
            {
                uint16_t raw =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(payload[index]) << 8) |
                        static_cast<uint16_t>(payload[index + 1])
                    );

                return static_cast<int16_t>(raw);
            };

            state.roll_deg =
                static_cast<float>(
                    read_int16_be(2)
                ) / 100.0f;

            state.pitch_deg =
                static_cast<float>(
                    read_int16_be(4)
                ) / 100.0f;

            state.gravity_deg =
                static_cast<float>(
                    read_int16_be(6)
                ) / 100.0f;

            state.gyro_roll_dps=
                static_cast<float>(
                    read_int16_be(8)
                ) / 100.0f;

            state.gyro_pitch_dps =
                static_cast<float>(
                    read_int16_be(10)
                ) / 100.0f;

            state.gyro_yaw_dps=
                static_cast<float>(
                    read_int16_be(12)
                ) / 100.0f;

            state.accel_x_mps2 =
                static_cast<float>(
                    read_int16_be(14)
                ) / 1000.0f;

            state.accel_y_mps2 =
                static_cast<float>(
                    read_int16_be(16)
                ) / 1000.0f;

            state.accel_z_mps2 =
                static_cast<float>(
                    read_int16_be(18)
                ) / 1000.0f;

            if (on_imu)
            {
                on_imu(state);
            }

            break;
        }

        // ----------------------------------------------------
        // 'L' - ACK de configuración STM32
        // ----------------------------------------------------

        case 'L':
        {
            if (n != 12)
            {
                return;
            }

            config_ack ack;

            ack.subcommand =
                payload[1];

            ack.body =
                payload[2];

            ack.status =
                payload[3];

            ack.value1 =
                (static_cast<uint32_t>(payload[4]) << 24) |
                (static_cast<uint32_t>(payload[5]) << 16) |
                (static_cast<uint32_t>(payload[6]) << 8)  |
                static_cast<uint32_t>(payload[7]);

            ack.value2 =
                (static_cast<uint32_t>(payload[8]) << 24) |
                (static_cast<uint32_t>(payload[9]) << 16) |
                (static_cast<uint32_t>(payload[10]) << 8) |
                static_cast<uint32_t>(payload[11]);

            if (on_config_ack)
            {
                on_config_ack(
                    ack
                );
            }

            break;
        }

        default:
        {
            set_error(
                protocol::packet_decoder::error_code::
                    unknown_opcode
            );

            break;
        }
    }
}


void stm32canbus_serialif::set_error(
    protocol::packet_decoder::error_code ec)
{
    std::cerr
        << "Error protocolo: "
        << static_cast<int>(ec)
        << std::endl;
}


// ============================================================
// Encoder - Jetson -> STM32
// ============================================================

// 'A' - Heartbeat

void stm32canbus_serialif::send_heartbeat()
{
    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'A';

    protocol::packet_encoder::send(1);
}


// 'B' - Comando de válvula

void stm32canbus_serialif::set_valve_command(
    uint8_t body,
    int16_t command)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    command =
        std::clamp<int16_t>(
            command,
            -1000,
            1000
        );

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'B';
    payload[1] = body;

    const uint16_t raw =
        static_cast<uint16_t>(command);

    payload[2] =
        static_cast<uint8_t>(
            (raw >> 8) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            raw & 0xFF
        );

    protocol::packet_encoder::send(4);
}


// 'C' - Detener todas las válvulas

void stm32canbus_serialif::stop_all_valves()
{
    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'C';

    protocol::packet_encoder::send(1);
}


// 'D' - Altura objetivo

void stm32canbus_serialif::set_target_height(
    uint8_t body,
    uint16_t height_mm)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'D';
    payload[1] = body;

    payload[2] =
        static_cast<uint8_t>(
            (height_mm >> 8) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            height_mm & 0xFF
        );

    protocol::packet_encoder::send(4);
}

// 'J' - Reconocer / limpiar falla NO_MOVEMENT

void stm32canbus_serialif::clear_body_fault(
    uint8_t body)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'J';
    payload[1] = body;

    protocol::packet_encoder::send(2);
}

// ============================================================
// OPCODE 'K' - Configuración runtime STM32
// ============================================================


// ------------------------------------------------------------
// K 0x01 - Límites de altura por cuerpo
// ------------------------------------------------------------

void stm32canbus_serialif::set_body_limits(
    uint8_t body,
    uint16_t min_height_mm,
    uint16_t max_height_mm)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    if (min_height_mm >= max_height_mm)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x01;
    payload[2] = body;

    payload[3] =
        static_cast<uint8_t>(
            (min_height_mm >> 8) & 0xFF
        );

    payload[4] =
        static_cast<uint8_t>(
            min_height_mm & 0xFF
        );

    payload[5] =
        static_cast<uint8_t>(
            (max_height_mm >> 8) & 0xFF
        );

    payload[6] =
        static_cast<uint8_t>(
            max_height_mm & 0xFF
        );

    protocol::packet_encoder::send(7);
}


// ------------------------------------------------------------
// K 0x02 - Umbral de comando
// ------------------------------------------------------------

void stm32canbus_serialif::set_move_command_threshold(
    uint16_t threshold)
{
    if (threshold == 0 ||
        threshold > 1000)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x02;

    payload[2] =
        static_cast<uint8_t>(
            (threshold >> 8) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            threshold & 0xFF
        );

    protocol::packet_encoder::send(4);
}


// ------------------------------------------------------------
// K 0x03 - Movimiento mínimo en mm
// ------------------------------------------------------------

void stm32canbus_serialif::set_min_body_movement(
    float movement_mm)
{
    if (movement_mm <= 0.0f)
    {
        return;
    }

    uint32_t scaled =
        static_cast<uint32_t>(
            movement_mm * 100.0f + 0.5f
        );

    if (scaled == 0 ||
        scaled > 65535)
    {
        return;
    }

    uint16_t raw =
        static_cast<uint16_t>(scaled);

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x03;

    payload[2] =
        static_cast<uint8_t>(
            (raw >> 8) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            raw & 0xFF
        );

    protocol::packet_encoder::send(4);
}


// ------------------------------------------------------------
// K 0x04 - Timeout NO_MOVEMENT
// ------------------------------------------------------------

void stm32canbus_serialif::set_no_movement_timeout(
    uint32_t timeout_ms)
{
    if (timeout_ms < 100 ||
        timeout_ms > 60000)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x04;

    payload[2] =
        static_cast<uint8_t>(
            (timeout_ms >> 24) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            (timeout_ms >> 16) & 0xFF
        );

    payload[4] =
        static_cast<uint8_t>(
            (timeout_ms >> 8) & 0xFF
        );

    payload[5] =
        static_cast<uint8_t>(
            timeout_ms & 0xFF
        );

    protocol::packet_encoder::send(6);
}


// ------------------------------------------------------------
// K 0x05 - Timeout de consigna AUTO
// ------------------------------------------------------------

void stm32canbus_serialif::set_target_timeout(
    uint32_t timeout_ms)
{
    if (timeout_ms < 100 ||
        timeout_ms > 60000)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x05;

    payload[2] =
        static_cast<uint8_t>(
            (timeout_ms >> 24) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            (timeout_ms >> 16) & 0xFF
        );

    payload[4] =
        static_cast<uint8_t>(
            (timeout_ms >> 8) & 0xFF
        );

    payload[5] =
        static_cast<uint8_t>(
            timeout_ms & 0xFF
        );

    protocol::packet_encoder::send(6);
}


// ------------------------------------------------------------
// K 0x06 - Sentido del encoder
// ------------------------------------------------------------

void stm32canbus_serialif::set_encoder_direction(
    uint8_t body,
    uint8_t direction)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    if (direction > 1)
    {
        return;
    }

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x06;
    payload[2] = body;
    payload[3] = direction;

    protocol::packet_encoder::send(4);
}


// ------------------------------------------------------------
// K 0x07 - Escala encoder en mm/pulso
// ------------------------------------------------------------

void stm32canbus_serialif::set_encoder_scale(
    uint8_t body,
    float mm_per_pulse)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    if (mm_per_pulse <= 0.0f)
    {
        return;
    }

    double scaled =
        static_cast<double>(
            mm_per_pulse
        ) * 100000.0;

    if (scaled < 1.0 ||
        scaled > 4294967295.0)
    {
        return;
    }

    uint32_t raw =
        static_cast<uint32_t>(
            scaled + 0.5
        );

    uint8_t* payload =
        protocol::packet_encoder::
            get_payload_buffer();

    payload[0] = 'K';
    payload[1] = 0x07;
    payload[2] = body;

    payload[3] =
        static_cast<uint8_t>(
            (raw >> 24) & 0xFF
        );

    payload[4] =
        static_cast<uint8_t>(
            (raw >> 16) & 0xFF
        );

    payload[5] =
        static_cast<uint8_t>(
            (raw >> 8) & 0xFF
        );

    payload[6] =
        static_cast<uint8_t>(
            raw & 0xFF
        );

    protocol::packet_encoder::send(7);
}

// ============================================================
// Envío físico por Boost.Asio
// ============================================================

void stm32canbus_serialif::send_impl(
    const uint8_t* buf,
    uint8_t n)
{
    if (!port.is_open())
    {
        return;
    }

    boost::asio::write(
        port,
        boost::asio::buffer(buf, n)
    );
}