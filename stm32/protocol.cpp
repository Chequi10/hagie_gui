#include "protocol.h"

namespace protocol
{

packet_decoder::packet_decoder()
{
    reset();
}

void packet_decoder::handle_pkt_state_idle()
{
    if (PACKET_SYNC_0_CHAR == NewFunction())
    {
        current_state = pkt_state::expecting_start_sync1;
    }
}

uint8_t packet_decoder::NewFunction()
{
return last_received_char;
}

void packet_decoder::handle_pkt_state_expecting_start_sync1()
{
    if (PACKET_SYNC_1_CHAR == last_received_char)
    {
        current_state = pkt_state::expecting_start_sync2;
    }
    else
    {
        set_error(error_code::bad_sync);
        reset();
    }
}

void packet_decoder::handle_pkt_state_expecting_start_sync2()
{
    if (PACKET_SYNC_2_CHAR == last_received_char)
    {
        current_state = pkt_state::expecting_start_sync3;
    }
    else
    {
        set_error(error_code::bad_sync);
        reset();
    }
}

void packet_decoder::handle_pkt_state_expecting_start_sync3()
{
    if (PACKET_SYNC_3_CHAR == last_received_char)
    {
        current_state = pkt_state::expecting_length;
    }
    else
    {
        set_error(error_code::bad_sync);
        reset();
    }
}

void packet_decoder::handle_pkt_state_expecting_length()
{
    payload_length =
        last_received_char;

    if (payload_length > 0 &&
        payload_length <= PAYLOAD_BUFFER_SIZE)
    {
        current_state = pkt_state::expecting_payload;
    }
    else
    {
        set_error(error_code::invalid_length);
        reset();
    }
}

void packet_decoder::handle_pkt_state_expecting_payload()
{
    received_payload_buffer[received_payload_index] =
        last_received_char;

    received_payload_index++;

    if (received_payload_index == payload_length)
    {
        expected_crc16 =
            calc_crc16(
                received_payload_buffer,
                static_cast<uint8_t>(received_payload_index)
            );

        received_payload_buffer[received_payload_index] = '\0';

        current_state = pkt_state::expecting_crc1;
    }
}

void packet_decoder::handle_pkt_state_expecting_crc1()
{
    crc16 =
        static_cast<uint16_t>(
            last_received_char
        ) << 8;

    current_state = pkt_state::expecting_crc2;
}

void packet_decoder::handle_pkt_state_expecting_crc2()
{
    crc16 |=
        last_received_char;

    if (expected_crc16 == crc16)
    {
        current_state = pkt_state::expecting_terminator;
    }
    else
    {
        set_error(error_code::bad_crc);
        reset();
    }
}

void packet_decoder::handle_pkt_state_expecting_terminator()
{
    if (PACKET_TERMINATOR_CHAR ==
        last_received_char)
    {
        handle_packet(
            received_payload_buffer,
            received_payload_index
        );
    }
    else
    {
        set_error(error_code::bad_terminator);
    }

    reset();
}

void packet_decoder::feed(uint8_t c)
{
    last_received_char = c;

    switch (current_state)
    {
        case pkt_state::idle:
            handle_pkt_state_idle();
            break;

        case pkt_state::expecting_start_sync1:
            handle_pkt_state_expecting_start_sync1();
            break;

        case pkt_state::expecting_start_sync2:
            handle_pkt_state_expecting_start_sync2();
            break;

        case pkt_state::expecting_start_sync3:
            handle_pkt_state_expecting_start_sync3();
            break;

        case pkt_state::expecting_length:
            handle_pkt_state_expecting_length();
            break;

        case pkt_state::expecting_payload:
            handle_pkt_state_expecting_payload();
            break;

        case pkt_state::expecting_crc1:
            handle_pkt_state_expecting_crc1();
            break;

        case pkt_state::expecting_crc2:
            handle_pkt_state_expecting_crc2();
            break;

        case pkt_state::expecting_terminator:
            handle_pkt_state_expecting_terminator();
            break;
    }
}

void packet_decoder::check_timeouts()
{
    auto now = std::chrono::steady_clock::now();

    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_of_packet_t0
        ).count();

    if (elapsed >= PACKET_TIMEOUT_IN_MS)
    {
        set_error(error_code::timeout);
        reset();
    }
}

void packet_decoder::reset()
{
    current_state = pkt_state::idle;
    received_payload_index = 0;
    payload_length = 0;
    expected_crc16 = 0;
    crc16 = 0;
    last_received_char = 0;

    start_of_packet_t0 =
        std::chrono::steady_clock::now();
}


packet_encoder::packet_encoder()
{
    buffer[0] = PACKET_SYNC_0_CHAR;
    buffer[1] = PACKET_SYNC_1_CHAR;
    buffer[2] = PACKET_SYNC_2_CHAR;
    buffer[3] = PACKET_SYNC_3_CHAR;
}

void packet_encoder::calc_crc_and_close_packet(uint8_t length)
{
    buffer[HEADER_SIZE - 1] = length;

    uint16_t crc =
        calc_crc16(
            get_payload_buffer(),
            length
        );

    buffer[HEADER_SIZE + length] =
        static_cast<uint8_t>(
            (crc >> 8) & 0xFF
        );

    buffer[HEADER_SIZE + length + 1] =
        static_cast<uint8_t>(
            crc & 0xFF
        );

    buffer[HEADER_SIZE + length + 2] =
        PACKET_TERMINATOR_CHAR;
}

uint8_t* packet_encoder::get_payload_buffer()
{
    return buffer + HEADER_SIZE;
}

const uint8_t* packet_encoder::get_packet() const
{
    return buffer;
}

void packet_encoder::send(uint8_t length)
{
    if (length == 0 ||
        length > PAYLOAD_BUFFER_SIZE)
    {
        return;
    }

    calc_crc_and_close_packet(length);

    send_impl(
        get_packet(),
        static_cast<uint8_t>(
            HEADER_SIZE +
            length +
            TRAILER_SIZE
        )
    );
}

uint16_t calc_crc16(
    const uint8_t* data_p,
    uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--)
    {
        x = static_cast<uint8_t>(
            (crc >> 8) ^ *data_p++
        );

        x ^= x >> 4;

        crc =
            static_cast<uint16_t>(
                (crc << 8) ^
                (static_cast<uint16_t>(x) << 12) ^
                (static_cast<uint16_t>(x) << 5) ^
                static_cast<uint16_t>(x)
            );
    }

    return crc;
}

}