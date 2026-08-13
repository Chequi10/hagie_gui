#pragma once

#include <cstdint>
#include <cstddef>
#include <chrono>

namespace protocol
{

constexpr std::size_t HEADER_SIZE = 5;   // "PKT!" + LENGTH
constexpr std::size_t PAYLOAD_BUFFER_SIZE = 96;
constexpr std::size_t TRAILER_SIZE = 3;  // CRC16 + '\n'

constexpr std::size_t MAX_PACKET_SIZE =
    HEADER_SIZE + PAYLOAD_BUFFER_SIZE + TRAILER_SIZE;

constexpr uint32_t PACKET_TIMEOUT_IN_MS = 500;

constexpr char PACKET_SYNC_0_CHAR = 'P';
constexpr char PACKET_SYNC_1_CHAR = 'K';
constexpr char PACKET_SYNC_2_CHAR = 'T';
constexpr char PACKET_SYNC_3_CHAR = '!';
constexpr char PACKET_TERMINATOR_CHAR = '\n';


class packet_encoder
{
public:
    packet_encoder();

    uint8_t* get_payload_buffer();

    void send(uint8_t length);

    virtual ~packet_encoder() = default;

protected:
    virtual void send_impl(
        const uint8_t* buf,
        uint8_t n
    ) = 0;

    const uint8_t* get_packet() const;

private:
    uint8_t buffer[MAX_PACKET_SIZE] {};

    void calc_crc_and_close_packet(uint8_t length);
};


class packet_decoder
{
public:

    enum class error_code
    {
        success = 0,
        bad_sync,
        invalid_length,
        bad_crc,
        bad_terminator,
        unknown_opcode,
        timeout
    };

    packet_decoder();

    virtual ~packet_decoder() = default;

    void feed(uint8_t c);

    void check_timeouts();

    void reset();

protected:

    virtual void handle_packet(
        const uint8_t* payload,
        std::size_t n
    ) = 0;

    virtual void set_error(error_code ec)
    {
        (void)ec;
    }

private:

    enum class pkt_state
    {
        idle = 0,
        expecting_start_sync1,
        expecting_start_sync2,
        expecting_start_sync3,
        expecting_length,
        expecting_payload,
        expecting_crc1,
        expecting_crc2,
        expecting_terminator
    };

    pkt_state current_state;

    uint8_t received_payload_buffer[
        PAYLOAD_BUFFER_SIZE + 1
    ] {};

    std::size_t received_payload_index;

    uint8_t payload_length;

    uint16_t expected_crc16;

    uint16_t crc16;

    uint8_t last_received_char;

    std::chrono::steady_clock::time_point
        start_of_packet_t0;

    void handle_pkt_state_idle();
    uint8_t NewFunction();
    void handle_pkt_state_expecting_start_sync1();
    void handle_pkt_state_expecting_start_sync2();
    void handle_pkt_state_expecting_start_sync3();
    void handle_pkt_state_expecting_length();
    void handle_pkt_state_expecting_payload();
    void handle_pkt_state_expecting_crc1();
    void handle_pkt_state_expecting_crc2();
    void handle_pkt_state_expecting_terminator();
};


uint16_t calc_crc16(
    const uint8_t* data_p,
    uint8_t length
);

}