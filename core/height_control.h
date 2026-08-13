#ifndef HEIGHT_CONTROL_H
#define HEIGHT_CONTROL_H

#include <array>
#include <atomic>
#include <cstdint>
#include <chrono>

class height_control
{
public:
    static constexpr std::size_t BODY_COUNT = 6;

    height_control();

    void set_target(
        uint8_t body,
        uint16_t height_mm
    );

    uint16_t get_target(
        uint8_t body
    ) const;

    bool has_target(
        uint8_t body
    ) const;

    void clear_target(
        uint8_t body
    );

    void clear_all();

private:
    std::array<
        std::atomic<uint16_t>,
        BODY_COUNT
    > target_height_mm;

    std::array<
        std::atomic<bool>,
        BODY_COUNT
    > target_valid;

    std::array<
        std::chrono::steady_clock::time_point,
        BODY_COUNT
    > target_timestamp;

    static constexpr std::array<uint16_t, BODY_COUNT> MIN_HEIGHT_MM =
    {
        50,
        50,
        50,
        50,
        50,
        50
    };

    static constexpr std::array<uint16_t, BODY_COUNT> MAX_HEIGHT_MM =
    {
        700,
        700,
        700,
        700,
        700,
        700
    };

    static constexpr std::chrono::milliseconds TARGET_TIMEOUT{500};
};

#endif