#include "height_control.h"

height_control::height_control()
{
    for (std::size_t i = 0; i < BODY_COUNT; ++i)
    {
        target_height_mm[i].store(0);
        target_valid[i].store(false);
        target_timestamp[i] =
        std::chrono::steady_clock::now();
    }
}


void height_control::set_target(
    uint8_t body,
    uint16_t height_mm)
{
    if (body >= BODY_COUNT)
    {
        return;
    }
    if (height_mm < MIN_HEIGHT_MM[body] ||
        height_mm > MAX_HEIGHT_MM[body])
    {
        target_valid[body].store(false);
        return;
    }
    target_height_mm[body].store(height_mm);
    target_valid[body].store(true);
    target_timestamp[body] =
    std::chrono::steady_clock::now();
}


uint16_t height_control::get_target(
    uint8_t body) const
{
    if (body >= BODY_COUNT)
    {
        return 0;
    }

    return target_height_mm[body].load();
}


bool height_control::has_target(
    uint8_t body) const
{
    if (body >= BODY_COUNT)
    {
        return false;
    }

    if (!target_valid[body].load())
    {
        return false;
    }

    auto now =
        std::chrono::steady_clock::now();

    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - target_timestamp[body]
        );

    return elapsed <= TARGET_TIMEOUT;
}


void height_control::clear_target(
    uint8_t body)
{
    if (body >= BODY_COUNT)
    {
        return;
    }

    target_valid[body].store(false);
}


void height_control::clear_all()
{
    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        target_valid[body].store(false);
    }
}