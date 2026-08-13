#include "valve_test.h"

void valve_test_set_targets(
    height_control& heightControl,
    const std::array<uint16_t, 6>& heights_mm)
{
    for (std::size_t body = 0;
         body < heights_mm.size();
         ++body)
    {
        heightControl.set_target(
            static_cast<uint8_t>(body),
            heights_mm[body]
        );
    }
}