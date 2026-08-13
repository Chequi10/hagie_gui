#ifndef VALVE_TEST_H
#define VALVE_TEST_H

#include <array>
#include <cstdint>

#include "height_control.h"

void valve_test_set_targets(
    height_control& heightControl,
    const std::array<uint16_t, 6>& heights_mm
);

#endif