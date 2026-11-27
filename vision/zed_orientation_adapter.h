#pragma once

#include "vision/point_cloud_source.h"

#ifdef HAGIE_ENABLE_ZED_SDK

#include <sl/Camera.hpp>
#include <cmath>

namespace ZedOrientationAdapter
{

inline PointCloudSource::CameraOrientation fromRotation(
    const sl::Rotation& rotation)
{
    PointCloudSource::CameraOrientation result {};

    // Direccion vertical expresada en los ejes Hagie.
    const float gx = rotation.r21;
    const float gy = rotation.r20;
    const float gz = rotation.r22;

    if (!std::isfinite(gx) ||
        !std::isfinite(gy) ||
        !std::isfinite(gz))
        return result;

    const float horizontal = std::hypot(gx, gz);

    if (horizontal < 0.0001f)
        return result;

    constexpr float RAD_TO_DEG =
        180.0f / 3.14159265358979323846f;

    result.roll_deg =
        std::atan2(gx, gz) * RAD_TO_DEG;

    result.pitch_deg =
        std::atan2(-gy, horizontal) * RAD_TO_DEG;

    result.valid = true;

    return result;
}

}

#endif
