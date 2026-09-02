#pragma once


/*
 * ============================================================
 * COMPATIBILIDAD TENSORRT
 * ============================================================
 *
 * Este archivo concentra las diferencias entre
 * versiones de TensorRT.
 *
 * La ThinkPad puede compilar Hagie sin TensorRT.
 *
 * La Jetson compilará este bloque cuando:
 *
 * HAGIE_ENABLE_TENSORRT
 *
 * esté habilitado.
 */


#ifdef HAGIE_ENABLE_TENSORRT

#include <NvInfer.h>


namespace TensorRtCompat
{

constexpr int majorVersion()
{
    return NV_TENSORRT_MAJOR;
}


constexpr bool isTensorRt8()
{
    return NV_TENSORRT_MAJOR == 8;
}


constexpr bool isTensorRt10OrNewer()
{
    return NV_TENSORRT_MAJOR >= 10;
}


static_assert(
    NV_TENSORRT_MAJOR >= 8,
    "Hagie requires TensorRT 8 or newer"
);


} // namespace TensorRtCompat


#else


namespace TensorRtCompat
{

constexpr int majorVersion()
{
    return 0;
}


constexpr bool isTensorRt8()
{
    return false;
}


constexpr bool isTensorRt10OrNewer()
{
    return false;
}


} // namespace TensorRtCompat


#endif