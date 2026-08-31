#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vision/rgb_frame_source.h"


class TasselDetector
{
public:

    struct Detection
    {
        float confidence =
            0.0f;

        int x =
            0;

        int y =
            0;

        int width =
            0;

        int height =
            0;

        std::size_t body_index =
            0;

        /*
        * Posición física 3D de la panoja
        * expresada en coordenadas de la máquina.
        *
        * X = lateral
        * Y = longitudinal
        * Z = vertical
        *
        * Unidad: metros.
        */
        float position_x =
            0.0f;

        float position_y =
            0.0f;

        float position_z =
            0.0f;

        bool position_3d_valid =
            false;
    };


    struct Result
    {
        std::size_t camera_index =
            0;

        std::uint64_t timestamp_ms =
            0;

        std::vector<Detection> detections;

        bool valid =
            false;
    };


    TasselDetector() = default;

    ~TasselDetector() = default;


    static std::size_t bodyFromImagePosition(
        std::size_t cameraIndex,
        int centerX,
        std::size_t imageWidth
    );


    bool processFrame(
        std::size_t cameraIndex,
        const RgbFrameSource::Frame& frame,
        Result& result
    );
};