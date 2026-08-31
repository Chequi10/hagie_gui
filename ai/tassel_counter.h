#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "ai/tassel_detector.h"


class TasselCounter
{
public:

    static constexpr std::size_t CAMERA_COUNT =
        7;


    struct State
    {
        std::uint64_t front_count =
            0;

        std::uint64_t rear_count =
            0;
    };


    TasselCounter() = default;

    ~TasselCounter() = default;


    void reset();


    std::vector<TasselDetector::Detection>
    processDetections(
        const TasselDetector::Result& result
    );


    State getState() const;


private:

    struct TrackedTassel
    {
        int center_x =
            0;

        int center_y =
            0;

        std::size_t body_index =
            0;

        std::uint64_t last_seen_timestamp_ms =
            0;
    };

        /*
     * Track físico compartido por todas
     * las cámaras frontales.
     *
     * Se utiliza para evitar contar dos veces
     * la misma panoja cuando aparece en
     * cámaras frontales solapadas.
     */
    struct FrontPhysicalTrack
    {
        float position_x =
            0.0f;

        float position_y =
            0.0f;

        float position_z =
            0.0f;

        std::size_t body_index =
            0;

        std::uint64_t last_seen_timestamp_ms =
            0;
    };

    std::vector<FrontPhysicalTrack>
    frontPhysicalTracks;


    static constexpr int MATCH_DISTANCE_PIXELS =
        60;

    static constexpr std::uint64_t TRACK_TIMEOUT_MS =
        1000;

        /*
     * Distancia física máxima para considerar
     * que dos detecciones frontales corresponden
     * a la misma panoja.
     *
     * Unidad: metros.
     *
     * Valor inicial para pruebas.
     */
    static constexpr float
        FRONT_3D_MATCH_DISTANCE_M =
            0.12f;


    /*
     * El matching entre cámaras debe ser corto.
     *
     * Queremos unir observaciones casi simultáneas
     * de cámaras solapadas, no panojas distintas
     * vistas mucho tiempo después.
     */
    static constexpr std::uint64_t
        FRONT_3D_TRACK_TIMEOUT_MS =
            300;    


    State state;


    std::array<
        std::vector<TrackedTassel>,
        CAMERA_COUNT
    > trackedTassels;

    
};