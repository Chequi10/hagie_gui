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


    void processDetections(
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

        std::uint64_t last_seen_timestamp_ms =
            0;
    };


    static constexpr int MATCH_DISTANCE_PIXELS =
        60;

    static constexpr std::uint64_t TRACK_TIMEOUT_MS =
        1000;


    State state;


    std::array<
        std::vector<TrackedTassel>,
        CAMERA_COUNT
    > trackedTassels;
};