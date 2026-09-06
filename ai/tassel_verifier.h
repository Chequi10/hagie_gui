#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "ai/tassel_detector.h"


class TasselVerifier
{
public:

    static constexpr std::size_t BODY_COUNT =
        6;


    struct State
    {
        std::uint64_t pending =
            0;

        std::uint64_t verified_remaining =
            0;

        std::uint64_t verified_removed =
            0;
    };


    TasselVerifier() = default;

    ~TasselVerifier() = default;


    void reset();

    void setVerificationWindow(
        std::uint64_t minDelayMs,
        std::uint64_t maxDelayMs
    );


    void processFrontDetections(
        const TasselDetector::Result& result
    );


    void processRearDetections(
        const TasselDetector::Result& result
    );


    void update(
        std::uint64_t currentTimestampMs
    );


    State getState() const;


private:

    struct PendingTassel
    {
        std::uint64_t timestamp_ms =
            0;

        std::size_t body_index =
            0;
    };


    std::uint64_t minDelayMs =
        1000;

    std::uint64_t maxDelayMs =
        5000;


    std::deque<PendingTassel>
        pendingTassels;


    State state;
};