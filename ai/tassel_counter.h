#pragma once

#include <cstddef>
#include <cstdint>

#include "ai/tassel_detector.h"


class TasselCounter
{
public:

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

    State state;
};
