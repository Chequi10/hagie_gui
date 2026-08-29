#include "ai/tassel_counter.h"


void TasselCounter::reset()
{
    state =
        State {};
}


void TasselCounter::processDetections(
    const TasselDetector::Result& result)
{
    if (!result.valid)
    {
        return;
    }


    const std::uint64_t detectionCount =
        static_cast<std::uint64_t>(
            result.detections.size()
        );


    /*
     * Cámaras 0..4:
     * delanteras.
     */
    if (result.camera_index < 5)
    {
        state.front_count +=
            detectionCount;

        return;
    }


    /*
     * Cámaras 5..6:
     * traseras.
     */
    if (result.camera_index < 7)
    {
        state.rear_count +=
            detectionCount;
    }
}


TasselCounter::State
TasselCounter::getState() const
{
    return state;
}