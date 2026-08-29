#include "ai/tassel_verifier.h"


void TasselVerifier::reset()
{
    pendingTassels.clear();

    state =
        State {};
}


void TasselVerifier::processFrontDetections(
    const TasselDetector::Result& result)
{
    if (!result.valid)
    {
        return;
    }


    if (result.camera_index >= 5)
    {
        return;
    }


    for (const auto& detection :
         result.detections)
    {
        (void)detection;

        PendingTassel pending;

        pending.timestamp_ms =
            result.timestamp_ms;

        pendingTassels.push_back(
            pending
        );
    }


    state.pending =
        pendingTassels.size();
}


void TasselVerifier::processRearDetections(
    const TasselDetector::Result& result)
{
    if (!result.valid)
    {
        return;
    }


    if (result.camera_index < 5 ||
        result.camera_index >= 7)
    {
        return;
    }


    for (const auto& detection :
         result.detections)
    {
        (void)detection;


        for (auto it =
                 pendingTassels.begin();
             it != pendingTassels.end();
             ++it)
        {
            if (result.timestamp_ms <
                it->timestamp_ms)
            {
                continue;
            }


            const std::uint64_t age =
                result.timestamp_ms -
                it->timestamp_ms;


            if (age < MIN_DELAY_MS)
            {
                continue;
            }


            if (age > MAX_DELAY_MS)
            {
                continue;
            }


            /*
             * Una detección trasera se asocia
             * con una detección frontal pendiente.
             *
             * Significa que esa panoja sigue presente.
             */
            pendingTassels.erase(
                it
            );

            ++state.verified_remaining;

            break;
        }
    }


    state.pending =
        pendingTassels.size();
}


void TasselVerifier::update(
    std::uint64_t currentTimestampMs)
{
    while (!pendingTassels.empty())
    {
        const PendingTassel& pending =
            pendingTassels.front();


        if (currentTimestampMs <
            pending.timestamp_ms)
        {
            break;
        }


        const std::uint64_t age =
            currentTimestampMs -
            pending.timestamp_ms;


        if (age <= MAX_DELAY_MS)
        {
            break;
        }


        /*
         * Venció la ventana y nunca apareció
         * en una cámara trasera.
         *
         * Se considera removida.
         */
        pendingTassels.pop_front();

        ++state.verified_removed;
    }


    state.pending =
        pendingTassels.size();
}


TasselVerifier::State
TasselVerifier::getState() const
{
    return state;
}