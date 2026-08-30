#include "ai/tassel_counter.h"

#include <algorithm>
#include <cmath>


void TasselCounter::reset()
{
    state =
        State {};


    for (auto& cameraTracks :
         trackedTassels)
    {
        cameraTracks.clear();
    }
}


std::vector<TasselDetector::Detection>
TasselCounter::processDetections(
    const TasselDetector::Result& result)
{
    std::vector<TasselDetector::Detection>
        newDetections;


    if (!result.valid)
    {
        return newDetections;
    }


    if (result.camera_index >=
        CAMERA_COUNT)
    {
        return newDetections;
    }


    auto& cameraTracks =
        trackedTassels[
            result.camera_index
        ];


    /*
     * Eliminamos tracks viejos.
     */
    cameraTracks.erase(
        std::remove_if(
            cameraTracks.begin(),
            cameraTracks.end(),

            [&result](
                const TrackedTassel& track)
            {
                if (result.timestamp_ms <
                    track.last_seen_timestamp_ms)
                {
                    return true;
                }


                return
                    (result.timestamp_ms -
                     track.last_seen_timestamp_ms)
                    >
                    TRACK_TIMEOUT_MS;
            }
        ),

        cameraTracks.end()
    );


    for (const auto& detection :
         result.detections)
    {
        const int centerX =
            detection.x +
            detection.width / 2;

        const int centerY =
            detection.y +
            detection.height / 2;


        TrackedTassel* matchedTrack =
            nullptr;


        for (auto& track :
             cameraTracks)
        {
            /*
             * Una panoja sólo puede continuar
             * un track del mismo cuerpo físico.
             */
            if (track.body_index !=
                detection.body_index)
            {
                continue;
            }


            const int dx =
                centerX -
                track.center_x;

            const int dy =
                centerY -
                track.center_y;


            const double distance =
                std::sqrt(
                    static_cast<double>(
                        dx * dx +
                        dy * dy
                    )
                );


            if (distance <=
                MATCH_DISTANCE_PIXELS)
            {
                matchedTrack =
                    &track;

                break;
            }
        }


        /*
         * Ya existía:
         * actualizamos posición y timestamp,
         * pero NO contamos otra vez.
         */
        if (matchedTrack != nullptr)
        {
            matchedTrack->center_x =
                centerX;

            matchedTrack->center_y =
                centerY;

            matchedTrack->body_index =
                detection.body_index;

            matchedTrack->last_seen_timestamp_ms =
                result.timestamp_ms;

            continue;
        }


        /*
         * ====================================================
         * NUEVA PANOJA
         * ====================================================
         */
        TrackedTassel newTrack;

        newTrack.center_x =
            centerX;

        newTrack.center_y =
            centerY;

        newTrack.body_index =
            detection.body_index;

        newTrack.last_seen_timestamp_ms =
            result.timestamp_ms;


        cameraTracks.push_back(
            newTrack
        );


        /*
         * Guardamos exactamente cuál fue
         * la detección nueva.
         *
         * Esto permitirá que TasselVerifier
         * reciba su body_index correcto.
         */
        newDetections.push_back(
            detection
        );


        if (result.camera_index < 5)
        {
            ++state.front_count;
        }
        else
        {
            ++state.rear_count;
        }
    }


    return newDetections;
}


TasselCounter::State
TasselCounter::getState() const
{
    return state;
}