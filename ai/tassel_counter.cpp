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


    frontPhysicalTracks.clear();
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

        /*
     * ========================================================
     * ELIMINAR TRACKS FISICOS FRONTALES VIEJOS
     * ========================================================
     */
    if (result.camera_index < 5)
    {
        frontPhysicalTracks.erase(
            std::remove_if(
                frontPhysicalTracks.begin(),
                frontPhysicalTracks.end(),

                [&result](
                    const FrontPhysicalTrack& track)
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
                        FRONT_3D_TRACK_TIMEOUT_MS;
                }
            ),

            frontPhysicalTracks.end()
        );
    }


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
         * DEDUPLICACION 3D ENTRE CAMARAS FRONTALES
         * ====================================================
         *
         * Llegamos acá porque esta detección es nueva
         * para ESTA cámara.
         *
         * Ahora comprobamos si otra cámara frontal
         * ya observó físicamente la misma panoja.
         */
        if (result.camera_index < 5 &&
            detection.position_3d_valid)
        {
            FrontPhysicalTrack* matchedPhysicalTrack =
                nullptr;


            for (auto& physicalTrack :
                 frontPhysicalTracks)
            {
                /*
                 * Nunca fusionar panojas asignadas
                 * a cuerpos diferentes.
                 */
                if (physicalTrack.body_index !=
                    detection.body_index)
                {
                    continue;
                }


                const float dx =
                    detection.position_x -
                    physicalTrack.position_x;

                const float dy =
                    detection.position_y -
                    physicalTrack.position_y;

                const float dz =
                    detection.position_z -
                    physicalTrack.position_z;


                const float distance =
                    std::sqrt(
                        dx * dx +
                        dy * dy +
                        dz * dz
                    );


                if (distance <=
                    FRONT_3D_MATCH_DISTANCE_M)
                {
                    matchedPhysicalTrack =
                        &physicalTrack;

                    break;
                }
            }


            /*
             * Ya fue vista físicamente por otra
             * cámara frontal.
             *
             * Creamos igualmente el track local
             * de esta cámara para que en el próximo
             * frame funcione el tracking por píxeles,
             * pero NO incrementamos el conteo.
             */
            if (matchedPhysicalTrack != nullptr)
            {
                TrackedTassel newCameraTrack;

                newCameraTrack.center_x =
                    centerX;

                newCameraTrack.center_y =
                    centerY;

                newCameraTrack.body_index =
                    detection.body_index;

                newCameraTrack.last_seen_timestamp_ms =
                    result.timestamp_ms;


                cameraTracks.push_back(
                    newCameraTrack
                );

                        /*
                


                /*
                 * Actualizar la posición física con
                 * la observación más reciente.
                 */
                matchedPhysicalTrack->position_x =
                    detection.position_x;

                matchedPhysicalTrack->position_y =
                    detection.position_y;

                matchedPhysicalTrack->position_z =
                    detection.position_z;

                matchedPhysicalTrack->last_seen_timestamp_ms =
                    result.timestamp_ms;


                /*
                 * IMPORTANTE:
                 *
                 * No se agrega a newDetections
                 * y no aumenta front_count.
                 */
                continue;
            }
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
         * Si esta panoja frontal tiene una posición
         * física 3D válida, la registramos en el
         * tracker compartido entre cámaras.
         */
        if (result.camera_index < 5 &&
            detection.position_3d_valid)
        {
            FrontPhysicalTrack physicalTrack;

            physicalTrack.position_x =
                detection.position_x;

            physicalTrack.position_y =
                detection.position_y;

            physicalTrack.position_z =
                detection.position_z;

            physicalTrack.body_index =
                detection.body_index;

            physicalTrack.last_seen_timestamp_ms =
                result.timestamp_ms;


            frontPhysicalTracks.push_back(
                physicalTrack
            );
        }


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