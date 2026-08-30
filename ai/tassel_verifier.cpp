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


    /*
     * Solamente cámaras frontales 0..4.
     */
    if (result.camera_index >= 5)
    {
        return;
    }


    for (const auto& detection :
         result.detections)
    {
        if (detection.body_index >=
            BODY_COUNT)
        {
            continue;
        }


        PendingTassel pending;

        pending.timestamp_ms =
            result.timestamp_ms;

        pending.body_index =
            detection.body_index;


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


    /*
     * Cámara 5 = trasera izquierda.
     * Cámara 6 = trasera derecha.
     */
    if (result.camera_index < 5 ||
        result.camera_index >= 7)
    {
        return;
    }


    const bool rearIsLeft =
        result.camera_index == 5;


    for (const auto& detection :
         result.detections)
    {
        (void)detection;


        for (auto it =
                 pendingTassels.begin();
             it != pendingTassels.end();
             ++it)
        {
            /*
             * =================================================
             * ZONA FÍSICA
             * =================================================
             *
             * body 0..2 -> cuerpos físicos 1..3 -> izquierda
             * body 3..5 -> cuerpos físicos 4..6 -> derecha
             */

            const bool pendingIsLeft =
                it->body_index < 3;


            /*
             * Una cámara trasera sólo puede verificar
             * panojas pertenecientes a su mitad.
             */
            if (pendingIsLeft !=
                rearIsLeft)
            {
                continue;
            }


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
             * Encontramos una panoja frontal pendiente
             * perteneciente a la misma zona física.
             *
             * Reapareció atrás:
             * NO fue removida.
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
    for (auto it =
             pendingTassels.begin();
         it != pendingTassels.end();)
    {
        if (currentTimestampMs <
            it->timestamp_ms)
        {
            ++it;

            continue;
        }


        const std::uint64_t age =
            currentTimestampMs -
            it->timestamp_ms;


        if (age <= MAX_DELAY_MS)
        {
            ++it;

            continue;
        }


        /*
         * Venció la ventana sin reaparecer
         * en la cámara trasera correspondiente.
         *
         * Se considera removida.
         */
        it =
            pendingTassels.erase(
                it
            );

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