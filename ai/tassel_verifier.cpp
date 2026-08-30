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
     * Cubre cuerpos 0,1,2.
     *
     * Cámara 6 = trasera derecha.
     * Cubre cuerpos 3,4,5.
     */
    if (result.camera_index < 5 ||
        result.camera_index >= 7)
    {
        return;
    }


    for (const auto& detection :
         result.detections)
    {
        /*
         * Detección inválida.
         */
        if (detection.body_index >=
            BODY_COUNT)
        {
            continue;
        }


        /*
         * =====================================================
         * VALIDACIÓN DE ZONA DE LA CÁMARA TRASERA
         * =====================================================
         *
         * CAM 5 solamente puede informar cuerpos 0..2.
         * CAM 6 solamente puede informar cuerpos 3..5.
         */
        if (result.camera_index == 5 &&
            detection.body_index >= 3)
        {
            continue;
        }


        if (result.camera_index == 6 &&
            detection.body_index < 3)
        {
            continue;
        }


        for (auto it =
                 pendingTassels.begin();
             it != pendingTassels.end();
             ++it)
        {
            /*
             * =================================================
             * MISMO CUERPO FÍSICO
             * =================================================
             *
             * La detección trasera solamente puede verificar
             * una panoja frontal perteneciente exactamente
             * al mismo cuerpo.
             */
            if (it->body_index !=
                detection.body_index)
            {
                continue;
            }


            /*
             * El tiempo trasero no puede ser anterior
             * al tiempo de detección frontal.
             */
            if (result.timestamp_ms <
                it->timestamp_ms)
            {
                continue;
            }


            const std::uint64_t age =
                result.timestamp_ms -
                it->timestamp_ms;


            /*
             * Todavía no pudo llegar físicamente
             * desde la cámara frontal a la trasera.
             */
            if (age < MIN_DELAY_MS)
            {
                continue;
            }


            /*
             * Ya salió de la ventana válida.
             */
            if (age > MAX_DELAY_MS)
            {
                continue;
            }


            /*
             * =================================================
             * PANOJA ENCONTRADA NUEVAMENTE
             * =================================================
             *
             * Fue vista adelante y reapareció atrás
             * en el mismo cuerpo.
             *
             * Por lo tanto:
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