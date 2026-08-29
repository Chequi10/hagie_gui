#include <iostream>

#include "ai/tassel_verifier.h"


static TasselDetector::Detection makeDetection()
{
    TasselDetector::Detection detection;

    detection.x =
        100;

    detection.y =
        100;

    detection.width =
        30;

    detection.height =
        50;

    detection.confidence =
        0.90f;

    return detection;
}


int main()
{
    TasselVerifier verifier;


    /*
     * ========================================================
     * CASO 1:
     * DOS PANOJAS DETECTADAS ADELANTE
     * ========================================================
     */

    TasselDetector::Result frontResult;

    frontResult.valid =
        true;

    frontResult.camera_index =
        0;

    frontResult.timestamp_ms =
        1000;

    frontResult.detections =
    {
        makeDetection(),
        makeDetection()
    };


    verifier.processFrontDetections(
        frontResult
    );


    TasselVerifier::State state =
        verifier.getState();


    std::cout
        << "Pendientes luego de frontal: "
        << state.pending
        << std::endl;


    if (state.pending != 2)
    {
        std::cerr
            << "ERROR: deberían existir 2 pendientes"
            << std::endl;

        return 1;
    }


    /*
     * ========================================================
     * CASO 2:
     * UNA APARECE ATRÁS
     * ========================================================
     *
     * La detección trasera llega 2000 ms después.
     * Está dentro de la ventana 1000..5000 ms.
     *
     * Se considera que ESA panoja sigue presente.
     */

    TasselDetector::Result rearResult;

    rearResult.valid =
        true;

    rearResult.camera_index =
        5;

    rearResult.timestamp_ms =
        3000;

    rearResult.detections =
    {
        makeDetection()
    };


    verifier.processRearDetections(
        rearResult
    );


    state =
        verifier.getState();


    std::cout
        << "Pendientes luego de trasera: "
        << state.pending
        << std::endl;

    std::cout
        << "Siguen presentes: "
        << state.verified_remaining
        << std::endl;


    if (state.pending != 1)
    {
        std::cerr
            << "ERROR: debería quedar 1 pendiente"
            << std::endl;

        return 1;
    }


    if (state.verified_remaining != 1)
    {
        std::cerr
            << "ERROR: debería existir 1 panoja no removida"
            << std::endl;

        return 1;
    }


    /*
     * ========================================================
     * CASO 3:
     * LA OTRA NUNCA APARECE ATRÁS
     * ========================================================
     *
     * Avanzamos más allá de los 5000 ms.
     */

    verifier.update(
        7000
    );


    state =
        verifier.getState();


    std::cout
        << "Pendientes finales: "
        << state.pending
        << std::endl;

    std::cout
        << "Removidas: "
        << state.verified_removed
        << std::endl;


    if (state.pending != 0)
    {
        std::cerr
            << "ERROR: no deberían quedar pendientes"
            << std::endl;

        return 1;
    }


    if (state.verified_removed != 1)
    {
        std::cerr
            << "ERROR: debería existir 1 panoja removida"
            << std::endl;

        return 1;
    }


    std::cout
        << "TEST OK"
        << std::endl;


    return 0;
}