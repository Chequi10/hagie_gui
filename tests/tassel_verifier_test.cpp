#include <iostream>

#include "ai/tassel_verifier.h"


static TasselDetector::Detection makeDetection(
    std::size_t bodyIndex = 0)
{
    TasselDetector::Detection detection;

    detection.body_index =
        bodyIndex;

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

    /*
    * ========================================================
    * CASO 4:
    * VERIFICACIÓN POR ZONA FÍSICA
    * ========================================================
    *
    * Una panoja del cuerpo 5 (índice 4)
    * pertenece a la mitad derecha.
    *
    * La cámara trasera izquierda (índice 5)
    * NO debe poder verificarla.
    */

    verifier.reset();


    TasselDetector::Result rightFrontResult;

    rightFrontResult.valid =
        true;

    rightFrontResult.camera_index =
        3;

    rightFrontResult.timestamp_ms =
        1000;

    rightFrontResult.detections =
    {
        makeDetection(
            4
        )
    };


    verifier.processFrontDetections(
        rightFrontResult
    );


    TasselDetector::Result leftRearResult;

    leftRearResult.valid =
        true;

    leftRearResult.camera_index =
        5;

    leftRearResult.timestamp_ms =
        3000;

    leftRearResult.detections =
    {
        makeDetection(
            0
        )
    };


    verifier.processRearDetections(
        leftRearResult
    );


    state =
        verifier.getState();


    std::cout
        << "Zona incorrecta - pendientes: "
        << state.pending
        << std::endl;

    std::cout
        << "Zona incorrecta - presentes: "
        << state.verified_remaining
        << std::endl;


    if (state.pending != 1)
    {
        std::cerr
            << "ERROR: la trasera izquierda verificó una panoja derecha"
            << std::endl;

        return 1;
    }


    if (state.verified_remaining != 0)
    {
        std::cerr
            << "ERROR: hubo verificación cruzada entre zonas"
            << std::endl;

        return 1;
    }


    /*
    * Ahora llega la cámara trasera derecha.
    *
    * Sí debe verificarla.
    */

    TasselDetector::Result rightRearResult;

    rightRearResult.valid =
        true;

    rightRearResult.camera_index =
        6;

    rightRearResult.timestamp_ms =
        3200;

    rightRearResult.detections =
    {
        makeDetection(
            4
        )
    };


    verifier.processRearDetections(
        rightRearResult
    );


    state =
        verifier.getState();


    std::cout
        << "Zona correcta - pendientes: "
        << state.pending
        << std::endl;

    std::cout
        << "Zona correcta - presentes: "
        << state.verified_remaining
        << std::endl;


    if (state.pending != 0)
    {
        std::cerr
            << "ERROR: la trasera derecha no verificó la panoja"
            << std::endl;

        return 1;
    }


    if (state.verified_remaining != 1)
    {
        std::cerr
            << "ERROR: verificación derecha incorrecta"
            << std::endl;

        return 1;
    }    


    return 0;
}