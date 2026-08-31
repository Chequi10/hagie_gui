#include <iostream>

#include "ai/tassel_counter.h"


static TasselDetector::Detection makeDetection(
    int x,
    int y,
    std::size_t bodyIndex = 0,
    int width = 30,
    int height = 50)
{
    TasselDetector::Detection detection;

    detection.x =
        x;

    detection.y =
        y;

    detection.body_index =
        bodyIndex;

    detection.width =
        width;

    detection.height =
        height;

    detection.confidence =
        0.90f;

    return detection;
}


int main()
{
    TasselCounter counter;


    /*
     * ========================================================
     * CÁMARA FRONTAL
     * ========================================================
     *
     * Frame 1:
     * aparecen dos panojas nuevas.
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
        makeDetection(
            100,
            100
        ),

        makeDetection(
            300,
            100
        )
    };


    counter.processDetections(
        frontResult
    );


    /*
     * Frame 2:
     * mismas dos panojas ligeramente desplazadas.
     *
     * NO deben sumarse nuevamente.
     */
    frontResult.timestamp_ms =
        1100;

    frontResult.detections =
    {
        makeDetection(
            105,
            103
        ),

        makeDetection(
            295,
            105
        )
    };


    counter.processDetections(
        frontResult
    );


    /*
     * Frame 3:
     * aparece una tercera panoja lejos
     * de las anteriores.
     *
     * Debe sumar solamente 1.
     */
    frontResult.timestamp_ms =
        1200;

    frontResult.detections =
    {
        makeDetection(
            105,
            103
        ),

        makeDetection(
            295,
            105
        ),

        makeDetection(
            500,
            150
        )
    };


    counter.processDetections(
        frontResult
    );


    /*
     * ========================================================
     * CÁMARA TRASERA
     * ========================================================
     *
     * Una panoja aparece en dos frames.
     * Debe contarse solamente una vez.
     */
    TasselDetector::Result rearResult;

    rearResult.valid =
        true;

    rearResult.camera_index =
        5;

    rearResult.timestamp_ms =
        2000;

    rearResult.detections =
    {
        makeDetection(
            200,
            120
        )
    };


    counter.processDetections(
        rearResult
    );


    rearResult.timestamp_ms =
        2100;

    rearResult.detections =
    {
        makeDetection(
            205,
            123
        )
    };


    counter.processDetections(
        rearResult
    );


    /*
     * Leemos resultado.
     */
    const TasselCounter::State state =
        counter.getState();


    std::cout
        << "Conteo frontal unico: "
        << state.front_count
        << std::endl;

    std::cout
        << "Conteo trasero unico: "
        << state.rear_count
        << std::endl;


    /*
     * Esperamos:
     *
     * frontal = 3
     * trasero  = 1
     */
    if (state.front_count != 3)
    {
        std::cerr
            << "ERROR: conteo frontal incorrecto"
            << std::endl;

        return 1;
    }


    if (state.rear_count != 1)
    {
        std::cerr
            << "ERROR: conteo trasero incorrecto"
            << std::endl;

        return 1;
    }


    /*
    * ========================================================
    * CUERPOS FÍSICOS DISTINTOS
    * ========================================================
    *
    * Dos detecciones prácticamente en la misma posición
    * de imagen, pero pertenecientes a cuerpos diferentes,
    * deben mantenerse como panojas distintas.
    */

    TasselCounter bodyCounter;


    TasselDetector::Result bodyResult;

    bodyResult.valid =
        true;

    bodyResult.camera_index =
        1;

    bodyResult.timestamp_ms =
        3000;

    bodyResult.detections =
    {
        makeDetection(
            200,
            120,
            1
        ),

        makeDetection(
            205,
            123,
            2
        )
    };


    bodyCounter.processDetections(
        bodyResult
    );


    const TasselCounter::State bodyState =
        bodyCounter.getState();


    std::cout
        << "Panojas en cuerpos distintos: "
        << bodyState.front_count
        << std::endl;


    if (bodyState.front_count != 2)
    {
        std::cerr
            << "ERROR: se mezclaron panojas de cuerpos distintos"
            << std::endl;

        return 1;
    }


    std::cout
        << "TEST CUERPOS OK"
        << std::endl;

    std::cout
        << "TEST OK"
        << std::endl;


        /*
     * ========================================================
     * TEST DEDUPLICACION 3D ENTRE CAMARAS FRONTALES
     * ========================================================
     */

    counter.reset();


    TasselDetector::Result frontCamera0;

    frontCamera0.valid =
        true;

    frontCamera0.camera_index =
        0;

    frontCamera0.timestamp_ms =
        1000;


    TasselDetector::Detection detectionCam0 =
        makeDetection(
            250,
            100,
            1
        );

    detectionCam0.position_x =
        -1.50f;

    detectionCam0.position_y =
        2.40f;

    detectionCam0.position_z =
        1.80f;

    detectionCam0.position_3d_valid =
        true;


    frontCamera0.detections.push_back(
        detectionCam0
    );


    counter.processDetections(
        frontCamera0
    );


    /*
     * La cámara frontal 1 ve la MISMA panoja,
     * pero en una posición de píxel totalmente
     * diferente.
     *
     * La posición física 3D es prácticamente
     * la misma.
     */
    TasselDetector::Result frontCamera1;

    frontCamera1.valid =
        true;

    frontCamera1.camera_index =
        1;

    frontCamera1.timestamp_ms =
        1050;


    TasselDetector::Detection detectionCam1 =
        makeDetection(
            40,
            120,
            1
        );

    detectionCam1.position_x =
        -1.47f;

    detectionCam1.position_y =
        2.36f;

    detectionCam1.position_z =
        1.82f;

    detectionCam1.position_3d_valid =
        true;


    frontCamera1.detections.push_back(
        detectionCam1
    );


    counter.processDetections(
        frontCamera1
    );


    TasselCounter::State state3D =
        counter.getState();


    std::cout
        << "Conteo frontal con solapamiento 3D: "
        << state3D.front_count
        << std::endl;


    if (state3D.front_count != 1)
    {
        std::cerr
            << "ERROR: la misma panoja fue contada "
            << "dos veces entre camaras"
            << std::endl;

        return 1;
    }


    std::cout
        << "TEST DEDUPLICACION 3D OK"
        << std::endl;    

    return 0;
}