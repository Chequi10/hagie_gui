#include <iostream>

#include "ai/tassel_counter.h"


static TasselDetector::Detection makeDetection(
    int x,
    int y,
    int width = 30,
    int height = 50)
{
    TasselDetector::Detection detection;

    detection.x =
        x;

    detection.y =
        y;

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


    std::cout
        << "TEST OK"
        << std::endl;

    return 0;
}