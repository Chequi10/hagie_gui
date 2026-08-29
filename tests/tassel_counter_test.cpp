#include <iostream>

#include "ai/tassel_counter.h"


int main()
{
    TasselCounter counter;


    /*
     * Simulamos una detección de cámara frontal.
     * Cámara 1 = índice 0.
     */
    TasselDetector::Result frontResult;

    frontResult.valid =
        true;

    frontResult.camera_index =
        0;

    frontResult.detections.resize(
        2
    );


    counter.processDetections(
        frontResult
    );


    /*
     * Simulamos otra cámara frontal.
     * Cámara 5 = índice 4.
     */
    frontResult.camera_index =
        4;

    frontResult.detections.resize(
        3
    );

    counter.processDetections(
        frontResult
    );


    /*
     * Simulamos cámara trasera.
     * Cámara 6 = índice 5.
     */
    TasselDetector::Result rearResult;

    rearResult.valid =
        true;

    rearResult.camera_index =
        5;

    rearResult.detections.resize(
        1
    );

    counter.processDetections(
        rearResult
    );


    /*
     * Cámara 7 = índice 6.
     */
    rearResult.camera_index =
        6;

    rearResult.detections.resize(
        2
    );

    counter.processDetections(
        rearResult
    );


    const TasselCounter::State state =
        counter.getState();


    std::cout
        << "Conteo frontal: "
        << state.front_count
        << std::endl;

    std::cout
        << "Conteo trasero: "
        << state.rear_count
        << std::endl;


    if (state.front_count != 5)
    {
        std::cerr
            << "ERROR: conteo frontal incorrecto"
            << std::endl;

        return 1;
    }


    if (state.rear_count != 3)
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