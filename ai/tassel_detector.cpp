#include "ai/tassel_detector.h"


bool TasselDetector::processFrame(
    std::size_t cameraIndex,
    const RgbFrameSource::Frame& frame,
    Result& result)
{
    result =
        Result {};


    if (!frame.valid ||
        frame.width == 0 ||
        frame.height == 0 ||
        frame.data.empty())
    {
        return false;
    }


    result.camera_index =
        cameraIndex;

    result.timestamp_ms =
        frame.timestamp_ms;


    /*
     * ========================================================
     * DETECTOR SIMULADO DE PANOJAS
     * ========================================================
     *
     * Cámaras frontales:
     *
     * CAM 0 -> cuerpos 0 y 1
     * CAM 1 -> cuerpos 1 y 2
     * CAM 2 -> cuerpos 2 y 3
     * CAM 3 -> cuerpos 3 y 4
     * CAM 4 -> cuerpos 4 y 5
     *
     * Cámaras traseras:
     *
     * CAM 5 -> zona izquierda, cuerpos 0..2
     * CAM 6 -> zona derecha, cuerpos 3..5
     */


    Detection detection1;

    detection1.confidence =
        0.92f;

    detection1.x =
        static_cast<int>(
            frame.width / 4
        );

    detection1.y =
        static_cast<int>(
            frame.height / 3
        );

    detection1.width =
        30;

    detection1.height =
        50;


    /*
     * Asignación del cuerpo.
     */
    if (cameraIndex < 5)
    {
        detection1.body_index =
            cameraIndex;
    }
    else if (cameraIndex == 5)
    {
        /*
         * Trasera izquierda.
         * Por simulación usamos cuerpo 1.
         */
        detection1.body_index =
            0;
    }
    else
    {
        /*
         * Trasera derecha.
         * Por simulación usamos cuerpo 4.
         */
        detection1.body_index =
            3;
    }


    result.detections.push_back(
        detection1
    );


    /*
     * Las cámaras frontales 0..4
     * simulan una segunda panoja.
     */
    if (cameraIndex < 5)
    {
        Detection detection2;

        detection2.confidence =
            0.87f;

        detection2.x =
            static_cast<int>(
                frame.width / 2
            );

        detection2.y =
            static_cast<int>(
                frame.height / 4
            );

        detection2.width =
            28;

        detection2.height =
            45;


        /*
         * Segundo cuerpo cubierto
         * por la cámara frontal.
         */
        detection2.body_index =
            cameraIndex + 1;


        result.detections.push_back(
            detection2
        );
    }


    result.valid =
        true;


    return true;
}