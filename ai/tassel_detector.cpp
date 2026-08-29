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
     * Por ahora generamos detecciones conocidas.
     * Más adelante este bloque será reemplazado
     * por YOLO / TensorRT.
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


        result.detections.push_back(
            detection2
        );
    }


    result.valid =
        true;


    return true;
}