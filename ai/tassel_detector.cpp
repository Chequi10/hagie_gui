#include "ai/tassel_detector.h"


std::size_t TasselDetector::bodyFromImagePosition(
    std::size_t cameraIndex,
    int centerX,
    std::size_t imageWidth)
{
    if (imageWidth == 0)
    {
        return 0;
    }


    /*
     * Evitamos posiciones fuera
     * de los límites de la imagen.
     */
    if (centerX < 0)
    {
        centerX =
            0;
    }


    if (centerX >=
        static_cast<int>(imageWidth))
    {
        centerX =
            static_cast<int>(
                imageWidth - 1
            );
    }


    /*
     * ========================================================
     * CÁMARAS FRONTALES
     * ========================================================
     *
     * CAM 0 -> cuerpos 0 y 1
     * CAM 1 -> cuerpos 1 y 2
     * CAM 2 -> cuerpos 2 y 3
     * CAM 3 -> cuerpos 3 y 4
     * CAM 4 -> cuerpos 4 y 5
     *
     * Mitad izquierda:
     * primer cuerpo.
     *
     * Mitad derecha:
     * segundo cuerpo.
     */
    if (cameraIndex < 5)
    {
        const bool rightSide =
            centerX >=
            static_cast<int>(
                imageWidth / 2
            );


        return cameraIndex +
            (rightSide ? 1 : 0);
    }


    /*
     * ========================================================
     * CÁMARAS TRASERAS
     * ========================================================
     *
     * CAM 5 -> cuerpos 0,1,2
     * CAM 6 -> cuerpos 3,4,5
     *
     * La imagen se divide
     * horizontalmente en tres zonas.
     */
    std::size_t zone =
        static_cast<std::size_t>(
            centerX
        ) * 3 / imageWidth;


    if (zone > 2)
    {
        zone =
            2;
    }


    if (cameraIndex == 5)
    {
        return zone;
    }


    if (cameraIndex == 6)
    {
        return 3 + zone;
    }


    /*
     * Cámara inválida.
     * Por seguridad devolvemos cuerpo 0.
     */
    return 0;
}


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
     * Más adelante este bloque será reemplazado
     * por el detector real.
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


    const int detection1CenterX =
        detection1.x +
        detection1.width / 2;


    detection1.body_index =
        bodyFromImagePosition(
            cameraIndex,
            detection1CenterX,
            frame.width
        );


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


        const int detection2CenterX =
            detection2.x +
            detection2.width / 2;


        detection2.body_index =
            bodyFromImagePosition(
                cameraIndex,
                detection2CenterX,
                frame.width
            );


        result.detections.push_back(
            detection2
        );
    }


    result.valid =
        true;


    return true;
}