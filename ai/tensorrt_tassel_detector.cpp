#include "ai/tensorrt_tassel_detector.h"


bool TensorRtTasselDetector::initialize(
    const char* enginePath)
{
    /*
     * Por ahora solamente validamos
     * que se haya recibido una ruta.
     *
     * La carga real del motor TensorRT
     * se implementará en el siguiente paso.
     */
    if (enginePath == nullptr ||
        enginePath[0] == '\0')
    {
        initialized =
            false;

        return false;
    }


    initialized =
        true;

    return true;
}


bool TensorRtTasselDetector::isInitialized() const
{
    return initialized;
}


bool TensorRtTasselDetector::processFrame(
    std::size_t cameraIndex,
    const RgbFrameSource::Frame& frame,
    TasselDetector::Result& result)
{
    result =
        TasselDetector::Result {};


    if (!initialized ||
        !frame.valid ||
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

    result.valid =
        true;


    /*
     * Todavía no ejecutamos inferencia.
     *
     * Por ahora devolvemos una lista
     * de detecciones vacía.
     */
    return true;
}