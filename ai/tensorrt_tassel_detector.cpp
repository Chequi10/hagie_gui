#include "ai/tensorrt_tassel_detector.h"
#include "ai/tensorrt_compat.h"

#include <fstream>
#include <iostream>
#include <vector>


TensorRtTasselDetector::TensorRtTasselDetector()
{
}


TensorRtTasselDetector::~TensorRtTasselDetector()
{
#ifdef HAGIE_ENABLE_TENSORRT

    /*
     * El orden importa:
     *
     * context depende de engine,
     * engine depende de runtime.
     */

#if NV_TENSORRT_MAJOR < 10

    if (context != nullptr)
    {
        context->destroy();
        context =
            nullptr;
    }

    if (engine != nullptr)
    {
        engine->destroy();
        engine =
            nullptr;
    }

    if (runtime != nullptr)
    {
        runtime->destroy();
        runtime =
            nullptr;
    }

#else

    delete context;
    context =
        nullptr;

    delete engine;
    engine =
        nullptr;

    delete runtime;
    runtime =
        nullptr;

#endif

#endif
}


#ifdef HAGIE_ENABLE_TENSORRT

void TensorRtTasselDetector::Logger::log(
    Severity severity,
    const char* message) noexcept
{
    /*
     * No mostramos mensajes puramente informativos
     * para evitar llenar el log de la GUI.
     */
    if (severity <=
        Severity::kWARNING)
    {
        std::cerr
            << "[TensorRT] "
            << message
            << std::endl;
    }
}

#endif


bool TensorRtTasselDetector::initialize(
    const char* enginePath)
{
    initialized =
        false;


    if (enginePath == nullptr ||
        enginePath[0] == '\0')
    {
        return false;
    }


#ifndef HAGIE_ENABLE_TENSORRT

    /*
     * Esta compilación no tiene TensorRT.
     */
    std::cerr
        << "[TensorRT] Support disabled"
        << std::endl;

    return false;

#else

    /*
     * ========================================================
     * LEER ENGINE DESDE DISCO
     * ========================================================
     */

    std::ifstream file(
        enginePath,
        std::ios::binary |
        std::ios::ate
    );


    if (!file)
    {
        std::cerr
            << "[TensorRT] Cannot open engine: "
            << enginePath
            << std::endl;

        return false;
    }


    const std::streamsize fileSize =
        file.tellg();


    if (fileSize <= 0)
    {
        return false;
    }


    file.seekg(
        0,
        std::ios::beg
    );


    std::vector<char> engineData(
        static_cast<std::size_t>(
            fileSize
        )
    );


    if (!file.read(
            engineData.data(),
            fileSize
        ))
    {
        return false;
    }


    /*
     * ========================================================
     * CREAR RUNTIME
     * ========================================================
     */

    runtime =
        nvinfer1::createInferRuntime(
            logger
        );


    if (runtime == nullptr)
    {
        return false;
    }


    /*
     * ========================================================
     * DESERIALIZAR ENGINE
     * ========================================================
     */

    engine =
        runtime->deserializeCudaEngine(
            engineData.data(),
            engineData.size()
        );


    if (engine == nullptr)
    {
        return false;
    }


    /*
     * ========================================================
     * CREAR CONTEXTO DE EJECUCIÓN
     * ========================================================
     */

    context =
        engine->createExecutionContext();


    if (context == nullptr)
    {
        return false;
    }


    initialized =
        true;


    std::cout
        << "[TensorRT] Engine loaded successfully"
        << std::endl;


    return true;

#endif
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
     * El engine ya está cargado.
     *
     * Todavía falta:
     *
     * - conocer input/output del YOLO;
     * - reservar CUDA buffers;
     * - preprocesar RGB;
     * - ejecutar TensorRT;
     * - decodificar detecciones;
     * - NMS.
     */

    return true;
}