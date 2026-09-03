#include "ai/tensorrt_tassel_detector.h"
#include "ai/tensorrt_compat.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>

#ifdef HAGIE_ENABLE_TENSORRT

#include <cuda_runtime.h>
#include <cuda_fp16.h>

namespace
{

void describeTensorDataType(
    nvinfer1::DataType type,
    std::string& name,
    std::size_t& elementSizeBytes)
{
    switch (type)
    {
        case nvinfer1::DataType::kFLOAT:

            name =
                "FP32";

            elementSizeBytes =
                4;

            break;


        case nvinfer1::DataType::kHALF:

            name =
                "FP16";

            elementSizeBytes =
                2;

            break;


        case nvinfer1::DataType::kINT8:

            name =
                "INT8";

            elementSizeBytes =
                1;

            break;


        case nvinfer1::DataType::kINT32:

            name =
                "INT32";

            elementSizeBytes =
                4;

            break;


        case nvinfer1::DataType::kBOOL:

            name =
                "BOOL";

            elementSizeBytes =
                1;

            break;


        default:

            name =
                "UNKNOWN";

            elementSizeBytes =
                0;

            break;
    }
}

} // namespace

#endif


TensorRtTasselDetector::TensorRtTasselDetector()
{
}

bool TensorRtTasselDetector::preprocessRgbFrame(
    const RgbFrameSource::Frame& frame,
    std::vector<float>& output,
    PreprocessInfo& info) const
{
    output.clear();

    info =
        PreprocessInfo {};


    /*
     * ========================================================
     * VALIDAR FRAME RGB888
     * ========================================================
     */

    if (!frame.valid ||
        frame.width == 0 ||
        frame.height == 0 ||
        inputWidth <= 0 ||
        inputHeight <= 0 ||
        inputChannels != 3)
    {
        return false;
    }


    const std::size_t expectedBytes =
        frame.width *
        frame.height *
        3;


    if (frame.data.size() != expectedBytes)
    {
        return false;
    }


    /*
     * ========================================================
     * CALCULAR LETTERBOX
     * ========================================================
     *
     * Conservamos relación de aspecto.
     */

    const float scaleX =
        static_cast<float>(inputWidth) /
        static_cast<float>(frame.width);

    const float scaleY =
        static_cast<float>(inputHeight) /
        static_cast<float>(frame.height);


    const float scale =
        std::min(
            scaleX,
            scaleY
        );


    int resizedWidth =
        static_cast<int>(
            std::round(
                static_cast<float>(frame.width) *
                scale
            )
        );


    int resizedHeight =
        static_cast<int>(
            std::round(
                static_cast<float>(frame.height) *
                scale
            )
        );


    resizedWidth =
        std::clamp(
            resizedWidth,
            1,
            inputWidth
        );


    resizedHeight =
        std::clamp(
            resizedHeight,
            1,
            inputHeight
        );


    const int padX =
        (inputWidth - resizedWidth) /
        2;

    const int padY =
        (inputHeight - resizedHeight) /
        2;


    info.originalWidth =
        frame.width;

    info.originalHeight =
        frame.height;

    info.networkWidth =
        inputWidth;

    info.networkHeight =
        inputHeight;

    info.resizedWidth =
        resizedWidth;

    info.resizedHeight =
        resizedHeight;

    info.scale =
        scale;

    info.padX =
        padX;

    info.padY =
        padY;


    /*
     * ========================================================
     * CREAR TENSOR NCHW
     * ========================================================
     *
     * Layout:
     *
     * RRRRR...
     * GGGGG...
     * BBBBB...
     *
     * Normalizado a 0.0 - 1.0.
     *
     * El padding utiliza 114, valor habitual
     * en modelos YOLO.
     */

    const std::size_t planeSize =
        static_cast<std::size_t>(inputWidth) *
        static_cast<std::size_t>(inputHeight);


    output.assign(
        planeSize * 3,
        114.0f / 255.0f
    );


    /*
     * ========================================================
     * RESIZE BILINEAL
     * ========================================================
     */

    for (int destinationY = 0;
         destinationY < resizedHeight;
         ++destinationY)
    {
        /*
         * Transformación al centro del pixel.
         */
        const float sourceY =
            (
                static_cast<float>(destinationY) +
                0.5f
            ) /
            scale -
            0.5f;


        int y0 =
            static_cast<int>(
                std::floor(sourceY)
            );

        int y1 =
            y0 + 1;


        const float fy =
            sourceY -
            static_cast<float>(y0);


        y0 =
            std::clamp(
                y0,
                0,
                static_cast<int>(frame.height) - 1
            );

        y1 =
            std::clamp(
                y1,
                0,
                static_cast<int>(frame.height) - 1
            );


        for (int destinationX = 0;
             destinationX < resizedWidth;
             ++destinationX)
        {
            const float sourceX =
                (
                    static_cast<float>(destinationX) +
                    0.5f
                ) /
                scale -
                0.5f;


            int x0 =
                static_cast<int>(
                    std::floor(sourceX)
                );

            int x1 =
                x0 + 1;


            const float fx =
                sourceX -
                static_cast<float>(x0);


            x0 =
                std::clamp(
                    x0,
                    0,
                    static_cast<int>(frame.width) - 1
                );

            x1 =
                std::clamp(
                    x1,
                    0,
                    static_cast<int>(frame.width) - 1
                );


            const std::size_t pixel00 =
                (
                    static_cast<std::size_t>(y0) *
                    frame.width +
                    static_cast<std::size_t>(x0)
                ) * 3;


            const std::size_t pixel01 =
                (
                    static_cast<std::size_t>(y0) *
                    frame.width +
                    static_cast<std::size_t>(x1)
                ) * 3;


            const std::size_t pixel10 =
                (
                    static_cast<std::size_t>(y1) *
                    frame.width +
                    static_cast<std::size_t>(x0)
                ) * 3;


            const std::size_t pixel11 =
                (
                    static_cast<std::size_t>(y1) *
                    frame.width +
                    static_cast<std::size_t>(x1)
                ) * 3;


            const int networkX =
                destinationX +
                padX;

            const int networkY =
                destinationY +
                padY;


            const std::size_t destinationIndex =
                static_cast<std::size_t>(networkY) *
                static_cast<std::size_t>(inputWidth) +
                static_cast<std::size_t>(networkX);


            /*
             * RGB -> NCHW.
             */
            for (std::size_t channel = 0;
                 channel < 3;
                 ++channel)
            {
                const float value00 =
                    static_cast<float>(
                        frame.data[
                            pixel00 +
                            channel
                        ]
                    );

                const float value01 =
                    static_cast<float>(
                        frame.data[
                            pixel01 +
                            channel
                        ]
                    );

                const float value10 =
                    static_cast<float>(
                        frame.data[
                            pixel10 +
                            channel
                        ]
                    );

                const float value11 =
                    static_cast<float>(
                        frame.data[
                            pixel11 +
                            channel
                        ]
                    );


                const float top =
                    value00 +
                    (value01 - value00) *
                    fx;


                const float bottom =
                    value10 +
                    (value11 - value10) *
                    fx;


                const float value =
                    top +
                    (bottom - top) *
                    fy;


                output[
                    channel *
                    planeSize +
                    destinationIndex
                ] =
                    value /
                    255.0f;
            }
        }
    }


    return true;
}

float TensorRtTasselDetector::intersectionOverUnion(
    const RawDetection& a,
    const RawDetection& b)
{
    const float intersectionX1 =
        std::max(a.x1, b.x1);

    const float intersectionY1 =
        std::max(a.y1, b.y1);

    const float intersectionX2 =
        std::min(a.x2, b.x2);

    const float intersectionY2 =
        std::min(a.y2, b.y2);


    const float intersectionWidth =
        std::max(
            0.0f,
            intersectionX2 - intersectionX1
        );

    const float intersectionHeight =
        std::max(
            0.0f,
            intersectionY2 - intersectionY1
        );


    const float intersectionArea =
        intersectionWidth *
        intersectionHeight;


    const float areaA =
        std::max(0.0f, a.x2 - a.x1) *
        std::max(0.0f, a.y2 - a.y1);

    const float areaB =
        std::max(0.0f, b.x2 - b.x1) *
        std::max(0.0f, b.y2 - b.y1);


    const float unionArea =
        areaA +
        areaB -
        intersectionArea;


    if (unionArea <= 0.0f)
    {
        return 0.0f;
    }


    return
        intersectionArea /
        unionArea;
}


void TensorRtTasselDetector::nonMaximumSuppression(
    std::vector<RawDetection>& detections,
    float iouThreshold)
{
    std::sort(
        detections.begin(),
        detections.end(),
        [](
            const RawDetection& a,
            const RawDetection& b)
        {
            return
                a.confidence >
                b.confidence;
        }
    );


    std::vector<RawDetection> filtered;

    filtered.reserve(
        detections.size()
    );


    for (const RawDetection& candidate :
         detections)
    {
        bool suppress =
            false;


        for (const RawDetection& accepted :
             filtered)
        {
            /*
             * No suprimir detecciones de clases diferentes.
             */
            if (candidate.classId !=
                accepted.classId)
            {
                continue;
            }


            if (intersectionOverUnion(
                    candidate,
                    accepted
                ) >
                iouThreshold)
            {
                suppress =
                    true;

                break;
            }
        }


        if (!suppress)
        {
            filtered.push_back(
                candidate
            );
        }
    }


    detections =
        std::move(filtered);
}


bool TensorRtTasselDetector::decodeOutputs(
    const PreprocessInfo& preprocessInfo,
    std::vector<RawDetection>& detections) const
{
    detections.clear();


#ifndef HAGIE_ENABLE_TENSORRT

    (void)preprocessInfo;

    return false;

#else

    if (outputTensorIndices.empty())
    {
        return false;
    }


    /*
     * Por ahora procesamos solamente el caso
     * de un único tensor de salida RAW.
     *
     * Más adelante agregaremos los engines
     * con NMS integrado y múltiples outputs.
     */
    if (outputTensorIndices.size() != 1)
    {
        std::cerr
            << "[TensorRT] Multiple output tensors detected. "
            << "Decoder not implemented yet."
            << std::endl;

        return false;
    }


    const std::size_t outputIndex =
        outputTensorIndices[0];


    if (outputIndex >= tensors.size() ||
        outputIndex >= hostOutputBuffers.size())
    {
        return false;
    }


    const TensorInfo& output =
        tensors[outputIndex];

    const std::vector<unsigned char>& hostBuffer =
        hostOutputBuffers[outputIndex];


    /*
     * Esperamos inicialmente:
     *
     * [1, attributes, detections]
     *
     * o
     *
     * [1, detections, attributes]
     */
    if (output.dimensions.size() != 3 ||
        output.dimensions[0] != 1)
    {
        std::cerr
            << "[TensorRT] Unsupported raw output dimensions"
            << std::endl;

        return false;
    }


    const int dimension1 =
        output.dimensions[1];

    const int dimension2 =
        output.dimensions[2];


    if (dimension1 <= 0 ||
        dimension2 <= 0)
    {
        return false;
    }


    bool attributesFirst =
        false;

    int attributeCount =
        0;

    int detectionCount =
        0;


    /*
     * Ejemplo moderno:
     *
     * [1, 5, 8400]
     *
     * frente a:
     *
     * [1, 8400, 5]
     */
    if (dimension1 < dimension2)
    {
        attributesFirst =
            true;

        attributeCount =
            dimension1;

        detectionCount =
            dimension2;
    }
    else
    {
        attributesFirst =
            false;

        detectionCount =
            dimension1;

        attributeCount =
            dimension2;
    }


    std::cout
        << "[TensorRT] Raw YOLO output: detections="
        << detectionCount
        << " attributes="
        << attributeCount
        << " layout="
        << (attributesFirst
                ? "1xAxN"
                : "1xNxA")
        << std::endl;


        /*
     * ========================================================
     * DETERMINAR FORMATO DE SALIDA
     * ========================================================
     *
     * Detector de una sola clase:
     *
     * ModernRaw:
     *   cx, cy, w, h, classScore
     *   => 5 atributos
     *
     * YoloV5Raw:
     *   cx, cy, w, h, objectness, classScore
     *   => 6 atributos
     *
     * Con Auto solamente aceptamos los casos
     * que podemos identificar sin ambigüedad.
     */

    ModelOutputFormat activeFormat =
        outputFormat;


    if (activeFormat ==
        ModelOutputFormat::Auto)
    {
        if (attributeCount == 5)
        {
            activeFormat =
                ModelOutputFormat::ModernRaw;
        }
        else if (attributeCount == 6)
        {
            std::cerr
                << "[TensorRT] AUTO cannot safely determine "
                << "the meaning of a 6-attribute output. "
                << "Select YoloV5Raw or EndToEndNms explicitly."
                << std::endl;

            return false;
        }
        else
        {
            std::cerr
                << "[TensorRT] AUTO does not recognize output with "
                << attributeCount
                << " attributes"
                << std::endl;

            return false;
        }
    }


    if (activeFormat ==
        ModelOutputFormat::ModernRaw)
    {
        if (attributeCount != 5)
        {
            std::cerr
                << "[TensorRT] ModernRaw expects 5 attributes, found "
                << attributeCount
                << std::endl;

            return false;
        }
    }
    else if (activeFormat ==
             ModelOutputFormat::YoloV5Raw)
    {
        if (attributeCount != 6)
        {
            std::cerr
                << "[TensorRT] YoloV5Raw expects 6 attributes, found "
                << attributeCount
                << std::endl;

            return false;
        }
    }
    else if (activeFormat ==
             ModelOutputFormat::EndToEndNms)
    {
        std::cerr
            << "[TensorRT] EndToEndNms decoder not implemented yet"
            << std::endl;

        return false;
    }

    /*
     * ========================================================
     * LECTOR GENERICO FP32 / FP16
     * ========================================================
     */

    auto readValue =
        [&output, &hostBuffer](
            std::size_t elementIndex,
            float& value) -> bool
    {
        if (elementIndex >=
            output.elementCount)
        {
            return false;
        }


        if (output.dataType == "FP32")
        {
            const std::size_t byteOffset =
                elementIndex *
                sizeof(float);


            if (byteOffset + sizeof(float) >
                hostBuffer.size())
            {
                return false;
            }


            float temporary =
                0.0f;


            std::memcpy(
                &temporary,
                hostBuffer.data() + byteOffset,
                sizeof(float)
            );


            value =
                temporary;

            return true;
        }


        if (output.dataType == "FP16")
        {
            const std::size_t byteOffset =
                elementIndex *
                sizeof(__half);


            if (byteOffset + sizeof(__half) >
                hostBuffer.size())
            {
                return false;
            }


            __half temporary;


            std::memcpy(
                &temporary,
                hostBuffer.data() + byteOffset,
                sizeof(__half)
            );


            value =
                __half2float(
                    temporary
                );

            return true;
        }


        return false;
    };


    auto tensorValue =
        [&](
            int detectionIndex,
            int attributeIndex,
            float& value) -> bool
    {
        std::size_t elementIndex =
            0;


        if (attributesFirst)
        {
            elementIndex =
                static_cast<std::size_t>(
                    attributeIndex
                ) *
                static_cast<std::size_t>(
                    detectionCount
                ) +
                static_cast<std::size_t>(
                    detectionIndex
                );
        }
        else
        {
            elementIndex =
                static_cast<std::size_t>(
                    detectionIndex
                ) *
                static_cast<std::size_t>(
                    attributeCount
                ) +
                static_cast<std::size_t>(
                    attributeIndex
                );
        }


        return readValue(
            elementIndex,
            value
        );
    };


    /*
     * ========================================================
     * DECODIFICAR CAJAS
     * ========================================================
     */

    constexpr float confidenceThreshold =
        0.25f;


    for (int detectionIndex = 0;
         detectionIndex < detectionCount;
         ++detectionIndex)
    {
                float centerX =
            0.0f;

        float centerY =
            0.0f;

        float width =
            0.0f;

        float height =
            0.0f;

        float confidence =
            0.0f;

        float objectness =
            1.0f;

        float classScore =
            0.0f;


                if (!tensorValue(
                detectionIndex,
                0,
                centerX
            ) ||
            !tensorValue(
                detectionIndex,
                1,
                centerY
            ) ||
            !tensorValue(
                detectionIndex,
                2,
                width
            ) ||
            !tensorValue(
                detectionIndex,
                3,
                height
            ))
        {
            return false;
        }


        if (activeFormat ==
            ModelOutputFormat::ModernRaw)
        {
            if (!tensorValue(
                    detectionIndex,
                    4,
                    classScore
                ))
            {
                return false;
            }


            confidence =
                classScore;
        }
        else if (activeFormat ==
                 ModelOutputFormat::YoloV5Raw)
        {
            if (!tensorValue(
                    detectionIndex,
                    4,
                    objectness
                ) ||
                !tensorValue(
                    detectionIndex,
                    5,
                    classScore
                ))
            {
                return false;
            }


            confidence =
                objectness *
                classScore;
        }


        if (confidence <
            confidenceThreshold)
        {
            continue;
        }


        float x1 =
            centerX -
            width * 0.5f;

        float y1 =
            centerY -
            height * 0.5f;

        float x2 =
            centerX +
            width * 0.5f;

        float y2 =
            centerY +
            height * 0.5f;


        /*
         * Sacar el letterbox y regresar
         * a coordenadas de la imagen original.
         */

        x1 =
            (x1 - preprocessInfo.padX) /
            preprocessInfo.scale;

        y1 =
            (y1 - preprocessInfo.padY) /
            preprocessInfo.scale;

        x2 =
            (x2 - preprocessInfo.padX) /
            preprocessInfo.scale;

        y2 =
            (y2 - preprocessInfo.padY) /
            preprocessInfo.scale;


        x1 =
            std::clamp(
                x1,
                0.0f,
                static_cast<float>(
                    preprocessInfo.originalWidth
                )
            );

        y1 =
            std::clamp(
                y1,
                0.0f,
                static_cast<float>(
                    preprocessInfo.originalHeight
                )
            );

        x2 =
            std::clamp(
                x2,
                0.0f,
                static_cast<float>(
                    preprocessInfo.originalWidth
                )
            );

        y2 =
            std::clamp(
                y2,
                0.0f,
                static_cast<float>(
                    preprocessInfo.originalHeight
                )
            );


        if (x2 <= x1 ||
            y2 <= y1)
        {
            continue;
        }


        RawDetection detection;

        detection.x1 =
            x1;

        detection.y1 =
            y1;

        detection.x2 =
            x2;

        detection.y2 =
            y2;

        detection.confidence =
            confidence;

        detection.classId =
            0;


        detections.push_back(
            detection
        );
    }


    nonMaximumSuppression(
        detections,
        0.45f
    );


    return true;

#endif
}

TensorRtTasselDetector::~TensorRtTasselDetector()

{
#ifdef HAGIE_ENABLE_TENSORRT

    releaseTensorRtResources();

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

bool TensorRtTasselDetector::validateEngineLayout() const
{
    /*
     * ========================================================
     * VALIDAR ESTRUCTURA GENERAL DEL ENGINE
     * ========================================================
     *
     * No validamos una versión concreta de YOLO.
     *
     * Solamente exigimos una estructura mínima
     * que Hagie pueda usar.
     */

    if (tensors.empty())
    {
        std::cerr
            << "[TensorRT] Engine has no tensors"
            << std::endl;

        return false;
    }


    /*
     * ========================================================
     * VALIDAR INPUT
     * ========================================================
     */

    if (inputTensorIndex < 0 ||
        static_cast<std::size_t>(
            inputTensorIndex
        ) >= tensors.size())
    {
        std::cerr
            << "[TensorRT] No valid input tensor"
            << std::endl;

        return false;
    }


    std::size_t inputCount =
        0;


    for (const TensorInfo& tensor :
         tensors)
    {
        if (tensor.isInput)
        {
            ++inputCount;
        }
    }


    /*
     * Para el detector de panojas esperamos
     * una única entrada de imagen.
     */
    if (inputCount != 1)
    {
        std::cerr
            << "[TensorRT] Expected exactly one input tensor, found "
            << inputCount
            << std::endl;

        return false;
    }


    const TensorInfo& input =
        tensors[
            static_cast<std::size_t>(
                inputTensorIndex
            )
        ];


    /*
     * Normalmente YOLO utiliza:
     *
     * N x C x H x W
     *
     * Permitimos dimensiones dinámicas (-1),
     * pero exigimos cuatro dimensiones.
     */
    if (input.dimensions.size() != 4)
    {
        std::cerr
            << "[TensorRT] Input tensor must have 4 dimensions"
            << std::endl;

        return false;
    }


    /*
     * Tipos que vamos a soportar inicialmente
     * como entrada del detector.
     */
    if (input.dataType != "FP32" &&
        input.dataType != "FP16")
    {
        std::cerr
            << "[TensorRT] Unsupported input type: "
            << input.dataType
            << std::endl;

        return false;
    }


    if (input.elementSizeBytes == 0)
    {
        std::cerr
            << "[TensorRT] Invalid input element size"
            << std::endl;

        return false;
    }


    /*
     * ========================================================
     * VALIDAR OUTPUTS
     * ========================================================
     */

    if (outputTensorIndices.empty())
    {
        std::cerr
            << "[TensorRT] Engine has no output tensors"
            << std::endl;

        return false;
    }


    for (const std::size_t outputIndex :
         outputTensorIndices)
    {
        if (outputIndex >=
            tensors.size())
        {
            std::cerr
                << "[TensorRT] Invalid output tensor index"
                << std::endl;

            return false;
        }


        const TensorInfo& output =
            tensors[
                outputIndex
            ];


        if (output.isInput)
        {
            std::cerr
                << "[TensorRT] Output tensor incorrectly marked as input"
                << std::endl;

            return false;
        }


        if (output.dimensions.empty())
        {
            std::cerr
                << "[TensorRT] Output tensor has no dimensions: "
                << output.name
                << std::endl;

            return false;
        }


        if (output.elementSizeBytes == 0)
        {
            std::cerr
                << "[TensorRT] Unsupported output type: "
                << output.dataType
                << std::endl;

            return false;
        }
    }


    std::cout
        << "[TensorRT] Engine layout validated"
        << std::endl;


    return true;
}

bool TensorRtTasselDetector::resolveInputDimensions()
{
    /*
     * ========================================================
     * RESOLVER DIMENSIONES DE ENTRADA
     * ========================================================
     *
     * Esperamos formato NCHW:
     *
     * N x C x H x W
     *
     * Ejemplo:
     *
     * 1 x 3 x 640 x 640
     *
     * También permitimos engines con dimensiones dinámicas.
     */

    if (inputTensorIndex < 0 ||
        static_cast<std::size_t>(
            inputTensorIndex
        ) >= tensors.size())
    {
        return false;
    }


    TensorInfo& input =
        tensors[
            static_cast<std::size_t>(
                inputTensorIndex
            )
        ];


    if (input.dimensions.size() != 4)
    {
        return false;
    }


    int batch =
        input.dimensions[0];

    int channels =
        input.dimensions[1];

    int height =
        input.dimensions[2];

    int width =
        input.dimensions[3];


    /*
     * ========================================================
     * DIMENSIONES DINAMICAS
     * ========================================================
     *
     * Si el engine permite dimensiones dinámicas,
     * utilizamos inicialmente 640 x 640.
     *
     * Esto NO ata Hagie a YOLO26.
     *
     * Más adelante este tamaño podrá venir de
     * configuración.
     */

    if (batch <= 0)
    {
        batch =
            1;
    }


    if (channels <= 0)
    {
        channels =
            3;
    }


    if (height <= 0)
    {
        height =
            640;
    }


    if (width <= 0)
    {
        width =
            640;
    }


    /*
     * Hagie procesa una imagen RGB por inferencia.
     */

    if (batch != 1)
    {
        std::cerr
            << "[TensorRT] Unsupported batch size: "
            << batch
            << std::endl;

        return false;
    }


    if (channels != 3)
    {
        std::cerr
            << "[TensorRT] Expected 3 input channels, found "
            << channels
            << std::endl;

        return false;
    }


    nvinfer1::Dims resolvedDims;

    resolvedDims.nbDims =
        4;

    resolvedDims.d[0] =
        batch;

    resolvedDims.d[1] =
        channels;

    resolvedDims.d[2] =
        height;

    resolvedDims.d[3] =
        width;


#if NV_TENSORRT_MAJOR >= 10

    if (!context->setInputShape(
            input.name.c_str(),
            resolvedDims
        ))
    {
        std::cerr
            << "[TensorRT] Cannot set input shape"
            << std::endl;

        return false;
    }

#else

    /*
     * TensorRT 8/9 utiliza el índice de binding.
     *
     * Buscamos el binding correspondiente por nombre
     * para no asumir que siempre sea el binding 0.
     */

    const int bindingIndex =
        engine->getBindingIndex(
            input.name.c_str()
        );


    if (bindingIndex < 0)
    {
        std::cerr
            << "[TensorRT] Cannot find input binding"
            << std::endl;

        return false;
    }


    if (!context->setBindingDimensions(
            bindingIndex,
            resolvedDims
        ))
    {
        std::cerr
            << "[TensorRT] Cannot set input dimensions"
            << std::endl;

        return false;
    }

#endif


    inputChannels =
        channels;

    inputHeight =
        height;

    inputWidth =
        width;


    std::cout
        << "[TensorRT] Input resolution: "
        << inputWidth
        << " x "
        << inputHeight
        << " x "
        << inputChannels
        << std::endl;


    return true;
}

bool TensorRtTasselDetector::resolveTensorSizes()
{
    /*
     * ========================================================
     * RESOLVER DIMENSIONES REALES Y TAMAÑOS
     * ========================================================
     *
     * Esto se hace DESPUES de configurar el tamaño
     * de entrada del contexto.
     *
     * Es importante para engines dinámicos porque las
     * dimensiones de salida pueden depender del input.
     */

    if (engine == nullptr ||
        context == nullptr)
    {
        return false;
    }


    for (std::size_t tensorIndex = 0;
         tensorIndex < tensors.size();
         ++tensorIndex)
    {
        TensorInfo& tensor =
            tensors[tensorIndex];


        nvinfer1::Dims dims;


#if NV_TENSORRT_MAJOR >= 10

        dims =
            context->getTensorShape(
                tensor.name.c_str()
            );

#else

        const int bindingIndex =
            engine->getBindingIndex(
                tensor.name.c_str()
            );


        if (bindingIndex < 0)
        {
            std::cerr
                << "[TensorRT] Cannot find binding: "
                << tensor.name
                << std::endl;

            return false;
        }


        dims =
            context->getBindingDimensions(
                bindingIndex
            );

#endif


        if (dims.nbDims <= 0)
        {
            std::cerr
                << "[TensorRT] Invalid dimensions for tensor: "
                << tensor.name
                << std::endl;

            return false;
        }


        tensor.dimensions.clear();


        std::size_t elementCount =
            1;


        for (int dimension = 0;
             dimension < dims.nbDims;
             ++dimension)
        {
            const int value =
                dims.d[dimension];


            /*
             * Si sigue existiendo una dimensión dinámica
             * después de configurar el contexto, todavía
             * no podemos reservar memoria de forma segura.
             */
            if (value <= 0)
            {
                std::cerr
                    << "[TensorRT] Unresolved dynamic dimension in tensor: "
                    << tensor.name
                    << std::endl;

                return false;
            }


            tensor.dimensions.push_back(
                value
            );


            const std::size_t dimensionSize =
                static_cast<std::size_t>(
                    value
                );


            /*
             * Protección contra overflow.
             */
            if (elementCount >
                static_cast<std::size_t>(-1) /
                dimensionSize)
            {
                std::cerr
                    << "[TensorRT] Tensor size overflow: "
                    << tensor.name
                    << std::endl;

                return false;
            }


            elementCount *=
                dimensionSize;
        }


        if (tensor.elementSizeBytes == 0)
        {
            std::cerr
                << "[TensorRT] Invalid element size: "
                << tensor.name
                << std::endl;

            return false;
        }


        if (elementCount >
            static_cast<std::size_t>(-1) /
            tensor.elementSizeBytes)
        {
            std::cerr
                << "[TensorRT] Tensor byte size overflow: "
                << tensor.name
                << std::endl;

            return false;
        }


        tensor.elementCount =
            elementCount;


        tensor.byteSize =
            elementCount *
            tensor.elementSizeBytes;


        std::cout
            << "[TensorRT] "
            << (tensor.isInput
                    ? "INPUT  "
                    : "OUTPUT ")
            << tensor.name
            << " [";


        for (std::size_t dimension = 0;
             dimension < tensor.dimensions.size();
             ++dimension)
        {
            if (dimension > 0)
            {
                std::cout
                    << " x ";
            }


            std::cout
                << tensor.dimensions[dimension];
        }


        std::cout
            << "] "
            << tensor.dataType
            << " elements="
            << tensor.elementCount
            << " bytes="
            << tensor.byteSize
            << std::endl;
    }


    return true;
}

bool TensorRtTasselDetector::allocateCudaBuffers()
{
    releaseCudaBuffers();

    tensorBuffers.clear();
    tensorBuffers.resize(
        tensors.size()
    );
    hostOutputBuffers.clear();

    hostOutputBuffers.resize(
        tensors.size()
    );


    for (std::size_t tensorIndex = 0;
         tensorIndex < tensors.size();
         ++tensorIndex)
    {
        const TensorInfo& tensor =
            tensors[tensorIndex];

        TensorBuffer& buffer =
            tensorBuffers[tensorIndex];


        if (tensor.byteSize == 0)
        {
            std::cerr
                << "[TensorRT] Invalid tensor size for CUDA allocation: "
                << tensor.name
                << std::endl;

            releaseCudaBuffers();

            return false;
        }


        void* devicePtr =
            nullptr;


        const cudaError_t error =
            cudaMalloc(
                &devicePtr,
                tensor.byteSize
            );


        if (error != cudaSuccess)
        {
            std::cerr
                << "[TensorRT] cudaMalloc failed for tensor "
                << tensor.name
                << ": "
                << cudaGetErrorString(error)
                << std::endl;

            releaseCudaBuffers();

            return false;
        }


        buffer.devicePtr =
            devicePtr;

        buffer.byteSize =
            tensor.byteSize;

        if (!tensor.isInput)
        {
            hostOutputBuffers[tensorIndex].resize(
                tensor.byteSize
            );
        }    


        std::cout
            << "[TensorRT] CUDA buffer allocated: "
            << tensor.name
            << " bytes="
            << buffer.byteSize
            << std::endl;
    }


    return true;
}


void TensorRtTasselDetector::releaseCudaBuffers()
{
    for (TensorBuffer& buffer :
         tensorBuffers)
    {
        if (buffer.devicePtr != nullptr)
        {
            cudaFree(
                buffer.devicePtr
            );

            buffer.devicePtr =
                nullptr;
        }


        buffer.byteSize =
            0;
    }


    tensorBuffers.clear();

    hostOutputBuffers.clear();
}

void TensorRtTasselDetector::releaseTensorRtResources()
{
    releaseCudaBuffers();


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


    tensors.clear();

    outputTensorIndices.clear();

    inputTensorIndex =
        -1;

    inputWidth =
        0;

    inputHeight =
        0;

    inputChannels =
        0;

    initialized =
        false;
}
#endif


bool TensorRtTasselDetector::initialize(
    const char* enginePath,
    ModelOutputFormat requestedOutputFormat)
    {

    outputFormat =
        requestedOutputFormat;
    #ifdef HAGIE_ENABLE_TENSORRT

        releaseTensorRtResources();

    #else

        initialized =
            false;

    #endif


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

    /*
    * ========================================================
    * INSPECCIONAR TENSORES DEL ENGINE
    * ========================================================
    *
    * No suponemos nombres, dimensiones ni formato
    * específico de una versión de YOLO.
    *
    * TensorRT 8/9 utiliza bindings.
    * TensorRT 10+ utiliza I/O tensors.
    */

    tensors.clear();

    inputTensorIndex =
        -1;

    outputTensorIndices.clear();


    #if NV_TENSORRT_MAJOR >= 10

        const int tensorCount =
            engine->getNbIOTensors();


        for (int index = 0;
            index < tensorCount;
            ++index)
        {
            const char* tensorName =
                engine->getIOTensorName(
                    index
                );


            if (tensorName == nullptr)
            {
                continue;
            }


            TensorInfo info;

            info.name =
                tensorName;


            info.isInput =
                engine->getTensorIOMode(
                    tensorName
                ) ==
                nvinfer1::TensorIOMode::kINPUT;



            describeTensorDataType(
                engine->getTensorDataType(
                    tensorName
                ),
                info.dataType,
                info.elementSizeBytes
            );


            const nvinfer1::Dims dims =
                engine->getTensorShape(
                    tensorName
                );


            for (int dimension = 0;
                dimension < dims.nbDims;
                ++dimension)
            {
                info.dimensions.push_back(
                    dims.d[dimension]
                );
            }

            const std::size_t tensorIndex =
                tensors.size();


            if (info.isInput)
            {
                if (inputTensorIndex == -1)
                {
                    inputTensorIndex =
                        static_cast<int>(
                            tensorIndex
                        );
                }
            }
            else
            {
                outputTensorIndices.push_back(
                    tensorIndex
                );
            }
            
            tensors.push_back(
                std::move(info)
            );
        }

    #else

        const int tensorCount =
            engine->getNbBindings();


        for (int index = 0;
            index < tensorCount;
            ++index)
        {
            const char* tensorName =
                engine->getBindingName(
                    index
                );


            if (tensorName == nullptr)
            {
                continue;
            }


            TensorInfo info;

            info.name =
                tensorName;


            info.isInput =
                engine->bindingIsInput(
                    index
                );


            describeTensorDataType(
                engine->getBindingDataType(
                    index
                ),
                info.dataType,
                info.elementSizeBytes
            );


            const nvinfer1::Dims dims =
                engine->getBindingDimensions(
                    index
                );


            for (int dimension = 0;
                dimension < dims.nbDims;
                ++dimension)
            {
                info.dimensions.push_back(
                    dims.d[dimension]
                );
            }


            const std::size_t tensorIndex =
                tensors.size();


            if (info.isInput)
            {
                /*
                * YOLO normalmente tiene una sola entrada
                * de imagen.
                */
                if (inputTensorIndex == -1)
                {
                    inputTensorIndex =
                        static_cast<int>(
                            tensorIndex
                        );
                }
            }
            else
            {
                outputTensorIndices.push_back(
                    tensorIndex
                );
            }


            tensors.push_back(
                std::move(info)
            );
        }

    #endif


    /*
    * Mostrar en consola qué engine fue cargado.
    */

    std::cout
        << "[TensorRT] Engine tensors:"
        << std::endl;


    for (const TensorInfo& tensor :
        tensors)
    {
        std::cout
            << "[TensorRT] "
            << (tensor.isInput
                    ? "INPUT  "
                    : "OUTPUT ")
            << tensor.name
            << " "
            << tensor.dataType
            << " [";


        for (std::size_t dimension = 0;
            dimension < tensor.dimensions.size();
            ++dimension)
        {
            if (dimension > 0)
            {
                std::cout
                    << " x ";
            }

            std::cout
                << tensor.dimensions[dimension];
        }


        std::cout
            << "]"
            << std::endl;
    }


    if (!validateEngineLayout())
    {
        std::cerr
            << "[TensorRT] Engine layout validation failed"
            << std::endl;

        return false;
    }

    if (!resolveInputDimensions())
    {
        std::cerr
            << "[TensorRT] Cannot resolve input dimensions"
            << std::endl;

        return false;
    }

    if (!resolveTensorSizes())
    {
        std::cerr
            << "[TensorRT] Cannot resolve tensor sizes"
            << std::endl;

        return false;
    }

    if (!allocateCudaBuffers())
    {
        std::cerr
            << "[TensorRT] Cannot allocate CUDA buffers"
            << std::endl;

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

    std::vector<float> inputData;

    PreprocessInfo preprocessInfo;


    if (!preprocessRgbFrame(
            frame,
            inputData,
            preprocessInfo
        ))
    {
        return false;
    }

    #ifdef HAGIE_ENABLE_TENSORRT

    if (inputTensorIndex < 0 ||
        static_cast<std::size_t>(inputTensorIndex) >=
            tensorBuffers.size())
    {
        return false;
    }


    const TensorInfo& inputTensor =
        tensors[
            static_cast<std::size_t>(
                inputTensorIndex
            )
        ];


    std::vector<__half> inputDataFp16;

    const void* hostInputPtr =
        nullptr;

    std::size_t inputBytes =
        0;


    if (inputTensor.dataType == "FP32")
    {
        hostInputPtr =
            inputData.data();

        inputBytes =
            inputData.size() *
            sizeof(float);
    }
    else if (inputTensor.dataType == "FP16")
    {
        inputDataFp16.resize(
            inputData.size()
        );


        for (std::size_t index = 0;
            index < inputData.size();
            ++index)
        {
            inputDataFp16[index] =
                __float2half(
                    inputData[index]
                );
        }


        hostInputPtr =
            inputDataFp16.data();

        inputBytes =
            inputDataFp16.size() *
            sizeof(__half);
    }
    else
    {
        std::cerr
            << "[TensorRT] Unsupported input type: "
            << inputTensor.dataType
            << std::endl;

        return false;
    }


    TensorBuffer& inputBuffer =
        tensorBuffers[
            static_cast<std::size_t>(
                inputTensorIndex
            )
        ];


    


    if (inputBytes != inputBuffer.byteSize)
    {
        std::cerr
            << "[TensorRT] Input byte size mismatch. Host="
            << inputBytes
            << " Device="
            << inputBuffer.byteSize
            << std::endl;

        return false;
    }


    const cudaError_t copyError =
        cudaMemcpy(
            inputBuffer.devicePtr,
            hostInputPtr,
            inputBytes,
            cudaMemcpyHostToDevice
        );


    if (copyError != cudaSuccess)
    {
        std::cerr
            << "[TensorRT] cudaMemcpy H2D failed: "
            << cudaGetErrorString(copyError)
            << std::endl;

        return false;
    }

    #if NV_TENSORRT_MAJOR >= 10

    /*
     * ========================================================
     * TENSORRT 10+
     * ========================================================
     *
     * Asociamos cada tensor con su buffer CUDA.
     */

    for (std::size_t tensorIndex = 0;
         tensorIndex < tensors.size();
         ++tensorIndex)
    {
        if (!context->setTensorAddress(
                tensors[tensorIndex].name.c_str(),
                tensorBuffers[tensorIndex].devicePtr
            ))
        {
            std::cerr
                << "[TensorRT] Cannot set tensor address: "
                << tensors[tensorIndex].name
                << std::endl;

            return false;
        }
    }


    cudaStream_t stream =
        nullptr;


    const cudaError_t streamError =
        cudaStreamCreate(
            &stream
        );


    if (streamError != cudaSuccess)
    {
        std::cerr
            << "[TensorRT] cudaStreamCreate failed: "
            << cudaGetErrorString(streamError)
            << std::endl;

        return false;
    }


    const bool inferenceOk =
        context->enqueueV3(
            stream
        );


    if (!inferenceOk)
    {
        std::cerr
            << "[TensorRT] enqueueV3 failed"
            << std::endl;

        cudaStreamDestroy(
            stream
        );

        return false;
    }


    const cudaError_t syncError =
        cudaStreamSynchronize(
            stream
        );


    cudaStreamDestroy(
        stream
    );


    if (syncError != cudaSuccess)
    {
        std::cerr
            << "[TensorRT] CUDA stream synchronization failed: "
            << cudaGetErrorString(syncError)
            << std::endl;

        return false;
    }

#else

    /*
     * ========================================================
     * TENSORRT 8 / 9
     * ========================================================
     *
     * executeV2 utiliza un array de bindings.
     */

    std::vector<void*> bindings(
        static_cast<std::size_t>(
            engine->getNbBindings()
        ),
        nullptr
    );


    for (std::size_t tensorIndex = 0;
         tensorIndex < tensors.size();
         ++tensorIndex)
    {
        const int bindingIndex =
            engine->getBindingIndex(
                tensors[tensorIndex].name.c_str()
            );


        if (bindingIndex < 0)
        {
            std::cerr
                << "[TensorRT] Cannot find binding: "
                << tensors[tensorIndex].name
                << std::endl;

            return false;
        }


        bindings[
            static_cast<std::size_t>(
                bindingIndex
            )
        ] =
            tensorBuffers[tensorIndex].devicePtr;
    }


    if (!context->executeV2(
            bindings.data()
        ))
    {
        std::cerr
            << "[TensorRT] executeV2 failed"
            << std::endl;

        return false;
    }

#endif
    /*
     * ========================================================
     * COPIAR OUTPUTS GPU -> CPU
     * ========================================================
     */

    for (const std::size_t outputIndex :
         outputTensorIndices)
    {
        if (outputIndex >= tensorBuffers.size() ||
            outputIndex >= hostOutputBuffers.size() ||
            outputIndex >= tensors.size())
        {
            return false;
        }


        const TensorBuffer& outputBuffer =
            tensorBuffers[outputIndex];

        std::vector<unsigned char>& hostBuffer =
            hostOutputBuffers[outputIndex];


        if (outputBuffer.devicePtr == nullptr ||
            outputBuffer.byteSize == 0 ||
            hostBuffer.size() != outputBuffer.byteSize)
        {
            std::cerr
                << "[TensorRT] Invalid output buffer: "
                << tensors[outputIndex].name
                << std::endl;

            return false;
        }


        const cudaError_t outputCopyError =
            cudaMemcpy(
                hostBuffer.data(),
                outputBuffer.devicePtr,
                outputBuffer.byteSize,
                cudaMemcpyDeviceToHost
            );


        if (outputCopyError != cudaSuccess)
        {
            std::cerr
                << "[TensorRT] cudaMemcpy D2H failed for output "
                << tensors[outputIndex].name
                << ": "
                << cudaGetErrorString(outputCopyError)
                << std::endl;

            return false;
        }
    }

        std::vector<RawDetection> rawDetections;


    if (!decodeOutputs(
            preprocessInfo,
            rawDetections
        ))
    {
        return false;
    }


    result.detections.clear();

    result.detections.reserve(
        rawDetections.size()
    );


    for (const RawDetection& raw :
         rawDetections)
    {
        TasselDetector::Detection detection;


        detection.confidence =
            raw.confidence;


        detection.x =
            static_cast<int>(
                std::round(
                    raw.x1
                )
            );


        detection.y =
            static_cast<int>(
                std::round(
                    raw.y1
                )
            );


        detection.width =
            static_cast<int>(
                std::round(
                    raw.x2 -
                    raw.x1
                )
            );


        detection.height =
            static_cast<int>(
                std::round(
                    raw.y2 -
                    raw.y1
                )
            );


        /*
         * body_index y posición 3D se completan
         * posteriormente en el pipeline Hagie.
         */

        detection.body_index =
            0;

        detection.position_3d_valid =
            false;


        result.detections.push_back(
            detection
        );
    }
#endif





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