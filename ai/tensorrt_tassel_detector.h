#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "ai/tassel_detector.h"
#include "vision/rgb_frame_source.h"


#ifdef HAGIE_ENABLE_TENSORRT
#include <NvInfer.h>
#endif


class TensorRtTasselDetector
{
public:

    TensorRtTasselDetector();

    ~TensorRtTasselDetector();


    bool initialize(
        const char* enginePath
    );


    bool isInitialized() const;


    bool processFrame(
        std::size_t cameraIndex,
        const RgbFrameSource::Frame& frame,
        TasselDetector::Result& result
    );


private:

    struct TensorInfo
    {
        std::string name;

        bool isInput =
            false;

        std::vector<int> dimensions;

        std::string dataType;

        std::size_t elementSizeBytes =
            0;

        std::size_t elementCount =
            0;

        std::size_t byteSize =
            0;
    };

    struct PreprocessInfo
    {
        std::size_t originalWidth =
            0;

        std::size_t originalHeight =
            0;

        int networkWidth =
            0;

        int networkHeight =
            0;

        int resizedWidth =
            0;

        int resizedHeight =
            0;

        float scale =
            1.0f;

        int padX =
            0;

        int padY =
            0;
    };

    bool preprocessRgbFrame(
        const RgbFrameSource::Frame& frame,
        std::vector<float>& output,
        PreprocessInfo& info
    ) const;


    bool initialized =
        false;


    int inputTensorIndex =
        -1;


    int inputWidth =
        0;

    int inputHeight =
        0;

    int inputChannels =
        0;


    std::vector<std::size_t>
        outputTensorIndices;


#ifdef HAGIE_ENABLE_TENSORRT

    /*
     * Valida la estructura general del engine.
     *
     * No depende de una versión concreta de YOLO.
     */
    bool validateEngineLayout() const;

    /*
    * Resuelve las dimensiones reales de entrada
    * que utilizará el contexto TensorRT.
    */
    bool resolveInputDimensions();

    /*
    * Obtiene las dimensiones reales de todos los tensores
    * después de configurar el contexto y calcula
    * cantidad de elementos y bytes necesarios.
    */
    bool resolveTensorSizes();

    struct TensorBuffer
    {
        void* devicePtr =
            nullptr;

        std::size_t byteSize =
            0;
    };


    bool allocateCudaBuffers();

    void releaseCudaBuffers();

    void releaseTensorRtResources();


    class Logger :
        public nvinfer1::ILogger
    {
    public:

        void log(
            Severity severity,
            const char* message
        ) noexcept override;
    };


    Logger logger;


    nvinfer1::IRuntime* runtime =
        nullptr;


    nvinfer1::ICudaEngine* engine =
        nullptr;


    nvinfer1::IExecutionContext* context =
        nullptr;


    std::vector<TensorInfo> tensors;

    std::vector<TensorBuffer> tensorBuffers;

    std::vector<std::vector<unsigned char>>
    hostOutputBuffers;

#endif
};