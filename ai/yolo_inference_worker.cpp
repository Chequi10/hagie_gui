#include "ai/yolo_inference_worker.h"
#include "vision/vision_3d_worker.h"

#include <chrono>


YoloInferenceWorker::YoloInferenceWorker(
    RgbCameraWorker& rgbCameraWorker,
    Vision3DWorker *vision3DWorker)
    : rgbCameraWorker(
        rgbCameraWorker
    ),
    vision3DWorker(
        vision3DWorker
    )
{
}


YoloInferenceWorker::~YoloInferenceWorker()
{
    stop();
}


bool YoloInferenceWorker::initialize(
    const char* enginePath,
    TensorRtTasselDetector::ModelOutputFormat outputFormat,
    float confidenceThreshold,
    float nmsThreshold)
{
    detector.setConfidenceThreshold(
        confidenceThreshold
    );

    detector.setNmsThreshold(
        nmsThreshold
    );

    return detector.initialize(
        enginePath,
        outputFormat
    );
}


bool YoloInferenceWorker::start()
{
    if (running.load())
    {
        return true;
    }


    if (!detector.isInitialized())
    {
        return false;
    }


    running.store(
        true
    );


    workerThread =
        std::thread(
            &YoloInferenceWorker::workerLoop,
            this
        );


    return true;
}


void YoloInferenceWorker::stop()
{
    running.store(
        false
    );


    if (workerThread.joinable())
    {
        workerThread.join();
    }
}


bool YoloInferenceWorker::isRunning() const
{
    return running.load();
}


bool YoloInferenceWorker::getLatestResult(
    std::size_t cameraIndex,
    Result& result) const
{
    if (cameraIndex >= CAMERA_COUNT)
    {
        result =
            Result {};

        return false;
    }


    std::lock_guard<std::mutex> lock(
        resultMutex
    );


    if (!latestResults[cameraIndex]
        .detectionResult.valid)
    {    
        result =
            Result {};

        return false;
    }


    result =
        latestResults[cameraIndex];


    return true;
}


void YoloInferenceWorker::workerLoop()
{
    /*
     * ========================================================
     * SCHEDULER DE CÁMARAS
     * ========================================================
     *
     * Las cinco cámaras frontales tienen
     * mayor prioridad.
     *
     * Las cámaras traseras solamente se
     * utilizan para verificación.
     *
     * Secuencia:
     *
     * 0 1 2 3 4
     * 0 1 2 3 4
     * 5 6
     *
     * y repetir.
     */
    static constexpr std::array<
        std::size_t,
        12
    > schedule =
    {
        0, 1, 2, 3, 4,
        0, 1, 2, 3, 4,
        5, 6
    };


    std::size_t scheduleIndex =
        0;


    while (running.load())
    {
        const std::size_t cameraIndex =
            schedule[scheduleIndex];


        scheduleIndex =
            (scheduleIndex + 1) %
            schedule.size();


        RgbFrameSource::Frame frame;


        if (!rgbCameraWorker.getFrame(
                cameraIndex,
                frame
            ))
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    1
                )
            );

            continue;
        }


        /*
         * Nunca procesar dos veces exactamente
         * el mismo frame.
         */
        if (frame.timestamp_ms ==
            lastProcessedTimestamp[cameraIndex])
        {
            std::this_thread::yield();

            continue;
        }

        /*
        * ========================================================
        * CAPTURA DE NUBE 3D SINCRONIZADA
        * ========================================================
        *
        * Las cámaras frontales 0..4 necesitan conservar
        * exactamente la nube correspondiente al frame RGB
        * que será enviado a YOLO.
        *
        * La copia se realiza ANTES de la inferencia.
        * De esta manera, aunque Vision3DWorker continúe
        * adquiriendo nuevas nubes mientras TensorRT trabaja,
        * este resultado conserva la nube original.
        */
        Vision3DProcessor::PointCloud synchronizedCloud;

        std::uint64_t synchronizedCloudTimestampMs =
            0;

        bool synchronizedCloudValid =
            false;


        if (cameraIndex <
                Vision3DWorker::CAMERA_COUNT)
        {
            if (vision3DWorker == nullptr)
            {
                continue;
            }


            if (!vision3DWorker->getLatestPointCloud(
                    cameraIndex,
                    synchronizedCloud,
                    synchronizedCloudTimestampMs
                ))
            {
                continue;
            }


            /*
            * En las cámaras frontales reales exigimos
            * correspondencia exacta RGB <-> XYZ.
            *
            * Si el RGB acaba de publicarse pero el worker
            * todavía no actualizó el cache de nube, simplemente
            * se espera al próximo turno de esta cámara.
            */
            if (synchronizedCloudTimestampMs !=
                frame.timestamp_ms)
            {
                continue;
            }


            synchronizedCloudValid =
                true;
        }


        TasselDetector::Result result;


        if (!detector.processFrame(
                cameraIndex,
                frame,
                result
            ))
        {
            continue;
        }


        lastProcessedTimestamp[cameraIndex] =
            frame.timestamp_ms;


        {
            std::lock_guard<std::mutex> lock(
                resultMutex
            );


            Result cachedResult;

            cachedResult.detectionResult =
                std::move(
                    result
                );

            cachedResult.pointCloud =
                std::move(
                    synchronizedCloud
                );

            cachedResult.pointCloudTimestampMs =
                synchronizedCloudTimestampMs;

            cachedResult.pointCloudValid =
                synchronizedCloudValid;


            latestResults[cameraIndex] =
                std::move(
                    cachedResult
                );
        }
    }
}