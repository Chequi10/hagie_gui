#include "vision/vision_3d_worker.h"


Vision3DWorker::Vision3DWorker(
    HagieState *state,
    Vision3DProcessor *processor,
    VisionHeightSource *heightSource)
    :
    state(state),
    processor(processor),
    heightSource(heightSource)
{
}


Vision3DWorker::~Vision3DWorker()
{
    stop();
}


// ============================================================
// FUENTES DE NUBE DE PUNTOS
// ============================================================

bool Vision3DWorker::setPointCloudSource(
    std::size_t camera,
    std::unique_ptr<PointCloudSource> source)
{
    /*
     * No modificar las fuentes mientras
     * el hilo está trabajando.
     */
    if (running.load())
    {
        return false;
    }


    if (camera >= CAMERA_COUNT)
    {
        return false;
    }


    if (source == nullptr)
    {
        return false;
    }


    /*
     * Verificación adicional:
     * la fuente debe corresponder con
     * la cámara donde se instala.
     */
    if (source->getCameraIndex() != camera)
    {
        return false;
    }


    pointCloudSources[camera] =
        std::move(source);


    return true;
}


void Vision3DWorker::clearPointCloudSource(
    std::size_t camera)
{
    if (running.load())
    {
        return;
    }


    if (camera >= CAMERA_COUNT)
    {
        return;
    }


    pointCloudSources[camera].reset();
}


bool Vision3DWorker::hasPointCloudSource(
    std::size_t camera) const
{
    if (camera >= CAMERA_COUNT)
    {
        return false;
    }


    return
        pointCloudSources[camera] != nullptr;
}


// ============================================================
// CONTROL
// ============================================================

bool Vision3DWorker::start()
{
    /*
     * Ya iniciado.
     */
    if (running.load())
    {
        return true;
    }


    /*
     * Dependencias obligatorias.
     */
    if (state == nullptr ||
        processor == nullptr ||
        heightSource == nullptr)
    {
        return false;
    }


    /*
     * Debe existir al menos una fuente.
     */
    bool sourceAvailable =
        false;


    for (std::size_t camera = 0;
         camera < CAMERA_COUNT;
         ++camera)
    {
        if (pointCloudSources[camera] != nullptr)
        {
            sourceAvailable =
                true;

            break;
        }
    }


    if (!sourceAvailable)
    {
        return false;
    }


    /*
     * Iniciar las fuentes.
     */
    for (std::size_t camera = 0;
         camera < CAMERA_COUNT;
         ++camera)
    {
        if (pointCloudSources[camera] == nullptr)
        {
            continue;
        }


        if (!pointCloudSources[camera]->start())
        {
            /*
             * Si una fuente falla al iniciar,
             * detener las que pudieran haberse
             * iniciado anteriormente.
             */
            for (std::size_t previous = 0;
                 previous < camera;
                 ++previous)
            {
                if (pointCloudSources[previous] != nullptr)
                {
                    pointCloudSources[previous]->stop();
                }
            }


            return false;
        }
    }


    running.store(
        true
    );


    workerThread =
        std::thread(
            &Vision3DWorker::workerLoop,
            this
        );


    return true;
}


void Vision3DWorker::stop()
{
    bool wasRunning =
        running.exchange(
            false
        );


    if (wasRunning &&
        workerThread.joinable())
    {
        workerThread.join();
    }
    else if (workerThread.joinable())
    {
        workerThread.join();
    }


    /*
     * Detener todas las fuentes.
     */
    for (std::size_t camera = 0;
         camera < CAMERA_COUNT;
         ++camera)
    {
        if (pointCloudSources[camera] != nullptr)
        {
            pointCloudSources[camera]->stop();
        }
    }
}


bool Vision3DWorker::isRunning() const
{
    return running.load();
}


// ============================================================
// PROCESAMIENTO DE UNA CÁMARA
// ============================================================

bool Vision3DWorker::processCamera(
    std::size_t camera,
    VisionHeightSource::VisionResult& result)
{
    if (camera >= CAMERA_COUNT)
    {
        return false;
    }


    if (processor == nullptr)
    {
        return false;
    }


    PointCloudSource *source =
        pointCloudSources[camera].get();


    if (source == nullptr)
    {
        return false;
    }


    /*
     * Pedir una nube nueva.
     */
    Vision3DProcessor::PointCloud cloud;


    if (!source->getPointCloud(cloud))
    {
        return false;
    }


    /*
     * Procesar la nube usando la configuración
     * correspondiente a esta cámara.
     */
    result =
        processor->processPointCloud(
            camera,
            cloud
        );


    return true;
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void Vision3DWorker::workerLoop()
{
    while (running.load())
    {   
        /*
        * ====================================================
        * ACTUALIZAR ORIENTACIÓN DESDE IMU
        * ====================================================
        *
        * La STM32 publica la IMU en HagieState.
        *
        * Vision3DProcessor utiliza roll y pitch para
        * compensar la inclinación de la máquina antes
        * de calcular las alturas.
        */
        if (state != nullptr &&
            processor != nullptr)
        {
            HagieState::ImuState imu =
                state->getImuState();


            Vision3DProcessor::Orientation orientation;


            orientation.valid =
                imu.valid;


            orientation.roll_deg =
                imu.roll_deg;


            orientation.pitch_deg =
                imu.pitch_deg;


            processor->setOrientation(
                orientation
            );
        }
        /*
         * Resultado independiente de cada cámara.
         */
        std::array<
            VisionHeightSource::VisionResult,
            CAMERA_COUNT
        > cameraResults {};


        bool anyResult =
            false;


        /*
         * ====================================================
         * OBTENER Y PROCESAR LAS CÁMARAS
         * ====================================================
         */
        for (std::size_t camera = 0;
             camera < CAMERA_COUNT;
             ++camera)
        {
            if (!running.load())
            {
                break;
            }


            if (pointCloudSources[camera] == nullptr)
            {
                continue;
            }


            if (processCamera(
                    camera,
                    cameraResults[camera]
                ))
            {
                anyResult =
                    true;
            }
        }


        /*
         * ====================================================
         * FUSIÓN MULTICÁMARA
         * ====================================================
         */
        if (anyResult &&
            processor != nullptr &&
            heightSource != nullptr)
        {
            uint64_t nowMs =
                static_cast<uint64_t>(
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                            std::chrono::steady_clock::now()
                                .time_since_epoch()
                        ).count()
                );


            VisionHeightSource::VisionResult
                mergedResult =
                    processor->mergeCameraResults(
                        cameraResults,
                        nowMs,
                        RESULT_TIMEOUT_MS
                    );


            /*
             * Publicar el resultado final.
             *
             * VisionHeightSource se encarga de
             * escribir las alturas en HagieState.
             */
            heightSource->submitResult(
                mergedResult
            );
        }


        /*
         * ====================================================
         * FRECUENCIA DEL WORKER
         * ====================================================
         */
        std::this_thread::sleep_for(
            LOOP_INTERVAL
        );
    }
}