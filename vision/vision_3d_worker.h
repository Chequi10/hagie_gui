#ifndef VISION_3D_WORKER_H
#define VISION_3D_WORKER_H


#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>


#include "core/hagie_state.h"
#include "vision/point_cloud_source.h"
#include "vision/vision_3d_processor.h"
#include "vision/vision_height_source.h"
#include <mutex>


class Vision3DWorker
{
public:

    static constexpr std::size_t CAMERA_COUNT =
        Vision3DProcessor::CAMERA_COUNT;


    /*
     * ========================================================
     * CONSTRUCTOR
     * ========================================================
     *
     * El worker NO es propietario de:
     *
     * - HagieState
     * - Vision3DProcessor
     * - VisionHeightSource
     *
     * Estos objetos existen durante toda la aplicación.
     */
    Vision3DWorker(
        HagieState *state,
        Vision3DProcessor *processor,
        VisionHeightSource *heightSource
    );


    ~Vision3DWorker();


    /*
     * No permitimos copiar el worker.
     *
     * Contiene un hilo propio.
     */
    Vision3DWorker(
        const Vision3DWorker&
    ) = delete;


    Vision3DWorker& operator=(
        const Vision3DWorker&
    ) = delete;


    void setMachineOrientationOverride(
        float rollDeg,
        float pitchDeg
    );

    void clearMachineOrientationOverride();

    PointCloudSource::CameraOrientation
    getMachineOrientation() const;


    // ========================================================
    // FUENTES DE NUBE DE PUNTOS
    // ========================================================

    /*
     * Asigna una fuente de nube a una cámara.
     *
     * camera:
     *
     * 0 -> cámara 1
     * 1 -> cámara 2
     * 2 -> cámara 3
     *
     * El worker pasa a ser propietario de la fuente.
     */
    bool setPointCloudSource(
        std::size_t camera,
        std::unique_ptr<PointCloudSource> source
    );

    /*
    * ========================================================
    * ORIENTACIÓN DE CADA CÁMARA
    * ========================================================
    *
    * Lectura opcional de la IMU integrada
    * de cada cámara.
    *
    * No reemplaza la IMU general de la Hagie.
    * Se usa para calibración y diagnóstico.
    */
    std::array<
        PointCloudSource::CameraOrientation,
        CAMERA_COUNT
    > cameraOrientations {};

    /*
     * Timestamp de captura de la última
     * nube válida de cada cámara.
     *
     * Unidad: milisegundos monotónicos.
     */
    std::array<
        std::uint64_t,
        CAMERA_COUNT
    > latestPointCloudTimestampMs {};



    /*
     * Elimina la fuente asociada a una cámara.
     *
     * Solo debe utilizarse con el worker detenido.
     */
    void clearPointCloudSource(
        std::size_t camera
    );

    bool getCameraMountingOffset(
        std::size_t camera,
        float& rollOffsetDeg,
        float& pitchOffsetDeg
    ) const;

    

    /*
     * Indica si existe una fuente instalada
     * para determinada cámara.
     */
    bool hasPointCloudSource(
        std::size_t camera
    ) const;

    bool getLatestPointCloud(
        std::size_t camera,
        Vision3DProcessor::PointCloud& cloud,
        std::uint64_t& timestampMs
    ) const;

        PointCloudSource::CameraOrientation
    getCameraOrientation(
        std::size_t camera
    ) const;


    // ========================================================
    // CONTROL DEL WORKER
    // ========================================================

    bool start();

    void stop();

    bool isRunning() const;


private:

    /*
     * ========================================================
     * LOOP PRINCIPAL
     * ========================================================
     */
    void workerLoop();

    


    /*
     * ========================================================
     * PROCESAMIENTO DE UNA CÁMARA
     * ========================================================
     *
     * Obtiene una nube de la fuente correspondiente
     * y la procesa mediante Vision3DProcessor.
     */
    bool processCamera(
        std::size_t camera,
        VisionHeightSource::VisionResult& result
    );


    /*
     * ========================================================
     * ESTADO DEL SISTEMA
     * ========================================================
     */

    HagieState *state = nullptr;

    Vision3DProcessor *processor = nullptr;

    VisionHeightSource *heightSource = nullptr;


    /*
     * Una fuente independiente por cámara.
     */
    std::array<
        std::unique_ptr<PointCloudSource>,
        CAMERA_COUNT
    > pointCloudSources {};

    /*
    * Última nube válida recibida
    * de cada cámara 3D frontal.
    *
    * Se usa para asociar detecciones RGB
    * con posiciones físicas XYZ.
    */
    std::array<
        Vision3DProcessor::PointCloud,
        CAMERA_COUNT
    > latestPointClouds {};


    std::array<
        bool,
        CAMERA_COUNT
    > latestPointCloudValid {};


    mutable std::mutex
        latestPointCloudMutex;


    /*
     * ========================================================
     * HILO
     * ========================================================
     */

    std::atomic<bool> running {false};

    std::thread workerThread;

    std::atomic<bool>
        machineOrientationOverrideEnabled {false};

    std::atomic<float>
        machineOrientationOverrideRollDeg {0.0f};

    std::atomic<float>
        machineOrientationOverridePitchDeg {0.0f};


    /*
     * Intervalo inicial del procesamiento.
     *
     * 50 ms = máximo aproximado de 20 ciclos/s.
     *
     * Más adelante podremos adaptarlo al FPS
     * real de las cámaras.
     */
    static constexpr std::chrono::milliseconds
        LOOP_INTERVAL {50};


    /*
     * Tiempo máximo permitido para considerar
     * válida una medición al fusionar cámaras.
     */
    static constexpr uint64_t
        RESULT_TIMEOUT_MS =
            500;
};


#endif