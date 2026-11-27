#ifndef VISION_HEIGHT_SOURCE_H
#define VISION_HEIGHT_SOURCE_H


#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <memory>
#include <vector>
#include <thread>


#include "core/hagie_state.h"


class VisionHeightSource
{
public:

    static constexpr std::size_t BODY_COUNT = 6;

    // ========================================================
    // FUENTE DE VISIÓN
    // ========================================================

    enum class SourceMode
    {
        SIMULATION,
        EXTERNAL
    };


    /*
     * Resultado de visión para un cuerpo.
     *
     * height_mm:
     * altura calculada por visión 3D.
     *
     * valid:
     * indica si la medición es válida.
     *
     * timestamp_ms:
     * instante asociado a la medición.
     * Más adelante nos permitirá detectar
     * datos viejos o una cámara congelada.
     */
    struct RegionPixel
    {
        int x;
        int y;
    };

    struct RegionLine
    {
        double x1;
        double y1;
        double x2;
        double y2;
    };

    struct BodyVisionResult
    {
        uint16_t height_mm = 0;

        bool valid = false;

        uint64_t timestamp_ms = 0;

        // Diagnostico del procesamiento 3D
        std::shared_ptr<const std::vector<RegionPixel>>
            region_pixels;

        uint64_t region_frame_timestamp_ms = 0;

        std::vector<RegionLine> region_lines;

        int region_image_width = 0;
        int region_image_height = 0;


        bool diagnostics_available = false;
        bool p95_valid = false;
        float p95_mm = 0.0f;
        std::size_t valid_points = 0;
        std::size_t required_points = 0;

        int camera_index = -1;
        float camera_height_mm = 0.0f;

        bool orientation_valid = false;
        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
    };


    /*
     * Resultado completo de visión
     * para los seis cuerpos.
     */
    struct VisionResult
    {
        std::array<
            BodyVisionResult,
            BODY_COUNT
        > bodies {};

        /*
         * Número incremental de muestra.
         */
        uint64_t sequence = 0;
    };


    /*
     * Recibimos HagieState para publicar
     * allí los resultados de visión.
     */
    explicit VisionHeightSource(
        HagieState *state
    );


    ~VisionHeightSource();


    /*
     * No permitimos copiar el objeto porque
     * posee un hilo interno y mutex.
     */
    VisionHeightSource(
        const VisionHeightSource&
    ) = delete;


    VisionHeightSource& operator=(
        const VisionHeightSource&
    ) = delete;


    // ========================================================
    // CONTROL
    // ========================================================

    bool start();


    void stop();


    bool isRunning() const;

    // ========================================================
    // SELECCIÓN DE FUENTE
    // ========================================================

    void setSourceMode(
        SourceMode mode
    );

    SourceMode getSourceMode() const;


    // ========================================================
    // RESULTADO ACTUAL
    // ========================================================

    VisionResult getResult() const;


    // ========================================================
    // ENTRADA DE VISIÓN REAL
    // ========================================================

    /*
     * Este método será utilizado por el módulo
     * que procese las cámaras 3D.
     *
     * El procesamiento real podrá generar:
     *
     * Cuerpo 1 -> altura, válido, timestamp
     * Cuerpo 2 -> altura, válido, timestamp
     * ...
     * Cuerpo 6 -> altura, válido, timestamp
     *
     * y entregar todo mediante submitResult().
     *
     * VisionHeightSource se encargará de:
     *
     * - almacenar el último resultado;
     * - actualizar la secuencia;
     * - publicar los datos en HagieState;
     * - ponerlos a disposición de AUTO VISIÓN.
     *
     * IMPORTANTE:
     *
     * Este método NO manda nada a STM32.
     * Tampoco controla válvulas.
     */
    void submitResult(
        const VisionResult& newResult
    );


private:

    // ========================================================
    // HILO
    // ========================================================

    void workerLoop();


    // ========================================================
    // SIMULACIÓN
    // ========================================================

    /*
     * Por ahora seguimos utilizando este método
     * para generar alturas simuladas.
     *
     * Más adelante podrá desactivarse cuando
     * entren datos de las cámaras reales.
     */
    void generateSimulatedResult();


    // ========================================================
    // PUBLICACIÓN
    // ========================================================

    /*
     * Publicar un resultado en HagieState.
     */
    void publishResult(
        const VisionResult& newResult
    );


    // ========================================================
    // ESTADO CENTRAL
    // ========================================================

    HagieState *state;


    // ========================================================
    // CONTROL DEL HILO
    // ========================================================

    std::atomic<bool> running;


    std::thread workerThread;

    // ========================================================
    // MODO DE FUENTE
    // ========================================================

    std::atomic<SourceMode> sourceMode {
        SourceMode::SIMULATION
    };

    // ========================================================
    // ÚLTIMO RESULTADO
    // ========================================================

    mutable std::mutex resultMutex;


    VisionResult result;


    // ========================================================
    // SIMULACIÓN
    // ========================================================

    uint32_t simulationStep = 0;
};


#endif