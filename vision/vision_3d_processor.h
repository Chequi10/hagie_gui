#ifndef VISION_3D_PROCESSOR_H
#define VISION_3D_PROCESSOR_H


#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>


#include "vision/vision_height_source.h"


class Vision3DProcessor
{
public:

    static constexpr std::size_t BODY_COUNT = 6;

    static constexpr std::size_t CAMERA_COUNT = 5;


    // ========================================================
    // PUNTO 3D GENÉRICO
    // ========================================================

    /*
     * Punto 3D genérico.
     *
     * No depende todavía de tipos específicos
     * del SDK ZED.
     */
    struct Point3D
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };


    using PointCloud =
        std::vector<Point3D>;


    // ========================================================
    // MAPEO DE EJES
    // ========================================================

    /*
     * Ejes posibles de la nube original.
     */
    enum class Axis
    {
        X,
        Y,
        Z
    };


    /*
     * Define cómo interpretamos los ejes de entrada.
     *
     * El sistema interno normalizado siempre será:
     *
     * X = lateral
     * Y = longitudinal
     * Z = vertical
     *
     * Los signos permiten invertir cada eje
     * sin modificar el resto del algoritmo.
     */
    struct AxisMapping
    {
        Axis lateral =
            Axis::X;

        Axis longitudinal =
            Axis::Y;

        Axis vertical =
            Axis::Z;


        int lateral_sign =
            1;

        int longitudinal_sign =
            1;

        int vertical_sign =
            1;
    };


    // ========================================================
    // ORIENTACIÓN IMU
    // ========================================================

    /*
     * Orientación instantánea de la máquina.
     *
     * Estos valores vendrán de la IMU.
     */
    struct Orientation
    {
        float roll_deg = 0.0f;

        float pitch_deg = 0.0f;

        bool valid = false;
    };


    // ========================================================
    // GEOMETRÍA DE CÁMARA
    // ========================================================

    /*
     * Geometría fija de montaje de la cámara.
     *
     * camera_height_mm:
     * altura de la cámara respecto del piso.
     *
     * roll_offset_deg / pitch_offset_deg:
     * correcciones fijas debidas al montaje físico
     * de la cámara.
     */
    struct CameraGeometry
    {
        float camera_height_mm = 0.0f;

        float roll_offset_deg = 0.0f;

        float pitch_offset_deg = 0.0f;


        /*
        * Posición física de la cámara
        * respecto del sistema de coordenadas
        * general de la máquina.
        *
        * Unidad: milímetros.
        */
        float position_x_mm = 0.0f;
        float position_y_mm = 0.0f;
        float position_z_mm = 0.0f;

    };

    /*
    * Configuración de una cámara 3D.
    *
    * enabled:
    * indica si la cámara está disponible.
    *
    * body_enabled:
    * indica qué cuerpos son atendidos
    * por esta cámara.
    */
    struct CameraConfig
    {
        bool enabled = false;

        CameraGeometry geometry;

        std::array<
            bool,
            BODY_COUNT
        > body_enabled {};
    };


    // ========================================================
    // REGIONES DE LOS CUERPOS
    // ========================================================

    /*
     * Configuración espacial de cada cuerpo.
     *
     * Por ahora solamente se definen
     * límites laterales sobre el eje X normalizado.
     */
    struct BodyRegion
    {
        float min_x = 0.0f;
        float max_x = 0.0f;

        float min_y = -10.0f;
        float max_y = 10.0f;

        float min_z = 0.0f;
        float max_z = 5.0f;

        std::size_t min_points = 1;
    };


    // ========================================================
    // Filtro
    // ========================================================

    struct TemporalFilterConfig
    {
        bool enabled = true;

        float alpha = 0.25f;
    };


    // ========================================================
    // CONSTRUCTOR
    // ========================================================

    Vision3DProcessor();


    // ========================================================
    // MAPEO DE EJES
    // ========================================================

    void setAxisMapping(
        const AxisMapping& mapping
    );


    AxisMapping getAxisMapping() const;


    // ========================================================
    // ORIENTACIÓN IMU
    // ========================================================

    void setOrientation(
        const Orientation& orientation
    );


    Orientation getOrientation() const;


    void setCameraConfig(
        std::size_t camera,
        const CameraConfig& config
    );

    CameraConfig getCameraConfig(
        std::size_t camera
    ) const;


    // ========================================================
    // GEOMETRÍA DE CÁMARA
    // ========================================================

    void setCameraGeometry(
        const CameraGeometry& geometry
    );


    CameraGeometry getCameraGeometry() const;


    // ========================================================
    // REGIONES POR CUERPO
    // ========================================================

    /*
     * Configurar la región lateral
     * correspondiente a un cuerpo.
     */
    void setBodyRegion(
        std::size_t body,
        const BodyRegion& region
    );


    BodyRegion getBodyRegion(
        std::size_t body
    ) const;


    // ========================================================
    // PROCESAMIENTO DE NUBE
    // ========================================================

    /*
     * Procesar una nube completa.
     *
     * La salida tiene exactamente el formato
     * que entiende VisionHeightSource.
     */
    VisionHeightSource::VisionResult
    processPointCloud(
        const PointCloud& cloud
    ) const;

    VisionHeightSource::VisionResult
    processPointCloud(
        std::size_t camera,
        const PointCloud& cloud
    ) const;

    void setTemporalFilterConfig(
        const TemporalFilterConfig& config
    );

    TemporalFilterConfig getTemporalFilterConfig() const;

    void resetTemporalFilter();

    bool isResultFresh(
        const VisionHeightSource::BodyVisionResult& result,
        uint64_t now_ms,
        uint64_t timeout_ms
    ) const;

    VisionHeightSource::VisionResult mergeCameraResults(
        const std::array<
            VisionHeightSource::VisionResult,
            CAMERA_COUNT
        >& cameraResults,
        uint64_t now_ms,
        uint64_t timeout_ms
    ) const;


private:

    // ========================================================
    // NORMALIZACIÓN DE EJES
    // ========================================================

    /*
     * Convierte los ejes de entrada al sistema interno:
     *
     * X = lateral
     * Y = longitudinal
     * Z = vertical
     */
    Point3D normalizeAxes(
        const Point3D& point
    ) const;

    /*
    * Compensa la inclinación de la máquina
    * utilizando roll y pitch de la IMU.
    *
    * La entrada ya debe estar normalizada:
    *
    * X = lateral
    * Y = longitudinal
    * Z = vertical
    */
    Point3D levelPointWithOrientation(
        const Point3D& point
    ) const;


    TemporalFilterConfig temporalFilterConfig;

    mutable std::array<
        float,
        BODY_COUNT
    > filteredHeightMm {};

    mutable std::array<
        bool,
        BODY_COUNT
    > temporalFilterInitialized {};


    // ========================================================
    // CÁLCULO DE ALTURA
    // ========================================================

    /*
     * Estimar la altura correspondiente
     * a un cuerpo determinado.
     */
    VisionHeightSource::BodyVisionResult
    calculateBodyHeight(
        const PointCloud& cloud,
        std::size_t body
    ) const;

    VisionHeightSource::BodyVisionResult
    calculateBodyHeight(
        const PointCloud& cloud,
        std::size_t body,
        const CameraGeometry& geometry
    ) const;


    // ========================================================
    // CONFIGURACIÓN ACTUAL
    // ========================================================

    Orientation orientation;


    CameraGeometry cameraGeometry;


    AxisMapping axisMapping;

    std::array<
        CameraConfig,
        CAMERA_COUNT
    > cameraConfigs {};


    /*
     * Regiones espaciales por cuerpo.
     */
    std::array<
        BodyRegion,
        BODY_COUNT
    > bodyRegions {};


    Point3D transformCameraToMachine(
        const Point3D& point,
        const CameraGeometry& geometry
    ) const;

    VisionHeightSource::BodyVisionResult
    applyTemporalFilter(
        std::size_t body,
        const VisionHeightSource::BodyVisionResult& input
    ) const;
};


#endif