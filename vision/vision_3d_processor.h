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


    /*
     * Punto 3D genérico.
     *
     * La idea es no depender todavía
     * de tipos específicos del SDK ZED.
     */
    struct Point3D
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };


    using PointCloud =
        std::vector<Point3D>;


    /*
     * Configuración espacial de cada cuerpo.
     *
     * Por ahora dejamos definidos límites
     * laterales simples.
     *
     * Más adelante podremos agregar:
     *
     * - profundidad mínima/máxima;
     * - región de interés;
     * - plano de suelo;
     * - inclinación IMU;
     * - filtros.
     */
    struct BodyRegion
    {
        float min_x = 0.0f;
        float max_x = 0.0f;
    };


    Vision3DProcessor();


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


    /*
     * Procesar una nube completa.
     *
     * La salida tiene exactamente el formato
     * que entiende VisionHeightSource.
     */
    VisionHeightSource::VisionResult processPointCloud(
        const PointCloud& cloud
    ) const;


private:

    /*
     * Estimar la altura correspondiente
     * a un cuerpo determinado.
     */
    VisionHeightSource::BodyVisionResult
    calculateBodyHeight(
        const PointCloud& cloud,
        std::size_t body
    ) const;


    /*
     * Regiones espaciales por cuerpo.
     */
    std::array<
        BodyRegion,
        BODY_COUNT
    > bodyRegions {};
};


#endif