#include "vision_3d_processor.h"


#include <algorithm>
#include <chrono>


Vision3DProcessor::Vision3DProcessor()
{
    /*
     * Configuración provisoria.
     *
     * Todavía no conocemos la geometría
     * definitiva de montaje de las cámaras
     * respecto de los seis cuerpos.
     *
     * Por ahora dividimos el eje X
     * en seis regiones consecutivas.
     */
    bodyRegions[0] = {-3.0f, -2.0f};
    bodyRegions[1] = {-2.0f, -1.0f};
    bodyRegions[2] = {-1.0f,  0.0f};
    bodyRegions[3] = { 0.0f,  1.0f};
    bodyRegions[4] = { 1.0f,  2.0f};
    bodyRegions[5] = { 2.0f,  3.0f};
}


void Vision3DProcessor::setBodyRegion(
    std::size_t body,
    const BodyRegion& region)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    if (region.min_x >= region.max_x)
    {
        return;
    }


    bodyRegions[body] =
        region;
}


Vision3DProcessor::BodyRegion
Vision3DProcessor::getBodyRegion(
    std::size_t body) const
{
    if (body >= BODY_COUNT)
    {
        return BodyRegion {};
    }


    return bodyRegions[body];
}


VisionHeightSource::VisionResult
Vision3DProcessor::processPointCloud(
    const PointCloud& cloud) const
{
    VisionHeightSource::VisionResult result;


    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        result.bodies[body] =
            calculateBodyHeight(
                cloud,
                body
            );
    }


    return result;
}


VisionHeightSource::BodyVisionResult
Vision3DProcessor::calculateBodyHeight(
    const PointCloud& cloud,
    std::size_t body) const
{
    VisionHeightSource::BodyVisionResult result;


    if (body >= BODY_COUNT)
    {
        return result;
    }


    const BodyRegion& region =
        bodyRegions[body];


    /*
     * Por ahora hacemos algo muy simple:
     *
     * - tomamos los puntos que caen dentro
     *   de la región X del cuerpo;
     * - buscamos el mayor valor Z;
     * - lo usamos como altura.
     *
     * ESTO ES PROVISORIO.
     *
     * Después probablemente usemos:
     *
     * - plano de suelo;
     * - percentil 90/95;
     * - rechazo de outliers;
     * - filtrado temporal.
     */
    bool foundPoint =
        false;


    float maxZ =
        0.0f;


    for (const Point3D& point : cloud)
    {
        if (point.x < region.min_x ||
            point.x >= region.max_x)
        {
            continue;
        }


        if (!foundPoint ||
            point.z > maxZ)
        {
            maxZ =
                point.z;

            foundPoint =
                true;
        }
    }


    if (!foundPoint)
    {
        result.valid =
            false;

        return result;
    }


    /*
     * Por ahora asumimos que Z viene
     * expresado en metros.
     */
    float heightMm =
        maxZ * 1000.0f;


    if (heightMm < 0.0f)
    {
        result.valid =
            false;

        return result;
    }


    if (heightMm > 65535.0f)
    {
        result.valid =
            false;

        return result;
    }


    result.height_mm =
        static_cast<uint16_t>(
            heightMm
        );


    result.valid =
        true;


    result.timestamp_ms =
        static_cast<uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()
                ).count()
        );


    return result;
}