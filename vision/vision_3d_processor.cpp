#include "vision_3d_processor.h"


#include <algorithm>
#include <chrono>
#include <cmath>



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
    bodyRegions[0].min_x = -3.0f;
    bodyRegions[0].max_x = -2.0f;

    bodyRegions[1].min_x = -2.0f;
    bodyRegions[1].max_x = -1.0f;

    bodyRegions[2].min_x = -1.0f;
    bodyRegions[2].max_x = 0.0f;

    bodyRegions[3].min_x = 0.0f;
    bodyRegions[3].max_x = 1.0f;

    bodyRegions[4].min_x = 1.0f;
    bodyRegions[4].max_x = 2.0f;

    bodyRegions[5].min_x = 2.0f;
    bodyRegions[5].max_x = 3.0f;
}

void Vision3DProcessor::setOrientation(
    const Orientation& orientation)
{
    this->orientation =
        orientation;
}


Vision3DProcessor::Orientation
Vision3DProcessor::getOrientation() const
{
    return orientation;
}


void Vision3DProcessor::setCameraGeometry(
    const CameraGeometry& geometry)
{
    cameraGeometry =
        geometry;
}


Vision3DProcessor::CameraGeometry
Vision3DProcessor::getCameraGeometry() const
{
    return cameraGeometry;
}

void Vision3DProcessor::setCameraConfig(
    std::size_t camera,
    const CameraConfig& config)
{
    /*
     * Protección contra un índice
     * de cámara inválido.
     */
    if (camera >= CAMERA_COUNT)
    {
        return;
    }

    cameraConfigs[camera] =
        config;
}


Vision3DProcessor::CameraConfig
Vision3DProcessor::getCameraConfig(
    std::size_t camera) const
{
    /*
     * Si el índice no existe,
     * devolvemos una configuración vacía
     * y deshabilitada.
     */
    if (camera >= CAMERA_COUNT)
    {
        return CameraConfig {};
    }

    return cameraConfigs[camera];
}

void Vision3DProcessor::setAxisMapping(
    const AxisMapping& mapping)
{
    axisMapping =
        mapping;
}


Vision3DProcessor::AxisMapping
    Vision3DProcessor::getAxisMapping() const
    {
        return axisMapping;
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

bool Vision3DProcessor::findBodyForPosition(
    std::size_t camera,
    const Point3D& position,
    std::size_t& body) const
{
    if (camera >= CAMERA_COUNT)
    {
        return false;
    }


    const CameraConfig& config =
        cameraConfigs[camera];


    if (!config.enabled)
    {
        return false;
    }


    for (std::size_t candidateBody = 0;
         candidateBody < BODY_COUNT;
         ++candidateBody)
    {
        if (!config.body_enabled[candidateBody])
        {
            continue;
        }


        const BodyRegion& region =
            bodyRegions[candidateBody];


        if (position.x < region.min_x ||
            position.x > region.max_x)
        {
            continue;
        }


        if (position.y < region.min_y ||
            position.y > region.max_y)
        {
            continue;
        }


        if (position.z < region.min_z ||
            position.z > region.max_z)
        {
            continue;
        }


        body =
            candidateBody;

        return true;
    }


    return false;
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
        VisionHeightSource::BodyVisionResult instantResult =
            calculateBodyHeight(
                cloud,
                body
            );


        result.bodies[body] =
            applyTemporalFilter(
                body,
                instantResult
            );
    }


    return result;
}

VisionHeightSource::VisionResult
Vision3DProcessor::processPointCloud(
    std::size_t camera,
    const PointCloud& cloud) const
{
    VisionHeightSource::VisionResult result;


    /*
     * Cámara inexistente.
     *
     * Devolvemos todos los cuerpos como
     * mediciones no válidas.
     */
    if (camera >= CAMERA_COUNT)
    {
        return result;
    }


    const CameraConfig& config =
        cameraConfigs[camera];


    /*
     * Cámara deshabilitada.
     */
    if (!config.enabled)
    {
        return result;
    }


    /*
     * Procesar solamente los cuerpos
     * asignados a esta cámara.
     */
    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        if (!config.body_enabled[body])
        {
            result.bodies[body].valid =
                false;

            continue;
        }


        VisionHeightSource::BodyVisionResult instantResult =
    calculateBodyHeight(
        cloud,
        body,
        config.geometry
    );


        result.bodies[body] =
            applyTemporalFilter(
                body,
                instantResult
            );
    }


    return result;
}

Vision3DProcessor::Point3D
Vision3DProcessor::normalizeAxes(
    const Point3D& point) const
{
    auto getAxisValue =
        [&point](Axis axis) -> float
        {
            switch (axis)
            {
                case Axis::X:
                    return point.x;

                case Axis::Y:
                    return point.y;

                case Axis::Z:
                    return point.z;
            }

            return 0.0f;
        };


    Point3D normalized =
        point;


    normalized.x =
        static_cast<float>(
            axisMapping.lateral_sign
        )
        *
        getAxisValue(
            axisMapping.lateral
        );


    normalized.y =
        static_cast<float>(
            axisMapping.longitudinal_sign
        )
        *
        getAxisValue(
            axisMapping.longitudinal
        );


    normalized.z =
        static_cast<float>(
            axisMapping.vertical_sign
        )
        *
        getAxisValue(
            axisMapping.vertical
        );


    return normalized;
}

Vision3DProcessor::Point3D
Vision3DProcessor::transformCameraToMachine(
    const Point3D& point,
    const CameraGeometry& geometry) const
{
    Point3D transformed =
        point;


    transformed.x =
        point.x +
        (
            geometry.position_x_mm
            / 1000.0f
        );


    transformed.y =
        point.y +
        (
            geometry.position_y_mm
            / 1000.0f
        );


    transformed.z =
        point.z +
        (
            geometry.position_z_mm
            / 1000.0f
        );


    return transformed;
}


Vision3DProcessor::Point3D
Vision3DProcessor::levelPointWithOrientation(
    const Point3D& point) const
{
    /*
     * Si la IMU todavía no es válida,
     * no modificamos el punto.
     */
    if (!orientation.valid)
    {
        return point;
    }


    constexpr float DEG_TO_RAD =
        3.14159265358979323846f / 180.0f;


    /*
     * Sumamos la inclinación instantánea
     * de la máquina y el offset fijo
     * de montaje de la cámara.
     */
    float roll =
        -(
            orientation.roll_deg +
            cameraGeometry.roll_offset_deg
        )
        * DEG_TO_RAD;

    float pitch =
        -(
            orientation.pitch_deg +
            cameraGeometry.pitch_offset_deg
        )
        * DEG_TO_RAD;


    float cosRoll =
        std::cos(roll);

    float sinRoll =
        std::sin(roll);

    float cosPitch =
        std::cos(pitch);

    float sinPitch =
        std::sin(pitch);


    /*
     * --------------------------------------------------------
     * CORRECCIÓN DE ROLL
     * --------------------------------------------------------
     *
     * Rotación alrededor del eje longitudinal Y.
     *
     * Sistema interno:
     *
     * X = lateral
     * Y = longitudinal
     * Z = vertical
     */
    Point3D afterRoll =
        point;

    afterRoll.x =
        cosRoll * point.x +
        sinRoll * point.z;

    afterRoll.y =
        point.y;

    afterRoll.z =
        -sinRoll * point.x +
        cosRoll * point.z;


    /*
     * --------------------------------------------------------
     * CORRECCIÓN DE PITCH
     * --------------------------------------------------------
     *
     * Rotación alrededor del eje lateral X.
     */
    Point3D leveled =
        afterRoll;

    leveled.x =
        afterRoll.x;

    leveled.y =
        cosPitch * afterRoll.y -
        sinPitch * afterRoll.z;

    leveled.z =
        sinPitch * afterRoll.y +
        cosPitch * afterRoll.z;


    return leveled;
}

bool Vision3DProcessor::getDetectionPosition3D(
    const PointCloud& cloud,
    int imageX,
    int imageY,
    int imageWidth,
    int imageHeight,
    const CameraGeometry& geometry,
    Point3D& position
) const
{
    if (imageWidth <= 0 ||
        imageHeight <= 0)
    {
        return false;
    }


    const int minX =
        imageX;

    const int maxX =
        imageX + imageWidth;

    const int minY =
        imageY;

    const int maxY =
        imageY + imageHeight;


    std::vector<float> positionsX;
    std::vector<float> positionsY;
    std::vector<float> positionsZ;


    for (const Point3D& point : cloud)
    {
        /*
         * Este punto debe conocer el píxel
         * de profundidad del cual provino.
         */
        if (!point.image_coordinates_valid)
        {
            continue;
        }


        /*
         * Quedarnos solamente con puntos 3D
         * que estén dentro del bounding box
         * de la panoja detectada en RGB.
         */
        if (point.image_x < minX ||
            point.image_x >= maxX ||
            point.image_y < minY ||
            point.image_y >= maxY)
        {
            continue;
        }


        /*
         * Llevar el punto al mismo sistema
         * físico usado por la máquina.
         */
        Point3D normalizedPoint =
            normalizeAxes(
                point
            );


        Point3D machinePoint =
            transformCameraToMachine(
                normalizedPoint,
                geometry
            );


        Point3D leveledPoint =
            levelPointWithOrientation(
                machinePoint
            );


        if (!std::isfinite(leveledPoint.x) ||
            !std::isfinite(leveledPoint.y) ||
            !std::isfinite(leveledPoint.z))
        {
            continue;
        }


        positionsX.push_back(
            leveledPoint.x
        );

        positionsY.push_back(
            leveledPoint.y
        );

        positionsZ.push_back(
            leveledPoint.z
        );
    }


    /*
     * No encontramos profundidad válida
     * dentro de esta detección.
     */
    if (positionsX.empty())
    {
        return false;
    }


    /*
     * Usamos MEDIANA en vez de un único punto.
     *
     * Esto hace la posición mucho más resistente
     * a errores de profundidad y puntos aislados.
     */
    std::sort(
        positionsX.begin(),
        positionsX.end()
    );

    std::sort(
        positionsY.begin(),
        positionsY.end()
    );

    std::sort(
        positionsZ.begin(),
        positionsZ.end()
    );


    const std::size_t middle =
        positionsX.size() / 2;


    position.x =
        positionsX[middle];

    position.y =
        positionsY[middle];

    position.z =
        positionsZ[middle];


    /*
     * La posición resultante representa
     * físicamente la detección completa,
     * no un píxel concreto.
     */
    position.image_x =
        imageX + imageWidth / 2;

    position.image_y =
        imageY + imageHeight / 2;

    position.image_coordinates_valid =
        true;


    return true;
}

void Vision3DProcessor::setTemporalFilterConfig(
    const TemporalFilterConfig& config)
{
    temporalFilterConfig =
        config;
}


Vision3DProcessor::TemporalFilterConfig
Vision3DProcessor::getTemporalFilterConfig() const
{
    return temporalFilterConfig;
}


void Vision3DProcessor::resetTemporalFilter()
{
    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        filteredHeightMm[body] =
            0.0f;

        temporalFilterInitialized[body] =
            false;
    }
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
     * Guardamos todas las alturas válidas
     * encontradas dentro de la región.
     */
    std::vector<float> validHeights;


    for (const Point3D& point : cloud)
    {
        /*
         * 1. Normalizar los ejes.
         *
         * Sistema interno:
         *
         * X = lateral
         * Y = longitudinal
         * Z = vertical
         */
        Point3D normalizedPoint =
            normalizeAxes(
                point
            );


        /*
         * 2. Corregir inclinación usando IMU.
         */
        Point3D leveledPoint =
            levelPointWithOrientation(
                normalizedPoint
            );


        /*
         * 3. Descartar NaN e infinitos.
         */
        if (!std::isfinite(leveledPoint.x) ||
            !std::isfinite(leveledPoint.y) ||
            !std::isfinite(leveledPoint.z))
        {
            continue;
        }


        /*
         * 4. Determinar la región lateral
         *    usando coordenadas de máquina.
         *
         * IMPORTANTE:
         *
         * Para decidir a qué cuerpo pertenece
         * el punto utilizamos X antes de la
         * corrección IMU.
         */
        if (normalizedPoint.x < region.min_x ||
            normalizedPoint.x >= region.max_x)
        {
            continue;
        }


        /*
         * 5. Filtrar rango longitudinal.
         */
        if (leveledPoint.y < region.min_y ||
            leveledPoint.y > region.max_y)
        {
            continue;
        }


        /*
         * 6. Filtrar rango vertical.
         */
        if (leveledPoint.z < region.min_z ||
            leveledPoint.z > region.max_z)
        {
            continue;
        }


        /*
         * El punto superó todos los filtros.
         */
        validHeights.push_back(
            leveledPoint.z
        );
    }


    /*
     * No aceptar la medición si no tenemos
     * suficientes puntos válidos.
     */
    if (validHeights.size() <
        region.min_points)
    {
        result.valid =
            false;

        return result;
    }


    /*
     * Ordenar alturas de menor a mayor.
     */
    std::sort(
        validHeights.begin(),
        validHeights.end()
    );


    /*
     * ========================================================
     * ALTURA REPRESENTATIVA: PERCENTIL 95
     * ========================================================
     *
     * Ya no usamos maxZ.
     *
     * Un punto espurio extremadamente alto
     * no debería dominar toda la medición.
     */
    constexpr float HEIGHT_PERCENTILE =
        0.95f;


    std::size_t percentileIndex =
        static_cast<std::size_t>(
            HEIGHT_PERCENTILE *
            static_cast<float>(
                validHeights.size() - 1
            )
        );


    float representativeZ =
        validHeights[
            percentileIndex
        ];


    /*
     * La nube trabaja en metros.
     * La salida del sistema está en milímetros.
     */
    float heightMm =
        representativeZ * 1000.0f;


    if (heightMm < 0.0f ||
        heightMm > 65535.0f)
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


// ============================================================
// CÁLCULO DE ALTURA CON GEOMETRÍA DE CÁMARA
// ============================================================

VisionHeightSource::BodyVisionResult
Vision3DProcessor::calculateBodyHeight(
    const PointCloud& cloud,
    std::size_t body,
    const CameraGeometry& geometry) const
{
    VisionHeightSource::BodyVisionResult result;


    if (body >= BODY_COUNT)
    {
        return result;
    }


    const BodyRegion& region =
        bodyRegions[body];


    /*
     * Alturas válidas detectadas
     * para este cuerpo.
     */
    std::vector<float> validHeights;


    for (const Point3D& point : cloud)
    {
        /*
         * 1. Normalizar ejes de la cámara.
         */
        Point3D normalizedPoint =
            normalizeAxes(
                point
            );


        /*
         * 2. Transformar desde coordenadas locales
         *    de la cámara al sistema común
         *    de coordenadas de la máquina.
         */
        Point3D machinePoint =
            transformCameraToMachine(
                normalizedPoint,
                geometry
            );


        /*
         * 3. Compensar inclinación de la máquina
         *    utilizando la IMU.
         */
        Point3D leveledPoint =
            levelPointWithOrientation(
                machinePoint
            );


        /*
         * 4. Descartar puntos inválidos.
         */
        if (!std::isfinite(leveledPoint.x) ||
            !std::isfinite(leveledPoint.y) ||
            !std::isfinite(leveledPoint.z))
        {
            continue;
        }


        /*
         * 5. Región lateral.
         *
         * Para decidir a qué cuerpo pertenece
         * usamos X en coordenadas de máquina,
         * antes de aplicar la nivelación IMU.
         */
        if (machinePoint.x < region.min_x ||
            machinePoint.x >= region.max_x)
        {
            continue;
        }


        /*
         * 6. Rango longitudinal permitido.
         */
        if (leveledPoint.y < region.min_y ||
            leveledPoint.y > region.max_y)
        {
            continue;
        }


        /*
         * 7. Rango vertical permitido.
         */
        if (leveledPoint.z < region.min_z ||
            leveledPoint.z > region.max_z)
        {
            continue;
        }


        /*
         * Punto aceptado.
         */
        validHeights.push_back(
            leveledPoint.z
        );
    }


    /*
     * Cantidad mínima de puntos.
     */
    if (validHeights.size() <
        region.min_points)
    {
        result.valid =
            false;

        return result;
    }


    /*
     * Ordenar alturas.
     */
    std::sort(
        validHeights.begin(),
        validHeights.end()
    );


    /*
     * Percentil 95.
     */
    constexpr float HEIGHT_PERCENTILE =
        0.95f;


    std::size_t percentileIndex =
        static_cast<std::size_t>(
            HEIGHT_PERCENTILE *
            static_cast<float>(
                validHeights.size() - 1
            )
        );


    float representativeZ =
        validHeights[
            percentileIndex
        ];


    float heightMm =
        representativeZ * 1000.0f;


    if (heightMm < 0.0f ||
        heightMm > 65535.0f)
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

VisionHeightSource::BodyVisionResult
Vision3DProcessor::applyTemporalFilter(
    std::size_t body,
    const VisionHeightSource::BodyVisionResult& input) const
{
    if (body >= BODY_COUNT)
    {
        return input;
    }


    /*
     * Una medición inválida NO modifica
     * el estado del filtro.
     */
    if (!input.valid)
    {
        return input;
    }


    /*
     * Filtro deshabilitado.
     */
    if (!temporalFilterConfig.enabled)
    {
        return input;
    }


    float alpha =
        temporalFilterConfig.alpha;


    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }


    /*
     * Primera medición:
     * inicializamos directamente.
     */
    if (!temporalFilterInitialized[body])
    {
        filteredHeightMm[body] =
            static_cast<float>(
                input.height_mm
            );

        temporalFilterInitialized[body] =
            true;

        return input;
    }


    float newHeight =
        static_cast<float>(
            input.height_mm
        );


    filteredHeightMm[body] =
        alpha * newHeight
        +
        (1.0f - alpha)
        * filteredHeightMm[body];


    VisionHeightSource::BodyVisionResult output =
        input;


    output.height_mm =
        static_cast<uint16_t>(
            std::lround(
                filteredHeightMm[body]
            )
        );


    return output;
}

bool Vision3DProcessor::isResultFresh(
    const VisionHeightSource::BodyVisionResult& result,
    uint64_t now_ms,
    uint64_t timeout_ms) const
{
    if (!result.valid)
    {
        return false;
    }

    if (result.timestamp_ms == 0)
    {
        return false;
    }

    if (now_ms < result.timestamp_ms)
    {
        return false;
    }

    return
        (now_ms - result.timestamp_ms)
        <= timeout_ms;
}

VisionHeightSource::VisionResult
Vision3DProcessor::mergeCameraResults(
    const std::array<
        VisionHeightSource::VisionResult,
        CAMERA_COUNT
    >& cameraResults,
    uint64_t now_ms,
    uint64_t timeout_ms) const
{
    VisionHeightSource::VisionResult merged;


    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        bool foundValid =
            false;


        VisionHeightSource::BodyVisionResult
            bestResult;


        for (std::size_t camera = 0;
             camera < CAMERA_COUNT;
             ++camera)
        {
            const auto& candidate =
                cameraResults[camera]
                    .bodies[body];


            if (!isResultFresh(
                    candidate,
                    now_ms,
                    timeout_ms))
            {
                continue;
            }


            /*
             * Si más de una cámara entrega
             * el mismo cuerpo, nos quedamos
             * con la medición más reciente.
             */
            if (!foundValid ||
                candidate.timestamp_ms >
                    bestResult.timestamp_ms)
            {
                bestResult =
                    candidate;

                foundValid =
                    true;
            }
        }


        if (foundValid)
        {
            merged.bodies[body] =
                bestResult;
        }
        else
        {
            merged.bodies[body].valid =
                false;
        }
    }


    return merged;
}