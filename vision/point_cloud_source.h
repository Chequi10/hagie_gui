#ifndef POINT_CLOUD_SOURCE_H
#define POINT_CLOUD_SOURCE_H

#include <cstddef>

#include "vision/vision_3d_processor.h"


class PointCloudSource
{
public:

    virtual ~PointCloudSource() = default;


    /*
     * ========================================================
     * CONTROL
     * ========================================================
     */

    virtual bool start() = 0;

    virtual void stop() = 0;

    virtual bool isRunning() const = 0;


    /*
     * ========================================================
     * ADQUISICIÓN
     * ========================================================
     *
     * Obtener una nube de puntos correspondiente
     * a una cámara.
     *
     * Devuelve:
     *
     * true  -> nube obtenida correctamente
     * false -> no hubo una nube válida disponible
     */
    virtual bool getPointCloud(
        Vision3DProcessor::PointCloud& cloud
    ) = 0;


    /*
     * ========================================================
     * IDENTIFICACIÓN
     * ========================================================
     *
     * Permite conocer qué cámara representa
     * esta fuente dentro de Vision3DProcessor.
     *
     * Valores esperados:
     *
     * 0 -> cámara 1
     * 1 -> cámara 2
     * 2 -> cámara 3
     */
    virtual std::size_t getCameraIndex() const = 0;
};


#endif