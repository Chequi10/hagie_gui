#ifndef POINT_CLOUD_SOURCE_H
#define POINT_CLOUD_SOURCE_H

#include <cstddef>
#include <cstdint>

#include "vision/vision_3d_processor.h"


class PointCloudSource
{
public:

        /*
     * ========================================================
     * ORIENTACIÓN PROPIA DE LA CÁMARA
     * ========================================================
     *
     * Orientación medida por una IMU integrada en la cámara,
     * si la fuente dispone de ella.
     *
     * NO reemplaza la IMU general de la Hagie.
     *
     * Se utilizará para:
     *
     * - calibración del montaje de cada cámara;
     * - diagnóstico;
     * - cálculo de roll_offset_deg / pitch_offset_deg.
     */
    struct CameraOrientation
    {
        float roll_deg = 0.0f;

        float pitch_deg = 0.0f;

        bool valid = false;
    };

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
     * TIMESTAMP DE ADQUISICIÓN OPCIONAL
     * ========================================================
     *
     * Permite que una fuente entregue el instante real
     * asociado a la nube obtenida.
     *
     * Es especialmente importante cuando RGB y XYZ
     * provienen del mismo grab() de una cámara.
     *
     * Las fuentes que no tengan timestamp propio pueden
     * dejar la implementación por defecto.
     */
    virtual bool getLastPointCloudTimestampMs(
        std::uint64_t& timestampMs
    ) const
    {
        timestampMs = 0;

        return false;
    }

        /*
     * ========================================================
     * IMU OPCIONAL DE LA CÁMARA
     * ========================================================
     *
     * Devuelve true si la fuente dispone de una orientación
     * válida proveniente de su propia IMU.
     *
     * Por defecto una cámara puede no tener IMU.
     *
     * Al tener implementación por defecto NO obliga a las
     * fuentes simuladas ni a futuras cámaras sin IMU a
     * implementar esta función.
     */
    virtual bool getCameraOrientation(
        CameraOrientation& orientation)
    {
        orientation = CameraOrientation {};

        return false;
    }


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