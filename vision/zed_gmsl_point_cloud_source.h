#ifndef ZED_GMSL_POINT_CLOUD_SOURCE_H
#define ZED_GMSL_POINT_CLOUD_SOURCE_H


#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <mutex>
#include <memory>
#include "vision/rgb_frame_source.h"

#include "vision/point_cloud_source.h"



/*
 * ============================================================
 * ZED SDK
 * ============================================================
 *
 * En la Jetson AGX Orin, cuando esté instalado el ZED SDK,
 * se habilitará:
 *
 * #include <sl/Camera.hpp>
 *
 * Por ahora queda comentado para permitir compilar
 * el proyecto en la PC de desarrollo sin ZED SDK.
 */
#ifdef HAGIE_ENABLE_ZED_SDK

#include <sl/Camera.hpp>

#endif


class ZedGmslPointCloudSource
    : public PointCloudSource
{
public:

    /*
     * cameraIndex:
     * índice lógico dentro de Hagie:
     *
     * 0 -> cámara 1
     * 1 -> cámara 2
     * 2 -> cámara 3
     *
     * serialNumber:
     * número de serie físico de la ZED.
     *
     * Usaremos el serial para que la asignación
     * física de cámaras no dependa del orden
     * detectado por Linux / ZED Link.
     */

    struct SharedRgbFrame
    {
        mutable std::mutex mutex;

        RgbFrameSource::Frame frame;
    };


    using SharedRgbFramePtr =
        std::shared_ptr<SharedRgbFrame>;


    ZedGmslPointCloudSource(
        std::size_t cameraIndex,
        uint32_t serialNumber
    );


    ~ZedGmslPointCloudSource() override;

    /*
    * ========================================================
    * DETECCIÓN DE CÁMARAS ZED
    * ========================================================
    *
    * Devuelve los números de serie de todas las
    * cámaras ZED visibles por el SDK.
    *
    * Con HAGIE_ENABLE_ZED_SDK deshabilitado
    * devuelve un vector vacío.
    */
    static std::vector<uint32_t>
    detectConnectedSerialNumbers();

    bool start() override;

    void stop() override;

    bool isRunning() const override;


    bool getPointCloud(
        Vision3DProcessor::PointCloud& cloud
    ) override;

    bool getLastPointCloudTimestampMs(
        std::uint64_t& timestampMs
    ) const override;

    bool getCameraOrientation(
        CameraOrientation& orientation
    ) override;

    bool getLatestRgbFrame(
        RgbFrameSource::Frame& frame
    ) const;

    SharedRgbFramePtr getSharedRgbFrame() const;


    std::size_t getCameraIndex() const override;


    uint32_t getSerialNumber() const;


private:

    std::size_t cameraIndex;

    uint32_t serialNumber;

    std::atomic<bool> running {false};

    SharedRgbFramePtr sharedRgbFrame;

    std::uint64_t lastPointCloudTimestampMs = 0;

    /*
     * ========================================================
     * OBJETO ZED SDK
     * ========================================================
     *
     * Cuando habilitemos el SDK en la Jetson:
     *
     * sl::Camera camera;
     *
     * También podremos mantener:
     *
     * sl::Mat pointCloud;
     * sl::RuntimeParameters runtimeParameters;
     *
     * Por ahora quedan comentados.
     */

    #ifdef HAGIE_ENABLE_ZED_SDK

        sl::Camera camera;

        sl::Mat zedPointCloud;

        sl::Mat zedRgbImage;

        sl::RuntimeParameters runtimeParameters;

    #endif
};


#endif