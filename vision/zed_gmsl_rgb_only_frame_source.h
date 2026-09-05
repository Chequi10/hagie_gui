#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "vision/rgb_frame_source.h"


#ifdef HAGIE_ENABLE_ZED_SDK

#include <sl/Camera.hpp>

#endif


/*
 * ============================================================
 * ZED GMSL2 - FUENTE RGB SOLAMENTE
 * ============================================================
 *
 * Utilizada por las cámaras traseras:
 *
 * Cámara 6 -> índice lógico 5
 * Cámara 7 -> índice lógico 6
 *
 * Estas cámaras se abren físicamente por número de serie
 * pero Hagie utiliza solamente su imagen RGB.
 *
 * No se genera nube de puntos ni profundidad.
 */
class ZedGmslRgbOnlyFrameSource
    : public RgbFrameSource
{
public:

    ZedGmslRgbOnlyFrameSource(
        std::size_t cameraIndex,
        uint32_t serialNumber
    );


    ~ZedGmslRgbOnlyFrameSource() override;


    bool start() override;

    void stop() override;

    bool isRunning() const override;


    bool getFrame(
        Frame& frame
    ) override;


    std::size_t getCameraIndex() const override;


    uint32_t getSerialNumber() const;


private:

    std::size_t cameraIndex;

    uint32_t serialNumber;

    std::atomic<bool> running {false};


#ifdef HAGIE_ENABLE_ZED_SDK

    sl::Camera camera;

    sl::Mat zedRgbImage;

    sl::RuntimeParameters runtimeParameters;

#endif
};
