#pragma once

#include <QWidget>
#include <array>

class MachineReferenceWidget : public QWidget
{
public:

    static constexpr int BODY_COUNT = 6;

    static constexpr int CAMERA_COUNT = 5;

    struct CameraPosition
    {
        double xM = 0.0;
        double yM = 0.0;
        bool enabled = false;
    };

    void setCameraPosition(
        int camera,
        double xM,
        double yM,
        bool enabled
    );

    struct BodyRegion
    {
        double xMinMm = 0.0;
        double xMaxMm = 0.0;

        double yMinMm = 0.0;
        double yMaxMm = 0.0;

        bool valid = false;
    };

    explicit MachineReferenceWidget(
        QWidget *parent = nullptr
    );

    void setBodyRegion(
        int body,
        double xMinMm,
        double xMaxMm,
        double yMinMm,
        double yMaxMm
    );

    void clearBodyRegion(
        int body
    );

    void clearAllRegions();

protected:

    void paintEvent(
        QPaintEvent *event
    ) override;

private:

    std::array<
        BodyRegion,
        BODY_COUNT
    > bodyRegions;

    std::array<
        CameraPosition,
        CAMERA_COUNT
    > cameraPositions;
};