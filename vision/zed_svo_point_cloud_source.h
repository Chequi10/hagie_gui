#pragma once

#include <QString>
#include <QImage>
#include <memory>
#include "vision/point_cloud_source.h"

class ZedSvoPointCloudSource final : public PointCloudSource
{
public:
    explicit ZedSvoPointCloudSource(QString file);
    ~ZedSvoPointCloudSource() override;

    bool start() override;
    void stop() override;
    bool isRunning() const override;

    bool getPointCloud(
        Vision3DProcessor::PointCloud& cloud) override;

    std::size_t getCameraIndex() const override
    { return 0; }

    bool seekFrame(std::size_t index);

    bool getCameraOrientation(
        CameraOrientation& orientation) override
    {
        orientation = orientation_;
        return running_ && orientation.valid;
    }

    std::size_t frameCount() const { return count_; }
    std::size_t frameIndex() const { return index_; }

    double recordingTime(std::size_t index) const
    { return double(index) / fps_; }

    double pairDeltaMs() const { return 0.0; }

    const QString& error() const { return error_; }
    const QImage& rgbImage() const { return rgb_; }
    const QImage& depthImage() const { return depth_; }

    const Vision3DProcessor::PointCloud& points() const
    { return cloud_; }

    bool pointAt(int u, int v,
        Vision3DProcessor::Point3D& point) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    QString file_, error_;
    QImage rgb_, depth_;

    Vision3DProcessor::PointCloud cloud_;

    CameraOrientation orientation_ {};

    bool running_ = false;
    std::size_t count_ = 0;
    std::size_t index_ = 0;
    double fps_ = 30.0;
};
