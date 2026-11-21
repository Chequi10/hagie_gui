#pragma once

#include <QString>
#include <QImage>
#include <vector>
#include "vision/point_cloud_source.h"

// Reproductor de capturas: acceso desde un solo hilo, con avance explicito.
// Conserva XYZ optico (X derecha, Y abajo, Z adelante), en milimetros.
class TumRgbdPointCloudSource final : public PointCloudSource
{
public:
    explicit TumRgbdPointCloudSource(QString directory, std::size_t camera = 0);
    bool start() override;
    void stop() override { running_ = false; }
    bool isRunning() const override { return running_; }
    bool getPointCloud(Vision3DProcessor::PointCloud& cloud) override;
    bool getLastPointCloudTimestampMs(std::uint64_t& timestamp) const override;
    std::size_t getCameraIndex() const override { return camera_; }

    bool seekFrame(std::size_t index);
    std::size_t frameCount() const { return pairs_.size(); }
    std::size_t frameIndex() const { return index_; }
    double recordingTime(std::size_t index) const;
    double pairDeltaMs() const;
    const QString& error() const { return error_; }
    const QImage& rgbImage() const { return rgb_; }
    const QImage& depthImage() const { return depth_; }
    const Vision3DProcessor::PointCloud& points() const { return cloud_; }
    bool pointAt(int u, int v, Vision3DProcessor::Point3D& point) const;

private:
    struct Entry { double time; QString file; };
    struct Pair { Entry rgb; Entry depth; };
    bool readList(const QString& file, std::vector<Entry>& entries);
    QString directory_;
    std::size_t camera_ = 0;
    bool running_ = false;
    std::size_t index_ = 0;
    std::uint64_t timestamp_ = 0;
    std::vector<Pair> pairs_;
    QImage rgb_, depth_;
    Vision3DProcessor::PointCloud cloud_;
    QString error_;
};
