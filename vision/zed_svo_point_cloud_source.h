#pragma once

#include <QString>
#include <QImage>
#include <memory>
#include <mutex>
#include <optional>
#include "vision/point_cloud_source.h"
#include "vision/zed_gmsl_point_cloud_source.h"

class ZedSvoPointCloudSource final : public PointCloudSource
{
public:
    explicit ZedSvoPointCloudSource(
        QString file,
        std::size_t cameraIndex = 0,
        bool automaticPlayback = false);
    ~ZedSvoPointCloudSource() override;

    bool start() override;
    void stop() override;
    bool isRunning() const override;

    bool getCameraIntrinsics(
        CameraIntrinsics& intrinsics
    ) const override;


    bool getPointCloud(
        Vision3DProcessor::PointCloud& cloud) override;

    std::size_t getCameraIndex() const override
    { return cameraIndex_; }

    bool seekFrame(std::size_t index);

    struct PlaybackControl
    {
        mutable std::mutex mutex;

        bool paused = false;
        bool ready = false;
        bool finished = false;

        std::size_t currentFrame = 0;
        std::size_t totalFrames = 0;

        double fps = 30.0;

        std::optional<std::size_t> requestedFrame;
    };

    using PlaybackControlPtr =
        std::shared_ptr<PlaybackControl>;

    PlaybackControlPtr getPlaybackControl() const
    {
        return playbackControl_;
    }

    using SharedRgbFramePtr =
        ZedGmslPointCloudSource::SharedRgbFramePtr;

    SharedRgbFramePtr getSharedRgbFrame() const
    { return sharedRgbFrame_; }

    bool getCaptureTimestampMs(
        std::uint64_t& timestampMs) const override
    {
        timestampMs = frameCaptureTimestampMs_;
        return running_ && timestampMs != 0;
    }

    bool getLastPointCloudTimestampMs(
        std::uint64_t& timestampMs) const override
    {
        timestampMs = lastTimestampMs_;
        return running_ && timestampMs != 0;
    }

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

    SharedRgbFramePtr sharedRgbFrame_ =
        std::make_shared<
            ZedGmslPointCloudSource::SharedRgbFrame>();

    std::uint64_t lastTimestampMs_ = 0;

    std::uint64_t frameCaptureTimestampMs_ = 0;

    void invalidateSharedFrame();

    std::size_t cameraIndex_ = 0;
    bool automaticPlayback_ = false;
    bool frameDelivered_ = false;

    PlaybackControlPtr playbackControl_ =
        std::make_shared<PlaybackControl>();

    bool running_ = false;
    std::size_t count_ = 0;
    std::size_t index_ = 0;
    double fps_ = 30.0;
};
