#pragma once
#include "vision/rgb_frame_source.h"
#include <memory>
#include <atomic>
#include <mutex>

// An RGB-only source: no synthetic depth is associated with these frames.
class VideoRgbFrameSource : public RgbFrameSource
{
public:
    struct Cache {
        mutable std::mutex mutex;
        Frame latest;
        std::uint64_t consumedTimestamp = 0;
        void publish(const Frame& frame) {
            std::lock_guard<std::mutex> lock(mutex);
            latest = frame;
            latest.rgb_only = true;
        }
        bool isConsumed() const {
            std::lock_guard<std::mutex> lock(mutex);
            return latest.valid && consumedTimestamp == latest.timestamp_ms;
        }
        void acknowledge(std::uint64_t timestamp) {
            std::lock_guard<std::mutex> lock(mutex);
            consumedTimestamp = timestamp;
        }
        bool read(Frame& frame) const {
            std::lock_guard<std::mutex> lock(mutex);
            frame = latest;
            return frame.valid;
        }
    };
    VideoRgbFrameSource(std::size_t camera, std::shared_ptr<Cache> cache)
        : camera(camera), cache(std::move(cache)) {}
    bool start() override { running = true; return true; }
    void stop() override { running = false; }
    bool isRunning() const override { return running; }
    bool getFrame(Frame& frame) override {
        if (!running) { frame = Frame{}; return false; }
        return cache->read(frame);
    }
    std::size_t getCameraIndex() const override { return camera; }
    void frameProcessed(std::uint64_t timestamp) override { cache->acknowledge(timestamp); }
private:
    std::size_t camera;
    std::shared_ptr<Cache> cache;
    std::atomic<bool> running {false};
};
