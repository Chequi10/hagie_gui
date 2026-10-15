#include "vision/video_rgb_frame_source.h"
#include "vision/rgb_camera_worker.h"
#include "vision/simulated_rgb_frame_source.h"
#include <cassert>
int main() {
    auto cache = std::make_shared<VideoRgbFrameSource::Cache>();
    RgbCameraWorker worker;
    worker.setFrameSource(0, std::make_unique<VideoRgbFrameSource>(0, cache));
    worker.start();
    RgbFrameSource::Frame frame;
    assert(!worker.getFrame(0, frame));
    frame.width = 2; frame.height = 1; frame.data = {1,2,3,4,5,6};
    frame.valid = true; frame.timestamp_ms = 123;
    cache->publish(frame);
    frame.data[0] = 99;
    RgbFrameSource::Frame received;
    assert(worker.getFrame(0, received));
    assert(received.data[0] == 1 && received.timestamp_ms == 123 && received.rgb_only);
    assert(!cache->isConsumed());
    worker.frameProcessed(0, 123);
    assert(cache->isConsumed());
    worker.setFrameSource(0, std::make_unique<SimulatedRgbFrameSource>(0));
    assert(worker.getFrame(0, received));
    assert(received.valid && received.width > 2 && !received.rgb_only);
    worker.stop();
    assert(!worker.getFrame(0, received));
}
