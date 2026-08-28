#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>


class RgbFrameSource
{
public:

    struct Frame
    {
        std::size_t width = 0;

        std::size_t height = 0;

        std::vector<std::uint8_t> data;

        std::uint64_t timestamp_ms = 0;

        bool valid = false;
    };


    virtual ~RgbFrameSource() = default;


    virtual bool start() = 0;

    virtual void stop() = 0;

    virtual bool isRunning() const = 0;


    virtual bool getFrame(
        Frame& frame
    ) = 0;


    virtual std::size_t getCameraIndex() const = 0;
};