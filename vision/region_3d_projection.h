#pragma once

#include "vision/point_cloud_source.h"

#include <QLineF>
#include <QPointF>

#include <array>
#include <vector>
#include <cmath>

namespace Region3DProjection
{

inline std::vector<QLineF> project(
    const std::array<Vision3DProcessor::Point3D, 8>& vertices,
    const PointCloudSource::CameraIntrinsics& intrinsics,
    int imageWidth,
    int imageHeight)
{
    std::vector<QLineF> lines;

    if (!intrinsics.valid ||
        intrinsics.width <= 0 ||
        intrinsics.height <= 0 ||
        imageWidth <= 0 ||
        imageHeight <= 0)
        return lines;

    const double sx =
        double(imageWidth) / intrinsics.width;

    const double sy =
        double(imageHeight) / intrinsics.height;

    constexpr double nearPlane = 50.0;

    constexpr int edges[12][2] = {
        {0,1}, {2,3}, {4,5}, {6,7},
        {0,2}, {1,3}, {4,6}, {5,7},
        {0,4}, {1,5}, {2,6}, {3,7}
    };

    auto clipNear = [&](auto& a, const auto& b)
    {
        const double t =
            (nearPlane - a.y) / (b.y - a.y);

        a.x += t * (b.x - a.x);
        a.z += t * (b.z - a.z);
        a.y = nearPlane;
    };

    auto pixel = [&](const auto& p)
    {
        return QPointF(
            (intrinsics.cx -
             intrinsics.fx * p.x / p.y) * sx,

            (intrinsics.cy -
             intrinsics.fy * p.z / p.y) * sy
        );
    };

    for (const auto& edge : edges)
    {
        auto a = vertices[edge[0]];
        auto b = vertices[edge[1]];

        if (a.y < nearPlane &&
            b.y < nearPlane)
            continue;

        if (a.y < nearPlane)
            clipNear(a, b);

        if (b.y < nearPlane)
            clipNear(b, a);

        const auto pa = pixel(a);
        const auto pb = pixel(b);

        if (!std::isfinite(pa.x()) ||
            !std::isfinite(pa.y()) ||
            !std::isfinite(pb.x()) ||
            !std::isfinite(pb.y()))
            continue;

        lines.emplace_back(pa, pb);
    }

    return lines;
}

}
