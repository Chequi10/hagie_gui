#include "vision/rgbd_reference_plane.h"
#include "vision/vision_3d_processor.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

static void check(bool condition, const char* message)
{
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main()
{
    using Vector = RgbdReferencePlane::Vector;
    const Vector a {-75.9, 281.5, 658.4};
    const Vector b {73.6, 230.6, 849.4};
    const Vector c {-377.1, 289.3, 823.2};
    RgbdReferencePlane plane;
    check(!plane.isValid(), "no plane initially");
    check(std::isnan(plane.heightMm(a)), "no height before reference");
    check(plane.fit(a, b, c), "fit the user's table points");
    for (const auto& p : {a, b, c})
        check(std::abs(plane.heightMm(p)) < 1e-8, "reference points lie on plane");
    const Vector leaf {-123.5, -145.7, 645.4};
    check(std::abs(plane.heightMm(leaf) - 425.85964) < 0.001, "known top-leaf distance, mm");
    check(std::abs(plane.heightMm({-136.4, -68.5, 672.4}) - 347.27279) < 0.001,
          "known previous leaf distance, mm");
    check(std::abs(plane.machinePoint(leaf)[2] - plane.heightMm(leaf)) < 1e-8,
          "HAGIE vertical equals signed distance");
    RgbdReferencePlane reversed;
    check(reversed.fit(c, b, a), "reverse order accepted");
    check(std::abs(reversed.heightMm(leaf) - plane.heightMm(leaf)) < 1e-8,
          "point ordering must not invert the height");
    // Rotation preserves physical distances in the local machine coordinates.
    const auto ma = plane.machinePoint(a), mb = plane.machinePoint(b);
    double original = 0, local = 0;
    for (int i = 0; i < 3; ++i) {
        original += (a[i] - b[i]) * (a[i] - b[i]);
        local += (ma[i] - mb[i]) * (ma[i] - mb[i]);
    }
    check(std::abs(original - local) < 1e-6, "orthonormal local coordinates");

    check(plane.fit({-200, 300, 600}, {200, 300, 600}, {-200, 300, 1000}), "level reference");
    Vision3DProcessor processor;
    Vision3DProcessor::CameraConfig config;
    config.enabled = true;
    config.body_enabled[0] = true;
    processor.setCameraConfig(0, config);
    Vision3DProcessor::TemporalFilterConfig filter;
    filter.enabled = false;
    processor.setTemporalFilterConfig(filter);
    Vision3DProcessor::BodyRegion region;
    region.min_x = -20000; region.max_x = 20000;
    region.min_y = -20000; region.max_y = 20000;
    region.min_z = 0; region.max_z = 10000;
    region.min_points = 30;
    processor.setBodyRegion(0, region);
    Vision3DProcessor::PointCloud cloud;
    for (int i = 0; i < 100; ++i) {
        const auto p = plane.machinePoint({0, 300.0 - (400 + i), 800});
        Vision3DProcessor::Point3D point;
        point.x = float(p[0]); point.y = float(p[1]); point.z = float(p[2]);
        cloud.push_back(point);
    }
    const auto result = processor.processPointCloud(0, cloud);
    check(result.bodies[0].valid && result.bodies[0].height_mm == 494,
          "original HAGIE processor calculates P95 after plane conversion");
    cloud.resize(29);
    check(!processor.processPointCloud(0, cloud).bodies[0].valid, "minimum valid point count");
    check(!plane.fit(a, a, c) && !plane.isValid(), "duplicate points invalidate old reference");
    check(!plane.fit({0,300,600}, {100,300,600}, {200,300,600}), "reject collinear points");
    check(!plane.fit({0,300,600}, {100,300,600}, {200,300,601}), "reject nearly collinear points");
    check(!plane.fit({0,0,600}, {100,0,600}, {0,100,600}), "reject vertical wall");
    plane.clear();
    check(!plane.isValid() && std::isnan(plane.heightMm(leaf)), "explicit reset clears height");
    std::cout << "OK: reference plane, 426 mm capture, point order, local axes, P95, invalid points and reset\n";
}
