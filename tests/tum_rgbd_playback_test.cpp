#include "vision/tum_rgbd_point_cloud_source.h"
#include <QApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QDir>
#include <cmath>
#include <cstdlib>
#include <iostream>

static void require(bool condition, const char* message)
{
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
static void writeFile(const QString& file, const QByteArray& contents)
{
    QFile output(file);
    require(output.open(QIODevice::WriteOnly), "open fixture");
    require(output.write(contents) == contents.size(), "write fixture");
}
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QTemporaryDir temporary;
    require(temporary.isValid(), "temporary directory");
    const QDir root(temporary.path());
    require(root.mkdir("rgb") && root.mkdir("depth"), "fixture folders");
    QImage rgb(640, 480, QImage::Format_RGB888);
    rgb.fill(Qt::green);
    require(rgb.save(root.filePath("rgb/a.png")), "save RGB PNG");
    QImage depth(640, 480, QImage::Format_Grayscale16);
    depth.fill(0);
    // 1 metre, with all four quadrants and an explicitly missing pixel.
    for (int v : {200, 280}) {
        auto* row = reinterpret_cast<quint16*>(depth.scanLine(v));
        for (int u : {280, 360}) row[u] = 5000;
    }
    require(depth.save(root.filePath("depth/a.png")), "save 16-bit PNG");
    writeFile(root.filePath("rgb.txt"), "# timestamps deliberately unsorted\n1.1 rgb/a.png\n1.0 rgb/a.png\n2.0 rgb/a.png\n");
    writeFile(root.filePath("depth.txt"), "1.116 depth/a.png\n1.016 depth/a.png\n2.1 depth/a.png\n");
    TumRgbdPointCloudSource source(temporary.path());
    require(source.start(), qPrintable(source.error()));
    require(source.frameCount() == 2, "discard pair outside 20 ms");
    require(source.points().size() == 4, "discard zero depth");
    Vision3DProcessor::Point3D point;
    require(source.pointAt(280, 200, point), "valid point");
    require(std::abs(point.z - 1000) < 0.001f, "depth scale /5 to millimetres");
    require(std::abs(point.x - ((280.0 - 319.5) * 1000 / 525)) < 0.001f, "X backprojection");
    require(std::abs(point.y - ((200.0 - 239.5) * 1000 / 525)) < 0.001f, "Y backprojection");
    require(point.image_coordinates_valid && point.image_x == 280 && point.image_y == 200, "pixel provenance");
    require(!source.pointAt(0, 0, point), "invalid depth pixel");
    require(!source.pointAt(640, 0, point), "pixel bounds");
    std::uint64_t first = 0, second = 0;
    require(source.getLastPointCloudTimestampMs(first), "monotonic capture timestamp");
    require(source.seekFrame(1), "seek second frame");
    require(source.getLastPointCloudTimestampMs(second) && second > first, "timestamp advances");
    require(std::abs(source.recordingTime(1) - 0.1) < 1e-6, "recording time separate from monotonic clock");
    require(!source.seekFrame(3) && source.frameIndex() == 1, "failed seek preserves frame");
    source.stop();
    Vision3DProcessor::PointCloud cloud;
    require(!source.getPointCloud(cloud), "stopped source");
    require(!source.getLastPointCloudTimestampMs(second) && second == 0, "no stopped timestamp");
    // An 8-bit depth PNG must never silently turn into metric depth.
    QImage wrong(640, 480, QImage::Format_Grayscale8);
    wrong.fill(200);
    require(wrong.save(root.filePath("depth/a.png")), "save invalid depth");
    require(!source.start(), "reject 8-bit depth");
    writeFile(root.filePath("rgb.txt"), "not-a-time rgb/a.png\n");
    require(!source.start(), "reject malformed timestamp");
    std::cout << "OK: TUM pairing, PNG16, XYZ, pixels, timestamps, seek and invalid input\n";
}
