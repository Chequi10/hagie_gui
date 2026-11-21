#include "vision/tum_rgbd_point_cloud_source.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QTextStream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <utility>

namespace {
// TUM recomienda estos intrinsecos para sus PNG de profundidad registrados.
constexpr float FX = 525.0f, FY = 525.0f, CX = 319.5f, CY = 239.5f;
Vision3DProcessor::Point3D reconstruct(int u, int v, quint16 raw)
{
    Vision3DProcessor::Point3D point;
    point.z = static_cast<float>(raw) / 5.0f;
    point.x = (static_cast<float>(u) - CX) * point.z / FX;
    point.y = (static_cast<float>(v) - CY) * point.z / FY;
    point.image_x = u;
    point.image_y = v;
    point.image_coordinates_valid = true;
    return point;
}
}

TumRgbdPointCloudSource::TumRgbdPointCloudSource(QString directory, std::size_t camera)
    : directory_(QDir(directory).absolutePath()), camera_(camera) {}

bool TumRgbdPointCloudSource::readList(const QString& file, std::vector<Entry>& entries)
{
    QFile input(QDir(directory_).filePath(file));
    if (!input.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error_ = "No se pudo abrir " + input.fileName();
        return false;
    }
    QTextStream stream(&input);
    const QString root = QFileInfo(directory_).canonicalFilePath() + '/';
    while (!stream.atEnd()) {
        const auto line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) continue;
        const auto fields = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        bool ok = false;
        const double time = fields.value(0).toDouble(&ok);
        if (fields.size() < 2 || !ok || !std::isfinite(time)) {
            error_ = "Lista de tiempos invalida: " + file;
            return false;
        }
        const auto path = QFileInfo(QDir(directory_).filePath(fields[1])).canonicalFilePath();
        if (path.isEmpty() || !path.startsWith(root) || !QFileInfo(path).isFile()) {
            error_ = "Imagen ausente o fuera de la carpeta: " + fields[1];
            return false;
        }
        entries.push_back({time, path});
    }
    std::stable_sort(entries.begin(), entries.end(),
        [](const Entry& a, const Entry& b) { return a.time < b.time; });
    if (entries.empty()) {
        error_ = "Lista de imagenes vacia: " + file;
        return false;
    }
    return true;
}

bool TumRgbdPointCloudSource::start()
{
    if (running_) return true;
    error_.clear();
    pairs_.clear();
    cloud_.clear();
    rgb_ = QImage();
    depth_ = QImage();
    std::vector<Entry> colors, depths;
    if (camera_ >= Vision3DProcessor::CAMERA_COUNT) {
        error_ = "Indice de camara invalido";
        return false;
    }
    if (!readList("rgb.txt", colors) || !readList("depth.txt", depths)) return false;
    for (const auto& color : colors) {
        auto it = std::lower_bound(depths.begin(), depths.end(), color.time,
            [](const Entry& entry, double time) { return entry.time < time; });
        auto best = it == depths.end() ? std::prev(it) : it;
        if (it != depths.begin()) {
            auto before = std::prev(it);
            if (std::abs(before->time - color.time) <= std::abs(best->time - color.time))
                best = before;
        }
        if (std::abs(best->time - color.time) <= 0.020 + 1e-9)
            pairs_.push_back({color, *best});
    }
    if (pairs_.empty()) {
        error_ = "No hay pares RGB/profundidad con diferencia de hasta 20 ms";
        return false;
    }
    running_ = true;
    if (!seekFrame(0)) { running_ = false; return false; }
    return true;
}

bool TumRgbdPointCloudSource::seekFrame(std::size_t index)
{
    if (!running_ || index >= pairs_.size()) {
        error_ = "Cuadro fuera de rango o fuente detenida";
        return false;
    }
    QImage color(pairs_[index].rgb.file), depth(pairs_[index].depth.file);
    if (color.isNull() || depth.isNull()) {
        error_ = "No se pudo decodificar el par RGB/profundidad";
        return false;
    }
    // No convertir una profundidad de 8 bits: destruiria la escala metrica.
    if (depth.format() != QImage::Format_Grayscale16 ||
        depth.size() != QSize(640, 480) || color.size() != depth.size()) {
        error_ = "Esta prueba requiere RGB y profundidad PNG gris de 16 bits, 640 x 480";
        return false;
    }
    Vision3DProcessor::PointCloud points;
    points.reserve(640 * 480);
    for (int v = 0; v < depth.height(); ++v) {
        const auto* row = reinterpret_cast<const quint16*>(depth.constScanLine(v));
        for (int u = 0; u < depth.width(); ++u) {
            if (row[u] != 0) points.push_back(reconstruct(u, v, row[u]));
        }
    }
    if (points.empty()) {
        error_ = "El cuadro no contiene profundidad valida";
        return false;
    }
    rgb_ = color.convertToFormat(QImage::Format_RGB888);
    depth_ = std::move(depth);
    cloud_ = std::move(points);
    index_ = index;
    const auto now = static_cast<std::uint64_t>(std::chrono::duration_cast<
        std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    timestamp_ = std::max(now, timestamp_ + 1);
    error_.clear();
    return true;
}

bool TumRgbdPointCloudSource::getPointCloud(Vision3DProcessor::PointCloud& cloud)
{
    if (!running_) { cloud.clear(); return false; }
    cloud = cloud_;
    return !cloud.empty();
}

bool TumRgbdPointCloudSource::getLastPointCloudTimestampMs(std::uint64_t& timestamp) const
{
    timestamp = running_ ? timestamp_ : 0;
    return timestamp != 0;
}

double TumRgbdPointCloudSource::recordingTime(std::size_t index) const
{
    return index < pairs_.size() ? pairs_[index].rgb.time - pairs_.front().rgb.time : 0.0;
}

double TumRgbdPointCloudSource::pairDeltaMs() const
{
    if (pairs_.empty()) return 0.0;
    return std::abs(pairs_[index_].rgb.time - pairs_[index_].depth.time) * 1000.0;
}

bool TumRgbdPointCloudSource::pointAt(int u, int v, Vision3DProcessor::Point3D& point) const
{
    point = {};
    if (!running_ || u < 0 || v < 0 || u >= depth_.width() || v >= depth_.height()) return false;
    const auto* row = reinterpret_cast<const quint16*>(depth_.constScanLine(v));
    if (row[u] == 0) return false;
    point = reconstruct(u, v, row[u]);
    return true;
}
