#include "ai/onnx_tassel_detector.h"
#include <QByteArray>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <algorithm>
#include <cmath>
#include <limits>

struct OnnxTasselDetector::Impl {
    QProcess process;
    QByteArray pending;
    QByteArray diagnostics;
    std::string lastError;
    double milliseconds = 0;
    bool receive(QJsonObject& packet, const std::atomic<bool>& running) {
        QElapsedTimer timer;
        timer.start();
        while (running.load() && timer.elapsed() < 45000) {
            pending += process.readAllStandardOutput();
            diagnostics += process.readAllStandardError();
            diagnostics = diagnostics.right(6000);
            const int newline = pending.indexOf('\n');
            if (newline >= 0) {
                const auto line = pending.left(newline);
                pending.remove(0, newline + 1);
                packet = QJsonDocument::fromJson(line).object();
                if (packet.isEmpty()) { lastError = "Respuesta ONNX inválida."; return false; }
                if (packet.value("type").toString() == "error") {
                    lastError = packet.value("message").toString().toStdString();
                    return false;
                }
                return true;
            }
            if (pending.size() > 128 * 1024 * 1024) break;
            if (process.state() == QProcess::NotRunning) {
                lastError = "Python ONNX finalizó: " + diagnostics.toStdString();
                return false;
            }
            process.waitForReadyRead(100);
        }
        lastError = running.load() ? "Tiempo de espera ONNX agotado." : "Inferencia detenida.";
        return false;
    }
};

OnnxTasselDetector::OnnxTasselDetector() : impl(std::make_unique<Impl>()) {}
OnnxTasselDetector::~OnnxTasselDetector() {
    if (impl->process.state() != QProcess::NotRunning) {
        impl->process.kill();
        impl->process.waitForFinished(1000);
    }
}
bool OnnxTasselDetector::initialize(const std::string& python, const std::string& helper,
                                   const std::string& model, const std::atomic<bool>& running) {
    const auto interpreter = QString::fromStdString(python);
    if (!QFileInfo(interpreter).isExecutable() ||
        !QFileInfo(QString::fromStdString(helper)).isFile() ||
        !QFileInfo(QString::fromStdString(model)).isFile()) {
        impl->lastError = "Revisá el modelo ONNX y el Python en DETECTOR DE PANOJAS.";
        return false;
    }
    impl->process.start(interpreter, {"-u", QString::fromStdString(helper),
                                     "--model", QString::fromStdString(model)});
    if (!impl->process.waitForStarted(1000)) {
        impl->lastError = impl->process.errorString().toStdString();
        return false;
    }
    QJsonObject ready;
    if (!impl->receive(ready, running)) return false;
    if (ready.value("type").toString() != "ready") {
        impl->lastError = "ONNX no confirmó la carga del modelo.";
        return false;
    }
    return true;
}
bool OnnxTasselDetector::processFrame(std::size_t camera, const RgbFrameSource::Frame& frame,
                                     TasselDetector::Result& result, float confidence, float iou,
                                     const std::atomic<bool>& running) {
    result = {};
    if (!frame.valid || frame.width == 0 || frame.height == 0 ||
        frame.width > 16384 || frame.height > 16384 ||
        frame.data.size() != frame.width * frame.height * 3 ||
        frame.data.size() > 96 * 1024 * 1024) {
        impl->lastError = "Fotograma RGB inválido para ONNX.";
        return false;
    }
    const QByteArray bytes(reinterpret_cast<const char*>(frame.data.data()),
                           static_cast<qsizetype>(frame.data.size()));
    QJsonObject request {
        {"width", static_cast<int>(frame.width)}, {"height", static_cast<int>(frame.height)},
        {"timestamp_ms", QString::number(static_cast<qulonglong>(frame.timestamp_ms))},
        {"confidence", static_cast<double>(confidence)}, {"iou", static_cast<double>(iou)},
        {"rgb", QString::fromLatin1(bytes.toBase64())}
    };
    impl->process.write(QJsonDocument(request).toJson(QJsonDocument::Compact) + '\n');
    QElapsedTimer writing;
    writing.start();
    while (impl->process.bytesToWrite() && running.load() && writing.elapsed() < 15000)
        impl->process.waitForBytesWritten(100);
    if (!running.load() || impl->process.bytesToWrite()) {
        impl->lastError = "No se pudo enviar el RGB al motor ONNX.";
        return false;
    }
    QJsonObject reply;
    if (!impl->receive(reply, running)) return false;
    if (reply.value("type").toString() != "result" ||
        reply.value("timestamp_ms").toString().toULongLong() != frame.timestamp_ms) {
        impl->lastError = "El resultado ONNX no corresponde al fotograma enviado.";
        return false;
    }
    result.camera_index = camera;
    result.timestamp_ms = frame.timestamp_ms;
    result.image_width = frame.width;
    result.image_height = frame.height;
    for (const auto& value : reply.value("boxes").toArray()) {
        const auto box = value.toArray();
        if (box.size() != 5) continue;
        bool finite = true;
        for (const auto& coordinate : box) finite = finite && std::isfinite(coordinate.toDouble());
        if (!finite) continue;
        TasselDetector::Detection detection;
        const auto clampX = [&](double x) { return std::clamp(x, 0.0, static_cast<double>(frame.width)); };
        const auto clampY = [&](double y) { return std::clamp(y, 0.0, static_cast<double>(frame.height)); };
        detection.x = static_cast<int>(std::floor(clampX(box[0].toDouble())));
        detection.y = static_cast<int>(std::floor(clampY(box[1].toDouble())));
        detection.width = static_cast<int>(std::ceil(clampX(box[2].toDouble()))) - detection.x;
        detection.height = static_cast<int>(std::ceil(clampY(box[3].toDouble()))) - detection.y;
        if (detection.width <= 0 || detection.height <= 0) continue;
        detection.confidence = static_cast<float>(box[4].toDouble());
        detection.body_index = TasselDetector::bodyFromImagePosition(camera,
            detection.x + detection.width / 2, frame.width);
        result.detections.push_back(detection);
    }
    impl->milliseconds = reply.value("inference_ms").toDouble();
    result.valid = true;
    return true;
}
std::string OnnxTasselDetector::error() const { return impl->lastError; }
double OnnxTasselDetector::inferenceMs() const { return impl->milliseconds; }
