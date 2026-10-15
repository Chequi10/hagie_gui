#include "gui/video_rgb_controls.h"
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QSettings>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <chrono>

namespace {
QString settingsPath() { return QDir::homePath() + "/.config/hagie_gui/hagie_config.ini"; }
}
VideoRgbControls::VideoRgbControls(QWidget* parent)
    : QWidget(parent), process(new QProcess(this)), killTimer(new QTimer(this)), poll(new QTimer(this)) {
    auto* layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("FUENTE RGB DE VIDEO — SIMULACIÓN CÁMARAS 3D", this));
    auto* description = new QLabel("El video reemplaza el RGB de la cámara elegida. "
        "Usa el detector y su configuración comunes. La nube continúa simulada.", this);
    description->setWordWrap(true);
    layout->addWidget(description);
    auto* row = new QHBoxLayout();
    row->addWidget(new QLabel("Cámara RGB:", this));
    camera = new QComboBox(this);
    for (int index = 0; index < 7; ++index) camera->addItem(QString("CÁMARA %1").arg(index + 1), index);
    row->addWidget(camera);
    row->addWidget(new QLabel("Rotación:", this));
    rotation = new QComboBox(this);
    rotation->addItem("0°", 0); rotation->addItem("90° horario", 90);
    rotation->addItem("180°", 180); rotation->addItem("270° horario (90° antihorario)", 270);
    row->addWidget(rotation); row->addStretch(); layout->addLayout(row);
    row = new QHBoxLayout();
    row->addWidget(new QLabel("Video RGB:", this));
    videoPath = new QLineEdit(this); row->addWidget(videoPath, 1);
    browse = new QPushButton("Elegir…", this); row->addWidget(browse); layout->addLayout(row);
    connect(browse, &QPushButton::clicked, this, [this]() {
        const auto path = QFileDialog::getOpenFileName(this, "Video RGB", videoPath->text(),
            "Videos (*.mp4 *.avi *.mov *.mkv *.m4v);;Todos (*)");
        if (!path.isEmpty()) videoPath->setText(path);
    });
    row = new QHBoxLayout();
    startButton = new QPushButton("INICIAR VIDEO", this); stopButton = new QPushButton("DETENER", this);
    row->addWidget(startButton); row->addWidget(stopButton); row->addStretch(); layout->addLayout(row);
    status = new QLabel("Seleccioná un video; detección en la página CÁMARAS.", this);
    status->setWordWrap(true); layout->addWidget(status);
    QSettings settings(settingsPath(), QSettings::IniFormat);
    settings.beginGroup("RgbVideoSimulation");
    camera->setCurrentIndex(std::clamp(settings.value("camera", 0).toInt(), 0, 6));
    const int rotateIndex = rotation->findData(settings.value("rotation", 0).toInt());
    rotation->setCurrentIndex(rotateIndex < 0 ? 0 : rotateIndex);
    videoPath->setText(settings.value("video_path").toString());
    setRunning(false);
    killTimer->setSingleShot(true);
    connect(killTimer, &QTimer::timeout, this, [this]() { if (process->state() != QProcess::NotRunning) process->kill(); });
    connect(startButton, &QPushButton::clicked, this, [this]() { startVideo(); });
    connect(stopButton, &QPushButton::clicked, this, [this]() { stopVideo(); });
    connect(process, &QProcess::readyReadStandardOutput, this, [this]() { consumeOutput(); });
    connect(process, &QProcess::readyReadStandardError, this, [this]() {
        diagnostics += process->readAllStandardError(); diagnostics = diagnostics.right(6000);
    });
    connect(process, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
            terminalMessage = true; status->setText("No se pudo abrir la fuente: " + process->errorString());
            setRunning(false);
        }
    });
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
        [this](int code, QProcess::ExitStatus exit) {
            killTimer->stop(); consumeOutput();
            if (!terminalMessage) status->setText(stopping ? "Video detenido; RGB simulado restaurado." :
                code == 0 && exit == QProcess::NormalExit ? "Video finalizado; RGB simulado restaurado." :
                "Error de lectura del video: " + QString::fromUtf8(diagnostics));
            setRunning(false);
        });
    poll->setInterval(20);
    connect(poll, &QTimer::timeout, this, [this]() {
        if (stopping) return;
        const QString error = detectorError ? detectorError() : QString();
        if (!error.isEmpty()) {
            terminalMessage = true; status->setText("Detector: " + error); stopVideo(); return;
        }
        if (awaitingConsumption && frameConsumed && frameConsumed()) {
            awaitingConsumption = false; process->write("next\n");
        }
    });
}
VideoRgbControls::~VideoRgbControls() {
    process->disconnect(this);
    if (process->state() != QProcess::NotRunning) { process->kill(); process->waitForFinished(1000); }
}
void VideoRgbControls::saveSettings() const {
    QSettings settings(settingsPath(), QSettings::IniFormat);
    settings.beginGroup("RgbVideoSimulation");
    settings.setValue("camera", camera->currentIndex());
    settings.setValue("rotation", rotation->currentData().toInt());
    settings.setValue("video_path", videoPath->text());
    // Detector options now belong solely to Vision/DETECTOR DE PANOJAS.
    for (const auto* key : {"model_path", "python_path", "confidence", "nms_iou"}) settings.remove(key);
}
void VideoRgbControls::setRunning(bool running) {
    if (running) poll->start(); else {
        poll->stop(); awaitingConsumption = false; if (onStopped) onStopped();
    }
    camera->setEnabled(!running); rotation->setEnabled(!running); videoPath->setEnabled(!running);
    browse->setEnabled(!running); startButton->setEnabled(!running); stopButton->setEnabled(running);
}
void VideoRgbControls::startVideo() {
    if (process->state() != QProcess::NotRunning) return;
    const QString python = pythonPath ? pythonPath() : QString();
    const QString helper = QCoreApplication::applicationDirPath() + "/video_rgb_source.py";
    if (!QFileInfo(videoPath->text()).isFile() || !QFileInfo(helper).isFile()) {
        status->setText("Revisá el video seleccionado y volvé a configurar con CMake."); return;
    }
    if (!QFileInfo(python).isExecutable()) {
        status->setText("Revisá Python CPU en DETECTOR DE PANOJAS."); return;
    }
    if (!onStarted || !onStarted(static_cast<std::size_t>(camera->currentData().toInt()))) {
        status->setText("No se pudo iniciar el detector; revisá su configuración."); return;
    }
    saveSettings(); pending.clear(); diagnostics.clear(); stopping = false; terminalMessage = false;
    awaitingConsumption = false;
    status->setText("Abriendo fuente RGB y detector…"); setRunning(true);
    process->start(python, {"-u", helper, "--video", videoPath->text(),
                           "--rotation", QString::number(rotation->currentData().toInt())});
}
void VideoRgbControls::stopVideo() {
    stopping = true; poll->stop(); awaitingConsumption = false;
    if (onStopped) onStopped();
    if (process->state() == QProcess::NotRunning) return;
    stopButton->setEnabled(false); process->terminate(); killTimer->start(1500);
}
void VideoRgbControls::consumeOutput() {
    pending += process->readAllStandardOutput();
    int newline;
    while ((newline = pending.indexOf('\n')) >= 0) {
        const QByteArray line = pending.left(newline); pending.remove(0, newline + 1);
        const auto object = QJsonDocument::fromJson(line).object();
        const QString type = object.value("type").toString();
        if (type == "frame" && !stopping) {
            const int width = object.value("width").toInt(), height = object.value("height").toInt();
            const QByteArray rgb = QByteArray::fromBase64(object.value("rgb").toString().toLatin1());
            if (width <= 0 || height <= 0 || width > 16384 || height > 16384 ||
                rgb.size() != static_cast<qint64>(width) * height * 3 || rgb.size() > 96 * 1024 * 1024) {
                terminalMessage = true; status->setText("Fotograma RGB inválido."); stopVideo(); return;
            }
            RgbFrameSource::Frame frame;
            frame.width = width; frame.height = height;
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(rgb.constData());
            frame.data.assign(bytes, bytes + rgb.size());
            const auto now = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
            frame.timestamp_ms = lastTimestamp = std::max(now, lastTimestamp + 1);
            frame.valid = true; frame.rgb_only = true;
            awaitingConsumption = true;
            if (onFrame) onFrame(frame);
            status->setText(QString("Video: %1 s | Fotograma %2 | Resultado en CÁMARAS")
                .arg(object.value("time_s").toDouble(), 0, 'f', 2).arg(object.value("frame").toInt()));
        } else if (type == "finished") {
            terminalMessage = true; status->setText("Video finalizado; RGB simulado restaurado.");
        } else if (type == "error") {
            terminalMessage = true; status->setText("Fuente RGB: " + object.value("message").toString());
        }
    }
    if (pending.size() > 128 * 1024 * 1024) {
        terminalMessage = true; status->setText("Fotograma demasiado grande."); pending.clear(); stopVideo();
    }
}
