#pragma once
#include <QByteArray>
#include <QWidget>
#include <functional>
#include "vision/rgb_frame_source.h"
class QComboBox;
class QLabel;
class QLineEdit;
class QProcess;
class QPushButton;
class QTimer;

// Acquisition controls only; detection belongs to YoloInferenceWorker.
class VideoRgbControls : public QWidget {
public:
    explicit VideoRgbControls(QWidget* parent = nullptr);
    ~VideoRgbControls() override;
    void saveSettings() const;
    void stopVideo();
    std::function<bool(std::size_t)> onStarted;
    std::function<void(const RgbFrameSource::Frame&)> onFrame;
    std::function<void()> onStopped;
    std::function<bool()> frameConsumed;
    std::function<QString()> detectorError;
    std::function<QString()> pythonPath;
private:
    void startVideo();
    void consumeOutput();
    void setRunning(bool running);
    QComboBox *camera = nullptr, *rotation = nullptr;
    QLineEdit* videoPath = nullptr;
    QLabel* status = nullptr;
    QPushButton *startButton = nullptr, *stopButton = nullptr, *browse = nullptr;
    QProcess* process = nullptr;
    QTimer *killTimer = nullptr, *poll = nullptr;
    QByteArray pending, diagnostics;
    bool stopping = false, terminalMessage = false, awaitingConsumption = false;
    std::uint64_t lastTimestamp = 0;
};
