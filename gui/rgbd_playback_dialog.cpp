#include <array>
#include "gui/rgbd_playback_dialog.h"
#include "vision/zed_svo_point_cloud_source.h"

#include <QApplication>
#include <QColor>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScreen>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace {
class RgbdCanvas : public QWidget
{
public:
    explicit RgbdCanvas(QWidget* parent) : QWidget(parent)
    {
        setMinimumSize(480, 180);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }
    QImage image;
    QRect roi;
    QPoint selected {-1, -1};
    std::function<void(QPoint)> picked;
    std::function<void(QRect)> regionChanged;

protected:
    QRectF displayRect() const
    {
        if (image.isNull()) return {};
        const QSizeF size = QSizeF(image.size()).scaled(QSizeF(this->size()), Qt::KeepAspectRatio);
        return QRectF(QPointF((width() - size.width()) / 2,
                            (height() - size.height()) / 2), size);
    }
    QPoint imagePoint(const QPointF& position) const
    {
        const auto rect = displayRect();
        if (rect.isEmpty() || !rect.contains(position)) return {-1, -1};
        return {std::clamp(int((position.x() - rect.x()) * image.width() / rect.width()), 0, image.width() - 1),
                std::clamp(int((position.y() - rect.y()) * image.height() / rect.height()), 0, image.height() - 1)};
    }
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(22, 25, 30));
        if (image.isNull()) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter, "Abrí una grabación ZED SVO2");
            return;
        }
        painter.drawImage(displayRect(), image);
        painter.save();
        painter.translate(displayRect().topLeft());
        painter.scale(displayRect().width() / image.width(), displayRect().height() / image.height());
        painter.setPen(QPen(Qt::yellow, 2));
        for (int offset : {0, image.width() / 2}) {
            painter.drawRect(roi.translated(offset, 0));
            if (selected.x() >= 0) {
                const QPoint p = selected + QPoint(offset, 0);
                painter.setPen(QPen(Qt::white, 2));
                painter.drawLine(p + QPoint(-8, 0), p + QPoint(8, 0));
                painter.drawLine(p + QPoint(0, -8), p + QPoint(0, 8));
                painter.setPen(QPen(Qt::yellow, 2));
            }
        }
        painter.restore();
    }
    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton) press_ = imagePoint(event->position());
    }
    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (event->button() != Qt::LeftButton || press_.x() < 0) return;
        if (image.isNull()) return;
        const int w = image.width() / 2;
        QPoint finish = imagePoint(event->position());
        if (finish.x() < 0 || finish.x() / w != press_.x() / w) { press_ = {-1, -1}; return; }
        QPoint first(press_.x() % w, press_.y());
        finish.setX(finish.x() % w);
        if ((finish - first).manhattanLength() < 6) {
            selected = finish;
            if (picked) picked(selected);
        } else {
            roi = QRect(first, finish).normalized().intersected(QRect(0, 0, w, image.height()));
            if (regionChanged) regionChanged(roi);
        }
        press_ = {-1, -1};
        update();
    }
private:
    QPoint press_ {-1, -1};
};

class RgbdPlaybackDialog final : public QDialog
{
public:
    explicit RgbdPlaybackDialog(QWidget* parent) : QDialog(parent)
    {
        setWindowTitle("Prueba de grabación ZED SVO2 — HAGIE");
        resize(1160, 700);
        if (auto* screen = QApplication::primaryScreen())
            resize(size().boundedTo(screen->availableGeometry().size() - QSize(60, 60)));
        auto* layout = new QVBoxLayout(this);
        auto* intro = new QLabel("Grabación de prueba: RGB y XYZ reales. No requiere STM32. "
            "Los resultados se muestran en esta ventana y no se envían al control hidráulico.", this);
        intro->setWordWrap(true);
        layout->addWidget(intro);

        auto* controls = new QHBoxLayout;
        auto* open = new QPushButton("Abrir grabación SVO2…", this);
        play_ = new QPushButton("Reproducir", this);
        auto* previous = new QPushButton("Anterior", this);
        auto* next = new QPushButton("Siguiente", this);
        auto* all = new QPushButton("Región completa", this);
        controls->addWidget(open);
        controls->addWidget(play_);
        controls->addWidget(previous);
        controls->addWidget(next);
        controls->addWidget(all);
        controls->addStretch();
        layout->addLayout(controls);
        status_ = new QLabel("Seleccioná una grabación ZED SVO2.", this);
        status_->setWordWrap(true);
        layout->addWidget(status_);
        canvas_ = new RgbdCanvas(this);
        layout->addWidget(canvas_, 1);
        coordinates_ = new QLabel("Clic: XYZ óptico en mm. Arrastrá sobre el cultivo para seleccionar la región.", this);
        coordinates_->setWordWrap(true);
        layout->addWidget(coordinates_);
        slider_ = new QSlider(Qt::Horizontal, this);
        slider_->setEnabled(false);
        layout->addWidget(slider_);

        auto* settings = new QHBoxLayout;
        height_ = new QDoubleSpinBox(this);
        height_->setRange(0, 5000);
        height_->setSuffix(" mm");
        height_->setToolTip("Altura conocida del centro óptico sobre el suelo o plano de referencia.");
        maximum_ = new QSpinBox(this);
        maximum_->setRange(100, 12000);
        maximum_->setValue(2000);
        maximum_->setSuffix(" mm");
        settings->addWidget(new QLabel("Altura de montaje", this));
        settings->addWidget(height_);
        settings->addWidget(new QLabel("Profundidad máxima", this));
        settings->addWidget(maximum_);
        layout->addLayout(settings);
        imuStatus_ = new QLabel("IMU ZED: sin datos", this);
        layout->addWidget(imuStatus_);

        result_ = new QLabel(this);
        result_->setStyleSheet(
            "font-size: 18px; font-weight: bold;");
        result_->setWordWrap(true);
        layout->addWidget(result_);
        auto* note = new QLabel(
            "Arrastrá sobre la imagen para seleccionar la región. "
            "Clic simple: consultar XYZ. Negro: sin profundidad.", this);
        note->setWordWrap(true);
        layout->addWidget(note);

        timer_.setSingleShot(true);
        connect(&timer_, &QTimer::timeout, this, [this] {
            if (!source_) return;
            if (source_->frameIndex() + 1 >= source_->frameCount()) { pause(); return; }
            QElapsedTimer elapsed;
            elapsed.start();

            slider_->setValue(int(source_->frameIndex() + 1));

            const auto processingMs = elapsed.elapsed();

            if (source_->frameIndex() % 10 == 0) {
                qInfo() << "[SVO GUI]"
                        << "Frame:" << source_->frameIndex()
                        << "Procesamiento completo:"
                        << processingMs << "ms";
            }

            if (playing_) schedule(processingMs);
        });
        connect(open, &QPushButton::clicked, this, [this] { openDirectory(); });
        connect(play_, &QPushButton::clicked, this, [this] {
            if (!source_) return;
            if (playing_) { pause(); return; }
            if (source_->frameIndex() + 1 >= source_->frameCount()) slider_->setValue(0);
            playing_ = true;
            play_->setText("Pausa");
            schedule();
        });
        connect(previous, &QPushButton::clicked, this, [this] { pause(); slider_->setValue(slider_->value() - 1); });
        connect(next, &QPushButton::clicked, this, [this] { pause(); slider_->setValue(slider_->value() + 1); });
        connect(slider_, &QSlider::sliderPressed, this, [this] { pause(); });
        connect(slider_, &QSlider::valueChanged, this, [this](int index) { loadFrame(index); });
        connect(all, &QPushButton::clicked, this, [this] {
            if (!source_) return;
        canvas_->roi = source_->rgbImage().rect();
            calculate();
            canvas_->update();
        });
        connect(height_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { calculate(); });
        connect(maximum_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] { calculate(); });
        canvas_->picked = [this](QPoint point) {
            showPoint(point);
        };

        canvas_->regionChanged = [this](QRect) { calculate(); };
        play_->setEnabled(false);
        calculate();
    }

private:
    void pause() { playing_ = false; timer_.stop(); play_->setText("Reproducir"); }
    void schedule(qint64 processingMs = 0)
    {
        if (!source_ || source_->frameIndex() + 1 >= source_->frameCount()) { pause(); return; }
        const auto index = source_->frameIndex();
        const double gap = source_->recordingTime(index + 1) - source_->recordingTime(index);
        // Reproduccion secuencial: no omite cuadros si la CPU tarda en decodificar.
        const int targetMs =
            int(std::round(gap * 1000.0));

        timer_.start(
            std::max(1, targetMs - int(processingMs)));
    }
    void openDirectory()
    {
        pause();
        const auto path = QFileDialog::getOpenFileName(
            this, "Abrir grabacion ZED",
            QDir::homePath() + "/hagie_recordings",
            "Grabaciones ZED (*.svo *.svo2)");
        if (path.isEmpty()) return;
        auto candidate = std::make_unique<ZedSvoPointCloudSource>(path);
        if (!candidate->start()) { QMessageBox::warning(this, "No se pudo abrir", candidate->error()); return; }
        source_ = std::move(candidate);
        if (!source_) return;
        canvas_->roi = source_->rgbImage().rect();
        canvas_->selected = QPoint(-1, -1);
        {
            QSignalBlocker blocker(slider_);
            slider_->setRange(0, int(source_->frameCount() - 1));
            slider_->setValue(0);
        }
        slider_->setEnabled(true);
        play_->setEnabled(true);
        coordinates_->setText("Clic: XYZ óptico en mm. Arrastrá para seleccionar una región.");
        render();
    }
    void loadFrame(int index)
    {
        if (!source_) return;
        if (!source_->seekFrame(std::size_t(index))) {
            pause();
            QSignalBlocker blocker(slider_);
            slider_->setValue(int(source_->frameIndex()));
            QMessageBox::warning(this, "No se pudo leer el cuadro", source_->error());
            return;
        }
        render();
    }
    void render()
    {
        const auto& depth = source_->depthImage();
        const int w = depth.width();
        const int h = depth.height();
        static const auto depthColors = [] {
            std::array<QRgb, 5001> colors {};

            colors[0] = qRgb(0, 0, 0);

            for (int i = 1; i <= 5000; ++i) {
                const double fraction = double(i) / 5000.0;

                colors[i] = QColor::fromHsvF(
                    (1.0 - fraction) * 0.67,
                    0.85, 0.95).rgb();
            }

            return colors;
        }();

        QImage colored(w, h, QImage::Format_RGB32);
        for (int v = 0; v < h; ++v) {
            const auto* row = reinterpret_cast<const quint16*>(depth.constScanLine(v));
            auto* output = reinterpret_cast<QRgb*>(colored.scanLine(v));
            for (int u = 0; u < w; ++u) {
                output[u] =
                    depthColors[std::min(int(row[u]), 5000)];
            }
        }
        const auto& rgb = source_->rgbImage();

        const int rw = rgb.width();
        const int rh = rgb.height();

        canvas_->image = QImage(
            2 * rw, rh, QImage::Format_RGB32);

        QPainter painter(&canvas_->image);

        painter.setRenderHint(
            QPainter::SmoothPixmapTransform, false);

        painter.drawImage(
            QRect(0, 0, rw, rh), rgb);

        painter.drawImage(
            QRect(rw, 0, rw, rh), colored);

        painter.end();
        status_->setText(QString("Cuadro %1 / %2 · tiempo %3 s · RGB/XYZ sincronizados · %4 puntos XYZ válidos")
            .arg(qulonglong(source_->frameIndex() + 1)).arg(qulonglong(source_->frameCount()))
            .arg(source_->recordingTime(source_->frameIndex()), 0, 'f', 2)
            .arg(qulonglong(source_->points().size())));
        canvas_->update();
        if (canvas_->selected.x() >= 0) showPoint(canvas_->selected);
        calculate();
    }
    void showPoint(QPoint pixel)
    {
        if (!source_) return;
        Vision3DProcessor::Point3D point;
        if (!source_->pointAt(pixel.x(), pixel.y(), point)) {
            coordinates_->setText(QString("Píxel (%1, %2): sin profundidad válida").arg(pixel.x()).arg(pixel.y()));
            return;
        }
        coordinates_->setText(QString("Píxel (%1, %2) · XYZ óptico: X = %3 mm | Y = %4 mm | Z = %5 mm")
            .arg(pixel.x()).arg(pixel.y()).arg(point.x, 0, 'f', 1)
            .arg(point.y, 0, 'f', 1).arg(point.z, 0, 'f', 1));
    }

    void calculate()
    {
        if (!source_) {
            imuStatus_->setText("IMU ZED: sin datos");
            result_->setText("Altura P95: —");
            return;
        }

        PointCloudSource::CameraOrientation imu;

        if (!source_->getCameraOrientation(imu)) {
            imuStatus_->setText("IMU ZED: no válida");
            result_->setText("Altura P95: no disponible");
            return;
        }

        imuStatus_->setText(
            QString("IMU ZED: válida | Roll: %1° | Pitch: %2°")
                .arg(imu.roll_deg, 0, 'f', 2)
                .arg(imu.pitch_deg, 0, 'f', 2));

        if (height_->value() <= 0) {
            result_->setText("Ingresá la altura de montaje");
            return;
        }

        Vision3DProcessor processor;

        Vision3DProcessor::AxisMapping axes;
        processor.setAxisMapping(axes);

        Vision3DProcessor::Orientation orientation;
        orientation.valid = true;
        orientation.roll_deg = imu.roll_deg;
        orientation.pitch_deg = imu.pitch_deg;

        processor.setOrientation(orientation);

        Vision3DProcessor::CameraConfig config;
        config.enabled = true;
        config.body_enabled[0] = true;
        config.geometry.camera_height_mm = height_->value();

        processor.setCameraConfig(0, config);

        Vision3DProcessor::TemporalFilterConfig filter;
        filter.enabled = false;
        processor.setTemporalFilterConfig(filter);

        Vision3DProcessor::BodyRegion region;
        region.min_x = -20000;
        region.max_x = 20000;
        region.min_y = -20000;
        region.max_y = 20000;
        region.min_z = 0;
        region.max_z = 10000;
        region.min_points = 30;

        processor.setBodyRegion(0, region);

        Vision3DProcessor::PointCloud selected;

        for (auto point : source_->points()) {
            if (!canvas_->roi.contains(
                    point.image_x, point.image_y) ||
                point.y > maximum_->value())
                continue;

            selected.push_back(point);
        }

        const auto result =
            processor.processPointCloud(0, selected).bodies[0];

        if (result.valid) {
            result_->setText(
                QString("Altura P95: %1 mm | Puntos seleccionados: %2")
                    .arg(result.height_mm)
                    .arg(qulonglong(selected.size())));
        } else {
            result_->setText(
                QString("Altura P95: no válida | Puntos seleccionados: %1")
                    .arg(qulonglong(selected.size())));
        }
    }

    std::unique_ptr<ZedSvoPointCloudSource> source_;
    QTimer timer_;
    bool playing_ = false;
    RgbdCanvas* canvas_ = nullptr;
    QLabel *status_ = nullptr, *coordinates_ = nullptr, *result_ = nullptr;
    QPushButton* play_ = nullptr;
    QSlider* slider_ = nullptr;
    QLabel* imuStatus_ = nullptr;
    QDoubleSpinBox* height_ = nullptr;
    QSpinBox* maximum_ = nullptr;
};
}

void openRgbdPlaybackDialog(QWidget* parent)
{
    RgbdPlaybackDialog dialog(parent);
    dialog.exec();
}
