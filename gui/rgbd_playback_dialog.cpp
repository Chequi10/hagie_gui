#include "gui/rgbd_playback_dialog.h"
#include "vision/tum_rgbd_point_cloud_source.h"
#include "vision/rgbd_reference_plane.h"

#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFileDialog>
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
    QRect roi {0, 0, 640, 480};
    QPoint selected {-1, -1};
    std::vector<QPoint> referencePixels;
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
        return {std::clamp(int((position.x() - rect.x()) * image.width() / rect.width()), 0, 1279),
                std::clamp(int((position.y() - rect.y()) * image.height() / rect.height()), 0, 479)};
    }
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(22, 25, 30));
        if (image.isNull()) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter, "Abrí la carpeta que contiene rgb.txt y depth.txt");
            return;
        }
        painter.drawImage(displayRect(), image);
        painter.save();
        painter.translate(displayRect().topLeft());
        painter.scale(displayRect().width() / image.width(), displayRect().height() / image.height());
        painter.setPen(QPen(Qt::yellow, 2));
        for (int offset : {0, 640}) {
            painter.drawRect(roi.translated(offset, 0));
            painter.setPen(QPen(QColor(255, 100, 60), 2));
            for (std::size_t i = 0; i < referencePixels.size(); ++i) {
                const auto p = referencePixels[i] + QPoint(offset, 0);
                painter.drawEllipse(p, 5, 5);
                painter.drawText(p + QPoint(8, -5), QString("P%1").arg(qulonglong(i + 1)));
            }
            painter.setPen(QPen(Qt::yellow, 2));
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
        QPoint finish = imagePoint(event->position());
        if (finish.x() < 0 || finish.x() / 640 != press_.x() / 640) { press_ = {-1, -1}; return; }
        QPoint first(press_.x() % 640, press_.y());
        finish.setX(finish.x() % 640);
        if ((finish - first).manhattanLength() < 6) {
            selected = finish;
            if (picked) picked(selected);
        } else {
            roi = QRect(first, finish).normalized().intersected(QRect(0, 0, 640, 480));
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
        setWindowTitle("Prueba de grabación RGB-D — HAGIE");
        resize(1160, 700);
        if (auto* screen = QApplication::primaryScreen())
            resize(size().boundedTo(screen->availableGeometry().size() - QSize(60, 60)));
        auto* layout = new QVBoxLayout(this);
        auto* intro = new QLabel("Grabación de prueba: RGB y XYZ reales. No requiere STM32. "
            "Los resultados se muestran en esta ventana y no se envían al control hidráulico.", this);
        intro->setWordWrap(true);
        layout->addWidget(intro);

        auto* controls = new QHBoxLayout;
        auto* open = new QPushButton("Abrir carpeta RGB-D…", this);
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
        status_ = new QLabel("Seleccioná rgbd_dataset_freiburg1_plant.", this);
        status_->setWordWrap(true);
        layout->addWidget(status_);
        canvas_ = new RgbdCanvas(this);
        layout->addWidget(canvas_, 1);
        coordinates_ = new QLabel("Clic: XYZ óptico en mm. Arrastrá sobre la planta para seleccionar la región.", this);
        coordinates_->setWordWrap(true);
        layout->addWidget(coordinates_);
        slider_ = new QSlider(Qt::Horizontal, this);
        slider_->setEnabled(false);
        layout->addWidget(slider_);

        auto* planeControls = new QHBoxLayout;
        auto* definePlane = new QPushButton("Marcar mesa (3 clics)", this);
        auto* clearPlane = new QPushButton("Borrar plano", this);
        planeStatus_ = new QLabel("Plano de mesa: sin definir.", this);
        planeStatus_->setWordWrap(true);
        planeControls->addWidget(definePlane);
        planeControls->addWidget(clearPlane);
        planeControls->addWidget(planeStatus_, 1);
        layout->addLayout(planeControls);

        auto* settings = new QHBoxLayout;
        reference_ = new QCheckBox("Referencia manual", this);
        reference_->setToolTip("Alternativa al plano de mesa: requiere altura e inclinación conocidas, con roll = 0°.");
        height_ = new QDoubleSpinBox(this);
        height_->setRange(0, 5000);
        height_->setSuffix(" mm");
        height_->setToolTip("Altura conocida del centro óptico sobre el suelo o plano de referencia.");
        pitch_ = new QDoubleSpinBox(this);
        pitch_->setRange(-89, 89);
        pitch_->setSuffix("°");
        pitch_->setToolTip("Inclinación óptica: positiva si la cámara mira hacia abajo. Roll de prueba = 0°.");
        maximum_ = new QSpinBox(this);
        maximum_->setRange(100, 12000);
        maximum_->setValue(2000);
        maximum_->setSuffix(" mm");
        settings->addWidget(reference_);
        settings->addWidget(height_);
        settings->addWidget(new QLabel("Inclinación ↓", this));
        settings->addWidget(pitch_);
        settings->addWidget(new QLabel("Prof. máx.", this));
        settings->addWidget(maximum_);
        layout->addLayout(settings);
        result_ = new QLabel(this);
        result_->setWordWrap(true);
        layout->addWidget(result_);
        auto* note = new QLabel("Pausá y marcá tres puntos separados de la mesa; después clic en una hoja para "
            "consultar su altura. Arrastrá una región que incluya las hojas superiores para calcular P95. "
            "El plano se descarta al cambiar de cuadro. La medida sobre la mesa incluye la maceta. "
            "XYZ óptico: X derecha, Y abajo, Z adelante. Negro = sin profundidad.", this);
        note->setWordWrap(true);
        layout->addWidget(note);

        timer_.setSingleShot(true);
        connect(&timer_, &QTimer::timeout, this, [this] {
            if (!source_) return;
            if (source_->frameIndex() + 1 >= source_->frameCount()) { pause(); return; }
            slider_->setValue(int(source_->frameIndex() + 1));
            if (playing_) schedule();
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
            canvas_->roi = QRect(0, 0, 640, 480);
            calculate();
            canvas_->update();
        });
        connect(reference_, &QCheckBox::toggled, this, [this] { calculate(); });
        connect(height_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { calculate(); });
        connect(pitch_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { calculate(); });
        connect(maximum_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] { calculate(); });
        connect(definePlane, &QPushButton::clicked, this, [this] {
            if (!source_) return;
            pause();
            resetPlane();
            reference_->setChecked(false);
            collectingPlane_ = true;
            planeStatus_->setText("Mesa: clic 1 de 3. Elegí puntos separados, lejos de los bordes.");
            calculate();
        });
        connect(clearPlane, &QPushButton::clicked, this, [this] {
            resetPlane();
            calculate();
            if (canvas_->selected.x() >= 0) showPoint(canvas_->selected);
        });
        canvas_->picked = [this](QPoint point) {
            if (collectingPlane_) collectPlanePoint(point);
            else showPoint(point);
        };
        canvas_->regionChanged = [this](QRect) { calculate(); };
        play_->setEnabled(false);
        calculate();
    }

private:
    void resetPlane()
    {
        plane_.clear();
        planePoints_.clear();
        canvas_->referencePixels.clear();
        collectingPlane_ = false;
        planeStatus_->setText("Plano de mesa: sin definir.");
        reference_->setEnabled(true);
        canvas_->update();
    }
    void collectPlanePoint(QPoint pixel)
    {
        Vision3DProcessor::Point3D point;
        if (!source_ || !source_->pointAt(pixel.x(), pixel.y(), point)) {
            showPoint(pixel);
            planeStatus_->setText(QString("Sin profundidad. Falta el clic %1 de 3 en la mesa.")
                .arg(qulonglong(planePoints_.size() + 1)));
            return;
        }
        planePoints_.push_back({point.x, point.y, point.z});
        canvas_->referencePixels.push_back(pixel);
        showPoint(pixel);
        if (planePoints_.size() < 3) {
            planeStatus_->setText(QString("Mesa: clic %1 de 3. Los puntos deben formar un triángulo amplio.")
                .arg(qulonglong(planePoints_.size() + 1)));
        } else {
            collectingPlane_ = false;
            if (!plane_.fit(planePoints_[0], planePoints_[1], planePoints_[2])) {
                resetPlane();
                planeStatus_->setText("Plano rechazado. Repetí con tres puntos separados sobre la misma mesa.");
            } else {
                reference_->setEnabled(false);
                planeStatus_->setText(QString("Plano listo para el cuadro %1. Clic en la planta: altura sobre la mesa.")
                    .arg(qulonglong(source_->frameIndex() + 1)));
            }
        }
        calculate();
        canvas_->update();
    }
    void pause() { playing_ = false; timer_.stop(); play_->setText("Reproducir"); }
    void schedule()
    {
        if (!source_ || source_->frameIndex() + 1 >= source_->frameCount()) { pause(); return; }
        const auto index = source_->frameIndex();
        const double gap = source_->recordingTime(index + 1) - source_->recordingTime(index);
        // Reproduccion secuencial: no omite cuadros si la CPU tarda en decodificar.
        timer_.start(std::max(1, int(std::round(gap * 1000.0))));
    }
    void openDirectory()
    {
        pause();
        const auto path = QFileDialog::getExistingDirectory(this, "Carpeta con rgb.txt y depth.txt",
            QStandardPaths::writableLocation(QStandardPaths::DownloadLocation));
        if (path.isEmpty()) return;
        auto candidate = std::make_unique<TumRgbdPointCloudSource>(path);
        if (!candidate->start()) { QMessageBox::warning(this, "No se pudo abrir", candidate->error()); return; }
        source_ = std::move(candidate);
        resetPlane();
        canvas_->roi = QRect(0, 0, 640, 480);
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
        const bool changingFrame = std::size_t(index) != source_->frameIndex();
        if (!source_->seekFrame(std::size_t(index))) {
            pause();
            QSignalBlocker blocker(slider_);
            slider_->setValue(int(source_->frameIndex()));
            QMessageBox::warning(this, "No se pudo leer el cuadro", source_->error());
            return;
        }
        if (changingFrame) {
            const bool hadReference = plane_.isValid() || collectingPlane_;
            resetPlane();
            if (hadReference)
                planeStatus_->setText("Cambió el cuadro: volvé a marcar la mesa para calcular alturas.");
        }
        render();
    }
    void render()
    {
        const auto& depth = source_->depthImage();
        QImage colored(640, 480, QImage::Format_RGB32);
        for (int v = 0; v < 480; ++v) {
            const auto* row = reinterpret_cast<const quint16*>(depth.constScanLine(v));
            auto* output = reinterpret_cast<QRgb*>(colored.scanLine(v));
            for (int u = 0; u < 640; ++u) {
                const double fraction = std::clamp(double(row[u]) / 25000.0, 0.0, 1.0);
                output[u] = row[u] == 0 ? qRgb(0, 0, 0)
                    : QColor::fromHsvF((1.0 - fraction) * 0.67, 0.85, 0.95).rgb();
            }
        }
        canvas_->image = QImage(1280, 480, QImage::Format_RGB32);
        QPainter painter(&canvas_->image);
        painter.drawImage(0, 0, source_->rgbImage());
        painter.drawImage(640, 0, colored);
        painter.end();
        status_->setText(QString("Cuadro %1 / %2 · tiempo %3 s · RGB/profundidad Δ %4 ms · %5 puntos XYZ válidos")
            .arg(qulonglong(source_->frameIndex() + 1)).arg(qulonglong(source_->frameCount()))
            .arg(source_->recordingTime(source_->frameIndex()), 0, 'f', 2)
            .arg(source_->pairDeltaMs(), 0, 'f', 1).arg(qulonglong(source_->points().size())));
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
        if (plane_.isValid()) {
            const double height = plane_.heightMm({point.x, point.y, point.z});
            coordinates_->setText(coordinates_->text() + QString(" | Altura sobre mesa = %1 mm")
                .arg(height, 0, 'f', 1));
        }
    }
    void calculate()
    {
        const bool usingPlane = plane_.isValid();
        const bool referenced = reference_->isChecked() && !usingPlane && !collectingPlane_;
        reference_->setEnabled(!usingPlane && !collectingPlane_);
        height_->setEnabled(referenced);
        pitch_->setEnabled(referenced);
        if (!source_) { result_->setText("Altura: —. Primero abrí una grabación."); return; }
        if (!referenced && !usingPlane) {
            result_->setText("Altura: —. XYZ disponible. Marcá tres puntos de la mesa para definir la referencia.");
            return;
        }
        // Instancia de prueba del mismo algoritmo, sin HagieState ni publicacion hidraulica.
        Vision3DProcessor processor;
        Vision3DProcessor::AxisMapping axes;
        if (!usingPlane) {
            axes.longitudinal = Vision3DProcessor::Axis::Z;
            axes.vertical = Vision3DProcessor::Axis::Y;
            axes.vertical_sign = -1;
        }
        processor.setAxisMapping(axes);
        Vision3DProcessor::Orientation orientation;
        orientation.valid = true;
        processor.setOrientation(orientation);
        Vision3DProcessor::CameraConfig config;
        config.enabled = true;
        config.body_enabled[0] = true;
        // Giro optico primero, luego traslacion vertical de la referencia.
        // Se aplica a los puntos debajo; geometria del procesador queda neutra.
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
        const double angle = pitch_->value() * 3.14159265358979323846 / 180.0;
        const float c = float(std::cos(angle)), s = float(std::sin(angle));
        Vision3DProcessor::PointCloud selected;
        for (auto point : source_->points()) {
            if (!canvas_->roi.contains(point.image_x, point.image_y) || point.z > maximum_->value()) continue;
            if (usingPlane) {
                const auto local = plane_.machinePoint({point.x, point.y, point.z});
                point.x = float(local[0]);
                point.y = float(local[1]);
                point.z = float(local[2]);
            } else {
                // Optico inclinado -> optico horizontal. Z interno = -Y optico.
                const float y = c * point.y + s * point.z - float(height_->value());
                const float z = -s * point.y + c * point.z;
                point.y = y; point.z = z;
            }
            selected.push_back(point);
        }
        const auto result = processor.processPointCloud(0, selected).bodies[0];
        const auto roi = canvas_->roi;
        if (result.valid)
            result_->setText(QString("Altura de prueba (P95): %1 mm · región (%2,%3)–(%4,%5) · "
                "%6 puntos seleccionados. Sin filtro temporal; requiere una referencia correcta.")
                .arg(result.height_mm).arg(roi.left()).arg(roi.top()).arg(roi.right()).arg(roi.bottom())
                .arg(qulonglong(selected.size())) + (usingPlane ? " Referencia: plano de la mesa." : " Referencia: manual."));
        else
            result_->setText(QString("Altura no válida: se requieren 30 puntos dentro de la región, "
                "el rango vertical y la profundidad máxima. Seleccionados antes del filtro vertical: %1.")
                .arg(qulonglong(selected.size())));
    }
    std::unique_ptr<TumRgbdPointCloudSource> source_;
    RgbdReferencePlane plane_;
    std::vector<RgbdReferencePlane::Vector> planePoints_;
    bool collectingPlane_ = false;
    QLabel* planeStatus_ = nullptr;
    QTimer timer_;
    bool playing_ = false;
    RgbdCanvas* canvas_ = nullptr;
    QLabel *status_ = nullptr, *coordinates_ = nullptr, *result_ = nullptr;
    QPushButton* play_ = nullptr;
    QSlider* slider_ = nullptr;
    QCheckBox* reference_ = nullptr;
    QDoubleSpinBox *height_ = nullptr, *pitch_ = nullptr;
    QSpinBox* maximum_ = nullptr;
};
}

void openRgbdPlaybackDialog(QWidget* parent)
{
    RgbdPlaybackDialog dialog(parent);
    dialog.exec();
}
