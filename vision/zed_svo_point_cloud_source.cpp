#include "vision/zed_svo_point_cloud_source.h"
#include "vision/zed_orientation_adapter.h"

#include <cmath>
#include <QElapsedTimer>
#include <QDebug>
#include <algorithm>
#include <utility>

#ifdef HAGIE_ENABLE_ZED_SDK
#include <sl/Camera.hpp>
#endif

struct ZedSvoPointCloudSource::Impl
{
#ifdef HAGIE_ENABLE_ZED_SDK
    sl::Camera camera;
    sl::Mat image;
    sl::Mat xyz;
#endif
};

ZedSvoPointCloudSource::ZedSvoPointCloudSource(QString file)
    : impl_(std::make_unique<Impl>()),
      file_(std::move(file))
{
}

ZedSvoPointCloudSource::~ZedSvoPointCloudSource()
{
    stop();
}

bool ZedSvoPointCloudSource::start()
{
    stop();
    error_.clear();

#ifdef HAGIE_ENABLE_ZED_SDK

    sl::InitParameters params;

    params.input.setFromSVOFile(
        file_.toUtf8().constData());

    params.svo_real_time_mode = false;
    params.depth_mode = sl::DEPTH_MODE::NEURAL;
    params.coordinate_units = sl::UNIT::MILLIMETER;
    params.coordinate_system =
        sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    auto result = impl_->camera.open(params);

    if (result != sl::ERROR_CODE::SUCCESS) {
        error_ = QString("Error al abrir SVO: %1")
            .arg(int(result));
        return false;
    }

    const int frames =
        impl_->camera.getSVONumberOfFrames();

    if (frames <= 0) {
        stop();
        error_ = "La grabacion no contiene cuadros.";
        return false;
    }

    count_ = std::size_t(frames);

    fps_ = impl_->camera.getCameraInformation()
        .camera_configuration.fps;

    if (fps_ <= 0) fps_ = 30.0;

    running_ = true;

    if (!seekFrame(0)) {
        stop();
        return false;
    }

    return true;

#else

    error_ = "SDK ZED no habilitado.";
    return false;

#endif
}

void ZedSvoPointCloudSource::stop()
{
#ifdef HAGIE_ENABLE_ZED_SDK
    if (impl_->camera.isOpened())
        impl_->camera.close();
#endif

    running_ = false;
    count_ = 0;
    cloud_.clear();
    rgb_ = QImage();
    depth_ = QImage();
}

bool ZedSvoPointCloudSource::isRunning() const
{
    return running_;
}

bool ZedSvoPointCloudSource::seekFrame(std::size_t index)
{
#ifdef HAGIE_ENABLE_ZED_SDK

    if (!running_ || index >= count_)
        return false;

    QElapsedTimer timer;
    timer.start();

    auto& camera = impl_->camera;

    const bool sequential =
        !rgb_.isNull() && index == index_ + 1;

    if ((!sequential &&
         camera.setSVOPosition(int(index))
            != sl::ERROR_CODE::SUCCESS) ||
        camera.grab() != sl::ERROR_CODE::SUCCESS) {
        error_ = "No se pudo leer el cuadro SVO.";
        return false;
    }

    const auto grabMs = timer.elapsed();

    CameraOrientation frameOrientation {};

    // IMU sincronizada con el fotograma SVO2.
    sl::SensorsData sensorsData;

    const auto imuResult = camera.getSensorsData(
        sensorsData,
        sl::TIME_REFERENCE::IMAGE);

    if (imuResult == sl::ERROR_CODE::SUCCESS &&
        sensorsData.imu.is_available) {

        frameOrientation =
            ZedOrientationAdapter::fromRotation(
                sensorsData.imu.pose.getRotationMatrix());
    }

    if (index % 10 == 0) {
        if (imuResult == sl::ERROR_CODE::SUCCESS &&
            sensorsData.imu.is_available) {

            const auto angles =
                sensorsData.imu.pose.getEulerAngles(false);

            const auto& orientation = frameOrientation;

            qInfo() << "[HAGIE IMU]"
                    << "Frame:" << index
                    << "Roll:" << orientation.roll_deg
                    << "Pitch:" << orientation.pitch_deg
                    << "Valida:" << orientation.valid;

            qInfo() << "[SVO IMU]"
                    << "Frame:" << index
                    << "Valida: SI"
                    << "Euler X:" << angles.x
                    << "Euler Y:" << angles.y
                    << "Euler Z:" << angles.z;

        } else {
            qInfo() << "[SVO IMU]"
                    << "Frame:" << index
                    << "Valida: NO"
                    << "Codigo:" << int(imuResult);
        }
    }


    const sl::Resolution rgbResolution(1920, 1200);
    const sl::Resolution xyzResolution(960, 600);

    if (camera.retrieveImage(
            impl_->image,
            sl::VIEW::LEFT,
            sl::MEM::CPU,
            rgbResolution)
            != sl::ERROR_CODE::SUCCESS ||
        camera.retrieveMeasure(
            impl_->xyz,
            sl::MEASURE::XYZ,
            sl::MEM::CPU,
            xyzResolution)
            != sl::ERROR_CODE::SUCCESS) {
        error_ = "Error al obtener RGB o XYZ.";
        return false;
    }

    const auto retrieveMs = timer.elapsed();

    const int rw = impl_->image.getWidth();
    const int rh = impl_->image.getHeight();

    const int dw = impl_->xyz.getWidth();
    const int dh = impl_->xyz.getHeight();

    if (rw <= 0 || rh <= 0 || dw <= 0 || dh <= 0) {
        error_ = "Dimensiones RGB o XYZ invalidas.";
        return false;
    }

    QImage rgb(rw, rh, QImage::Format_RGB888);
    QImage depth(dw, dh, QImage::Format_Grayscale16);

    if (rgb.isNull() || depth.isNull()) {
        error_ = "No hay memoria para las imagenes.";
        return false;
    }

    Vision3DProcessor::PointCloud cloud;
    cloud.reserve(std::size_t(dw) * dh);

    const auto* imagePtr =
        impl_->image.getPtr<sl::uchar4>(sl::MEM::CPU);

    const auto* xyzPtr =
        impl_->xyz.getPtr<sl::float4>(sl::MEM::CPU);

    const auto imageStep =
        impl_->image.getStep<sl::uchar4>(sl::MEM::CPU);

    const auto xyzStep =
        impl_->xyz.getStep<sl::float4>(sl::MEM::CPU);

    if (!imagePtr || !xyzPtr) {
        error_ = "Error de acceso a memoria RGB/XYZ.";
        return false;
    }

    // Imagen RGB en resolucion completa.
    for (int y = 0; y < rh; ++y) {
        const auto* imageRow = imagePtr + y * imageStep;
        auto* rgbRow = rgb.scanLine(y);

        for (int x = 0; x < rw; ++x) {
            const auto& pixel = imageRow[x];

            rgbRow[3*x]     = pixel[2];
            rgbRow[3*x + 1] = pixel[1];
            rgbRow[3*x + 2] = pixel[0];
        }
    }

    // Profundidad y nube XYZ en resolucion reducida.
    for (int y = 0; y < dh; ++y) {
        const auto* xyzRow = xyzPtr + y * xyzStep;

        auto* depthRow =
            reinterpret_cast<quint16*>(depth.scanLine(y));

        for (int x = 0; x < dw; ++x) {
            depthRow[x] = 0;

            const auto& p = xyzRow[x];

            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z) ||
                p.x <= 0)
                continue;

            depthRow[x] = quint16(
                std::clamp(p.x, 1.0f, 65535.0f));

            Vision3DProcessor::Point3D point;

            point.x = -p.y;
            point.y = -p.z;
            point.z = p.x;

            // Coordenadas referidas a la imagen RGB.
            point.image_x = int(
                (x + 0.5) * rw / dw);

            point.image_y = int(
                (y + 0.5) * rh / dh);

            point.image_coordinates_valid = true;

            cloud.push_back(point);
        }
    }

    rgb_ = std::move(rgb);
    depth_ = std::move(depth);
    cloud_ = std::move(cloud);

    orientation_ = frameOrientation;
    index_ = index;

    const auto totalMs = timer.elapsed();

    if (index % 10 == 0) {
        qInfo() << "[SVO PERF]"
                << "Frame:" << index
                << "Grab:" << grabMs << "ms"
                << "Retrieve:" << retrieveMs - grabMs << "ms"
                << "Procesamiento:" << totalMs - retrieveMs << "ms"
                << "Total:" << totalMs << "ms"
                << "FPS fuente:" << 1000.0 / qMax(totalMs, qint64(1));
    }

    return true;

#else

    return false;

#endif
}

bool ZedSvoPointCloudSource::getPointCloud(
    Vision3DProcessor::PointCloud& cloud)
{
    if (!running_) return false;

    cloud = cloud_;
    return !cloud.empty();
}

bool ZedSvoPointCloudSource::pointAt(
    int u, int v,
    Vision3DProcessor::Point3D& point) const
{
#ifdef HAGIE_ENABLE_ZED_SDK

    if (!running_ ||
        u < 0 || v < 0 ||
        u >= rgb_.width() || v >= rgb_.height())
        return false;

    if (depth_.isNull())
        return false;

    const int du = std::clamp(
        int((u + 0.5) * depth_.width() / rgb_.width()),
        0, depth_.width() - 1);

    const int dv = std::clamp(
        int((v + 0.5) * depth_.height() / rgb_.height()),
        0, depth_.height() - 1);

    sl::float4 p;

    if (impl_->xyz.getValue(du, dv, &p)
            != sl::ERROR_CODE::SUCCESS)
        return false;

    if (!std::isfinite(p.x) ||
        !std::isfinite(p.y) ||
        !std::isfinite(p.z) ||
        p.x <= 0)
        return false;

    point.x = -p.y;
    point.y = -p.z;
    point.z = p.x;

    point.image_x = u;
    point.image_y = v;
    point.image_coordinates_valid = true;

    return true;

#else

    return false;

#endif
}
