#pragma once

#include <QWidget>
#include <QVector>
#include <QElapsedTimer>

class HeightTrendWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HeightTrendWidget(QWidget *parent = nullptr);

    void addSample(
        int body,
        double target_mm,
        double encoder_mm
    );

    void setSelectedBody(
        int body
    );

    void setVisibleWindowSeconds(
        double seconds
    );

    void setAutoScale(
        bool enabled
    );

    void setFixedRange(
        double min_mm,
        double max_mm
    );

protected:
    void paintEvent(
        QPaintEvent *event
    ) override;

private:
    struct Sample
    {
        qint64 time_ms;
        double target_mm;
        double encoder_mm;
    };

    static constexpr int BODY_COUNT = 6;

    QVector<Sample> samples[BODY_COUNT];

    int selectedBody = 0;

    QElapsedTimer timer;

    double visibleWindowSeconds = 30.0;

    static constexpr double MAX_HISTORY_SECONDS =
        1800.0; // 30 minutos

    bool autoScale = false;

    double fixedMinMm = 50.0;
    double fixedMaxMm = 700.0;
};