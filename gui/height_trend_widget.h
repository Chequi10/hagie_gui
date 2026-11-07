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
        double encoder_mm,
        double current_up_ma = 0.0,
        double current_down_ma = 0.0,
        bool current_valid = false
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
        double current_up_ma;
        double current_down_ma;
        bool current_valid;
    };

    // HAGIE_AX020800_TREND_V1: OUT1/OUT2 solo para cuerpo 1.
    static constexpr int BODY_COUNT = 6;

    QVector<Sample> samples[BODY_COUNT];

    int selectedBody = 0;

    QElapsedTimer timer;

    double visibleWindowSeconds = 15.0;

    static constexpr double MAX_HISTORY_SECONDS =
        1800.0; // 30 minutos

    bool autoScale = false;

    double fixedMinMm = 50.0;
    double fixedMaxMm = 700.0;
};