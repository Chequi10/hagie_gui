#include "height_trend_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <algorithm>
#include <QPainterPath>
#include <limits>


HeightTrendWidget::HeightTrendWidget(
    QWidget *parent
)
    : QWidget(parent)
{
    setMinimumHeight(180);

    timer.start();

    setAutoFillBackground(true);
}


void HeightTrendWidget::setVisibleWindowSeconds(
    double seconds
)
{
    if (seconds > 1.0)
    {
        visibleWindowSeconds = seconds;
        update();
    }
}

void HeightTrendWidget::setAutoScale(
    bool enabled
)
{
    autoScale = enabled;
    update();
}


void HeightTrendWidget::setFixedRange(
    double min_mm,
    double max_mm
)
{
    if (max_mm <= min_mm)
    {
        return;
    }

    fixedMinMm = min_mm;
    fixedMaxMm = max_mm;

    update();
}


void HeightTrendWidget::addSample(
    int body,
    double target_mm,
    double encoder_mm
)
{
    if (body < 0 ||
        body >= BODY_COUNT)
    {
        return;
    }

    Sample sample;

    sample.time_ms =
        timer.elapsed();

    sample.target_mm =
        target_mm;

    sample.encoder_mm =
        encoder_mm;

    samples[body].push_back(sample);


    const qint64 oldestAllowedTime =
        sample.time_ms -
        static_cast<qint64>(
            MAX_HISTORY_SECONDS *
            1000.0
        );


    while (!samples[body].isEmpty() &&
           samples[body].front().time_ms <
               oldestAllowedTime)
    {
        samples[body].removeFirst();
    }


    if (body == selectedBody)
    {
        update();
    }
}

void HeightTrendWidget::setSelectedBody(
    int body
)
{
    if (body < 0 ||
        body >= BODY_COUNT)
    {
        return;
    }

    selectedBody =
        body;

    update();
}


void HeightTrendWidget::paintEvent(
    QPaintEvent *event
)
{
    Q_UNUSED(event);


    QPainter painter(this);

    painter.setRenderHint(
        QPainter::Antialiasing,
        true
    );


    /*
     * Fondo.
     */
    painter.fillRect(
        rect(),
        palette().window()
    );


    const int leftMargin   = 85;
    const int rightMargin  = 20;
    const int topMargin    = 30;
    const int bottomMargin = 35;


    QRect graphRect(
        leftMargin,
        topMargin,
        width() -
            leftMargin -
            rightMargin,
        height() -
            topMargin -
            bottomMargin
    );


    if (graphRect.width() <= 0 ||
        graphRect.height() <= 0)
    {
        return;
    }


    /*
     * Título.
     */
    painter.setPen(
        palette().text().color()
    );

    QFont titleFont =
        painter.font();

    titleFont.setBold(true);
    titleFont.setPointSize(11);

    painter.setFont(titleFont);

    painter.drawText(
        QRect(
            0,
            2,
            width(),
            25
        ),
        Qt::AlignCenter,
        QString("TENDENCIA DE ALTURA - CUERPO %1")
            .arg(selectedBody + 1)
    );


    /*
     * Ejes.
     */
    painter.setPen(
        QPen(
            palette().text().color(),
            1
        )
    );

    painter.drawRect(
        graphRect
    );


    


    const qint64 now =
        timer.elapsed();


    const qint64 visibleStart =
        now -
        static_cast<qint64>(
            visibleWindowSeconds *
            1000.0
        );

    const QVector<Sample> &bodySamples =
    samples[selectedBody];



    if (bodySamples.isEmpty())
    {
        painter.drawText(
            graphRect,
            Qt::AlignCenter,
            "Sin datos"
        );

        return;
    }
      

    /*
     * Buscar mínimo y máximo
     * solamente dentro de la ventana visible.
     */
    double minValue;
    double maxValue;


if (autoScale)
{
    /*
     * Escala automática:
     * buscar mínimo y máximo solamente
     * dentro de la ventana visible.
     */

        minValue =
            std::numeric_limits<double>::max();

        maxValue =
            std::numeric_limits<double>::lowest();


        for (const Sample &sample : bodySamples)
        {
            if (sample.time_ms <
                visibleStart)
            {
                continue;
            }


            minValue =
                std::min(
                    minValue,
                    std::min(
                        sample.target_mm,
                        sample.encoder_mm
                    )
                );


            maxValue =
                std::max(
                    maxValue,
                    std::max(
                        sample.target_mm,
                        sample.encoder_mm
                    )
                );
        }


        if (minValue >
            maxValue)
        {
            return;
        }


        double range =
            maxValue - minValue;


        if (range < 50.0)
        {
            range = 50.0;
        }


        minValue -=
            range * 0.15;

        maxValue +=
            range * 0.15;


        /*
        * Una altura física negativa
        * no tiene sentido en este sistema.
        */
        if (minValue < 0.0)
        {
            minValue = 0.0;
}
    }
    else
    {
        /*
        * Escala fija.
        */
        minValue =
            fixedMinMm;

        maxValue =
            fixedMaxMm;
    }


    /*
     * Funciones de conversión.
     */
    auto mapX =
        [&](qint64 time_ms)
        {
            double normalized =
                static_cast<double>(
                    time_ms -
                    visibleStart
                ) /
                (
                    visibleWindowSeconds *
                    1000.0
                );


            return graphRect.left() +
                normalized *
                graphRect.width();
        };


    auto mapY =
        [&](double value)
        {
            double normalized =
                (value - minValue) /
                (maxValue - minValue);


            return graphRect.bottom() -
                normalized *
                graphRect.height();
        };


    /*
     * Grilla horizontal.
     */
    painter.setPen(
        QPen(
            QColor(180, 180, 180),
            1,
            Qt::DashLine
        )
    );


    constexpr int horizontalDivisions =
        5;


    for (int i = 0;
         i <= horizontalDivisions;
         ++i)
    {
        double value =
            minValue +
            (
                maxValue - minValue
            ) *
            static_cast<double>(i) /
            horizontalDivisions;


        int y =
            static_cast<int>(
                mapY(value)
            );


        painter.drawLine(
            graphRect.left(),
            y,
            graphRect.right(),
            y
        );


        painter.setPen(
            palette().text().color()
        );


        painter.drawText(
                5,
                y - 10,
                leftMargin - 12,
                20,
            Qt::AlignRight |
            Qt::AlignVCenter,
            QString::number(
                value,
                'f',
                0
            ) + " mm"
        );


        painter.setPen(
            QPen(
                QColor(180, 180, 180),
                1,
                Qt::DashLine
            )
        );
    }


    /*
     * Línea objetivo.
     */

    /*
    * Ninguna curva puede dibujarse
    * fuera del área del gráfico.
    */
    painter.save();
    painter.setClipRect(graphRect); 
    QPainterPath targetPath;

    bool targetStarted =
        false;


    for (const Sample &sample : bodySamples)
    {
        if (sample.time_ms <
            visibleStart)
        {
            continue;
        }


        QPointF point(
            mapX(sample.time_ms),
            mapY(sample.target_mm)
        );


        if (!targetStarted)
        {
            targetPath.moveTo(point);
            targetStarted = true;
        }
        else
        {
            targetPath.lineTo(point);
        }
    }


    painter.setPen(
        QPen(
            QColor(220, 40, 40),
            2
        )
    );

    painter.drawPath(
        targetPath
    );


    /*
     * Línea encoder.
     */
    QPainterPath encoderPath;

    bool encoderStarted =
        false;


    for (const Sample &sample : bodySamples)
    {
        if (sample.time_ms <
            visibleStart)
        {
            continue;
        }


        QPointF point(
            mapX(sample.time_ms),
            mapY(sample.encoder_mm)
        );


        if (!encoderStarted)
        {
            encoderPath.moveTo(point);
            encoderStarted = true;
        }
        else
        {
            encoderPath.lineTo(point);
        }
    }


    painter.setPen(
        QPen(
            QColor(30, 90, 220),
            2
        )
    );

    painter.drawPath(
        encoderPath
    );

    painter.restore();


    /*
     * Leyenda.
     */
    painter.setFont(
        QWidget::font()
    );


    painter.setPen(
        QColor(220, 40, 40)
    );

    painter.drawText(
        graphRect.left() + 10,
        graphRect.top() + 20,
        "Objetivo 3D"
    );


    painter.setPen(
        QColor(30, 90, 220)
    );

    painter.drawText(
        graphRect.left() + 120,
        graphRect.top() + 20,
        "Encoder"
    );


    /*
     * Escala temporal.
     */
    painter.setPen(
        palette().text().color()
    );


    painter.drawText(
        graphRect.left(),
        graphRect.bottom() + 5,
        100,
        25,
        Qt::AlignLeft |
        Qt::AlignVCenter,
        QString("-%1 s")
            .arg(
                visibleWindowSeconds,
                0,
                'f',
                0
            )
    );


    painter.drawText(
        graphRect.right() - 100,
        graphRect.bottom() + 5,
        100,
        25,
        Qt::AlignRight |
        Qt::AlignVCenter,
        "AHORA"
    );
}