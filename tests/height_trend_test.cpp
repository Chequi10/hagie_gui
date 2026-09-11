#include <QApplication>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>

#include <cmath>

#include "gui/height_trend_widget.h"


int main(
    int argc,
    char *argv[]
)
{
    QApplication app(
        argc,
        argv
    );


    QWidget window;

    window.setWindowTitle(
        "Height Trend Test"
    );


    QVBoxLayout *layout =
        new QVBoxLayout(
            &window
        );


    QLabel *infoLabel =
        new QLabel(
            "Simulación de objetivo 3D y respuesta del encoder"
        );

    infoLabel->setAlignment(
        Qt::AlignCenter
    );

    layout->addWidget(
        infoLabel
    );


    HeightTrendWidget *trend =
        new HeightTrendWidget();

    trend->setVisibleWindowSeconds(
        30.0
    );

    trend->setAutoScale(
        false
    );

    trend->setFixedRange(
        0.0,
        700.0
    );

    trend->setSelectedBody(
        0
    );

    layout->addWidget(
        trend,
        1
    );


    /*
     * ========================================================
     * SIMULACIÓN
     * ========================================================
     *
     * target_mm:
     *     representa la altura objetivo entregada por visión 3D.
     *
     * encoder_mm:
     *     representa la posición real medida por el encoder.
     *
     * El encoder sigue al objetivo con retardo,
     * imitando la respuesta de un sistema hidráulico.
     */

    double encoder_mm =
        300.0;

    double time_s =
        0.0;


    QTimer timer;

    timer.setInterval(
        100
    );


    QObject::connect(
        &timer,
        &QTimer::timeout,
        [&]()
        {
            time_s +=
                0.1;


            /*
             * Objetivo escalonado.
             */
            double target_mm =
                400.0;


            if (time_s >= 5.0 &&
                time_s < 10.0)
            {
                target_mm =
                    500.0;
            }
            else if (time_s >= 10.0 &&
                     time_s < 15.0)
            {
                target_mm =
                    350.0;
            }
            else if (time_s >= 15.0 &&
                     time_s < 20.0)
            {
                target_mm =
                    550.0;
            }
            else if (time_s >= 20.0 &&
                     time_s < 25.0)
            {
                target_mm =
                    420.0;
            }
            else if (time_s >= 25.0)
            {
                time_s =
                    0.0;
            }


            /*
             * Respuesta simulada del encoder.
             *
             * Modelo simple de primer orden:
             *
             * encoder += alpha *
             *            (target - encoder)
             *
             * Cuanto menor alpha,
             * más lenta la respuesta.
             */
            constexpr double alpha =
                0.08;


            encoder_mm +=
                alpha *
                (
                    target_mm -
                    encoder_mm
                );


            /*
             * Pequeña perturbación para que
             * la señal no sea perfectamente ideal.
             */
            encoder_mm +=
                std::sin(
                    time_s * 3.0
                ) * 0.5;


            /*
             * Cargar muestra del cuerpo 1.
             */
            trend->addSample(
                0,
                target_mm,
                encoder_mm
            );


            infoLabel->setText(
                QString(
                    "Objetivo 3D: %1 mm   |   Encoder: %2 mm"
                )
                .arg(
                    target_mm,
                    0,
                    'f',
                    0
                )
                .arg(
                    encoder_mm,
                    0,
                    'f',
                    1
                )
            );
        }
    );


    timer.start();


    window.resize(
        1200,
        600
    );

    window.show();


    return app.exec();
}