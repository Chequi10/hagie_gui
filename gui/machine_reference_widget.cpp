#include "machine_reference_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QFontMetrics>
#include <algorithm>


MachineReferenceWidget::MachineReferenceWidget(
    QWidget *parent
)
    : QWidget(parent)
{
    setMinimumSize(
        420,
        300
    );
}


void MachineReferenceWidget::setBodyRegion(
    int body,
    double xMinMm,
    double xMaxMm,
    double yMinMm,
    double yMaxMm
)
{
    if (body < 0 ||
        body >= BODY_COUNT)
    {
        return;
    }

    BodyRegion &region =
        bodyRegions[
            static_cast<std::size_t>(body)
        ];

    region.xMinMm =
        std::min(xMinMm, xMaxMm);

    region.xMaxMm =
        std::max(xMinMm, xMaxMm);

    region.yMinMm =
        std::min(yMinMm, yMaxMm);

    region.yMaxMm =
        std::max(yMinMm, yMaxMm);

    region.valid = true;

    update();
}


void MachineReferenceWidget::clearBodyRegion(
    int body
)
{
    if (body < 0 ||
        body >= BODY_COUNT)
    {
        return;
    }

    bodyRegions[
        static_cast<std::size_t>(body)
    ].valid = false;

    update();
}


void MachineReferenceWidget::clearAllRegions()
{
    for (BodyRegion &region :
         bodyRegions)
    {
        region.valid = false;
    }

    update();
}

void MachineReferenceWidget::setCameraPosition(
    int camera,
    double xM,
    double yM,
    bool enabled
)
{
    if (camera < 0 ||
        camera >= CAMERA_COUNT)
    {
        return;
    }

    CameraPosition &position =
        cameraPositions[
            static_cast<std::size_t>(camera)
        ];

    position.xM = xM;
    position.yM = yM;
    position.enabled = enabled;

    update();
}


void MachineReferenceWidget::paintEvent(
    QPaintEvent *event
)
{
    Q_UNUSED(event);

    QPainter painter(this);

    painter.setRenderHint(
        QPainter::Antialiasing,
        true
    );

    painter.fillRect(
        rect(),
        palette().base()
    );


    /*
     * Márgenes.
     */
    const int margin = 35;

    const QRectF graphRect(
        margin,
        margin,
        width() - 2 * margin,
        height() - 2 * margin
    );


    /*
     * Origen.
     */
    const QPointF origin(
        graphRect.center().x(),
        graphRect.center().y()
    );


    /*
     * Determinar extensión necesaria.
     *
     * Como mínimo mostramos +/- 2000 mm.
     */
    double maxAbsX = 1.0;
    double maxAbsY = 1.0;

    for (const BodyRegion &region :
         bodyRegions)
    {
        if (!region.valid)
        {
            continue;
        }

        maxAbsX =
            std::max(
                maxAbsX,
                std::max(
                    std::abs(region.xMinMm),
                    std::abs(region.xMaxMm)
                )
            );

        maxAbsY =
            std::max(
                maxAbsY,
                std::max(
                    std::abs(region.yMinMm),
                    std::abs(region.yMaxMm)
                )
            );
    }


    /*
     * Dejamos margen visual.
     */
    maxAbsX *= 1.15;
    maxAbsY *= 1.15;


    const double scaleX =
        (graphRect.width() * 0.5) /
        maxAbsX;

    const double scaleY =
        (graphRect.height() * 0.5) /
        maxAbsY;

    


    /*
     * Conversión coordenadas máquina -> pantalla.
     *
     * +X = derecha
     * +Y = adelante
     *
     * Ojo:
     * en pantalla Y positivo va hacia abajo,
     * por eso se resta.
     */
    auto machineToScreen =
        [&](double xMm,
            double yMm)
        {
            return QPointF(
            origin.x() +
                xMm * scaleX,

            origin.y() -
                yMm * scaleY
        );
        };


    /*
     * ========================================================
     * Ejes
     * ========================================================
     */

    QPen axisPen(
        palette().text().color(),
        2
    );

    painter.setPen(
        axisPen
    );


    /*
     * Eje X
     */
    painter.drawLine(
        QPointF(
            graphRect.left(),
            origin.y()
        ),
        QPointF(
            graphRect.right(),
            origin.y()
        )
    );


    /*
     * Eje Y
     */
    painter.drawLine(
        QPointF(
            origin.x(),
            graphRect.bottom()
        ),
        QPointF(
            origin.x(),
            graphRect.top()
        )
    );


    /*
     * Flechas.
     */
    const double arrow = 8.0;


    /*
     * +X
     */
    painter.drawLine(
        QPointF(
            graphRect.right(),
            origin.y()
        ),
        QPointF(
            graphRect.right() - arrow,
            origin.y() - arrow
        )
    );

    painter.drawLine(
        QPointF(
            graphRect.right(),
            origin.y()
        ),
        QPointF(
            graphRect.right() - arrow,
            origin.y() + arrow
        )
    );


    /*
     * -X
     */
    painter.drawLine(
        QPointF(
            graphRect.left(),
            origin.y()
        ),
        QPointF(
            graphRect.left() + arrow,
            origin.y() - arrow
        )
    );

    painter.drawLine(
        QPointF(
            graphRect.left(),
            origin.y()
        ),
        QPointF(
            graphRect.left() + arrow,
            origin.y() + arrow
        )
    );


    /*
     * +Y
     */
    painter.drawLine(
        QPointF(
            origin.x(),
            graphRect.top()
        ),
        QPointF(
            origin.x() - arrow,
            graphRect.top() + arrow
        )
    );

    painter.drawLine(
        QPointF(
            origin.x(),
            graphRect.top()
        ),
        QPointF(
            origin.x() + arrow,
            graphRect.top() + arrow
        )
    );


    /*
     * -Y
     */
    painter.drawLine(
        QPointF(
            origin.x(),
            graphRect.bottom()
        ),
        QPointF(
            origin.x() - arrow,
            graphRect.bottom() - arrow
        )
    );

    painter.drawLine(
        QPointF(
            origin.x(),
            graphRect.bottom()
        ),
        QPointF(
            origin.x() + arrow,
            graphRect.bottom() - arrow
        )
    );


    /*
     * Etiquetas de ejes.
     */
    painter.drawText(
        QPointF(
            graphRect.right() - 25,
            origin.y() - 8
        ),
        "+X"
    );

    painter.drawText(
        QPointF(
            graphRect.left() + 5,
            origin.y() - 8
        ),
        "-X"
    );

    painter.drawText(
        QPointF(
            origin.x() + 8,
            graphRect.top() + 18
        ),
        "+Y"
    );

    painter.drawText(
        QPointF(
            origin.x() + 8,
            graphRect.bottom() - 8
        ),
        "-Y"
    );


    /*
     * Origen.
     */
    painter.drawEllipse(
        origin,
        4,
        4
    );

    painter.drawText(
        origin +
            QPointF(
                7,
                -7
            ),
        "(0,0)"
    );


    /*
     * Sentido de avance.
     */
    painter.drawText(
        QPointF(
            origin.x() + 25,
            graphRect.top() + 18
        ),
        "ADELANTE"
    );


    /*
     * ========================================================
     * Regiones de los 6 cuerpos
     * ========================================================
     */

    QPen regionPen(
        palette().highlight().color(),
        2,
        Qt::DashLine
    );

    painter.setPen(
        regionPen
    );


    for (int body = 0;
         body < BODY_COUNT;
         ++body)
    {
        const BodyRegion &region =
            bodyRegions[
                static_cast<std::size_t>(body)
            ];

        if (!region.valid)
        {
            continue;
        }

        const bool yValid =
            region.yMinMm >= 0.0 &&
            region.yMaxMm >= 0.0;

        QColor regionColor =
            yValid
                ? QColor(0, 150, 0)
                : QColor(200, 0, 0);

        QPen regionPen(
            regionColor,
            2,
            Qt::DashLine
        );

        painter.setPen(
            regionPen
        );


        const QPointF p1 =
            machineToScreen(
                region.xMinMm,
                region.yMaxMm
            );

        const QPointF p2 =
            machineToScreen(
                region.xMaxMm,
                region.yMinMm
            );


        QRectF bodyRect(
            p1,
            p2
        );

        bodyRect =
            bodyRect.normalized();


        painter.drawRect(
            bodyRect
        );


        painter.setPen(
            QPen(
                regionColor,
                2
            )
        );


        /*
         * Nombre del cuerpo.
         */
        painter.drawText(
            bodyRect,
            Qt::AlignCenter,
            QString(
                "C%1"
            ).arg(
                body + 1
            )
        );

        painter.setPen(
            QColor(0, 150, 0)
        );

        painter.drawText(
            20,
            height() - 12,
            "VERDE: región válida"
        );

        painter.setPen(
            QColor(200, 0, 0)
        );

        painter.drawText(
            170,
            height() - 12,
            "ROJO: Y negativo"
        );
    }


    /*
    * ========================================================
    * Cámaras frontales 3D
    * ========================================================
    */
    for (int camera = 0;
        camera < CAMERA_COUNT;
        ++camera)
    {
        const CameraPosition &position =
            cameraPositions[
                static_cast<std::size_t>(camera)
            ];

        if (!position.enabled)
        {
            continue;
        }

        const QPointF cameraPoint =
            machineToScreen(
                position.xM,
                position.yM
            );

        painter.setPen(
            QPen(
                QColor(140, 0, 200),
                2
            )
        );

        painter.setBrush(
            QColor(140, 0, 200)
        );

        painter.drawEllipse(
            cameraPoint,
            6,
            6
        );

        painter.setBrush(
            Qt::NoBrush
        );

        const double labelOffsetY =
            -10.0 -
            static_cast<double>(camera % 3) * 14.0;

        const double labelOffsetX =
            8.0;

        painter.drawText(
            cameraPoint +
                QPointF(
                    labelOffsetX,
                    labelOffsetY
                ),
            QString("CAM%1").arg(camera + 1)
        );
    }

    /*
     * Marco externo.
     */
    painter.setPen(
        QPen(
            palette().mid().color(),
            1
        )
    );

    painter.drawRect(
        rect().adjusted(
            0,
            0,
            -1,
            -1
        )
    );
}