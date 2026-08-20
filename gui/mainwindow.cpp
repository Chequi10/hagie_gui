#include "mainwindow.h"

#include <QAction>
#include <QApplication>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QStackedWidget>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QTimer>
#include "stm32/stm32_worker.h"
#include "core/height_target_controller.h"
#include <QStringList>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSettings>
#include <QScrollArea>
#include "vision/vision_height_source.h"
#include "vision/vision_3d_worker.h"



MainWindow::MainWindow(
    HagieState *hagieState,
    STM32Worker *stm32Worker,
    HeightTargetController *heightTargetController,
    VisionHeightSource *visionHeightSource,
    Vision3DProcessor *vision3DProcessor,
    Vision3DWorker *vision3DWorker,
    QWidget *parent)
    : QMainWindow(parent),
    state(hagieState),
    stm32Worker(stm32Worker),
    heightTargetController(heightTargetController),
    visionHeightSource(visionHeightSource),
    vision3DProcessor(vision3DProcessor),
    vision3DWorker(vision3DWorker),
    centralStack(nullptr)
{
    setWindowTitle(
        "Hagie Control"
    );

    resize(
        1280,
        720
    );


    /*
     * ========================================================
     * Widget central con múltiples páginas
     * ========================================================
     */
    centralStack =
        new QStackedWidget(this);


    /*
     * Página 0
     */
    centralStack->addWidget(
        createDashboardPage()
    );


    /*
     * Página 1
     */
    centralStack->addWidget(
        createCamerasPage()
    );


    /*
     * Página 2
     */
    centralStack->addWidget(
        createFaultsPage()
    );


    /*
     * Página 3
     */
    centralStack->addWidget(
        createTestsPage()
    );


    /*
     * Página 4
     */
    centralStack->addWidget(
        createConfigurationPage()
    );


    setCentralWidget(
        centralStack
    );


    /*
     * ========================================================
     * Callback de conexión STM32
     * ========================================================
     */
    if (this->stm32Worker != nullptr)
    {
        this->stm32Worker->setConnectionCallback(
            [this](bool connected)
            {
                /*
                 * El callback viene desde el hilo STM32.
                 *
                 * Pasamos la modificación de widgets
                 * al hilo principal de Qt.
                 */
                QTimer::singleShot(
                    0,
                    this,
                    [this, connected]()
                    {
                        /*
                         * ========================================
                         * STM32 DESCONECTADA
                         * ========================================
                         */
                        if (!connected)
                        {
                            for (
                                std::size_t body = 0;
                                body < HagieState::BODY_COUNT;
                                ++body
                            )
                            {
                                /*
                                 * Cancelar AUTO local.
                                 */
                                testAutoEnabled[body] =
                                    false;
                                testVisionAutoEnabled[body] =
                                    false;    

                                /*
                                 * Estado visual MANUAL.
                                 */
                                if (testModeLabels[body] != nullptr)
                                {
                                    testModeLabels[body]->setText(
                                        "Modo: MANUAL"
                                    );

                                    testModeLabels[body]->setStyleSheet(
                                        "font-weight: bold;"
                                    );
                                }


                                /*
                                 * Sin STM32 no permitimos
                                 * órdenes de movimiento.
                                 */
                                if (testDownButtons[body] != nullptr)
                                {
                                    testDownButtons[body]->setEnabled(
                                        false
                                    );
                                }


                                if (testUpButtons[body] != nullptr)
                                {
                                    testUpButtons[body]->setEnabled(
                                        false
                                    );
                                }


                                if (testAutoButtons[body] != nullptr)
                                {
                                    testAutoButtons[body]->setEnabled(
                                        false
                                    );
                                }


                                if (testManualButtons[body] != nullptr)
                                {
                                    testManualButtons[body]->setEnabled(
                                        false
                                    );
                                }
                            }


                            /*
                             * Estado compartido también seguro.
                             */
                            if (state != nullptr)
                            {
                                for (
                                    std::size_t body = 0;
                                    body < HagieState::BODY_COUNT;
                                    ++body
                                )
                                {
                                    state->setBodyAutoMode(
                                        body,
                                        false
                                    );

                                    state->setBodyValveCommand(
                                        body,
                                        0
                                    );
                                }
                            }


                            return;
                        }


                        /*
                         * ========================================
                         * STM32 RECONECTADA
                         * ========================================
                         *
                         * Nunca recuperar automáticamente
                         * el AUTO anterior.
                         */
                        for (
                            std::size_t body = 0;
                            body < HagieState::BODY_COUNT;
                            ++body
                        )
                        {
                            testAutoEnabled[body] =
                                false;

                            testVisionAutoEnabled[body] =
                                false;


                            if (testModeLabels[body] != nullptr)
                            {
                                testModeLabels[body]->setText(
                                    "Modo: MANUAL"
                                );

                                testModeLabels[body]->setStyleSheet(
                                    "font-weight: bold;"
                                );
                            }


                            if (testAutoButtons[body] != nullptr)
                            {
                                testAutoButtons[body]->setEnabled(
                                    true
                                );
                            }


                            if (testManualButtons[body] != nullptr)
                            {
                                testManualButtons[body]->setEnabled(
                                    true
                                );
                            }
                        }


                        /*
                         * No reenviamos objetivos viejos.
                         *
                         * El STM32Worker ya envía STOP ALL
                         * inmediatamente al reconectar.
                         */
                    }
                );
            }
        );
    }


    /*
     * ========================================================
     * Configuración
     * ========================================================
     */
    loadConfiguration();

    syncConfigurationToWorker();


    /*
     * ========================================================
     * Menús / barra de estado
     * ========================================================
     */
    createMenus();

    createStatusBar();

    updateDashboard();

    updateFaultPage();


    /*
     * ========================================================
     * Timer de actualización GUI
     * ========================================================
     */
    dashboardTimer =
        new QTimer(this);


    connect(
        dashboardTimer,
        &QTimer::timeout,
        this,
        &MainWindow::updateDashboard
    );


    dashboardTimer->start(
        100
    );
}

void MainWindow::updateTestPage()
{
    if (state == nullptr)
    {
        return;
    }


    constexpr int32_t
        ENCODER_MOVEMENT_THRESHOLD_MM =
            2;


    constexpr int64_t
        ENCODER_STILL_TIMEOUT_MS =
            300;


    auto now =
        std::chrono::steady_clock::now();


    /*
     * Estado de conexión real.
     */
    HagieState::SystemState system =
        state->getSystemState();


    bool stm32Connected =
        system.stm32_connected;


    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        HagieState::BodyState bodyState =
            state->getBodyState(
                body
            );

        uint64_t nowMs =
            static_cast<uint64_t>(
                std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                        std::chrono::steady_clock::now()
                            .time_since_epoch()
                    ).count()
            );

        constexpr uint64_t VISION_STALE_TIMEOUT_MS =
            500;

        bool visionFresh =
            bodyState.vision_valid &&
            bodyState.vision_timestamp_ms != 0 &&
            (nowMs - bodyState.vision_timestamp_ms) <=
                VISION_STALE_TIMEOUT_MS;    


        /*
         * ====================================================
         * DIAGNÓSTICO DEL ENCODER
         * ====================================================
         */

        if (!testEncoderInitialized[body])
        {
            testPreviousHeight[body] =
                bodyState.height_mm;


            testEncoderInitialized[body] =
                true;


            testEncoderDirection[body] =
                0;


            testLastEncoderMovementTime[body] =
                now;


            testEncoderStatusLabels[body]
                ->setText(
                    "Encoder: QUIETO"
                );


            testEncoderStatusLabels[body]
                ->setStyleSheet(
                    "font-weight: bold;"
                    "color: gray;"
                );


            testEncoderDeltaLabels[body]
                ->setText(
                    "Cambio: 0 mm"
                );
        }
        else
        {
            int32_t delta =
                static_cast<int32_t>(
                    bodyState.height_mm
                )
                -
                static_cast<int32_t>(
                    testPreviousHeight[body]
                );


            if (
                delta >=
                ENCODER_MOVEMENT_THRESHOLD_MM
            )
            {
                testEncoderDirection[body] =
                    1;


                testLastEncoderMovementTime[body] =
                    now;


                testEncoderStatusLabels[body]
                    ->setText(
                        "Encoder: SUBIENDO"
                    );


                testEncoderStatusLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "color: green;"
                    );


                testEncoderDeltaLabels[body]
                    ->setText(
                        QString(
                            "Cambio: +%1 mm"
                        )
                            .arg(delta)
                    );


                testPreviousHeight[body] =
                    bodyState.height_mm;
            }
            else if (
                delta <=
                -ENCODER_MOVEMENT_THRESHOLD_MM
            )
            {
                testEncoderDirection[body] =
                    -1;


                testLastEncoderMovementTime[body] =
                    now;


                testEncoderStatusLabels[body]
                    ->setText(
                        "Encoder: BAJANDO"
                    );


                testEncoderStatusLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "color: green;"
                    );


                testEncoderDeltaLabels[body]
                    ->setText(
                        QString(
                            "Cambio: %1 mm"
                        )
                            .arg(delta)
                    );


                testPreviousHeight[body] =
                    bodyState.height_mm;
            }
            else
            {
                auto elapsed =
                    std::chrono::
                        duration_cast<
                            std::chrono::
                                milliseconds
                        >(
                            now -
                            testLastEncoderMovementTime[
                                body
                            ]
                        ).count();


                if (
                    elapsed >=
                    ENCODER_STILL_TIMEOUT_MS
                )
                {
                    testEncoderDirection[body] =
                        0;


                    testEncoderStatusLabels[body]
                        ->setText(
                            "Encoder: QUIETO"
                        );


                    testEncoderStatusLabels[body]
                        ->setStyleSheet(
                            "font-weight: bold;"
                            "color: gray;"
                        );


                    testEncoderDeltaLabels[body]
                        ->setText(
                            "Cambio: 0 mm"
                        );
                }
            }
        }


        /*
         * ====================================================
         * TELEMETRÍA
         * ====================================================
         */

        testHeightLabels[body]
            ->setText(
                QString(
                    "Altura encoder: %1 mm"
                )
                    .arg(
                        bodyState.height_mm
                    )
            );


        if (visionFresh)
        {
            testVisionHeightLabels[body]
                ->setText(
                    QString(
                        "Visión 3D: %1 mm"
                    )
                        .arg(
                            bodyState.vision_height_mm
                        )
                );


            if (heightTargetController != nullptr)
                {
                    HeightTargetController::BodyConfiguration
                        configuration =
                            heightTargetController
                                ->getBodyConfiguration(
                                    body
                                );

                    configuration.offset_mm =
                        configVisionOffsetSpin[body]
                            ->value();

                    heightTargetController
                        ->setBodyConfiguration(
                            body,
                            configuration
                        );

                    HeightTargetController::TargetResult
                        targetResult =
                            heightTargetController
                                ->calculateTarget(
                                    body,
                                    bodyState.vision_height_mm,
                                    bodyState.vision_valid
                                );

                if (targetResult.valid)
                {
                    testVisionTargetLabels[body]
                        ->setText(
                            QString(
                                "Objetivo 3D: %1 mm"
                            )
                                .arg(
                                    targetResult.target_mm
                                )
                        );
                }
                else
                {
                    testVisionTargetLabels[body]
                        ->setText(
                            "Objetivo 3D: NO VÁLIDO"
                        );
                }
            }
            else
            {
                testVisionTargetLabels[body]
                    ->setText(
                        "Objetivo 3D: ---"
                    );
            }
        }
        else
        {
            testVisionHeightLabels[body]
                ->setText(
                    "Visión 3D: NO VÁLIDA"
                );

            testVisionTargetLabels[body]
                ->setText(
                    "Objetivo 3D: NO VÁLIDO"
                );
        }


        testValveLabels[body]
            ->setText(
                QString(
                    "Válvula: %1"
                )
                    .arg(
                        bodyState.valve_command
                    )
            );


        /*
         * ====================================================
         * FALLAS
         * ====================================================
         */

        if (bodyState.faults == 0)
        {
            testFaultLabels[body]
                ->setText(
                    "Estado: OK"
                );


            testFaultLabels[body]
                ->setStyleSheet(
                    "font-weight: bold;"
                    "color: green;"
                );


            testBodyFrames[body]
                ->setStyleSheet(
                    ""
                );


            /*
             * Movimiento manual solamente si:
             *
             * - STM32 conectada
             * - cuerpo NO está AUTO
             */
            bool manualEnabled =
                stm32Connected &&
                !testAutoEnabled[body] &&
                !testVisionAutoEnabled[body];


            testUpButtons[body]
                ->setEnabled(
                    manualEnabled
                );


            testDownButtons[body]
                ->setEnabled(
                    manualEnabled
                );
        }
        else
        {
            QStringList faults;


            if (
                (bodyState.faults & 0x01U)
                != 0
            )
            {
                faults
                    << "ENCODER_TIMEOUT";
            }


            if (
                (bodyState.faults & 0x02U)
                != 0
            )
            {
                faults
                    << "ENCODER_RANGE";
            }


            if (
                (bodyState.faults & 0x04U)
                != 0
            )
            {
                faults
                    << "NO_MOVEMENT";
            }


            if (
                (bodyState.faults & 0x08U)
                != 0
            )
            {
                faults
                    << "MIN_LIMIT";
            }


            if (
                (bodyState.faults & 0x10U)
                != 0
            )
            {
                faults
                    << "MAX_LIMIT";
            }


            if (
                (bodyState.faults & 0x20U)
                != 0
            )
            {
                faults
                    << "TARGET_TIMEOUT";
            }


            if (
                (bodyState.faults & 0x40U)
                != 0
            )
            {
                faults
                    << "VALVE_ERROR";
            }


            testFaultLabels[body]
                ->setText(
                    "FALLA: "
                    + faults.join(
                        " | "
                    )
                );


            testFaultLabels[body]
                ->setStyleSheet(
                    "font-weight: bold;"
                    "color: red;"
                );


            testBodyFrames[body]
                ->setStyleSheet(
                    "QFrame {"
                    "border: 2px solid red;"
                    "}"
                );


            testUpButtons[body]
                ->setEnabled(
                    false
                );


            testDownButtons[body]
                ->setEnabled(
                    false
                );
        }


        /*
         * ====================================================
         * AUTO / MANUAL SEGÚN CONEXIÓN
         * ====================================================
         */

        if (!stm32Connected)
        {
            /*
             * Sin comunicación:
             * ninguna orden nueva.
             */
            testAutoButtons[body]
                ->setEnabled(
                    false
                );


            testManualButtons[body]
                ->setEnabled(
                    false
                );


            testAutoEnabled[body] =
                false;

            testVisionAutoEnabled[body] =
                false;


            testModeLabels[body]
                ->setText(
                    "Modo: MANUAL"
                );


            testModeLabels[body]
                ->setStyleSheet(
                    "font-weight: bold;"
                );
        }
        else
        {
            testAutoButtons[body]
                ->setEnabled(
                    true
                );


            testManualButtons[body]
                ->setEnabled(
                    true
                );
        }
    }
}


MainWindow::~MainWindow()
{
}


/*
 * ============================================================
 * MENÚS
 * ============================================================
 */

void MainWindow::createMenus()
{
    /*
     * ARCHIVO
     */
    QMenu *fileMenu =
        menuBar()->addMenu("Archivo");

    QAction *exitAction =
        fileMenu->addAction("Salir");

    connect(
        exitAction,
        &QAction::triggered,
        qApp,
        &QApplication::quit
    );


    /*
     * OPERACIÓN
     */
    QMenu *operationMenu =
        menuBar()->addMenu("Operación");

    QAction *dashboardAction =
        operationMenu->addAction(
            "Vista general"
        );

    QAction *cameraAction =
        operationMenu->addAction(
            "Cámaras"
        );

    operationMenu->addSeparator();

    QAction *stopAction =
        operationMenu->addAction(
            "PARADA TOTAL"
        );


    connect(
        dashboardAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(0);
        }
    );

    connect(
        cameraAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(1);
        }
    );

    /*
     * Más adelante stopAction llamará:
     *
     * stm32.stop_all_valves();
     */
    connect(
        stopAction,
        &QAction::triggered,
        this,
        []()
        {
            // TODO:
            // parada total real
        }
    );


    /*
     * CONFIGURACIÓN
     */
    QMenu *configMenu =
        menuBar()->addMenu(
            "Configuración"
        );

    QAction *configurationAction =
        configMenu->addAction(
            "Configuración general"
        );

    configMenu->addAction("Máquina");
    configMenu->addAction("Cuerpos");
    configMenu->addAction("Encoders");
    configMenu->addAction("Cámaras");
    configMenu->addAction("IA");
    configMenu->addAction("STM32");
    configMenu->addAction("CAN / Axiomatic");

    connect(
        configurationAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(4);
        }
    );


    /*
     * TEST
     */
    QMenu *testMenu =
        menuBar()->addMenu("Test");

    QAction *testPageAction =
        testMenu->addAction(
            "Panel de test"
        );

    testMenu->addSeparator();

    testMenu->addAction("Válvulas");
    testMenu->addAction("Encoders");
    testMenu->addAction("Comunicación STM32");
    testMenu->addAction("IMU");
    testMenu->addAction("Cámaras");

    connect(
        testPageAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(3);
        }
    );


    /*
     * DIAGNÓSTICO
     */
    QMenu *diagnosticMenu =
        menuBar()->addMenu(
            "Diagnóstico"
        );

    QAction *faultAction =
        diagnosticMenu->addAction(
            "Fallas"
        );

    diagnosticMenu->addAction(
        "Estado del sistema"
    );

    diagnosticMenu->addAction(
        "Comunicaciones"
    );

    diagnosticMenu->addAction(
        "Logs"
    );

    connect(
        faultAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(2);
        }
    );


    /*
     * AYUDA
     */
    QMenu *helpMenu =
        menuBar()->addMenu("Ayuda");

    helpMenu->addAction(
        "Información del sistema"
    );

    helpMenu->addAction(
        "Acerca de Hagie Control"
    );
}


/*
 * ============================================================
 * VISTA GENERAL
 * ============================================================
 */

QWidget *MainWindow::createDashboardPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *mainLayout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "HAGIE - CONTROL DE ALTURA"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 24px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(title);


    /*
     * Panel de 6 cuerpos.
     */
    QGridLayout *bodyGrid =
        new QGridLayout();


    for (int body = 0;
         body < 6;
         ++body)
    {
        QFrame *frame =
            new QFrame();

        frame->setFrameShape(
            QFrame::StyledPanel
        );

        QVBoxLayout *layout =
            new QVBoxLayout(frame);


        QLabel *bodyTitle =
            new QLabel(
                QString("CUERPO %1")
                    .arg(body + 1)
            );

        bodyTitle->setAlignment(
            Qt::AlignCenter
        );

        bodyTitle->setStyleSheet(
            "font-weight: bold;"
            "font-size: 16px;"
        );


        heightLabels[body] =
            new QLabel("Altura: --- mm");

        targetLabels[body] =
            new QLabel("Objetivo: --- mm");

        modeLabels[body] =
            new QLabel("Modo: MANUAL");

        valveLabels[body] =
            new QLabel("Válvula: 0");

        faultLabels[body] =
            new QLabel("Falla: OK");


        layout->addWidget(heightLabels[body]);
        layout->addWidget(targetLabels[body]);
        layout->addWidget(modeLabels[body]);
        layout->addWidget(valveLabels[body]);
        layout->addWidget(faultLabels[body]);


        bodyGrid->addWidget(
            frame,
            body / 3,
            body % 3
        );
    }


    mainLayout->addLayout(bodyGrid);


    /*
     * Parada total.
     */
    QPushButton *stopButton =
        new QPushButton(
            "PARADA TOTAL"
        );

    stopButton->setMinimumHeight(70);

    stopButton->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
        "background-color: red;"
        "color: white;"
    );

    mainLayout->addWidget(
        stopButton
    );


    return page;
}


/*
 * ============================================================
 * CÁMARAS
 * ============================================================
 */

QWidget *MainWindow::createCamerasPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *layout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "CÁMARAS"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    layout->addWidget(title);


    /*
     * Cámara predominante.
     *
     * Más adelante acá irá el widget
     * de video real.
     */
    QLabel *mainCamera =
        new QLabel(
            "CÁMARA PRINCIPAL\n\n"
            "VIDEO"
        );

    mainCamera->setAlignment(
        Qt::AlignCenter
    );

    mainCamera->setMinimumHeight(250);

    mainCamera->setStyleSheet(
        "background-color: black;"
        "color: white;"
        "font-size: 28px;"
    );

    layout->addWidget(
        mainCamera,
        1
    );


    /*
     * Miniaturas.
     */
    QHBoxLayout *cameraSelector =
        new QHBoxLayout();


    for (int camera = 0;
         camera < 4;
         ++camera)
    {
        QPushButton *button =
            new QPushButton(
                QString("Cámara %1")
                    .arg(camera + 1)
            );

        button->setMinimumHeight(
            60
        );

        cameraSelector->addWidget(
            button
        );
    }


    layout->addLayout(
        cameraSelector
    );


    return page;
}


/*
 * ============================================================
 * FALLAS
 * ============================================================
 */

QWidget *MainWindow::createFaultsPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *mainLayout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "DIAGNÓSTICO DE FALLAS"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(title);


    /*
     * Fallas globales.
     */
    systemFaultsLabel =
        new QLabel(
            "Fallas globales: ---"
        );

    systemFaultsLabel->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(
        systemFaultsLabel
    );


    /*
     * Fallas individuales.
     */
    QGridLayout *faultGrid =
        new QGridLayout();


    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        QFrame *frame =
            new QFrame();

        frame->setFrameShape(
            QFrame::StyledPanel
        );

        QVBoxLayout *layout =
            new QVBoxLayout(frame);


        QLabel *bodyTitle =
            new QLabel(
                QString("CUERPO %1")
                    .arg(body + 1)
            );

        bodyTitle->setAlignment(
            Qt::AlignCenter
        );

        bodyTitle->setStyleSheet(
            "font-size: 16px;"
            "font-weight: bold;"
        );


        bodyFaultDetailLabels[body] =
            new QLabel("OK");

        bodyFaultDetailLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        bodyFaultDetailLabels[body]
            ->setWordWrap(true);


        QPushButton *clearButton =
            new QPushButton(
                "Borrar NO_MOVEMENT"
            );

        connect(
            clearButton,
            &QPushButton::clicked,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr)
                {
                    return;
                }

                stm32Worker->clearBodyFault(
                    static_cast<uint8_t>(body)
                );
            }
        );


        layout->addWidget(
            bodyTitle
        );

        layout->addWidget(
            bodyFaultDetailLabels[body]
        );

        layout->addWidget(
            clearButton
        );


        faultGrid->addWidget(
            frame,
            body / 3,
            body % 3
        );
    }


    mainLayout->addLayout(
        faultGrid
    );

    mainLayout->addStretch();

    return page;
}


/*
 * ============================================================
 * TEST
 * ============================================================
 */

QWidget *MainWindow::createTestsPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *mainLayout =
        new QVBoxLayout(page);

    mainLayout->setSizeConstraint(
        QLayout::SetMinimumSize
    );


    QLabel *title =
        new QLabel(
            "PANEL DE TEST - VÁLVULAS Y ENCODERS"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(title);


    /*
     * ========================================================
     * Selector de intensidad para modo MANUAL
     * ========================================================
     */
    QHBoxLayout *commandLayout =
        new QHBoxLayout();

    QLabel *commandLabel =
        new QLabel(
            "Comando de válvula:"
        );

    QComboBox *commandCombo =
        new QComboBox();

    commandCombo->addItem("100", 100);
    commandCombo->addItem("200", 200);
    commandCombo->addItem("300", 300);
    commandCombo->addItem("400", 400);
    commandCombo->addItem("500", 500);
    commandCombo->addItem("750", 750);
    commandCombo->addItem("1000", 1000);

    /*
     * Valor inicial = 300.
     */
    commandCombo->setCurrentIndex(2);

    testValveCommand = 300;


    connect(
        commandCombo,
        QOverload<int>::of(
            &QComboBox::currentIndexChanged
        ),
        this,
        [this, commandCombo](int)
        {
            testValveCommand =
                commandCombo
                    ->currentData()
                    .toInt();
        }
    );


    commandLayout->addStretch();

    commandLayout->addWidget(
        commandLabel
    );

    commandLayout->addWidget(
        commandCombo
    );

    commandLayout->addStretch();


    mainLayout->addLayout(
        commandLayout
    );


    /*
     * ========================================================
     * Grid de los seis cuerpos
     * ========================================================
     */
    QGridLayout *bodyGrid =
        new QGridLayout();


    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        testBodyFrames[body] =
            new QFrame();

        QFrame *frame =
            testBodyFrames[body];

        frame->setFrameShape(
            QFrame::StyledPanel
        );

        frame->setMinimumHeight(
            280
        );


        QVBoxLayout *bodyLayout =
            new QVBoxLayout(frame);

        bodyLayout->setContentsMargins(
            8,
            5,
            8,
            5
        );

        bodyLayout->setSpacing(
            3
        );    


        /*
         * Título.
         */
        QLabel *bodyTitle =
            new QLabel(
                QString("CUERPO %1")
                    .arg(body + 1)
            );

        bodyTitle->setAlignment(
            Qt::AlignCenter
        );

        bodyTitle->setStyleSheet(
            "font-size: 16px;"
            "font-weight: bold;"
        );


        /*
         * Estado actual.
         */
        testHeightLabels[body] =
            new QLabel(
                "Altura: --- mm"
            );

        testHeightLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );


        /*
         * Altura detectada por visión 3D.
         */
        testVisionHeightLabels[body] =
            new QLabel(
                "Visión 3D: --- mm"
            );

        testVisionHeightLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        testVisionHeightLabels[body]
            ->setStyleSheet(
                "font-weight: bold;"
                "color: #2b6cff;"
            );


        /*
         * Objetivo calculado a partir de visión.
         * Todavía NO se envía a la STM32.
         */
        testVisionTargetLabels[body] =
            new QLabel(
                "Objetivo 3D: --- mm"
            );

        testVisionTargetLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        testVisionTargetLabels[body]
            ->setStyleSheet(
                "font-weight: bold;"
                "color: #7a2cff;"
            );


        testEncoderStatusLabels[body] =
            new QLabel(
                "Encoder: ---"
            );

        testEncoderStatusLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );


        testEncoderDeltaLabels[body] =
            new QLabel(
                "Cambio: 0 mm"
            );

        testEncoderDeltaLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );


        testValveLabels[body] =
            new QLabel(
                "Válvula: 0"
            );

        testValveLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );


        testFaultLabels[body] =
            new QLabel(
                "Estado: OK"
            );

        testFaultLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        testFaultLabels[body]
            ->setStyleSheet(
                "font-weight: bold;"
            );


        /*
         * ====================================================
         * Altura objetivo AUTO
         * ====================================================
         */
        QHBoxLayout *targetLayout =
            new QHBoxLayout();

        QLabel *targetLabel =
            new QLabel(
                "Objetivo:"
            );


        testTargetHeightSpin[body] =
            new QSpinBox();

        testTargetHeightSpin[body]
            ->setRange(
                0,
                2000
            );

        testTargetHeightSpin[body]
            ->setSuffix(
                " mm"
            );

        /*
        * Recuperar el último objetivo utilizado
        * solamente como valor de interfaz.
        *
        * Esto NO activa AUTO.
        */
        QSettings testSettings(
            "hagie_config.ini",
            QSettings::IniFormat
        );

        QString targetKey =
            QString("Test/target_body_%1")
                .arg(body);

        int savedTarget =
            testSettings.value(
                targetKey,
                500
            ).toInt();

        testTargetHeightSpin[body]
            ->setValue(
                savedTarget
            );

        connect(
            testTargetHeightSpin[body],
            QOverload<int>::of(
                &QSpinBox::valueChanged
            ),
            this,
            [body](int value)
            {
                QSettings settings(
                    "hagie_config.ini",
                    QSettings::IniFormat
                );

                QString key =
                    QString("Test/target_body_%1")
                        .arg(body);

                settings.setValue(
                    key,
                    value
                );

                settings.sync();
            }
        );    



        targetLayout->addWidget(
            targetLabel
        );

        targetLayout->addWidget(
            testTargetHeightSpin[body]
        );


        /*
         * ====================================================
         * Estado MANUAL / AUTO
         * ====================================================
         */
        testModeLabels[body] =
            new QLabel(
                "Modo: MANUAL"
            );

        testModeLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        testModeLabels[body]
            ->setStyleSheet(
                "font-weight: bold;"
            );


        /*
         * ====================================================
         * Botones MANUAL / AUTO
         * ====================================================
         */
        QHBoxLayout *modeButtonLayout =
            new QHBoxLayout();


        testManualButtons[body] =
            new QPushButton(
                "MANUAL"
            );

        testAutoButtons[body] =
            new QPushButton(
                "AUTO TEST"
            );

        testVisionAutoButtons[body] =
            new QPushButton(
                "AUTO VISIÓN"
            );


        /*
         * Estado inicial.
         */
        testAutoEnabled[body] =
            false;

        testVisionAutoEnabled[body] =
            false;    


        /*
         * MANUAL.
         *
         * El opcode B con comando cero hace que
         * la STM32 vuelva a modo MANUAL.
         */
        connect(
            testManualButtons[body],
            &QPushButton::clicked,
            this,
            [this, body]()
            {
                testAutoEnabled[body] =
                    false;

                testVisionAutoEnabled[body] =
                    false;

                testAutoButtons[body]
                    ->setStyleSheet(
                        ""
                    );

                testVisionAutoButtons[body]
                    ->setStyleSheet(
                        ""
                    );

                testManualButtons[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "background-color: lightgreen;"
                    );

                testModeLabels[body]
                    ->setText(
                        "Modo: MANUAL"
                    );

                testModeLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                    );


                testDownButtons[body]
                    ->setEnabled(true);

                testUpButtons[body]
                    ->setEnabled(true);


                if (stm32Worker == nullptr)
                {
                    return;
                }


                stm32Worker->setValveCommand(
                    static_cast<uint8_t>(body),
                    0
                );
            }
        );


        /*
         * AUTO.
         *
         * Enviar inmediatamente la primera
         * consigna D.
         */
        connect(
            testAutoButtons[body],
            &QPushButton::clicked,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr)
                {
                    return;
                }


                uint16_t target =
                    static_cast<uint16_t>(
                        testTargetHeightSpin[body]
                            ->value()
                    );


                testAutoEnabled[body] =
                    true;

                testVisionAutoEnabled[body] =
                    false;

                testManualButtons[body]
                    ->setStyleSheet(
                        ""
                    );

                testVisionAutoButtons[body]
                    ->setStyleSheet(
                        ""
                    );

                testAutoButtons[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "background-color: lightgreen;"
                    );


                testModeLabels[body]
                    ->setText(
                        "Modo: AUTO TEST"
                    );
                    

                testModeLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "color: green;"
                    );


                /*
                 * Mientras estamos en AUTO no
                 * permitimos SUBIR / BAJAR manual.
                 */
                testDownButtons[body]
                    ->setEnabled(false);

                testUpButtons[body]
                    ->setEnabled(false);


                stm32Worker->setTargetHeight(
                    static_cast<uint8_t>(body),
                    target
                );
            }
        );

       
        connect(
            testVisionAutoButtons[body],
            &QPushButton::clicked,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr ||
                    state == nullptr ||
                    heightTargetController == nullptr)
                {
                    return;
                }

                HagieState::SystemState system =
                    state->getSystemState();

                HagieState::BodyState bodyState =
                    state->getBodyState(body);

                uint64_t nowMs =
                    static_cast<uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                                std::chrono::steady_clock::now()
                                    .time_since_epoch()
                            ).count()
                    );

                constexpr uint64_t VISION_STALE_TIMEOUT_MS =
                    500;

                bool visionFresh =
                    bodyState.vision_valid &&
                    bodyState.vision_timestamp_ms != 0 &&
                    (nowMs - bodyState.vision_timestamp_ms) <=
                        VISION_STALE_TIMEOUT_MS;    


                /*
                * Condiciones mínimas para permitir AUTO VISIÓN.
                */
                if (!system.stm32_connected ||
                    !system.vision_running ||
                    !visionFresh ||
                    bodyState.faults != 0)
                {
                    return;
                }


                /*
                * AUTO TEST y AUTO VISIÓN son excluyentes.
                */
                testAutoEnabled[body] =
                    false;

                testVisionAutoEnabled[body] =
                    true;


                /*
                * Estado visual de botones.
                */
                testManualButtons[body]
                    ->setStyleSheet(
                        ""
                    );

                testAutoButtons[body]
                    ->setStyleSheet(
                        ""
                    );

                testVisionAutoButtons[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "background-color: lightgreen;"
                    );


                testModeLabels[body]
                    ->setText(
                        "Modo: AUTO VISIÓN"
                    );

                testModeLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                        "color: green;"
                    );


                /*
                * En AUTO VISIÓN no permitimos
                * movimiento manual.
                */
                testDownButtons[body]
                    ->setEnabled(false);

                testUpButtons[body]
                    ->setEnabled(false);


                /*
                * Calcular objetivo usando visión + offset.
                */
                HeightTargetController::TargetResult
                    targetResult =
                        heightTargetController
                            ->calculateTarget(
                                body,
                                bodyState.vision_height_mm,
                                bodyState.vision_valid
                            );


                if (!targetResult.valid)
                {
                    testVisionAutoEnabled[body] =
                        false;

                    return;
                }


                /*
                * Primera consigna hacia STM32.
                */
                stm32Worker->setTargetHeight(
                    static_cast<uint8_t>(body),
                    targetResult.target_mm
                );
            }
        );
        
        modeButtonLayout->addWidget(
            testManualButtons[body]
        );

        modeButtonLayout->addWidget(
            testAutoButtons[body]
        );

        modeButtonLayout->addWidget(
            testVisionAutoButtons[body]
        );


        /*
         * ====================================================
         * Botones de movimiento manual
         * ====================================================
         */
        QWidget *manualControlWidget =
            new QWidget();

        manualControlWidget->setMinimumHeight(
            40
        );

        QHBoxLayout *buttonLayout =
            new QHBoxLayout(
                manualControlWidget
            );

        buttonLayout->setContentsMargins(
            0,
            3,
            0,
            3
        );

        buttonLayout->setSpacing(
            5
        );


        testDownButtons[body] =
            new QPushButton(
                "BAJAR"
            );

        QPushButton *stopButton =
            new QPushButton(
                "STOP"
            );

        testUpButtons[body] =
            new QPushButton(
                "SUBIR"
            );


        testDownButtons[body]->setMinimumHeight(30);
        stopButton->setMinimumHeight(30);
        testUpButtons[body]->setMinimumHeight(30);


        QPushButton *downButton =
            testDownButtons[body];

        QPushButton *upButton =
            testUpButtons[body];

        buttonLayout->addWidget(
            downButton
        );

        buttonLayout->addWidget(
            stopButton
        );

        buttonLayout->addWidget(
            upButton
        );




        /*
         * BAJAR.
         */
        connect(
            downButton,
            &QPushButton::pressed,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr)
                {
                    return;
                }


                stm32Worker->setValveCommand(
                    static_cast<uint8_t>(body),
                    static_cast<int16_t>(
                        -testValveCommand
                    )
                );
            }
        );


        connect(
            downButton,
            &QPushButton::released,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr)
                {
                    return;
                }


                stm32Worker->setValveCommand(
                    static_cast<uint8_t>(body),
                    0
                );
            }
        );


        /*
         * STOP.
         *
         * STOP también cancela AUTO.
         */
        connect(
            stopButton,
            &QPushButton::clicked,
            this,
            [this, body]()
            {
                testAutoEnabled[body] =
                    false;

                testVisionAutoEnabled[body] =
                    false;


                testModeLabels[body]
                    ->setText(
                        "Modo: MANUAL"
                    );

                testModeLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                    );


                


                if (stm32Worker == nullptr)
                {
                    return;
                }


                stm32Worker->setValveCommand(
                    static_cast<uint8_t>(body),
                    0
                );
            }
        );


        /*
         * SUBIR.
         */
        connect(
            upButton,
            &QPushButton::pressed,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr)
                {
                    return;
                }


                stm32Worker->setValveCommand(
                    static_cast<uint8_t>(body),
                    static_cast<int16_t>(
                        testValveCommand
                    )
                );
            }
        );


        connect(
            upButton,
            &QPushButton::released,
            this,
            [this, body]()
            {
                if (stm32Worker == nullptr)
                {
                    return;
                }


                stm32Worker->setValveCommand(
                    static_cast<uint8_t>(body),
                    0
                );
            }
        );


        /*
         * ====================================================
         * Construcción visual del cuerpo
         * ====================================================
         */
        bodyLayout->addWidget(
            bodyTitle
        );

        bodyLayout->addWidget(
            testHeightLabels[body]
        );

        bodyLayout->addWidget(
            testVisionHeightLabels[body]
        );

        bodyLayout->addWidget(
            testVisionTargetLabels[body]
        );

        bodyLayout->addWidget(
            testEncoderStatusLabels[body]
        );

        bodyLayout->addWidget(
            testEncoderDeltaLabels[body]
        );

        bodyLayout->addWidget(
            testValveLabels[body]
        );

        bodyLayout->addWidget(
            testFaultLabels[body]
        );


        bodyLayout->addLayout(
            targetLayout
        );

        bodyLayout->addWidget(
            testModeLabels[body]
        );

        bodyLayout->addLayout(
            modeButtonLayout
        );

        bodyLayout->addWidget(
            manualControlWidget
        );


        bodyGrid->addWidget(
            frame,
            body / 3,
            body % 3
        );
    }


    mainLayout->addLayout(
        bodyGrid
    );


    /*
     * ========================================================
     * Renovación periódica de consignas AUTO
     * ========================================================
     *
     * La STM32 tiene watchdog individual de consigna.
     * Mientras un cuerpo esté en AUTO, reenviamos D
     * periódicamente.
     */
    testAutoTimer =
        new QTimer(this);


    connect(
        testAutoTimer,
        &QTimer::timeout,
        this,
        [this]()
        {
            if (stm32Worker == nullptr)
            {
                return;
            }


            for (std::size_t body = 0;
                body < HagieState::BODY_COUNT;
                ++body)
            {
                /*
                * ====================================================
                * AUTO TEST
                * ====================================================
                */
                if (testAutoEnabled[body])
                {
                    uint16_t target =
                        static_cast<uint16_t>(
                            testTargetHeightSpin[body]
                                ->value()
                        );

                    stm32Worker->setTargetHeight(
                        static_cast<uint8_t>(body),
                        target
                    );

                    continue;
                }


                /*
                * ====================================================
                * AUTO VISIÓN
                * ====================================================
                */
                if (testVisionAutoEnabled[body])
                {
                    if (state == nullptr ||
                        heightTargetController == nullptr)
                    {
                        continue;
                    }

                    HagieState::SystemState system =
                        state->getSystemState();

                    HagieState::BodyState bodyState =
                        state->getBodyState(body);


                    /*
                    * Si se pierde alguna condición necesaria,
                    * salir de AUTO VISIÓN.
                    */
                   uint64_t nowMs =
                        static_cast<uint64_t>(
                            std::chrono::duration_cast<
                                std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now()
                                        .time_since_epoch()
                                ).count()
                        );

                    constexpr uint64_t VISION_STALE_TIMEOUT_MS =
                        500;

                    bool visionFresh =
                        bodyState.vision_valid &&
                        bodyState.vision_timestamp_ms != 0 &&
                        (nowMs - bodyState.vision_timestamp_ms) <=
                            VISION_STALE_TIMEOUT_MS;

                    if (!system.stm32_connected ||
                        !system.vision_running ||
                        !visionFresh ||
                        bodyState.faults != 0)
                    {
                        testVisionAutoEnabled[body] =
                            false;

                        testModeLabels[body]
                            ->setText(
                                "Modo: MANUAL"
                            );

                        testModeLabels[body]
                            ->setStyleSheet(
                                "font-weight: bold;"
                            );

                        testVisionAutoButtons[body]
                            ->setStyleSheet(
                                ""
                            );

                        testManualButtons[body]
                            ->setStyleSheet(
                                "font-weight: bold;"
                                "background-color: lightgreen;"
                            );

                        stm32Worker->setValveCommand(
                            static_cast<uint8_t>(body),
                            0
                        );

                        continue;
                    }


                    /*
                    * Calcular el objetivo 3D actual.
                    */
                    HeightTargetController::TargetResult
                        targetResult =
                            heightTargetController
                                ->calculateTarget(
                                    body,
                                    bodyState.vision_height_mm,
                                    visionFresh
                                );


                    if (!targetResult.valid)
                    {
                        testVisionAutoEnabled[body] =
                            false;

                        stm32Worker->setValveCommand(
                            static_cast<uint8_t>(body),
                            0
                        );

                        continue;
                    }


                    /*
                    * Renovar consigna hacia STM32.
                    */
                    stm32Worker->setTargetHeight(
                        static_cast<uint8_t>(body),
                        targetResult.target_mm
                    );
                }
            }
        }
    );


    /*
     * Reenviar consigna AUTO cada 500 ms.
     */
    testAutoTimer->start(
        500
    );


    /*
     * ========================================================
     * Botón global de seguridad
     * ========================================================
     */
    QPushButton *stopAllButton =
        new QPushButton(
            "DETENER TODAS LAS VÁLVULAS"
        );


    stopAllButton->setMinimumHeight(
        60
    );


    stopAllButton->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
        "background-color: red;"
        "color: white;"
    );


    connect(
        stopAllButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            /*
             * Cancelar AUTO de los seis cuerpos.
             */
            for (std::size_t body = 0;
                 body < HagieState::BODY_COUNT;
                 ++body)
            {
                testAutoEnabled[body] =
                    false;

                testVisionAutoEnabled[body] =
                    false;


                testModeLabels[body]
                    ->setText(
                        "Modo: MANUAL"
                    );

                testModeLabels[body]
                    ->setStyleSheet(
                        "font-weight: bold;"
                    );


                testDownButtons[body]
                    ->setEnabled(true);

                testUpButtons[body]
                    ->setEnabled(true);
            }


            if (stm32Worker == nullptr)
            {
                return;
            }


            stm32Worker->stopAllValves();
        }
    );


    /*
    * Contenedor exterior:
    * - panel de test desplazable
    * - parada total siempre visible abajo
    */
    QWidget *container =
        new QWidget();

    QVBoxLayout *containerLayout =
        new QVBoxLayout(container);

    containerLayout->setContentsMargins(
        0,
        0,
        0,
        0
    );


    /*
    * Todo el panel de test queda dentro
    * del área desplazable.
    */
    QScrollArea *scrollArea =
        new QScrollArea();

    scrollArea->setWidgetResizable(
        true
    );

    scrollArea->setWidget(
        page
    );

    scrollArea->setFrameShape(
        QFrame::NoFrame
    );

    containerLayout->addWidget(
        scrollArea,
        1
    );


    /*
    * Parada de emergencia siempre visible.
    */
    containerLayout->addWidget(
        stopAllButton
    );

    return container;
}

/*
 * ============================================================
 * CONFIGURACIÓN
 * ============================================================
 */

QWidget *MainWindow::createConfigurationPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *mainLayout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "CONFIGURACIÓN DE CUERPOS"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(title);


    
   

    /*
    * ========================================================
    * FUENTE DE VISIÓN 3D
    * ========================================================
    */
    QFrame *visionSourceFrame =
        new QFrame();

    visionSourceFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QHBoxLayout *visionSourceLayout =
        new QHBoxLayout(
            visionSourceFrame
        );

    QLabel *visionSourceLabel =
        new QLabel(
            "Fuente de visión 3D:"
        );

    configVisionSourceCombo =
    new QComboBox();


    configVisionSourceCombo->addItem(
        "SIMULACIÓN ALTURAS",
        0
    );

    configVisionSourceCombo->addItem(
        "SIMULACIÓN CÁMARAS 3D",
        1
    );

    configVisionSourceCombo->addItem(
        "CÁMARAS 3D REALES",
        2
    );


    connect(
        configVisionSourceCombo,
        QOverload<int>::of(
            &QComboBox::currentIndexChanged
        ),
        this,
        [this](int index)
        {
            if (visionHeightSource == nullptr)
            {
                return;
            }


            /*
            * ====================================================
            * MODO 0
            * SIMULACIÓN DIRECTA DE ALTURAS
            * ====================================================
            */
            if (index == 0)
            {
                /*
                * Detener el worker de nubes.
                */
                if (vision3DWorker != nullptr)
                {
                    vision3DWorker->stop();
                }


                /*
                * VisionHeightSource vuelve a generar
                * alturas simuladas internamente.
                */
                visionHeightSource->setSourceMode(
                    VisionHeightSource::SourceMode::SIMULATION
                );


                return;
            }


            /*
            * ====================================================
            * MODO 1
            * SIMULACIÓN DE CÁMARAS 3D
            * ====================================================
            */
            if (index == 1)
            {
                /*
                * Desactivar la simulación directa
                * de alturas.
                */
                visionHeightSource->setSourceMode(
                    VisionHeightSource::SourceMode::EXTERNAL
                );


                /*
                * Arrancar Vision3DWorker.
                *
                * En este momento tiene instaladas las
                * SimulatedPointCloudSource creadas
                * en main.cpp.
                */
                if (vision3DWorker != nullptr)
                {
                    if (!vision3DWorker->isRunning())
                    {
                        if (!vision3DWorker->start())
                        {
                            qWarning(
                                "No se pudo iniciar Vision3DWorker"
                            );
                        }
                    }
                }


                return;
            }


            /*
            * ====================================================
            * MODO 2
            * CÁMARAS 3D REALES
            * ====================================================
            *
            * Todavía no tenemos instalada una fuente
            * real ZED GMSL2.
            *
            * Por seguridad dejamos EXTERNAL pero
            * detenemos el worker actual, que todavía
            * contiene las fuentes simuladas.
            */
            if (index == 2)
            {
                if (vision3DWorker != nullptr)
                {
                    vision3DWorker->stop();
                }


                visionHeightSource->setSourceMode(
                    VisionHeightSource::SourceMode::EXTERNAL
                );


                return;
            }
        }
    );
    
    visionSourceLayout->addWidget(
        visionSourceLabel
    );

    visionSourceLayout->addWidget(
        configVisionSourceCombo
    );

    visionSourceLayout->addStretch();

    mainLayout->addWidget(
        visionSourceFrame
    );

    /*
    * ========================================================
    * CONFIGURACIÓN DE CÁMARAS 3D
    * ========================================================
    */
    QFrame *visionCameraFrame =
        new QFrame();

    visionCameraFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QVBoxLayout *visionCameraMainLayout =
        new QVBoxLayout(
            visionCameraFrame
        );


    QLabel *visionCameraTitle =
        new QLabel(
            "CONFIGURACIÓN DE CÁMARAS 3D"
        );

    visionCameraTitle->setAlignment(
        Qt::AlignCenter
    );

    visionCameraTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    visionCameraMainLayout->addWidget(
        visionCameraTitle
    );


    /*
    * Selector de cámara.
    */
    QHBoxLayout *cameraSelectorLayout =
        new QHBoxLayout();

    QLabel *cameraSelectorLabel =
        new QLabel(
            "Cámara:"
        );

    configVisionCameraCombo =
        new QComboBox();

    configVisionCameraCombo->addItem(
        "CÁMARA 1",
        0
    );

    configVisionCameraCombo->addItem(
        "CÁMARA 2",
        1
    );

    configVisionCameraCombo->addItem(
        "CÁMARA 3",
        2
    );


    connect(
        configVisionCameraCombo,
        QOverload<int>::of(
            &QComboBox::currentIndexChanged
        ),
        this,
        [this](int index)
        {
            if (index < 0)
            {
                return;
            }


            std::size_t newCamera =
                static_cast<std::size_t>(
                    index
                );


            if (newCamera >=
                Vision3DProcessor::CAMERA_COUNT)
            {
                return;
            }


            /*
            * Guardar en memoria los valores
            * de la cámara que estábamos editando.
            */
            saveVisionCameraFromWidgets(
                currentVisionCamera
            );


            /*
            * Cambiar de cámara.
            */
            currentVisionCamera =
                newCamera;


            /*
            * Mostrar los valores propios
            * de la nueva cámara.
            */
            loadVisionCameraIntoWidgets(
                currentVisionCamera
            );
        }
    );

    cameraSelectorLayout->addWidget(
        cameraSelectorLabel
    );

    cameraSelectorLayout->addWidget(
        configVisionCameraCombo
    );

    cameraSelectorLayout->addStretch();

    visionCameraMainLayout->addLayout(
        cameraSelectorLayout
    );


    /*
    * Cámara habilitada.
    */
    configVisionCameraEnabled =
        new QCheckBox(
            "Cámara habilitada"
        );

    visionCameraMainLayout->addWidget(
        configVisionCameraEnabled
    );


    /*
    * Cuerpos atendidos por la cámara.
    */
    QLabel *cameraBodiesLabel =
        new QLabel(
            "Cuerpos atendidos:"
        );

    visionCameraMainLayout->addWidget(
        cameraBodiesLabel
    );

    QHBoxLayout *cameraBodiesLayout =
        new QHBoxLayout();

    for (std::size_t body = 0;
        body < HagieState::BODY_COUNT;
        ++body)
    {
        configVisionCameraBodyChecks[body] =
            new QCheckBox(
                QString("C%1")
                    .arg(body + 1)
            );

        cameraBodiesLayout->addWidget(
            configVisionCameraBodyChecks[body]
        );
    }

    cameraBodiesLayout->addStretch();

    visionCameraMainLayout->addLayout(
        cameraBodiesLayout
    );


    /*
    * Geometría física de cámara.
    */
    QGridLayout *cameraGeometryLayout =
        new QGridLayout();


    QLabel *cameraPositionXLabel =
        new QLabel(
            "Posición X (mm)"
        );

    configVisionCameraPositionX =
        new QDoubleSpinBox();

    configVisionCameraPositionX->setRange(
        -20000.0,
        20000.0
    );

    configVisionCameraPositionX->setDecimals(
        1
    );


    QLabel *cameraPositionYLabel =
        new QLabel(
            "Posición Y (mm)"
        );

    configVisionCameraPositionY =
        new QDoubleSpinBox();

    configVisionCameraPositionY->setRange(
        -20000.0,
        20000.0
    );

    configVisionCameraPositionY->setDecimals(
        1
    );


    QLabel *cameraPositionZLabel =
        new QLabel(
            "Posición Z (mm)"
        );

    configVisionCameraPositionZ =
        new QDoubleSpinBox();

    configVisionCameraPositionZ->setRange(
        -5000.0,
        10000.0
    );

    configVisionCameraPositionZ->setDecimals(
        1
    );


    QLabel *cameraHeightLabel =
        new QLabel(
            "Altura cámara/piso (mm)"
        );

    configVisionCameraHeight =
        new QDoubleSpinBox();

    configVisionCameraHeight->setRange(
        0.0,
        10000.0
    );

    configVisionCameraHeight->setDecimals(
        1
    );


    QLabel *cameraRollLabel =
        new QLabel(
            "Roll montaje (°)"
        );

    configVisionCameraRoll =
        new QDoubleSpinBox();

    configVisionCameraRoll->setRange(
        -180.0,
        180.0
    );

    configVisionCameraRoll->setDecimals(
        2
    );


    QLabel *cameraPitchLabel =
        new QLabel(
            "Pitch montaje (°)"
        );

    configVisionCameraPitch =
        new QDoubleSpinBox();

    configVisionCameraPitch->setRange(
        -180.0,
        180.0
    );

    configVisionCameraPitch->setDecimals(
        2
    );


    cameraGeometryLayout->addWidget(
        cameraPositionXLabel,
        0,
        0
    );

    cameraGeometryLayout->addWidget(
        configVisionCameraPositionX,
        0,
        1
    );

    cameraGeometryLayout->addWidget(
        cameraPositionYLabel,
        0,
        2
    );

    cameraGeometryLayout->addWidget(
        configVisionCameraPositionY,
        0,
        3
    );


    cameraGeometryLayout->addWidget(
        cameraPositionZLabel,
        1,
        0
    );

    cameraGeometryLayout->addWidget(
        configVisionCameraPositionZ,
        1,
        1
    );

    cameraGeometryLayout->addWidget(
        cameraHeightLabel,
        1,
        2
    );

    cameraGeometryLayout->addWidget(
        configVisionCameraHeight,
        1,
        3
    );


    cameraGeometryLayout->addWidget(
        cameraRollLabel,
        2,
        0
    );

    cameraGeometryLayout->addWidget(
        configVisionCameraRoll,
        2,
        1
    );

    cameraGeometryLayout->addWidget(
        cameraPitchLabel,
        2,
        2
    );

    cameraGeometryLayout->addWidget(
        configVisionCameraPitch,
        2,
        3
    );


    visionCameraMainLayout->addLayout(
        cameraGeometryLayout
    );


    /*
    * Agregar panel completo a Configuración.
    */
    mainLayout->addWidget(
        visionCameraFrame
    );

    QGridLayout *bodyGrid =
        new QGridLayout();


    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        QFrame *frame =
            new QFrame();

        frame->setFrameShape(
            QFrame::StyledPanel
        );

        QVBoxLayout *bodyLayout =
            new QVBoxLayout(frame);


        QLabel *bodyTitle =
            new QLabel(
                QString("CUERPO %1")
                    .arg(body + 1)
            );

        bodyTitle->setAlignment(
            Qt::AlignCenter
        );

        bodyTitle->setStyleSheet(
            "font-size: 16px;"
            "font-weight: bold;"
        );

        bodyLayout->addWidget(
            bodyTitle
        );


        /*
         * Altura mínima.
         */
        QLabel *minLabel =
            new QLabel(
                "Altura mínima (mm)"
            );

        configMinHeightSpin[body] =
            new QSpinBox();

        configMinHeightSpin[body]
            ->setRange(
                0,
                2000
            );

        configMinHeightSpin[body]
            ->setValue(
                50
            );


        /*
         * Altura máxima.
         */
        QLabel *maxLabel =
            new QLabel(
                "Altura máxima (mm)"
            );

        configMaxHeightSpin[body] =
            new QSpinBox();

        configMaxHeightSpin[body]
            ->setRange(
                0,
                2000
            );

        configMaxHeightSpin[body]
            ->setValue(
                700
            );


        /*
         * Escala encoder.
         */
        QLabel *scaleLabel =
            new QLabel(
                "Escala encoder (mm/pulso)"
            );

        configEncoderScaleSpin[body] =
            new QDoubleSpinBox();

        configEncoderScaleSpin[body]
            ->setDecimals(5);

        configEncoderScaleSpin[body]
            ->setRange(
                0.00001,
                100.0
            );

        configEncoderScaleSpin[body]
            ->setSingleStep(
                0.001
            );

        configEncoderScaleSpin[body]
            ->setValue(
                1.0
            );


        /*
         * Sentido encoder.
         */
        QLabel *directionLabel =
            new QLabel(
                "Sentido encoder"
            );

        configEncoderDirectionCombo[body] =
            new QComboBox();

        configEncoderDirectionCombo[body]
            ->addItem(
                "Normal",
                1
            );

        configEncoderDirectionCombo[body]
            ->addItem(
                "Invertido",
                -1
            );


        bodyLayout->addWidget(
            minLabel
        );

        bodyLayout->addWidget(
            configMinHeightSpin[body]
        );

        bodyLayout->addWidget(
            maxLabel
        );

        bodyLayout->addWidget(
            configMaxHeightSpin[body]
        );

        bodyLayout->addWidget(
            scaleLabel
        );

        bodyLayout->addWidget(
            configEncoderScaleSpin[body]
        );

        bodyLayout->addWidget(
            directionLabel
        );


        QLabel *visionOffsetLabel =
            new QLabel(
                "Offset visión 3D (mm)"
            );

        configVisionOffsetSpin[body] =
            new QSpinBox();

        configVisionOffsetSpin[body]
            ->setRange(
                -1000,
                1000
            );

        configVisionOffsetSpin[body]
            ->setSingleStep(
                10
            );

        configVisionOffsetSpin[body]
            ->setValue(
                0
            );

        bodyLayout->addWidget(
            configEncoderDirectionCombo[body]
        );

        bodyLayout->addWidget(
            visionOffsetLabel
        );

        bodyLayout->addWidget(
            configVisionOffsetSpin[body]
        );


        bodyGrid->addWidget(
            frame,
            body / 3,
            body % 3
        );
    }


    mainLayout->addLayout(
        bodyGrid
    );


    /*
    * ========================================================
    * PARÁMETROS GLOBALES DE CONTROL Y SEGURIDAD
    * ========================================================
    */

    QFrame *controlFrame =
        new QFrame();

    controlFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QGridLayout *controlLayout =
        new QGridLayout(controlFrame);


    QLabel *controlTitle =
        new QLabel(
            "PARÁMETROS DE CONTROL Y SEGURIDAD"
        );

    controlTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    controlLayout->addWidget(
        controlTitle,
        0,
        0,
        1,
        4
    );


    /*
    * K 0x02
    * Umbral mínimo de comando.
    */
    QLabel *moveThresholdLabel =
        new QLabel(
            "Umbral comando movimiento"
        );

    configMoveThresholdSpin =
        new QSpinBox();

    configMoveThresholdSpin->setRange(
        1,
        1000
    );

    configMoveThresholdSpin->setValue(
        100
    );


    /*
    * K 0x03
    * Movimiento mínimo esperado.
    */
    QLabel *minMovementLabel =
        new QLabel(
            "Movimiento mínimo (mm)"
        );

    configMinMovementSpin =
        new QDoubleSpinBox();

    configMinMovementSpin->setDecimals(
        2
    );

    configMinMovementSpin->setRange(
        0.01,
        100.0
    );

    configMinMovementSpin->setSingleStep(
        0.1
    );

    configMinMovementSpin->setValue(
        2.0
    );


    /*
    * K 0x04
    * Timeout NO_MOVEMENT.
    */
    QLabel *noMovementTimeoutLabel =
        new QLabel(
            "Timeout NO_MOVEMENT (ms)"
        );

    configNoMovementTimeoutSpin =
        new QSpinBox();

    configNoMovementTimeoutSpin->setRange(
        100,
        60000
    );

    configNoMovementTimeoutSpin->setSingleStep(
        100
    );

    configNoMovementTimeoutSpin->setValue(
        1000
    );


    /*
    * K 0x05
    * Timeout de consigna AUTO.
    */
    QLabel *targetTimeoutLabel =
        new QLabel(
            "Timeout consigna AUTO (ms)"
        );

    configTargetTimeoutSpin =
        new QSpinBox();

    configTargetTimeoutSpin->setRange(
        100,
        60000
    );

    configTargetTimeoutSpin->setSingleStep(
        100
    );

    configTargetTimeoutSpin->setValue(
        1000
    );


    /*
    * Primera fila de parámetros.
    */
    controlLayout->addWidget(
        moveThresholdLabel,
        1,
        0
    );

    controlLayout->addWidget(
        configMoveThresholdSpin,
        1,
        1
    );

    controlLayout->addWidget(
        minMovementLabel,
        1,
        2
    );

    controlLayout->addWidget(
        configMinMovementSpin,
        1,
        3
    );


    /*
    * Segunda fila.
    */
    controlLayout->addWidget(
        noMovementTimeoutLabel,
        2,
        0
    );

    controlLayout->addWidget(
        configNoMovementTimeoutSpin,
        2,
        1
    );

    controlLayout->addWidget(
        targetTimeoutLabel,
        2,
        2
    );

    controlLayout->addWidget(
        configTargetTimeoutSpin,
        2,
        3
    );


    mainLayout->addWidget(
        controlFrame
    );


    QPushButton *saveButton =
            new QPushButton(
                "GUARDAR CONFIGURACIÓN"
            );

        saveButton->setMinimumHeight(
            50
        );

        saveButton->setStyleSheet(
            "font-size: 18px;"
            "font-weight: bold;"
        );

        connect(
        saveButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            if (stm32Worker == nullptr)
            {
                return;
            }

            saveConfiguration();

            syncConfigurationToWorker();

            
        }
    );

    
    


    /*
    * Contenedor exterior:
    * - contenido desplazable arriba
    * - botón GUARDAR fijo abajo
    */
    QWidget *container =
        new QWidget();

    QVBoxLayout *containerLayout =
        new QVBoxLayout(container);

    containerLayout->setContentsMargins(
        0,
        0,
        0,
        0
    );


    /*
    * Solo la configuración entra en el scroll.
    */
    QScrollArea *scrollArea =
        new QScrollArea();

    scrollArea->setWidgetResizable(
        true
    );

    scrollArea->setWidget(
        page
    );

    scrollArea->setFrameShape(
        QFrame::NoFrame
    );

    containerLayout->addWidget(
        scrollArea,
        1
    );


    /*
    * El botón queda siempre visible.
    */
    containerLayout->addWidget(
        saveButton
    );

    return container;
}    

/*
 * ============================================================
 * BARRA DE ESTADO
 * ============================================================
 */

void MainWindow::createStatusBar()
{
    systemStatusLabel =
        new QLabel(this);

    statusBar()->addPermanentWidget(
        systemStatusLabel,
        1
    );

    configSyncLabel =
        new QLabel(this);

    statusBar()->addPermanentWidget(
        configSyncLabel
    );

    updateSystemStatus();
}

void MainWindow::updateDashboard()
{
    if (state == nullptr)
    {
        return;
    }

    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        HagieState::BodyState bodyState =
            state->getBodyState(body);

        heightLabels[body]->setText(
            QString("Altura: %1 mm")
                .arg(bodyState.height_mm)
        );

        targetLabels[body]->setText(
            QString("Objetivo: %1 mm")
                .arg(bodyState.target_mm)
        );

        modeLabels[body]->setText(
            bodyState.auto_mode
                ? "Modo: AUTO"
                : "Modo: MANUAL"
        );

        valveLabels[body]->setText(
            QString("Válvula: %1")
                .arg(bodyState.valve_command)
        );

        if (bodyState.faults == 0)
        {
            faultLabels[body]->setText(
                "Falla: OK"
            );
        }
        else
        {
            faultLabels[body]->setText(
                QString("Falla: 0x%1")
                    .arg(
                        bodyState.faults,
                        0,
                        16
                    )
            );
        }
    }
    updateSystemStatus();
    updateFaultPage();
    updateTestPage();
}

void MainWindow::updateSystemStatus()
{
    if (state == nullptr ||
        systemStatusLabel == nullptr)
    {
        return;
    }

    HagieState::SystemState system =
        state->getSystemState();

    QString stm32Text =
        system.stm32_connected
            ? "STM32: CONECTADA"
            : "STM32: DESCONECTADA";

    QString canText =
        system.can_ok
            ? "CAN: OK"
            : "CAN: FALLA";

    QString imuText =
        system.imu_valid
            ? "IMU: OK"
            : "IMU: NO VÁLIDA";

    QString visionText;

    if (!system.vision_running)
    {
        visionText =
            "VISIÓN: DETENIDA";
    }
    else if (visionHeightSource == nullptr)
    {
        visionText =
            "VISIÓN: ACTIVA";
    }
    else
    {
        VisionHeightSource::SourceMode mode =
            visionHeightSource->getSourceMode();

        if (mode ==
            VisionHeightSource::SourceMode::SIMULATION)
        {
            visionText =
                "VISIÓN: ACTIVA (SIMULACIÓN)";
        }
        else
        {
            visionText =
                "VISIÓN: ACTIVA (CÁMARAS 3D)";
        }
    }

    QString aiText =
        system.ai_running
            ? "IA: ACTIVA"
            : "IA: DETENIDA";

    systemStatusLabel->setText(
        stm32Text
        + " | "
        + canText
        + " | "
        + imuText
        + " | "
        + visionText
        + " | "
        + aiText
    );


    if (stm32Worker != nullptr &&
        configSyncLabel != nullptr)
    {
        STM32Worker::ConfigSyncStatus configStatus =
            stm32Worker->getConfigSyncStatus();

        switch (configStatus)
        {
            case STM32Worker::ConfigSyncStatus::PENDING:
            {
                configSyncLabel->setText(
                    "CONFIG STM32: PENDIENTE"
                );

                configSyncLabel->setStyleSheet(
                    "font-weight: bold;"
                    "color: orange;"
                );

                break;
            }

            case STM32Worker::ConfigSyncStatus::SYNCHRONIZED:
            {
                configSyncLabel->setText(
                    "CONFIG STM32: SINCRONIZADA"
                );

                configSyncLabel->setStyleSheet(
                    "font-weight: bold;"
                    "color: green;"
                );

                break;
            }

            case STM32Worker::ConfigSyncStatus::ERROR:
            {
                configSyncLabel->setText(
                    "CONFIG STM32: ERROR"
                );

                configSyncLabel->setStyleSheet(
                    "font-weight: bold;"
                    "color: red;"
                );

                break;
            }
        }
    }
}

void MainWindow::updateFaultPage()
{
    if (state == nullptr)
    {
        return;
    }

    /*
     * ========================================================
     * FALLAS GLOBALES
     * ========================================================
     */

    HagieState::SystemState system =
        state->getSystemState();

    QStringList systemFaultList;

    if ((system.system_faults & 0x01U) != 0)
    {
        systemFaultList
            << "JETSON_TIMEOUT";
    }

    if ((system.system_faults & 0x02U) != 0)
    {
        systemFaultList
            << "IMU_TIMEOUT";
    }

    if ((system.system_faults & 0x04U) != 0)
    {
        systemFaultList
            << "UART_RX";
    }

    if ((system.system_faults & 0x08U) != 0)
    {
        systemFaultList
            << "UART_TX";
    }

    if ((system.system_faults & 0x10U) != 0)
    {
        systemFaultList
            << "CAN";
    }


    if (systemFaultList.isEmpty())
    {
        systemFaultsLabel->setText(
            "Fallas globales: OK"
        );
    }
    else
    {
        systemFaultsLabel->setText(
            "Fallas globales: "
            + systemFaultList.join(" | ")
        );
    }


    /*
     * ========================================================
     * FALLAS POR CUERPO
     * ========================================================
     */

    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        HagieState::BodyState bodyState =
            state->getBodyState(body);

        QStringList faults;


        if ((bodyState.faults & 0x01U) != 0)
        {
            faults
                << "ENCODER_TIMEOUT";
        }

        if ((bodyState.faults & 0x02U) != 0)
        {
            faults
                << "ENCODER_RANGE";
        }

        if ((bodyState.faults & 0x04U) != 0)
        {
            faults
                << "NO_MOVEMENT";
        }

        if ((bodyState.faults & 0x08U) != 0)
        {
            faults
                << "MIN_LIMIT";
        }

        if ((bodyState.faults & 0x10U) != 0)
        {
            faults
                << "MAX_LIMIT";
        }

        if ((bodyState.faults & 0x20U) != 0)
        {
            faults
                << "TARGET_TIMEOUT";
        }

        if ((bodyState.faults & 0x40U) != 0)
        {
            faults
                << "VALVE_ERROR";
        }


        if (faults.isEmpty())
        {
            bodyFaultDetailLabels[body]
                ->setText("OK");
        }
        else
        {
            bodyFaultDetailLabels[body]
                ->setText(
                    faults.join("\n")
                );
        }
    }
}

void MainWindow::saveVisionCameraFromWidgets(
    std::size_t camera)
{
    if (camera >= Vision3DProcessor::CAMERA_COUNT)
    {
        return;
    }


    Vision3DProcessor::CameraConfig& config =
        visionCameraConfigs[camera];


    /*
     * Cámara habilitada.
     */
    config.enabled =
        configVisionCameraEnabled->isChecked();


    /*
     * Cuerpos atendidos.
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        config.body_enabled[body] =
            configVisionCameraBodyChecks[body]
                ->isChecked();
    }


    /*
     * Geometría física.
     */
    config.geometry.position_x_mm =
        static_cast<float>(
            configVisionCameraPositionX->value()
        );

    config.geometry.position_y_mm =
        static_cast<float>(
            configVisionCameraPositionY->value()
        );

    config.geometry.position_z_mm =
        static_cast<float>(
            configVisionCameraPositionZ->value()
        );


    /*
     * Altura respecto del piso.
     */
    config.geometry.camera_height_mm =
        static_cast<float>(
            configVisionCameraHeight->value()
        );


    /*
     * Orientación fija de montaje.
     */
    config.geometry.roll_offset_deg =
        static_cast<float>(
            configVisionCameraRoll->value()
        );

    config.geometry.pitch_offset_deg =
        static_cast<float>(
            configVisionCameraPitch->value()
        );
}


void MainWindow::loadVisionCameraIntoWidgets(
    std::size_t camera)
{
    if (camera >= Vision3DProcessor::CAMERA_COUNT)
    {
        return;
    }


    const Vision3DProcessor::CameraConfig& config =
        visionCameraConfigs[camera];


    /*
     * Cámara habilitada.
     */
    configVisionCameraEnabled->setChecked(
        config.enabled
    );


    /*
     * Cuerpos atendidos.
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        configVisionCameraBodyChecks[body]
            ->setChecked(
                config.body_enabled[body]
            );
    }


    /*
     * Geometría física.
     */
    configVisionCameraPositionX->setValue(
        config.geometry.position_x_mm
    );

    configVisionCameraPositionY->setValue(
        config.geometry.position_y_mm
    );

    configVisionCameraPositionZ->setValue(
        config.geometry.position_z_mm
    );


    /*
     * Altura respecto del piso.
     */
    configVisionCameraHeight->setValue(
        config.geometry.camera_height_mm
    );


    /*
     * Orientación fija de montaje.
     */
    configVisionCameraRoll->setValue(
        config.geometry.roll_offset_deg
    );

    configVisionCameraPitch->setValue(
        config.geometry.pitch_offset_deg
    );
}

void MainWindow::saveConfiguration()
{
    QSettings settings(
        "hagie_config.ini",
        QSettings::IniFormat
    );

    /*
     * ========================================================
     * Configuración por cuerpo
     * ========================================================
     */

    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        QString group =
            QString("Body%1")
                .arg(body + 1);

        settings.beginGroup(group);

        settings.setValue(
            "min_height_mm",
            configMinHeightSpin[body]->value()
        );

        settings.setValue(
            "max_height_mm",
            configMaxHeightSpin[body]->value()
        );

        settings.setValue(
            "encoder_scale_mm_per_pulse",
            configEncoderScaleSpin[body]->value()
        );

        settings.setValue(
            "encoder_direction",
            configEncoderDirectionCombo[body]
                ->currentData()
                .toInt()
        );

        settings.setValue(
            "vision_offset_mm",
            configVisionOffsetSpin[body]
                ->value()
        );

        settings.endGroup();
    }


    /*
     * ========================================================
     * Parámetros globales
     * ========================================================
     */

    settings.beginGroup(
        "Control"
    );

    settings.setValue(
        "move_command_threshold",
        configMoveThresholdSpin->value()
    );

    settings.setValue(
        "min_body_movement_mm",
        configMinMovementSpin->value()
    );

    settings.setValue(
        "no_movement_timeout_ms",
        configNoMovementTimeoutSpin->value()
    );

    settings.setValue(
        "target_timeout_ms",
        configTargetTimeoutSpin->value()
    );

    settings.endGroup();

    

    /*
    * ========================================================
    * Fuente de visión 3D
    * ========================================================
    */
    settings.beginGroup(
        "Vision"
    );

    settings.setValue(
        "source_mode",
        configVisionSourceCombo
            ->currentData()
            .toInt()
    );

    settings.endGroup();


    /*
    * ========================================================
    * Configuración de cámaras 3D
    * ========================================================
    */

    /*
    * Guardar primero lo que actualmente está
    * visible en los widgets.
    */
    if (configVisionCameraCombo != nullptr)
    {
        std::size_t currentCamera =
            static_cast<std::size_t>(
                configVisionCameraCombo->currentIndex()
            );

        if (currentCamera <
            Vision3DProcessor::CAMERA_COUNT)
        {
            saveVisionCameraFromWidgets(
                currentCamera
            );
        }
    }


    /*
    * Guardar las tres configuraciones.
    */
    for (std::size_t camera = 0;
        camera < Vision3DProcessor::CAMERA_COUNT;
        ++camera)
    {
        QString group =
            QString("Camera%1")
                .arg(camera + 1);

        settings.beginGroup(group);


        const auto& config =
            visionCameraConfigs[camera];


        settings.setValue(
            "enabled",
            config.enabled
        );


        /*
        * Cuerpos atendidos por esta cámara.
        */
        for (std::size_t body = 0;
            body < HagieState::BODY_COUNT;
            ++body)
        {
            settings.setValue(
                QString("body_%1")
                    .arg(body + 1),
                config.body_enabled[body]
            );
        }


        /*
        * Posición física de la cámara.
        */
        settings.setValue(
            "position_x_mm",
            config.geometry.position_x_mm
        );

        settings.setValue(
            "position_y_mm",
            config.geometry.position_y_mm
        );

        settings.setValue(
            "position_z_mm",
            config.geometry.position_z_mm
        );


        /*
        * Altura cámara / piso.
        */
        settings.setValue(
            "camera_height_mm",
            config.geometry.camera_height_mm
        );


        /*
        * Correcciones de montaje.
        */
        settings.setValue(
            "roll_offset_deg",
            config.geometry.roll_offset_deg
        );

        settings.setValue(
            "pitch_offset_deg",
            config.geometry.pitch_offset_deg
        );


        settings.endGroup();
    }

    /*
     * Forzar escritura en disco.
     */

         /*
     * ========================================================
     * Aplicar configuración al procesador 3D
     * ========================================================
     */
    if (vision3DProcessor != nullptr)
    {
        for (std::size_t camera = 0;
             camera < Vision3DProcessor::CAMERA_COUNT;
             ++camera)
        {
            vision3DProcessor->setCameraConfig(
                camera,
                visionCameraConfigs[camera]
            );


            
        }
    }
    settings.sync();
}

void MainWindow::loadConfiguration()
{
    QSettings settings(
        "hagie_config.ini",
        QSettings::IniFormat
    );

    /*
     * ========================================================
     * Configuración por cuerpo
     * ========================================================
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        QString group =
            QString("Body%1")
                .arg(body + 1);

        settings.beginGroup(group);

        configMinHeightSpin[body]->setValue(
            settings.value(
                "min_height_mm",
                50
            ).toInt()
        );

        configMaxHeightSpin[body]->setValue(
            settings.value(
                "max_height_mm",
                700
            ).toInt()
        );

        configEncoderScaleSpin[body]->setValue(
            settings.value(
                "encoder_scale_mm_per_pulse",
                1.0
            ).toDouble()
        );

        int direction =
            settings.value(
                "encoder_direction",
                1
            ).toInt();


        configVisionOffsetSpin[body]->setValue(
                settings.value(
                    "vision_offset_mm",
                    0
                ).toInt()
            );

        int index =
            configEncoderDirectionCombo[body]
                ->findData(direction);

        if (index >= 0)
        {
            configEncoderDirectionCombo[body]
                ->setCurrentIndex(index);
        }

        settings.endGroup();
    }


    /*
     * ========================================================
     * Parámetros globales
     * ========================================================
     */
    settings.beginGroup(
        "Control"
    );

    configMoveThresholdSpin->setValue(
        settings.value(
            "move_command_threshold",
            100
        ).toInt()
    );

    configMinMovementSpin->setValue(
        settings.value(
            "min_body_movement_mm",
            2.0
        ).toDouble()
    );

    configNoMovementTimeoutSpin->setValue(
        settings.value(
            "no_movement_timeout_ms",
            1000
        ).toInt()
    );

    configTargetTimeoutSpin->setValue(
        settings.value(
            "target_timeout_ms",
            1000
        ).toInt()
    );

    settings.endGroup();

    /*
    * ========================================================
    * Fuente de visión 3D
    * ========================================================
    */
    settings.beginGroup(
        "Vision"
    );

    int visionSourceMode =
        settings.value(
            "source_mode",
            0
        ).toInt();

    settings.endGroup();


    int visionSourceIndex =
        configVisionSourceCombo
            ->findData(
                visionSourceMode
            );

    if (visionSourceIndex >= 0)
    {
        configVisionSourceCombo
            ->setCurrentIndex(
                visionSourceIndex
            );
    }

    /*
    * ========================================================
    * Configuración de cámaras 3D
    * ========================================================
    */
    for (std::size_t camera = 0;
        camera < Vision3DProcessor::CAMERA_COUNT;
        ++camera)
    {
        QString group =
            QString("Camera%1")
                .arg(camera + 1);

        settings.beginGroup(group);


        Vision3DProcessor::CameraConfig& config =
            visionCameraConfigs[camera];


        /*
        * Cámara habilitada.
        */
        config.enabled =
            settings.value(
                "enabled",
                false
            ).toBool();


        /*
        * Cuerpos atendidos.
        */
        for (std::size_t body = 0;
            body < HagieState::BODY_COUNT;
            ++body)
        {
            config.body_enabled[body] =
                settings.value(
                    QString("body_%1")
                        .arg(body + 1),
                    false
                ).toBool();
        }


        /*
        * Posición física.
        */
        config.geometry.position_x_mm =
            settings.value(
                "position_x_mm",
                0.0
            ).toFloat();

        config.geometry.position_y_mm =
            settings.value(
                "position_y_mm",
                0.0
            ).toFloat();

        config.geometry.position_z_mm =
            settings.value(
                "position_z_mm",
                0.0
            ).toFloat();


        /*
        * Altura cámara / piso.
        */
        config.geometry.camera_height_mm =
            settings.value(
                "camera_height_mm",
                0.0
            ).toFloat();


        /*
        * Correcciones fijas de montaje.
        */
        config.geometry.roll_offset_deg =
            settings.value(
                "roll_offset_deg",
                0.0
            ).toFloat();

        config.geometry.pitch_offset_deg =
            settings.value(
                "pitch_offset_deg",
                0.0
            ).toFloat();


        settings.endGroup();
    }


    /*
    * ========================================================
    * Aplicar configuración al procesador 3D
    * ========================================================
    */
    if (vision3DProcessor != nullptr)
    {
        for (std::size_t camera = 0;
            camera < Vision3DProcessor::CAMERA_COUNT;
            ++camera)
        {
            vision3DProcessor->setCameraConfig(
                camera,
                visionCameraConfigs[camera]
            );
        }
    }

    

    /*
    * Mostrar en la GUI la cámara
    * actualmente seleccionada.
    */
    if (configVisionCameraCombo != nullptr)
    {
        int currentCamera =
            configVisionCameraCombo
                ->currentIndex();

        if (currentCamera >= 0 &&
            static_cast<std::size_t>(currentCamera) <
                Vision3DProcessor::CAMERA_COUNT)
        {
            loadVisionCameraIntoWidgets(
                static_cast<std::size_t>(
                    currentCamera
                )
            );
        }
    }
}
void MainWindow::syncConfigurationToWorker()
{
    if (stm32Worker == nullptr)
    {
        return;
    }

    

    /*
     * K01 - Límites por cuerpo
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        uint16_t minHeight =
            static_cast<uint16_t>(
                configMinHeightSpin[body]->value()
            );

        uint16_t maxHeight =
            static_cast<uint16_t>(
                configMaxHeightSpin[body]->value()
            );

        if (minHeight >= maxHeight)
        {
            continue;
        }

        stm32Worker->setBodyLimits(
            static_cast<uint8_t>(body),
            minHeight,
            maxHeight
        );
    }

    /*
     * K02
     */
    stm32Worker->setMoveCommandThreshold(
        static_cast<uint16_t>(
            configMoveThresholdSpin->value()
        )
    );

    /*
     * K03
     */
    stm32Worker->setMinBodyMovement(
        static_cast<float>(
            configMinMovementSpin->value()
        )
    );

    /*
     * K04
     */
    stm32Worker->setNoMovementTimeout(
        static_cast<uint32_t>(
            configNoMovementTimeoutSpin->value()
        )
    );

    /*
     * K05
     */
    stm32Worker->setTargetTimeout(
        static_cast<uint32_t>(
            configTargetTimeoutSpin->value()
        )
    );

    /*
     * K06 y K07
     */
    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        int directionData =
            configEncoderDirectionCombo[body]
                ->currentData()
                .toInt();

        uint8_t protocolDirection =
            (directionData == 1)
                ? 0
                : 1;

        stm32Worker->setEncoderDirection(
            static_cast<uint8_t>(body),
            protocolDirection
        );

        stm32Worker->setEncoderScale(
            static_cast<uint8_t>(body),
            static_cast<float>(
                configEncoderScaleSpin[body]->value()
            )
        );
    }
    /*
    * Ya cargamos toda la configuración en runtimeConfig.
    * Ahora comenzar el envío secuencial K -> ACK -> K.
    */
    stm32Worker->beginConfigurationSync();
}