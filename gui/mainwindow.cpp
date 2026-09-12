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
#include <QPainter>
#include <QPen>
#include <QStandardPaths>
#include <QDir>
#include <QLineEdit>
#include <QFileInfo>
#include <QDialog>
#include <QTextBrowser>
#include <QDialogButtonBox>
#include "ai/tassel_detector.h"
#include "stm32/stm32_worker.h"
#include "core/height_target_controller.h"
#include <QStringList>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSettings>
#include <QScrollArea>
#include <iostream>
#include <QPlainTextEdit>
#include <QDateTime>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QImage>
#include <QPixmap>
#include "vision/vision_height_source.h"
#include "vision/vision_3d_worker.h"

#include <memory>

#include "vision/simulated_point_cloud_source.h"
#include "vision/zed_gmsl_point_cloud_source.h"
#include "vision/zed_gmsl_rgb_frame_source.h"
#include "vision/zed_gmsl_rgb_only_frame_source.h"
#include "height_trend_widget.h"
#include <QShortcut>
#include <QKeySequence>




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
    visionHeightSource(visionHeightSource),
    heightTargetController(heightTargetController),
    vision3DProcessor(vision3DProcessor),
    vision3DWorker(vision3DWorker),
    yoloInferenceWorker(
    rgbCameraWorker,
    vision3DWorker
)
{
    setWindowTitle(
        "Hagie Control"
    );

    QShortcut *fullScreenShortcut =
    new QShortcut(
        QKeySequence(Qt::Key_F11),
        this
    );

    fullScreenShortcut->setContext(
        Qt::ApplicationShortcut
    );

    connect(
        fullScreenShortcut,
        &QShortcut::activated,
        this,
        [this]()
        {
            if (isFullScreen())
            {
                showNormal();
            }
            else
            {
                showFullScreen();
            }
        }
    );

    QShortcut *escapeShortcut =
        new QShortcut(
            QKeySequence(Qt::Key_Escape),
            this
        );

    escapeShortcut->setContext(
        Qt::ApplicationShortcut
    );

    connect(
        escapeShortcut,
        &QShortcut::activated,
        this,
        [this]()
        {
            if (isFullScreen())
            {
                showNormal();
            }
        }
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

    /*
    * Página 5
    * Comunicaciones
    */
    centralStack->addWidget(
        createCommunicationsPage()
    );

    /*
    * Página 6
    * Logs
    */
    centralStack->addWidget(
        createLogsPage()
    );


    setCentralWidget(
        centralStack
    );

    /*
    * ========================================================
    * CÁMARAS RGB SIMULADAS
    * ========================================================
    */
    for (std::size_t camera = 0;
        camera < RgbCameraWorker::CAMERA_COUNT;
        ++camera)
    {
        auto source =
            std::make_unique<
                SimulatedRgbFrameSource
            >(
                camera
            );

        rgbCameraWorker.setFrameSource(
            camera,
            std::move(source)
        );
    }

    rgbCameraWorker.start();


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

    /*
    * ========================================================
    * DETECCIÓN INICIAL DE CÁMARAS ZED
    * ========================================================
    *
    * La GUI ya existe y la configuración ya fue cargada.
    * Ahora sí podemos comparar los seriales guardados
    * contra las cámaras detectadas físicamente.
    */
    refreshZedCameraDetection();

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

    connect(
        dashboardTimer,
        &QTimer::timeout,
        this,
        &MainWindow::updateCameraPage
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

        const uint64_t visionStaleTimeoutMs =
        static_cast<uint64_t>(
            configVisionDataTimeoutSpin != nullptr
                ? configVisionDataTimeoutSpin->value()
                : 1000
        );

        bool visionFresh =
            bodyState.vision_valid &&
            bodyState.vision_timestamp_ms != 0 &&
            (nowMs - bodyState.vision_timestamp_ms) <=
                visionStaleTimeoutMs;
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
     yoloInferenceWorker.stop();
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

    QAction *communicationsAction =
        diagnosticMenu->addAction(
            "Comunicaciones"
        );

    QAction *logsAction =
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

    connect(
        communicationsAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(
                5
            );
        }
    );

    connect(
        logsAction,
        &QAction::triggered,
        this,
        [this]()
        {
            centralStack->setCurrentIndex(
                6
            );
        }
    );


    /*
 * ========================================================
 * AYUDA
 * ========================================================
 */
QMenu *helpMenu =
    menuBar()->addMenu("Ayuda");


/*
 * Manual de uso
 */
QAction *manualAction =
    helpMenu->addAction(
        "Manual de uso"
    );


/*
 * Información del sistema
 */
QAction *systemInfoAction =
    helpMenu->addAction(
        "Información del sistema"
    );


helpMenu->addSeparator();


/*
 * Acerca de
 */
QAction *aboutAction =
    helpMenu->addAction(
        "Acerca de Hagie Control"
    );


/*
 * ========================================================
 * MANUAL DE USO
 * ========================================================
 */
connect(
    manualAction,
    &QAction::triggered,
    this,
    [this]()
    {
        QDialog dialog(this);

        dialog.setWindowTitle(
            "Manual de uso - Hagie Control"
        );

        dialog.resize(
            900,
            650
        );


        QVBoxLayout *layout =
            new QVBoxLayout(&dialog);


        QTextBrowser *text =
            new QTextBrowser();

        text->setOpenExternalLinks(
            true
        );

        text->setHtml(
            R"(
            <h1>Hagie Control</h1>

            <p>
            Sistema inteligente para el control automático de altura,
            detección de panojas y verificación de la efectividad
            del despanojado.
            </p>

            <hr>

            <h2>1. Vista general</h2>

            <p>
            La pantalla principal permite observar el
            estado de los seis cuerpos de la máquina.
            </p>

            <p>
            Para cada cuerpo se muestran:
            </p>

            <ul>
                <li><b>Altura:</b> posición medida por el encoder.</li>
                <li><b>Objetivo:</b> altura solicitada por el sistema.</li>
                <li><b>Modo:</b> MANUAL o automático.</li>
                <li><b>Válvula:</b> comando aplicado al actuador hidráulico.</li>
                <li><b>Falla:</b> estado de diagnóstico del cuerpo.</li>
            </ul>


            <h2>2. Cámaras</h2>

            <p>
            El sistema utiliza siete cámaras físicas.
            </p>

            <ul>
                <li>
                <b>Cámaras 1 a 5:</b>
                cámaras frontales 3D + RGB.
                Se utilizan para observar el cultivo,
                obtener información tridimensional y
                detectar panojas mediante inteligencia artificial.
                </li>

                <li>
                <b>Cámaras 6 y 7:</b>
                cámaras traseras RGB utilizadas para
                observar el resultado del trabajo realizado.
                </li>
            </ul>

            <p>
            Desde esta pantalla puede seleccionarse la
            cámara que se desea visualizar.
            </p>


            <h2>3. Fallas</h2>

            <p>
            La pantalla de fallas concentra las condiciones
            anormales detectadas por el sistema.
            </p>

            <p>
            Debe utilizarse para identificar rápidamente
            problemas de comunicación, sensores, actuadores
            o subsistemas de control.
            </p>


            <h2>4. Test</h2>

            <p>
            El panel de test permite verificar individualmente
            el funcionamiento de los cuerpos.
            </p>

            <p>
            Desde esta pantalla pueden realizarse pruebas de:
            </p>

            <ul>
                <li>Movimiento manual.</li>
                <li>Lectura de encoders.</li>
                <li>Accionamiento de válvulas.</li>
                <li>Control automático.</li>
                <li>Control automático utilizando visión 3D.</li>
            </ul>

            <p>
            Las funciones de movimiento quedan inhibidas
            cuando no existe comunicación válida con la STM32.
            </p>


            <h2>5. Configuración</h2>

            <p>
            Permite configurar los parámetros utilizados
            por el sistema de control.
            </p>

            <p>
            Entre ellos se encuentran:
            </p>

            <ul>
                <li>Límites de altura.</li>
                <li>Configuración de encoders.</li>
                <li>Parámetros de cada cuerpo.</li>
                <li>Configuración de visión.</li>
                <li>Selección de la fuente de visión 3D.</li>
            </ul>

            <p>
            Las fuentes disponibles son:
            </p>

            <ul>
                <li><b>SIMULACIÓN ALTURAS</b></li>
                <li><b>SIMULACIÓN CÁMARAS 3D</b></li>
                <li><b>CÁMARAS 3D REALES</b></li>
            </ul>


            <h2>6. Comunicaciones</h2>

            <p>
            Esta pantalla permite supervisar las
            comunicaciones entre los distintos módulos
            electrónicos del sistema.
            </p>

            <p>
            Incluye principalmente la comunicación con
            la STM32 y el sistema CAN utilizado para el
            accionamiento de las válvulas.
            </p>


            <h2>7. Logs</h2>

            <p>
            La pantalla de Logs registra cronológicamente
            eventos importantes ocurridos durante el
            funcionamiento de Hagie Control.
            </p>

            <p>
            Los registros permiten diagnosticar fallas
            y analizar el comportamiento del sistema.
            </p>


            <h2>8. Barra de estado</h2>

            <p>
            En la parte inferior de la pantalla se muestra
            permanentemente el estado general del sistema.
            </p>

            <ul>
                <li><b>STM32:</b> estado de comunicación con el controlador.</li>
                <li><b>CAN:</b> estado de la red de actuadores.</li>
                <li><b>IMU:</b> validez del sensor de orientación.</li>
                <li><b>VISIÓN:</b> estado del sistema de visión 3D.</li>
                <li><b>IA:</b> estado del sistema de detección.</li>
                <li><b>CONFIG STM32:</b> estado de sincronización de configuración.</li>
            </ul>


            <h2>9. Parada total</h2>

            <p>
            El botón rojo <b>PARADA TOTAL</b> cancela
            las órdenes de movimiento y ordena detener
            los actuadores.
            </p>

            <p>
            Debe utilizarse ante una situación anormal
            o cuando sea necesario detener inmediatamente
            el movimiento controlado por el sistema.
            </p>


            <hr>

            <p>
            <b>Hagie Control</b><br>
            Sistema de visión y control automático de altura.
            </p>
            )"
        );


        layout->addWidget(
            text
        );


        QDialogButtonBox *buttons =
            new QDialogButtonBox(
                QDialogButtonBox::Close
            );

        connect(
            buttons,
            &QDialogButtonBox::rejected,
            &dialog,
            &QDialog::reject
        );

        layout->addWidget(
            buttons
        );


        dialog.exec();
    }
);


/*
 * ========================================================
 * INFORMACIÓN DEL SISTEMA
 * ========================================================
 */
connect(
    systemInfoAction,
    &QAction::triggered,
    this,
    [this]()
    {
        QDialog dialog(this);

        dialog.setWindowTitle(
            "Información del sistema"
        );

        dialog.resize(
            650,
            450
        );


        QVBoxLayout *layout =
            new QVBoxLayout(&dialog);


        QTextBrowser *text =
            new QTextBrowser();

        text->setHtml(
            R"(
            <h2>Arquitectura de Hagie Control</h2>

            <p>
            El sistema está compuesto por varios
            subsistemas que trabajan en conjunto.
            </p>

            <ul>
                <li><b>Computadora NVIDIA Jetson:</b>
                    procesamiento principal.</li>

                <li><b>STM32:</b>
                    control en tiempo real, encoders
                    y comunicación con actuadores.</li>

                <li><b>CAN:</b>
                    comunicación con los módulos
                    de control de válvulas.</li>

                <li><b>Visión 3D:</b>
                    medición tridimensional del cultivo.</li>

                <li><b>IA:</b>
                    detección de panojas y verificación
                    de la efectividad del despanojado.</li>

                <li><b>IMU:</b>
                    compensación de la orientación
                    de la máquina.</li>

                <li><b>Encoders:</b>
                    medición de posición de los
                    seis cuerpos.</li>
            </ul>

            <p>
            La Jetson procesa la información de visión
            y genera los objetivos de altura.
            La STM32 ejecuta el control de los actuadores
            y supervisa las señales de campo.
            </p>
            )"
        );


        layout->addWidget(
            text
        );


        QDialogButtonBox *buttons =
            new QDialogButtonBox(
                QDialogButtonBox::Close
            );

        connect(
            buttons,
            &QDialogButtonBox::rejected,
            &dialog,
            &QDialog::reject
        );

        layout->addWidget(
            buttons
        );


        dialog.exec();
    }
);


/*
 * ========================================================
 * ACERCA DE HAGIE CONTROL
 * ========================================================
 */
connect(
    aboutAction,
    &QAction::triggered,
    this,
    [this]()
    {
        QDialog dialog(this);

        dialog.setWindowTitle(
            "Acerca de Hagie Control"
        );

        dialog.resize(
            500,
            300
        );


        QVBoxLayout *layout =
            new QVBoxLayout(&dialog);


        QTextBrowser *text =
            new QTextBrowser();

        text->setHtml(
            R"(
            <div align="center">

            <h1 style="margin-bottom: 4px;">
            Hagie Control
            </h1>

            <h3 style="margin: 4px;">
            Control inteligente y verificación de despanojado
            </h3>

            <p style="margin: 6px;">
            Control automático de altura mediante visión 3D
            y detección de panojas con inteligencia artificial.
            </p>

            
            <p style="margin: 4px;">
            <b>Plataforma:</b> NVIDIA Jetson + STM32
            </p>

            <p style="margin: 4px;">
            <b>Visión:</b> Cámaras 3D + RGB
            </p>

            <hr>

            <p style="margin: 6px;">
            <b>Desarrollado por:</b><br>
            Esp. Ing. Ezequiel Acerbo
            </p>

            <p style="margin: 4px;">
            <b>Correo:</b> acerboezequiel@live.com
            </p>

            </div>
            )"
        );


        layout->addWidget(
            text
        );


        QDialogButtonBox *buttons =
            new QDialogButtonBox(
                QDialogButtonBox::Close
            );

        connect(
            buttons,
            &QDialogButtonBox::rejected,
            &dialog,
            &QDialog::reject
        );

        layout->addWidget(
            buttons
        );


        dialog.exec();
    }
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
    
    mainLayout->setContentsMargins(
        8,
        8,
        8,
        8
    );

    mainLayout->setSpacing(
        6
    );    


    QLabel *title =
    new QLabel(
        "HAGIE CONTROL"
    );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 26px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(title);


    QLabel *subtitle =
        new QLabel(
            "Control de altura · Detección y verificación de despanojado"
        );

    subtitle->setAlignment(
        Qt::AlignCenter
    );

    subtitle->setStyleSheet(
        "font-size: 15px;"
        "font-weight: normal;"
    );

    mainLayout->addWidget(subtitle);


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
     * ========================================================
     * RENDIMIENTO DE DESPANOJADO
     * ========================================================
     */

    QFrame *tasselPerformanceFrame =
        new QFrame();

    tasselPerformanceFrame->setFrameShape(
        QFrame::StyledPanel
    );


    QVBoxLayout *tasselPerformanceLayout =
        new QVBoxLayout(
            tasselPerformanceFrame
        );


    QLabel *tasselPerformanceTitle =
        new QLabel(
            "RENDIMIENTO DE DESPANOJADO"
        );

    tasselPerformanceTitle->setAlignment(
        Qt::AlignCenter
    );

    tasselPerformanceTitle->setStyleSheet(
        "font-size: 17px;"
        "font-weight: bold;"
    );

    tasselPerformanceLayout->addWidget(
        tasselPerformanceTitle
    );


    QHBoxLayout *tasselStatsLayout =
        new QHBoxLayout();


    tasselDetectedLabel =
        new QLabel(
            "Detectadas: 0"
        );

    tasselRemovedLabel =
        new QLabel(
            "Removidas: 0"
        );

    tasselRemainingLabel =
        new QLabel(
            "Presentes: 0"
        );

    tasselEfficiencyLabel =
        new QLabel(
            "Efectividad: -- %"
        );


    tasselDetectedLabel->setAlignment(
        Qt::AlignCenter
    );

    tasselRemovedLabel->setAlignment(
        Qt::AlignCenter
    );

    tasselRemainingLabel->setAlignment(
        Qt::AlignCenter
    );

    tasselEfficiencyLabel->setAlignment(
        Qt::AlignCenter
    );


        tasselDetectedLabel->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
        "color: #1565C0;"
    );

    tasselRemovedLabel->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
        "color: #2E7D32;"
    );

    tasselRemainingLabel->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
        "color: #C62828;"
    );

    tasselEfficiencyLabel->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
        "color: #616161;"
    );


    tasselStatsLayout->addWidget(
        tasselDetectedLabel
    );

    tasselStatsLayout->addWidget(
        tasselRemovedLabel
    );

    tasselStatsLayout->addWidget(
        tasselRemainingLabel
    );

    tasselStatsLayout->addWidget(
        tasselEfficiencyLabel
    );


    tasselPerformanceLayout->addLayout(
        tasselStatsLayout
    );


   mainLayout->addWidget(
        tasselPerformanceFrame
    );


    /*
    * ========================================================
    * TENDENCIA DE ALTURA
    * ========================================================
    */

    QHBoxLayout *trendControlsLayout =
        new QHBoxLayout();


    QLabel *trendBodyLabel =
        new QLabel("Cuerpo:");

    QComboBox *trendBodyCombo =
        new QComboBox();

    for (int body = 0;
        body < 6;
        ++body)
    {
        trendBodyCombo->addItem(
            QString("Cuerpo %1")
                .arg(body + 1)
        );
    }


    QLabel *trendTimeLabel =
        new QLabel("Ventana:");

    QComboBox *trendTimeCombo =
        new QComboBox();

    trendTimeCombo->addItem("30 s", 30.0);
    trendTimeCombo->addItem("1 min", 60.0);
    trendTimeCombo->addItem("5 min", 300.0);
    trendTimeCombo->addItem("10 min", 600.0);
    trendTimeCombo->addItem("30 min", 1800.0);


    QLabel *trendScaleLabel =
        new QLabel("Escala:");

    QComboBox *trendScaleCombo =
        new QComboBox();

    trendScaleCombo->addItem("FIJA");
    trendScaleCombo->addItem("AUTO");


    trendControlsLayout->addStretch();

    trendControlsLayout->addWidget(
        trendBodyLabel
    );

    trendControlsLayout->addWidget(
        trendBodyCombo
    );

    trendControlsLayout->addSpacing(20);

    trendControlsLayout->addWidget(
        trendTimeLabel
    );

    trendControlsLayout->addWidget(
        trendTimeCombo
    );

    trendControlsLayout->addSpacing(20);

    trendControlsLayout->addWidget(
        trendScaleLabel
    );

    trendControlsLayout->addWidget(
        trendScaleCombo
    );

    trendControlsLayout->addStretch();


    mainLayout->addLayout(
        trendControlsLayout
    );

    heightTrendWidget =
        new HeightTrendWidget();

    heightTrendWidget->setAutoScale(
        false
    );

    heightTrendWidget->setFixedRange(
        0.0,
        800.0
    );

    connect(
        trendTimeCombo,
        &QComboBox::currentIndexChanged,
        this,
        [this, trendTimeCombo](int)
        {
            if (heightTrendWidget == nullptr)
            {
                return;
            }

            heightTrendWidget->setVisibleWindowSeconds(
                trendTimeCombo->currentData()
                    .toDouble()
            );
        }
    );


    connect(
        trendScaleCombo,
        &QComboBox::currentIndexChanged,
        this,
        [this, trendScaleCombo](int)
        {
            if (heightTrendWidget == nullptr)
            {
                return;
            }

            const bool autoScale =
                trendScaleCombo->currentText() ==
                "AUTO";

            heightTrendWidget->setAutoScale(
                autoScale
            );
        }
    );


    connect(
        trendBodyCombo,
        &QComboBox::currentIndexChanged,
        this,
        [this](int index)
        {
            trendSelectedBody =
                index;

            if (heightTrendWidget != nullptr)
            {
                heightTrendWidget->setSelectedBody(
                    index
                );
            }
        }
    );

    heightTrendWidget->setVisibleWindowSeconds(
        30.0
    );

    mainLayout->addWidget(
        heightTrendWidget,
        1
    );


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

    connect(
        stopButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            /*
            * Cancelar todos los modos AUTO locales.
            */
            for (std::size_t body = 0;
                body < HagieState::BODY_COUNT;
                ++body)
            {
                testAutoEnabled[body] =
                    false;

                testVisionAutoEnabled[body] =
                    false;

                if (state != nullptr)
                {
                    state->setBodyAutoMode(
                        body,
                        false
                    );
                }
            }


            /*
            * Orden real de parada a la STM32.
            */
            if (stm32Worker != nullptr)
            {
                stm32Worker->stopAllValves();
                qInfo() << "PARADA TOTAL DASHBOARD -> stopAllValves()";
            }
        }
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
    mainCameraLabel =
        new QLabel(
            "CÁMARA PRINCIPAL\n\n"
            "VIDEO"
        );

    mainCameraLabel->setAlignment(
        Qt::AlignCenter
    );

    mainCameraLabel->setMinimumHeight(
        250
    );

    mainCameraLabel->setStyleSheet(
        "background-color: black;"
        "color: white;"
        "font-size: 28px;"
    );

    layout->addWidget(
        mainCameraLabel,
        1
    );


    /*
     * Miniaturas.
     */
    QHBoxLayout *cameraSelector =
        new QHBoxLayout();


    constexpr int PHYSICAL_CAMERA_COUNT =
        7;

    for (int camera = 0;
        camera < PHYSICAL_CAMERA_COUNT;
        ++camera)
    {
        QString cameraText;

        if (camera < 5)
        {
            cameraText =
                QString("Cámara %1\nFRONTAL 3D + RGB")
                    .arg(camera + 1);
        }
        else
        {
            cameraText =
                QString("Cámara %1\nTRASERA RGB")
                    .arg(camera + 1);
        }

        QPushButton *button =
            new QPushButton(
                cameraText
            );

        button->setMinimumHeight(
            60
        );

        connect(
            button,
            &QPushButton::clicked,
            this,
            [this, camera]()
            {
                selectedRgbCamera =
                    static_cast<std::size_t>(
                        camera
                    );

                if (mainCameraLabel != nullptr)
                {
                    mainCameraLabel->setText(
                        QString(
                            "CÁMARA %1\n\n"
                            "RGB SIMULADA"
                        ).arg(
                            camera + 1
                        )
                    );
                }
            }
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

void MainWindow::updateCameraPage()
{
    if (mainCameraLabel == nullptr)
    {
        return;
    }


    /*
     * ========================================================
     * PROCESAMIENTO Y CONTEO DE LAS 7 CÁMARAS
     * ========================================================
     */

    constexpr std::size_t RGB_CAMERA_COUNT =
        7;


    std::uint64_t latestTimestampMs =
        0;
    bool aiProcessedFrame =
        false;

    const bool realYoloMode =
        configVisionSourceCombo != nullptr &&
        configVisionSourceCombo->currentIndex() == 2;


    for (std::size_t camera = 0;
        camera < RGB_CAMERA_COUNT;
        ++camera)
    
    {
        TasselDetector::Result cameraResult;

        Vision3DProcessor::PointCloud cloud3D;

        std::uint64_t cloudTimestampMs =
            0;

        bool cloud3DValid =
            false;


        /*
        * =====================================================
        * OBTENCIÓN DEL RESULTADO DE IA
        * =====================================================
        *
        * SIMULACIÓN:
        * TasselDetector continúa ejecutándose directamente.
        *
        * MODO REAL:
        * La GUI solamente consume resultados ya procesados
        * por YoloInferenceWorker.
        */
        if (realYoloMode)
        {
            YoloInferenceWorker::Result yoloResult;


            if (!yoloInferenceWorker.getLatestResult(
                    camera,
                    yoloResult
                ))
            {
                continue;
            }


            if (!yoloResult.detectionResult.valid)
            {
                continue;
            }


            /*
            * Nunca consumir dos veces el mismo
            * resultado producido por YOLO.
            */
            if (yoloResult.detectionResult.timestamp_ms ==
                lastConsumedYoloTimestamp[camera])
            {
                continue;
            }


            cameraResult =
                std::move(
                    yoloResult.detectionResult
                );


            /*
            * Para las cinco cámaras frontales,
            * YoloInferenceWorker conserva la nube
            * correspondiente exactamente al mismo
            * frame RGB utilizado por TensorRT.
            */
            if (camera < Vision3DWorker::CAMERA_COUNT)
            {
                if (!yoloResult.pointCloudValid)
                {
                    continue;
                }


                if (yoloResult.pointCloudTimestampMs !=
                    cameraResult.timestamp_ms)
                {
                    continue;
                }


                cloud3D =
                    std::move(
                        yoloResult.pointCloud
                    );

                cloudTimestampMs =
                    yoloResult.pointCloudTimestampMs;

                cloud3DValid =
                    true;
            }


            lastConsumedYoloTimestamp[camera] =
                cameraResult.timestamp_ms;
        }
        else
        {
            /*
            * =================================================
            * DETECTOR SIMULADO
            * =================================================
            */
            RgbFrameSource::Frame cameraFrame;


            if (!rgbCameraWorker.getFrame(
                    camera,
                    cameraFrame
                ))
            {
                continue;
            }


            if (!cameraFrame.valid ||
                cameraFrame.width == 0 ||
                cameraFrame.height == 0 ||
                cameraFrame.data.empty())
            {
                continue;
            }


            if (!tasselDetector.processFrame(
                    camera,
                    cameraFrame,
                    cameraResult
                ))
            {
                continue;
            }


            /*
            * En simulación RGB y nube provienen de
            * fuentes independientes.
            *
            * Conservamos la tolerancia temporal
            * que ya utilizábamos.
            */
            if (camera < Vision3DWorker::CAMERA_COUNT &&
                vision3DWorker != nullptr)
            {
                if (vision3DWorker->getLatestPointCloud(
                        camera,
                        cloud3D,
                        cloudTimestampMs
                    ))
                {
                    constexpr std::uint64_t
                        MAX_RGB_CLOUD_DELTA_MS =
                            150;


                    const std::uint64_t timeDeltaMs =
                        (cameraResult.timestamp_ms >
                        cloudTimestampMs)
                        ?
                        (cameraResult.timestamp_ms -
                        cloudTimestampMs)
                        :
                        (cloudTimestampMs -
                        cameraResult.timestamp_ms);


                    if (timeDeltaMs <=
                        MAX_RGB_CLOUD_DELTA_MS)
                    {
                        cloud3DValid =
                            true;
                    }
                }
            }
        }


        /*
        * El detector produjo un resultado válido.
        */
        aiProcessedFrame =
            true;


        if (cameraResult.timestamp_ms >
            latestTimestampMs)
        {
            latestTimestampMs =
                cameraResult.timestamp_ms;
        }


        /*
        * =====================================================
        * ASOCIACIÓN RGB -> POSICIÓN 3D
        * =====================================================
        *
        * Solo las cámaras frontales 0..4
        * disponen de nube de puntos.
        */
        if (camera < Vision3DWorker::CAMERA_COUNT)
        {
            std::vector<
                TasselDetector::Detection
            > validDetections;


            if (cloud3DValid &&
                vision3DProcessor != nullptr)
            {
                Vision3DProcessor::CameraConfig
                    cameraConfig =
                        vision3DProcessor->getCameraConfig(
                            camera
                        );


                for (TasselDetector::Detection detection :
                    cameraResult.detections)
                {
                    Vision3DProcessor::Point3D position3D;


                    if (!vision3DProcessor->getDetectionPosition3D(
                            cloud3D,
                            detection.x,
                            detection.y,
                            detection.width,
                            detection.height,
                            cameraConfig.geometry,
                            position3D
                        ))
                    {
                        continue;
                    }


                    std::size_t detectedBody =
                        0;


                    if (!vision3DProcessor->findBodyForPosition(
                            camera,
                            position3D,
                            detectedBody
                        ))
                    {
                        continue;
                    }


                    detection.position_x =
                        position3D.x;

                    detection.position_y =
                        position3D.y;

                    detection.position_z =
                        position3D.z;

                    detection.position_3d_valid =
                        true;

                    detection.body_index =
                        detectedBody;


                    validDetections.push_back(
                        detection
                    );
                }
            }


            /*
            * Una detección frontal sin nube 3D válida
            * no entra al contador.
            */
            cameraResult.detections =
                std::move(
                    validDetections
                );
        }

        else
        {
            /*
            * =====================================================
            * ASIGNACIÓN DE CUERPO PARA CÁMARAS TRASERAS
            * =====================================================
            *
            * Las cámaras 5 y 6 no utilizan nube 3D.
            *
            * El cuerpo se determina por la posición horizontal
            * del centro del bounding box dentro de la imagen.
            *
            * CAM 5 -> cuerpos 0,1,2
            * CAM 6 -> cuerpos 3,4,5
            */
            for (auto& detection :
                cameraResult.detections)
            {
                const int centerX =
                    detection.x +
                    detection.width / 2;


                detection.body_index =
                    TasselDetector::bodyFromImagePosition(
                        camera,
                        centerX,
                        cameraResult.image_width
                    );
            }
        }

         /*
         * =====================================================
         * TRACKING DE PANOJAS ÚNICAS
         * =====================================================
         *
         * TasselCounter devuelve exactamente cuáles
         * detecciones son nuevas.
         *
         * Esto conserva correctamente:
         *
         * - posición
         * - body_index
         * - tamaño
         * - confianza
         *
         * y evita usar solamente una cantidad.
         */
        const std::vector<
            TasselDetector::Detection
        > newDetections =
            tasselCounter.processDetections(
                cameraResult
            );


        /*
         * No apareció ninguna panoja nueva.
         */
        if (newDetections.empty())
        {
            continue;
        }


        /*
         * Creamos un resultado que contiene
         * exclusivamente las detecciones nuevas
         * confirmadas por el tracker.
         */
        TasselDetector::Result uniqueResult =
            cameraResult;


        uniqueResult.detections =
            newDetections;


        /*
        * Cámaras 0..4:
        * detección frontal.
        */
        if (camera < 5)
        {
            tasselVerifier.processFrontDetections(
                uniqueResult
            );
        }



        /*
        * Cámaras 5..6:
        * verificación trasera.
        */
        else
        {
            tasselVerifier.processRearDetections(
                uniqueResult
            );
        }
    }

    /*
    * ========================================================
    * ESTADO DEL MÓDULO DE IA
    * ========================================================
    *
    * La IA se considera activa cuando el detector
    * pudo procesar al menos un frame RGB válido
    * durante este ciclo de actualización.
    *
    * Actualmente TasselDetector utiliza detecciones
    * simuladas. Más adelante YOLO reemplazará solamente
    * esa etapa de detección.
    */
    if (state != nullptr)
    {
        HagieState::SystemState system =
            state->getSystemState();

        if (system.ai_running != aiProcessedFrame)
        {
            system.ai_running =
                aiProcessedFrame;

            state->setSystemState(
                system
            );
        }
    }


    /*
    * Actualizamos vencimientos de panojas
    * pendientes de verificación.
    */
    if (latestTimestampMs != 0)
    {
        tasselVerifier.update(
            latestTimestampMs
        );
    }


    /*
     * ========================================================
     * VISUALIZACIÓN DE LA CÁMARA SELECCIONADA
     * ========================================================
     */

    RgbFrameSource::Frame frame;


    if (!rgbCameraWorker.getFrame(
            selectedRgbCamera,
            frame
        ))
    {
        mainCameraLabel->setText(
            QString(
                "CÁMARA %1\n\n"
                "SIN FRAME RGB"
            ).arg(
                selectedRgbCamera + 1
            )
        );

        return;
    }


    if (!frame.valid ||
        frame.width == 0 ||
        frame.height == 0 ||
        frame.data.empty())
    {
        return;
    }


    QImage image(
        frame.data.data(),
        static_cast<int>(
            frame.width
        ),
        static_cast<int>(
            frame.height
        ),
        static_cast<int>(
            frame.width * 3
        ),
        QImage::Format_RGB888
    );


    QImage displayImage =
        image.copy();


    TasselDetector::Result detectionResult;

    bool detectionResultValid =
        false;


    /*
    * ========================================================
    * RESULTADO PARA VISUALIZACIÓN
    * ========================================================
    *
    * En simulación ejecutamos el detector simulado.
    *
    * En modo real reutilizamos exclusivamente el resultado
    * producido por YoloInferenceWorker.
    *
    * La GUI nunca ejecuta TensorRT.
    */
    if (realYoloMode)
    {
        YoloInferenceWorker::Result yoloDisplayResult;


        if (yoloInferenceWorker.getLatestResult(
                selectedRgbCamera,
                yoloDisplayResult
            ))
        {
            /*
            * Solo dibujamos bounding boxes cuando el resultado
            * corresponde exactamente al frame que estamos
            * mostrando.
            *
            * Esto evita dibujar cajas viejas sobre una imagen
            * RGB más nueva.
            */
            if (yoloDisplayResult.detectionResult.valid &&
                yoloDisplayResult.detectionResult.timestamp_ms ==
                    frame.timestamp_ms)
            {
                detectionResult =
                    std::move(
                        yoloDisplayResult.detectionResult
                    );

                detectionResultValid =
                    true;
            }
        }
    }
    else
    {
        detectionResultValid =
            tasselDetector.processFrame(
                selectedRgbCamera,
                frame,
                detectionResult
            );
    }


    if (detectionResultValid)
    {
        QPainter painter(
            &displayImage
        );


        QPen pen;

        pen.setWidth(
            3
        );

        painter.setPen(
            pen
        );


        for (const auto& detection :
            detectionResult.detections)
        {
            painter.drawRect(
                detection.x,
                detection.y,
                detection.width,
                detection.height
            );
        }


        const TasselCounter::State counterState =
            tasselCounter.getState();

        const TasselVerifier::State verifierState =
            tasselVerifier.getState();


        painter.drawText(
            10,
            25,
            QString(
                "Detectadas: %1"
            ).arg(
                detectionResult.detections.size()
            )
        );


        painter.drawText(
            10,
            50,
            QString(
                "Frontal acumulado: %1"
            ).arg(
                counterState.front_count
            )
        );


        painter.drawText(
            10,
            75,
            QString(
                "Trasero acumulado: %1"
            ).arg(
                counterState.rear_count
            )
        );


        painter.drawText(
            10,
            100,
            QString(
                "Pendientes: %1"
            ).arg(
                verifierState.pending
            )
        );


        painter.drawText(
            10,
            125,
            QString(
                "Removidas: %1"
            ).arg(
                verifierState.verified_removed
            )
        );


        painter.drawText(
            10,
            150,
            QString(
                "Siguen presentes: %1"
            ).arg(
                verifierState.verified_remaining
            )
        );
    }


    QPixmap pixmap =
        QPixmap::fromImage(
            displayImage
        );


    mainCameraLabel->setPixmap(
        pixmap.scaled(
            mainCameraLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
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


        clearNoMovementButtons[body] =
            new QPushButton(
                "Borrar NO_MOVEMENT"
            );

        QPushButton *clearButton =
            clearNoMovementButtons[body];

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
 * COMUNICACIONES
 * ============================================================
 */

QWidget *MainWindow::createCommunicationsPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *mainLayout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "DIAGNÓSTICO DE COMUNICACIONES"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(
        title
    );


   /*
    * ========================================================
    * STM32
    * ========================================================
    */
    QFrame *stm32Frame =
        new QFrame();

    stm32Frame->setFrameShape(
        QFrame::StyledPanel
    );

    QVBoxLayout *stm32Layout =
        new QVBoxLayout(
            stm32Frame
        );

    QLabel *stm32Title =
        new QLabel(
            "STM32"
        );

    stm32Title->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
    );

    communicationsStm32StatusLabel =
        new QLabel(
            "Estado: ---"
        );

    communicationsStm32UptimeLabel =
        new QLabel(
            "Uptime: ---"
        );

    communicationsUartErrorsLabel =
        new QLabel(
            "Errores UART: ---"
        );

    communicationsTxDroppedLabel =
        new QLabel(
            "TX descartados: ---"
        );

    stm32Layout->addWidget(
        stm32Title
    );

    stm32Layout->addWidget(
        communicationsStm32StatusLabel
    );

    stm32Layout->addWidget(
        communicationsStm32UptimeLabel
    );

    stm32Layout->addWidget(
        communicationsUartErrorsLabel
    );

    stm32Layout->addWidget(
        communicationsTxDroppedLabel
    );


    /*
    * ========================================================
    * CAN / AXIOMATIC
    * ========================================================
    */
    QFrame *canFrame =
        new QFrame();

    canFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QVBoxLayout *canLayout =
        new QVBoxLayout(
            canFrame
        );

    QLabel *canTitle =
        new QLabel(
            "CAN / AXIOMATIC"
        );

    canTitle->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
    );

    communicationsCanStatusLabel =
        new QLabel(
            "Estado CAN: ---"
        );

    communicationsAxiomaticModulesLabel =
        new QLabel(
            "Módulos detectados: ---"
        );

    communicationsCanDroppedLabel =
        new QLabel(
            "RX descartados: ---"
        );

    canLayout->addWidget(
        canTitle
    );

    canLayout->addWidget(
        communicationsCanStatusLabel
    );

    canLayout->addWidget(
        communicationsAxiomaticModulesLabel
    );

    canLayout->addWidget(
        communicationsCanDroppedLabel
    );


    /*
    * ========================================================
    * IMU HAGIE
    * ========================================================
    */
    QFrame *imuFrame =
        new QFrame();

    imuFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QVBoxLayout *imuLayout =
        new QVBoxLayout(
            imuFrame
        );

    QLabel *imuTitle =
        new QLabel(
            "IMU HAGIE"
        );

    imuTitle->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
    );

    communicationsImuStatusLabel =
        new QLabel(
            "Estado: ---"
        );

    imuLayout->addWidget(
        imuTitle
    );

    imuLayout->addWidget(
        communicationsImuStatusLabel
    );


    /*
    * ========================================================
    * VISIÓN 3D
    * ========================================================
    */
    QFrame *visionFrame =
        new QFrame();

    visionFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QVBoxLayout *visionLayout =
        new QVBoxLayout(
            visionFrame
        );

    QLabel *visionTitle =
        new QLabel(
            "VISIÓN 3D"
        );

    visionTitle->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
    );

    communicationsVisionStatusLabel =
        new QLabel(
            "Estado: ---"
        );

    visionLayout->addWidget(
        visionTitle
    );

    visionLayout->addWidget(
        communicationsVisionStatusLabel
    );


    /*
    * ========================================================
    * DISTRIBUCIÓN
    * ========================================================
    */
    QGridLayout *communicationsGrid =
        new QGridLayout();

    communicationsGrid->addWidget(
        stm32Frame,
        0,
        0
    );

    communicationsGrid->addWidget(
        canFrame,
        0,
        1
    );

    communicationsGrid->addWidget(
        imuFrame,
        1,
        0
    );

    communicationsGrid->addWidget(
        visionFrame,
        1,
        1
    );

    mainLayout->addLayout(
        communicationsGrid
    );

    mainLayout->addStretch();


    return page;
}

/*
 * ============================================================
 * LOGS
 * ============================================================
 */

QWidget *MainWindow::createLogsPage()
{
    QWidget *page =
        new QWidget();

    QVBoxLayout *mainLayout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "DIAGNÓSTICO - LOGS"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    mainLayout->addWidget(
        title
    );


   logsTextEdit =
        new QPlainTextEdit();

    logsTextEdit->setReadOnly(
        true
    );

    logsTextEdit->setLineWrapMode(
        QPlainTextEdit::NoWrap
    );


    QHBoxLayout *buttonsLayout =
        new QHBoxLayout();


    logsClearButton =
        new QPushButton(
            "LIMPIAR"
        );

    logsSaveButton =
        new QPushButton(
            "GUARDAR LOG"
        );

    logsPauseButton =
        new QPushButton(
            "PAUSAR"
        );


    logsClearButton->setMinimumHeight(
        50
    );

    logsSaveButton->setMinimumHeight(
        50
    );

    logsPauseButton->setMinimumHeight(
        50
    );


    buttonsLayout->addWidget(
        logsClearButton
    );

    buttonsLayout->addWidget(
        logsSaveButton
    );

    buttonsLayout->addWidget(
        logsPauseButton
    );


    mainLayout->addWidget(
        logsTextEdit,
        1
    );

    mainLayout->addLayout(
        buttonsLayout
    );

    /*
    * Limpiar visor de logs.
    */
    connect(
        logsClearButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            if (logsTextEdit != nullptr)
            {
                logsTextEdit->clear();
            }
        }
    );


    /*
    * Pausar / reanudar visualización de logs.
    */
    connect(
        logsPauseButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            logsPaused =
                !logsPaused;

            logsPauseButton->setText(
                logsPaused
                    ? "REANUDAR"
                    : "PAUSAR"
            );
        }
    );


    /*
    * Guardar logs en archivo de texto.
    */
    connect(
        logsSaveButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            if (logsTextEdit == nullptr)
            {
                return;
            }

            const QString defaultName =
                QString(
                    "hagie_log_%1.txt"
                ).arg(
                    QDateTime::currentDateTime()
                        .toString(
                            "yyyyMMdd_HHmmss"
                        )
                );

            const QString fileName =
                QFileDialog::getSaveFileName(
                    this,
                    "Guardar log",
                    defaultName,
                    "Archivos de texto (*.txt)"
                );

            if (fileName.isEmpty())
            {
                return;
            }

            QFile file(
                fileName
            );

            if (!file.open(
                    QIODevice::WriteOnly |
                    QIODevice::Text))
            {
                return;
            }

            QTextStream stream(
                &file
            );

            stream <<
                logsTextEdit->toPlainText();

            file.close();
        }
    );




    addLogMessage(
        "Sistema de logs iniciado"
    );


    return page;
}

/*
 * ============================================================
 * AGREGAR MENSAJE AL LOG
 * ============================================================
 */

void MainWindow::addLogMessage(
    const QString& message)
{
    if (logsPaused)
    {
        return;
    }

    if (logsTextEdit == nullptr)
    {
        return;
    }


    const QString timestamp =
        QDateTime::currentDateTime()
            .toString(
                "HH:mm:ss"
            );


    logsTextEdit->appendPlainText(
        QString("[%1] %2")
            .arg(
                timestamp,
                message
            )
    );
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
            configurationFilePath(),
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
            [this, body](int value)
            {
                QSettings settings(
                    configurationFilePath(),
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

                const uint64_t visionStaleTimeoutMs =
                    static_cast<uint64_t>(
                        configVisionDataTimeoutSpin != nullptr
                            ? configVisionDataTimeoutSpin->value()
                            : 1000
                    );

                bool visionFresh =
                    bodyState.vision_valid &&
                    bodyState.vision_timestamp_ms != 0 &&
                    (nowMs - bodyState.vision_timestamp_ms) <=
                        visionStaleTimeoutMs;
                /*
                * Condiciones mínimas para permitir AUTO VISIÓN.
                */
                if (!system.stm32_connected ||
                    !system.vision_running ||
                    !visionFresh ||
                    bodyState.faults != 0 ||
                    !isBodyCoveredByActiveCamera(body))
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

                    const uint64_t visionStaleTimeoutMs =
                        static_cast<uint64_t>(
                            configVisionDataTimeoutSpin != nullptr
                                ? configVisionDataTimeoutSpin->value()
                                : 1000
                        );

                    bool visionFresh =
                        bodyState.vision_valid &&
                        bodyState.vision_timestamp_ms != 0 &&
                        (nowMs - bodyState.vision_timestamp_ms) <=
                            visionStaleTimeoutMs;

                    bool bodyCovered =
                        isBodyCoveredByActiveCamera(body);

                    if (!system.stm32_connected ||
                        !system.vision_running ||
                        !visionFresh ||
                        bodyState.faults != 0 ||
                        !bodyCovered)
                    {
                        std::cout
                            << "AUTO VISION CANCEL body="
                            << body
                            << " stm32="
                            << system.stm32_connected
                            << " vision_running="
                            << system.vision_running
                            << " visionFresh="
                            << visionFresh
                            << " vision_valid="
                            << bodyState.vision_valid
                            << " vision_age_ms="
                            << (
                                bodyState.vision_timestamp_ms != 0
                                    ? nowMs - bodyState.vision_timestamp_ms
                                    : 0
                            )
                            << " faults=0x"
                            << std::hex
                            << bodyState.faults
                            << std::dec
                            << " covered="
                            << bodyCovered
                            << std::endl;
                    }


                    if (!system.stm32_connected ||
                    !system.vision_running ||
                    !visionFresh ||
                    bodyState.faults != 0 ||
                    !bodyCovered)
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
        250
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


void MainWindow::refreshZedCameraDetection()
{
    /*
     * Guardar cualquier cambio manual del serial
     * de la cámara actualmente seleccionada.
     */
    saveVisionCameraFromWidgets(
        currentVisionCamera
    );


    /*
     * Obtener las ZED físicamente detectadas.
     */
    /*
    * Consultar físicamente las cámaras una sola vez
    * y guardar el resultado en cache.
    */
    detectedZedSerialNumbers =
        ZedGmslPointCloudSource::
            detectConnectedSerialNumbers();


    const auto& detectedSerials =
        detectedZedSerialNumbers;


    /*
     * Limpiar la lista de ZED nuevas.
     */
    if (configVisionDetectedSerialCombo != nullptr)
    {
        configVisionDetectedSerialCombo->clear();
    }

    if (configRearRgbDetectedSerialCombo != nullptr)
    {
        configRearRgbDetectedSerialCombo->clear();
    }


    /*
     * ========================================================
     * 1. REVISAR CÁMARAS ASIGNADAS
     * ========================================================
     */
    for (std::size_t camera = 0;
         camera < Vision3DProcessor::CAMERA_COUNT;
         ++camera)
    {
        const uint32_t assignedSerial =
            visionCameraSerialNumbers[camera];


        if (assignedSerial == 0)
        {
            qInfo(
                "Camera %zu: SIN SERIAL ASIGNADO",
                camera + 1
            );

            continue;
        }


        bool detected = false;


        for (uint32_t detectedSerial :
             detectedSerials)
        {
            if (detectedSerial == assignedSerial)
            {
                detected = true;
                break;
            }
        }


        if (detected)
        {
            qInfo(
                "Camera %zu: DETECTADA - Serial %u",
                camera + 1,
                assignedSerial
            );
        }
        else
        {
            qWarning(
                "Camera %zu: NO ENCONTRADA - Serial %u",
                camera + 1,
                assignedSerial
            );
        }
    }


    /*
     * ========================================================
     * 2. BUSCAR ZED DETECTADAS PERO NO ASIGNADAS
     * ========================================================
     */
    for (uint32_t detectedSerial :
         detectedSerials)
    {
        bool alreadyAssigned = false;


        for (std::size_t camera = 0;
             camera < Vision3DProcessor::CAMERA_COUNT;
             ++camera)
        {
            if (visionCameraSerialNumbers[camera] ==
                detectedSerial)
            {
                alreadyAssigned = true;
                break;
            }
        }

            if (!alreadyAssigned)
        {
            for (std::size_t camera = 0;
                camera < rearRgbCameraSerialNumbers.size();
                ++camera)
            {
                if (rearRgbCameraSerialNumbers[camera] ==
                    detectedSerial)
                {
                    alreadyAssigned = true;
                    break;
                }
            }
        }


        if (!alreadyAssigned)
        {
            qInfo(
                "ZED NUEVA DETECTADA - Serial %u",
                detectedSerial
            );


            if (configVisionDetectedSerialCombo != nullptr)
            {
                configVisionDetectedSerialCombo->addItem(
                    QString::number(
                        detectedSerial
                    ),
                    QVariant::fromValue(
                        static_cast<qulonglong>(
                            detectedSerial
                        )
                    )
                );
            }

            if (configRearRgbDetectedSerialCombo != nullptr)
            {
                configRearRgbDetectedSerialCombo->addItem(
                    QString::number(
                        detectedSerial
                    ),
                    QVariant::fromValue(
                        static_cast<qulonglong>(
                            detectedSerial
                        )
                    )
                );
            }
        }
    }


    /*
     * Habilitar asignación solamente cuando
     * exista alguna ZED nueva.
     *
     * IMPORTANTE:
     * Esto queda FUERA del for anterior.
     */
    const bool hasNewCameras =
        configVisionDetectedSerialCombo != nullptr &&
        configVisionDetectedSerialCombo->count() > 0;


    if (configVisionDetectedSerialCombo != nullptr)
    {
        configVisionDetectedSerialCombo->setEnabled(
            hasNewCameras
        );
    }


    if (configVisionAssignDetectedButton != nullptr)
    {
        configVisionAssignDetectedButton->setEnabled(
            hasNewCameras
        );
    }

    if (configRearRgbDetectedSerialCombo != nullptr)
    {
        configRearRgbDetectedSerialCombo->setEnabled(
            hasNewCameras
        );
    }

    if (configRearRgbAssignDetectedButton != nullptr)
    {
        configRearRgbAssignDetectedButton->setEnabled(
            hasNewCameras
        );
    }


    /*
     * ========================================================
     * 3. ESTADO DE LA CÁMARA SELECCIONADA
     * ========================================================
     */
    const uint32_t selectedSerial =
        visionCameraSerialNumbers[
            currentVisionCamera
        ];


    if (configVisionCameraStatusLabel != nullptr)
    {
        if (selectedSerial == 0)
        {
            configVisionCameraStatusLabel->setText(
                "Estado ZED: SIN CÁMARA ASIGNADA"
            );
        }
        else
        {
            bool selectedDetected = false;


            for (uint32_t detectedSerial :
                 detectedSerials)
            {
                if (detectedSerial ==
                    selectedSerial)
                {
                    selectedDetected = true;
                    break;
                }
            }


            if (selectedDetected)
            {
                configVisionCameraStatusLabel->setText(
                    QString(
                        "Estado ZED: DETECTADA - Serial %1"
                    ).arg(
                        selectedSerial
                    )
                );
            }
            else
            {
                configVisionCameraStatusLabel->setText(
                    QString(
                        "Estado ZED: NO ENCONTRADA - Serial %1"
                    ).arg(
                        selectedSerial
                    )
                );
            }
        }
    }

        if (configRearRgbCameraStatusLabel != nullptr)
    {
        const uint32_t rearSelectedSerial =
            rearRgbCameraSerialNumbers[
                currentRearRgbCamera
            ];

        if (rearSelectedSerial == 0)
        {
            configRearRgbCameraStatusLabel->setText(
                "Estado ZED: SIN CÁMARA ASIGNADA"
            );
        }
        else
        {
            bool rearSelectedDetected = false;

            for (uint32_t detectedSerial :
                 detectedSerials)
            {
                if (detectedSerial ==
                    rearSelectedSerial)
                {
                    rearSelectedDetected = true;
                    break;
                }
            }

            if (rearSelectedDetected)
            {
                configRearRgbCameraStatusLabel->setText(
                    QString(
                        "Estado ZED: DETECTADA - Serial %1"
                    ).arg(
                        rearSelectedSerial
                    )
                );
            }
            else
            {
                configRearRgbCameraStatusLabel->setText(
                    QString(
                        "Estado ZED: NO ENCONTRADA - Serial %1"
                    ).arg(
                        rearSelectedSerial
                    )
                );
            }
        }
    }


    qInfo(
        "ZED: %zu cámara(s) física(s) detectada(s)",
        detectedSerials.size()
    );
}


bool MainWindow::isBodyCoveredByActiveCamera(
    std::size_t body) const
{
    if (body >= HagieState::BODY_COUNT)
    {
        return false;
    }


    for (std::size_t camera = 0;
         camera < Vision3DProcessor::CAMERA_COUNT;
         ++camera)
    {
        const auto& config =
            visionCameraConfigs[camera];


        if (!config.enabled)
        {
            continue;
        }


        if (!config.body_enabled[body])
        {
            continue;
        }


        const uint32_t serial =
            visionCameraSerialNumbers[camera];


        if (serial == 0)
        {
            continue;
        }


        for (uint32_t detectedSerial :
             detectedZedSerialNumbers)
        {
            if (detectedSerial == serial)
            {
                return true;
            }
        }
    }


    return false;
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


    /*
    * ========================================================
    * NAVEGACIÓN DE CONFIGURACIÓN
    * ========================================================
    */
    QHBoxLayout *configurationContentLayout =
        new QHBoxLayout();


    /*
    * Menú lateral.
    */
    QFrame *configurationMenuFrame =
        new QFrame();

    configurationMenuFrame->setFrameShape(
        QFrame::StyledPanel
    );

    configurationMenuFrame->setMaximumWidth(
        220
    );

    QVBoxLayout *configurationMenuLayout =
        new QVBoxLayout(
            configurationMenuFrame
        );


    /*
    * Páginas internas.
    */
    QStackedWidget *configurationStack =
        new QStackedWidget();


    QWidget *generalPage =
        new QWidget();

    QVBoxLayout *generalPageLayout =
        new QVBoxLayout(
            generalPage
        );


    QWidget *bodiesPage =
        new QWidget();

    QVBoxLayout *bodiesPageLayout =
        new QVBoxLayout(
            bodiesPage
        );


    QWidget *camerasPage =
        new QWidget();

    QVBoxLayout *camerasPageLayout =
        new QVBoxLayout(
            camerasPage
        );
        QWidget *rearRgbCamerasPage =
        new QWidget();

    QVBoxLayout *rearRgbCamerasPageLayout =
        new QVBoxLayout(
            rearRgbCamerasPage
        );

    QLabel *rearRgbCamerasTitle =
        new QLabel(
            "CONFIGURACIÓN DE CÁMARAS TRASERAS RGB"
        );

    rearRgbCamerasTitle->setAlignment(
        Qt::AlignCenter
    );

    rearRgbCamerasTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    rearRgbCamerasPageLayout->addWidget(
        rearRgbCamerasTitle
    );

        QHBoxLayout *rearRgbCameraSelectorLayout =
        new QHBoxLayout();

    QLabel *rearRgbCameraSelectorLabel =
        new QLabel(
            "Cámara:"
        );

    configRearRgbCameraCombo =
        new QComboBox();

    configRearRgbCameraCombo->addItem(
        "CÁMARA 6",
        5
    );

    configRearRgbCameraCombo->addItem(
        "CÁMARA 7",
        6
    );

    connect(
        configRearRgbCameraCombo,
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

            if (currentRearRgbCamera >=
                rearRgbCameraSerialNumbers.size())
            {
                return;
            }

            rearRgbCameraSerialNumbers[
                currentRearRgbCamera
            ] =
                static_cast<uint32_t>(
                    configRearRgbCameraSerial->value()
                );

            currentRearRgbCamera =
                static_cast<std::size_t>(
                    index
                );

            if (currentRearRgbCamera >=
                rearRgbCameraSerialNumbers.size())
            {
                return;
            }

            configRearRgbCameraSerial->setValue(
                static_cast<int>(
                    rearRgbCameraSerialNumbers[
                        currentRearRgbCamera
                    ]
                )
            );
            refreshZedCameraDetection();
        }
    );

    rearRgbCameraSelectorLayout->addWidget(
        rearRgbCameraSelectorLabel
    );

    rearRgbCameraSelectorLayout->addWidget(
        configRearRgbCameraCombo
    );

    rearRgbCameraSelectorLayout->addStretch();

    rearRgbCamerasPageLayout->addLayout(
        rearRgbCameraSelectorLayout
    );

        QHBoxLayout *rearRgbCameraSerialLayout =
        new QHBoxLayout();

    QLabel *rearRgbCameraSerialLabel =
        new QLabel(
            "Serial ZED:"
        );

    configRearRgbCameraSerial =
        new QSpinBox();

    configRearRgbCameraSerial ->setRange(
        0,
        999999999
    );

    configRearRgbCameraSerial->setSpecialValueText(
        "NO CONFIGURADO"
    );

    rearRgbCameraSerialLayout->addWidget(
        rearRgbCameraSerialLabel
    );

    rearRgbCameraSerialLayout->addWidget(
        configRearRgbCameraSerial
    );

    rearRgbCameraSerialLayout->addStretch();

    rearRgbCamerasPageLayout->addLayout(
        rearRgbCameraSerialLayout
    );

    configRearRgbCameraStatusLabel =
    new QLabel(
        "Estado ZED: NO DETECTADA"
    );

    configRearRgbCameraStatusLabel->setStyleSheet(
        "font-weight: bold;"
    );

    rearRgbCamerasPageLayout->addWidget(
       configRearRgbCameraStatusLabel
    );

    configRearRgbDetectButton =
        new QPushButton(
            "DETECTAR CÁMARAS ZED"
        );

    rearRgbCamerasPageLayout->addWidget(
        configRearRgbDetectButton
    );

    connect(
        configRearRgbDetectButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            if (configRearRgbCameraSerial != nullptr &&
                currentRearRgbCamera <
                    rearRgbCameraSerialNumbers.size())
            {
                rearRgbCameraSerialNumbers[
                    currentRearRgbCamera
                ] =
                    static_cast<uint32_t>(
                        configRearRgbCameraSerial->value()
                    );
            }

            refreshZedCameraDetection();
        }
    );

    QHBoxLayout *rearRgbDetectedSerialLayout =
        new QHBoxLayout();

    QLabel *rearRgbDetectedSerialLabel =
        new QLabel(
            "ZED nueva detectada:"
        );

    configRearRgbDetectedSerialCombo =
        new QComboBox();

    configRearRgbDetectedSerialCombo ->setEnabled(
        false
    );

    rearRgbDetectedSerialLayout->addWidget(
        rearRgbDetectedSerialLabel
    );

    rearRgbDetectedSerialLayout->addWidget(
        configRearRgbDetectedSerialCombo
    );

    rearRgbDetectedSerialLayout->addStretch();

    rearRgbCamerasPageLayout->addLayout(
        rearRgbDetectedSerialLayout
    );

    configRearRgbAssignDetectedButton =
        new QPushButton(
            "ASIGNAR A CÁMARA SELECCIONADA"
        );

    configRearRgbAssignDetectedButton->setEnabled(
        false
    );

    rearRgbCamerasPageLayout->addWidget(
        configRearRgbAssignDetectedButton
    );

    connect(
        configRearRgbAssignDetectedButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            if (configRearRgbDetectedSerialCombo == nullptr)
            {
                return;
            }

            if (configRearRgbDetectedSerialCombo->count() == 0)
            {
                return;
            }

            if (currentRearRgbCamera >=
                rearRgbCameraSerialNumbers.size())
            {
                return;
            }

            const QVariant serialData =
                configRearRgbDetectedSerialCombo->currentData();

            const uint32_t selectedSerial =
                static_cast<uint32_t>(
                    serialData.toULongLong()
                );

            rearRgbCameraSerialNumbers[
                currentRearRgbCamera
            ] =
                selectedSerial;

            if (configRearRgbCameraSerial != nullptr)
            {
                configRearRgbCameraSerial->setValue(
                    static_cast<int>(
                        selectedSerial
                    )
                );
            }

            refreshZedCameraDetection();
        }
    );

    rearRgbCamerasPageLayout->addStretch();    

    QWidget *regionsPage =
        new QWidget();

    QVBoxLayout *regionsPageLayout =
        new QVBoxLayout(
            regionsPage
        );

            


    QWidget *controlPage =
        new QWidget();

    QVBoxLayout *controlPageLayout =
        new QVBoxLayout(
            controlPage
        );

        QWidget *tasselVerificationPage =
        new QWidget();

    QVBoxLayout *tasselVerificationPageLayout =
        new QVBoxLayout(
            tasselVerificationPage
        );


    QLabel *tasselVerificationTitle =
        new QLabel(
            "VERIFICACIÓN DE PANOJAS"
        );

    tasselVerificationTitle->setAlignment(
        Qt::AlignCenter
    );

    tasselVerificationTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    tasselVerificationPageLayout->addWidget(
        tasselVerificationTitle
    );


    /*
     * Velocidad estimada.
     */
    QHBoxLayout *tasselSpeedLayout =
        new QHBoxLayout();

    QLabel *tasselSpeedLabel =
        new QLabel(
            "Velocidad de trabajo estimada (km/h):"
        );

    configTasselSpeedSpin =
        new QDoubleSpinBox();

    configTasselSpeedSpin->setRange(
        0.1,
        30.0
    );

    configTasselSpeedSpin->setDecimals(
        1
    );

    configTasselSpeedSpin->setSingleStep(
        0.5
    );

    configTasselSpeedSpin->setValue(
        6.0
    );

    tasselSpeedLayout->addWidget(
        tasselSpeedLabel
    );

    tasselSpeedLayout->addWidget(
        configTasselSpeedSpin
    );

    tasselSpeedLayout->addStretch();

    tasselVerificationPageLayout->addLayout(
        tasselSpeedLayout
    );


    /*
     * Distancia entre cámaras delanteras y traseras.
     */
    QHBoxLayout *tasselDistanceLayout =
        new QHBoxLayout();

    QLabel *tasselDistanceLabel =
        new QLabel(
            "Distancia cámaras delanteras / traseras (mm):"
        );

    configTasselCameraDistanceSpin =
        new QSpinBox();

    configTasselCameraDistanceSpin->setRange(
        100,
        20000
    );

    configTasselCameraDistanceSpin->setSingleStep(
        100
    );

    configTasselCameraDistanceSpin->setValue(
        3000
    );

    tasselDistanceLayout->addWidget(
        tasselDistanceLabel
    );

    tasselDistanceLayout->addWidget(
        configTasselCameraDistanceSpin
    );

    tasselDistanceLayout->addStretch();

    tasselVerificationPageLayout->addLayout(
        tasselDistanceLayout
    );


    /*
     * Tolerancia temporal.
     */
    QHBoxLayout *tasselToleranceLayout =
        new QHBoxLayout();

    QLabel *tasselToleranceLabel =
        new QLabel(
            "Tolerancia temporal (%):"
        );

    configTasselTimingToleranceSpin =
        new QDoubleSpinBox();

    configTasselTimingToleranceSpin->setRange(
        0.0,
        100.0
    );

    configTasselTimingToleranceSpin->setDecimals(
        1
    );

    configTasselTimingToleranceSpin->setSingleStep(
        5.0
    );

    configTasselTimingToleranceSpin->setValue(
        30.0
    );

    tasselToleranceLayout->addWidget(
        tasselToleranceLabel
    );

    tasselToleranceLayout->addWidget(
        configTasselTimingToleranceSpin
    );

    tasselToleranceLayout->addStretch();

    tasselVerificationPageLayout->addLayout(
        tasselToleranceLayout
    );


    /*
     * Valores calculados.
     */
    configTasselExpectedTimeLabel =
        new QLabel(
            "Tiempo estimado hasta cámara trasera: --- ms"
        );

    configTasselVerificationWindowLabel =
        new QLabel(
            "Ventana de verificación: --- ms"
        );

    configTasselExpectedTimeLabel->setStyleSheet(
        "font-weight: bold;"
    );

    configTasselVerificationWindowLabel->setStyleSheet(
        "font-weight: bold;"
    );

    tasselVerificationPageLayout->addSpacing(
        20
    );

    tasselVerificationPageLayout->addWidget(
        configTasselExpectedTimeLabel
    );

    tasselVerificationPageLayout->addWidget(
        configTasselVerificationWindowLabel
    );

        connect(
        configTasselSpeedSpin,
        QOverload<double>::of(
            &QDoubleSpinBox::valueChanged
        ),
        this,
        [this](double)
        {
            updateTasselVerificationTiming();
        }
    );

    connect(
        configTasselCameraDistanceSpin,
        QOverload<int>::of(
            &QSpinBox::valueChanged
        ),
        this,
        [this](int)
        {
            updateTasselVerificationTiming();
        }
    );

    connect(
        configTasselTimingToleranceSpin,
        QOverload<double>::of(
            &QDoubleSpinBox::valueChanged
        ),
        this,
        [this](double)
        {
            updateTasselVerificationTiming();
        }
    );

    updateTasselVerificationTiming();

    tasselVerificationPageLayout->addStretch();    


    configurationStack->addWidget(
        generalPage
    );

    configurationStack->addWidget(
        bodiesPage
    );

    configurationStack->addWidget(
        camerasPage
    );

    configurationStack->addWidget(
        rearRgbCamerasPage
    );

    configurationStack->addWidget(
        regionsPage
    );

    configurationStack->addWidget(
        controlPage
    );

        configurationStack->addWidget(
        tasselVerificationPage
    );

    QPushButton *generalButton =
    new QPushButton(
        "GENERAL"
    );

    QPushButton *bodiesButton =
        new QPushButton(
            "CUERPOS / ENCODERS"
        );

    QPushButton *camerasButton =
        new QPushButton(
            "CÁMARAS 3D"
        );

    QPushButton *rearRgbCamerasButton =
    new QPushButton(
        "CÁMARAS TRASERAS RGB"
        );    

    QPushButton *regionsButton =
        new QPushButton(
            "REGIONES 3D"
        );

    QPushButton *controlButton =
        new QPushButton(
            "CONTROL / SEGURIDAD"
        );

    QPushButton *tasselVerificationButton =
        new QPushButton(
            "VERIFICACIÓN PANOJAS"
        );


    generalButton->setMinimumHeight(50);
    bodiesButton->setMinimumHeight(50);
    camerasButton->setMinimumHeight(50);
    rearRgbCamerasButton->setMinimumHeight(50);
    regionsButton->setMinimumHeight(50);
    controlButton->setMinimumHeight(50);
        tasselVerificationButton->setMinimumHeight(
        50
    );


    configurationMenuLayout->addWidget(
        generalButton
    );

    configurationMenuLayout->addWidget(
        bodiesButton
    );

    configurationMenuLayout->addWidget(
        camerasButton
    );

    configurationMenuLayout->addWidget(
        rearRgbCamerasButton
    );

    configurationMenuLayout->addWidget(
        regionsButton
    );

    configurationMenuLayout->addWidget(
        controlButton
    );

    configurationMenuLayout->addWidget(
        tasselVerificationButton
    );

    configurationMenuLayout->addStretch();

    connect(
        generalButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                0
            );
        }
    );

    connect(
        bodiesButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                1
            );
        }
    );

    connect(
        camerasButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                2
            );
        }
    );

    connect(
        rearRgbCamerasButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                3
            );
        }
    );

    connect(
        regionsButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                4
            );
        }
    );

    connect(
        controlButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                5
            );
        }
    );

        connect(
        tasselVerificationButton,
        &QPushButton::clicked,
        this,
        [configurationStack]()
        {
            configurationStack->setCurrentIndex(
                6
            );
        }
    );


    configurationContentLayout->addWidget(
        configurationMenuFrame
    );

    configurationContentLayout->addWidget(
        configurationStack,
        1
    );

    
    mainLayout->addLayout(
        configurationContentLayout,
        1
    );



         /*
     * ========================================================
     * PARÁMETROS GENERALES DE CÁMARAS Y PROCESAMIENTO
     * ========================================================
     */
    QFrame *cameraGeneralConfigFrame =
        new QFrame();

    cameraGeneralConfigFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QVBoxLayout *cameraGeneralConfigLayout =
        new QVBoxLayout(
            cameraGeneralConfigFrame
        );


    QLabel *cameraGeneralConfigTitle =
        new QLabel(
            "PARÁMETROS GENERALES DE CÁMARAS Y PROCESAMIENTO"
        );

    cameraGeneralConfigTitle->setAlignment(
        Qt::AlignCenter
    );

    cameraGeneralConfigTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    cameraGeneralConfigLayout->addWidget(
        cameraGeneralConfigTitle
    );


    /*
     * FPS cámaras.
     */
    QHBoxLayout *cameraFpsLayout =
        new QHBoxLayout();

    QLabel *cameraFpsLabel =
        new QLabel(
            "FPS cámaras:"
        );

    configCameraFpsCombo =
        new QComboBox();

    configCameraFpsCombo->addItem(
        "15",
        15
    );

    configCameraFpsCombo->addItem(
        "30",
        30
    );

    configCameraFpsCombo->addItem(
        "60",
        60
    );

    cameraFpsLayout->addWidget(
        cameraFpsLabel
    );

    cameraFpsLayout->addWidget(
        configCameraFpsCombo
    );

    cameraFpsLayout->addStretch();

    cameraGeneralConfigLayout->addLayout(
        cameraFpsLayout
    );


    /*
     * Resolución RGB.
     */
    QHBoxLayout *cameraResolutionLayout =
        new QHBoxLayout();

    QLabel *cameraResolutionLabel =
        new QLabel(
            "Resolución RGB:"
        );

    configCameraResolutionCombo =
        new QComboBox();

    configCameraResolutionCombo->addItem(
        "HD720",
        "HD720"
    );

    configCameraResolutionCombo->addItem(
        "HD1080",
        "HD1080"
    );

    cameraResolutionLayout->addWidget(
        cameraResolutionLabel
    );

    cameraResolutionLayout->addWidget(
        configCameraResolutionCombo
    );

    cameraResolutionLayout->addStretch();

    cameraGeneralConfigLayout->addLayout(
        cameraResolutionLayout
    );


    /*
     * Timeout de cámara.
     */
    QHBoxLayout *cameraTimeoutLayout =
        new QHBoxLayout();

    QLabel *cameraTimeoutLabel =
        new QLabel(
            "Timeout cámara (ms):"
        );

    configCameraTimeoutSpin =
        new QSpinBox();

    configCameraTimeoutSpin->setRange(
        100,
        10000
    );

    configCameraTimeoutSpin->setSingleStep(
        100
    );

    configCameraTimeoutSpin->setValue(
        1000
    );

    cameraTimeoutLayout->addWidget(
        cameraTimeoutLabel
    );

    cameraTimeoutLayout->addWidget(
        configCameraTimeoutSpin
    );

    cameraTimeoutLayout->addStretch();

    cameraGeneralConfigLayout->addLayout(
        cameraTimeoutLayout
    );


    /*
     * Reconexión automática.
     */
    configCameraAutoReconnectCheck =
        new QCheckBox(
            "Reconexión automática de cámaras"
        );

    configCameraAutoReconnectCheck->setChecked(
        true
    );

    cameraGeneralConfigLayout->addWidget(
        configCameraAutoReconnectCheck
    );


    /*
     * Intervalo de reconexión.
     */
    QHBoxLayout *cameraReconnectIntervalLayout =
        new QHBoxLayout();

    QLabel *cameraReconnectIntervalLabel =
        new QLabel(
            "Intervalo reconexión (ms):"
        );

    configCameraReconnectIntervalSpin =
        new QSpinBox();

    configCameraReconnectIntervalSpin->setRange(
        500,
        30000
    );

    configCameraReconnectIntervalSpin->setSingleStep(
        500
    );

    configCameraReconnectIntervalSpin->setValue(
        2000
    );

    cameraReconnectIntervalLayout->addWidget(
        cameraReconnectIntervalLabel
    );

    cameraReconnectIntervalLayout->addWidget(
        configCameraReconnectIntervalSpin
    );

    cameraReconnectIntervalLayout->addStretch();

    cameraGeneralConfigLayout->addLayout(
        cameraReconnectIntervalLayout
    );


    /*
     * Procesar IA cada N frames.
     */
    QHBoxLayout *aiFrameIntervalLayout =
        new QHBoxLayout();

    QLabel *aiFrameIntervalLabel =
        new QLabel(
            "Procesar IA cada N frames:"
        );

    configAiFrameIntervalSpin =
        new QSpinBox();

    configAiFrameIntervalSpin->setRange(
        1,
        30
    );

    configAiFrameIntervalSpin->setValue(
        1
    );

    connect(
        configAiFrameIntervalSpin,
        QOverload<int>::of(
            &QSpinBox::valueChanged
        ),
        this,
        [this](int value)
        {
            yoloInferenceWorker.setFrameInterval(
                static_cast<std::size_t>(
                    value
                )
            );
        }
    );

    aiFrameIntervalLayout->addWidget(
        aiFrameIntervalLabel
    );

    aiFrameIntervalLayout->addWidget(
        configAiFrameIntervalSpin
    );

    aiFrameIntervalLayout->addStretch();

    cameraGeneralConfigLayout->addLayout(
        aiFrameIntervalLayout
    );


    /*
     * Timeout de datos 3D.
     */
    QHBoxLayout *visionDataTimeoutLayout =
        new QHBoxLayout();

    QLabel *visionDataTimeoutLabel =
        new QLabel(
            "Timeout datos 3D (ms):"
        );

    configVisionDataTimeoutSpin =
        new QSpinBox();

    configVisionDataTimeoutSpin->setRange(
        100,
        10000
    );

    configVisionDataTimeoutSpin->setSingleStep(
        100
    );

    configVisionDataTimeoutSpin->setValue(
        500
    );

    visionDataTimeoutLayout->addWidget(
        visionDataTimeoutLabel
    );

    visionDataTimeoutLayout->addWidget(
        configVisionDataTimeoutSpin
    );

    visionDataTimeoutLayout->addStretch();

    cameraGeneralConfigLayout->addLayout(
        visionDataTimeoutLayout
    );


    generalPageLayout->addWidget(
        cameraGeneralConfigFrame
    );   
   

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
            /*
            * Detener inferencia antes de reemplazar
            * cualquier fuente RGB.
            */
            yoloInferenceWorker.stop();

            lastConsumedYoloTimestamp.fill(
                0
            );

            tasselCounter.reset();
            tasselVerifier.reset();
            
            if (visionHeightSource == nullptr)
            {
                return;
            }

            QString visionSourceText;

            if (index == 0)
            {
                visionSourceText =
                    "SIMULACIÓN ALTURAS";
            }
            else if (index == 1)
            {
                visionSourceText =
                    "SIMULACIÓN CÁMARAS 3D";
            }
            else if (index == 2)
            {
                visionSourceText =
                    "CÁMARAS 3D REALES";
            }
            else
            {
                visionSourceText =
                    "DESCONOCIDA";
            }

            addLogMessage(
                QString(
                    "Fuente de visión cambiada a %1"
                ).arg(
                    visionSourceText
                )
            );


            /*
            * ====================================================
            * MODO 0
            * SIMULACIÓN DIRECTA DE ALTURAS
            * ====================================================
            */
            if (index == 0)
            {

            if (vision3DWorker != nullptr)
            {
                vision3DWorker->clearMachineOrientationOverride();
            }
            
       
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
                * La salida de visión vendrá desde
                * Vision3DWorker.
                */
                visionHeightSource->setSourceMode(
                    VisionHeightSource::SourceMode::EXTERNAL
                );

                if (vision3DWorker != nullptr)
                {
                    vision3DWorker->setMachineOrientationOverride(
                        2.0f,
                        -1.0f
                    );
                }

                


                if (vision3DWorker == nullptr)
                {
                    return;
                }


                /*
                * Detener antes de reemplazar las fuentes.
                */
                vision3DWorker->stop();


                /*
                * Restaurar RGB simulado de las
                * siete cámaras físicas.
                */
                for (std::size_t camera = 0;
                    camera < RgbCameraWorker::CAMERA_COUNT;
                    ++camera)
                {
                    rgbCameraWorker.clearFrameSource(
                        camera
                    );


                    auto rgbSource =
                        std::make_unique<
                            SimulatedRgbFrameSource
                        >(
                            camera
                        );


                    if (!rgbCameraWorker.setFrameSource(
                            camera,
                            std::move(rgbSource)
                        ))
                    {
                        qWarning(
                            "No se pudo restaurar RGB simulado "
                            "de la cámara %zu",
                            camera + 1
                        );
                    }
                }


                /*
                * Instalar las cinco fuentes simuladas.
                */
                for (std::size_t camera = 0;
                    camera < Vision3DProcessor::CAMERA_COUNT;
                    ++camera)
                {
                    vision3DWorker->clearPointCloudSource(
                        camera
                    );


                    auto source =
                        std::make_unique<
                            SimulatedPointCloudSource
                        >(
                            camera
                        );
                     /*
                    * Orientación IMU simulada para comprobar
                    * el diagnóstico en tiempo real de la GUI.
                    */
                    if (camera == 0)
                    {
                        source->setSimulatedOrientation(
                            3.5f,
                            -0.2f
                        );
                    }
                    else if (camera == 1)
                    {
                        source->setSimulatedOrientation(
                            1.0f,
                            0.5f
                        );
                    }
                    else if (camera == 2)
                    {
                        source->setSimulatedOrientation(
                            -1.2f,
                            0.3f
                        );
                    }   
                    
                    else if (camera == 3)
                    {
                        source->setSimulatedOrientation(
                            0.8f,
                            -0.4f
                        );
                    }
                    else if (camera == 4)
                    {
                        source->setSimulatedOrientation(
                            -0.6f,
                            0.2f
                        );
                    }
                                        
                        

                    if (!vision3DWorker->setPointCloudSource(
                            camera,
                            std::move(source)
                        ))
                    {
                        qWarning(
                            "No se pudo instalar la fuente simulada "
                            "de la cámara %zu",
                            camera
                        );
                    }
                }


                if (!vision3DWorker->start())
                {
                    qWarning(
                        "No se pudo iniciar Vision3DWorker "
                        "en simulación de cámaras 3D"
                    );
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
                
                                /*
                 * ========================================================
                 * VALIDAR CONFIGURACIÓN TENSORRT
                 * ========================================================
                 */
                const QString enginePath =
                    configYoloEngineEdit
                        ->text()
                        .trimmed();


                bool tensorRtConfigValid =
                    true;


                if (enginePath.isEmpty())
                {
                    qWarning(
                        "TensorRT: no se configuró ningún engine"
                    );

                    addLogMessage(
                        "TensorRT: no se configuró ningún engine"
                    );

                    tensorRtConfigValid =
                        false;
                }
                else if (!QFileInfo::exists(
                             enginePath
                         ))
                {
                    qWarning(
                        "TensorRT: el engine no existe: %s",
                        qPrintable(enginePath)
                    );

                    addLogMessage(
                        QString(
                            "TensorRT: el engine no existe: %1"
                        ).arg(
                            enginePath
                        )
                    );

                    tensorRtConfigValid =
                        false;
                }
                else
                {
                    addLogMessage(
                        QString(
                            "TensorRT: engine encontrado: %1"
                        ).arg(
                            enginePath
                        )
                    );
                }
                /*
                * Invalidar cualquier IMU Hagie simulada
                * al entrar en modo de cámaras reales.
                */

                if (vision3DWorker != nullptr)
                {
                    vision3DWorker->clearMachineOrientationOverride();
                }
                
                /*
                * Actualizar el cache de cámaras ZED
                * antes de iniciar visión real.
                */
                refreshZedCameraDetection();
                
                /*
                * Los resultados vendrán desde
                * las cámaras ZED reales.
                */
                visionHeightSource->setSourceMode(
                    VisionHeightSource::SourceMode::EXTERNAL
                );


                if (vision3DWorker == nullptr)
                {
                    return;
                }


                /*
                * Detener antes de reemplazar fuentes.
                */
                vision3DWorker->stop();

                


                /*
                * Al entrar en modo REAL eliminamos
                * todas las fuentes RGB anteriores.
                *
                * Esto evita mezclar:
                *
                * - frontales ZED reales
                * - traseras simuladas
                */
                for (std::size_t camera = 0;
                    camera < RgbCameraWorker::CAMERA_COUNT;
                    ++camera)
                {
                    rgbCameraWorker.clearFrameSource(
                        camera
                    );
                }


                /*
                * Detectar las ZED físicas disponibles.
                */
                const auto& detectedSerials =
                    detectedZedSerialNumbers;


                std::size_t usableCameraCount =
                    0;


                for (std::size_t camera = 0;
                    camera < Vision3DProcessor::CAMERA_COUNT;
                    ++camera)
                {
                    /*
                    * Limpiar cualquier fuente anterior.
                    */
                    vision3DWorker->clearPointCloudSource(
                        camera
                    );


                    /*
                    * Si la cámara está deshabilitada,
                    * se ignora completamente.
                    */
                    if (!visionCameraConfigs[camera].enabled)
                    {
                        qInfo(
                            "Camera %zu: DESHABILITADA",
                            camera + 1
                        );

                        continue;
                    }


                    const uint32_t serial =
                        visionCameraSerialNumbers[camera];


                    /*
                    * Cámara habilitada pero sin serial.
                    */
                    if (serial == 0)
                    {
                        qWarning(
                            "Camera %zu habilitada pero sin serial ZED configurado",
                            camera + 1
                        );

                        continue;
                    }


                    /*
                    * Verificar si esa ZED está físicamente presente.
                    */
                    bool physicallyDetected =
                        false;


                    for (uint32_t detectedSerial :
                        detectedSerials)
                    {
                        if (detectedSerial == serial)
                        {
                            physicallyDetected =
                                true;

                            break;
                        }
                    }


                    if (!physicallyDetected)
                    {
                        qWarning(
                            "Camera %zu habilitada pero NO ENCONTRADA - Serial %u",
                            camera + 1,
                            serial
                        );

                        /*
                        * No bloqueamos las demás cámaras.
                        */
                        continue;
                    }


                    /*
                    * Instalar la fuente ZED real.
                    */
                    auto source =
                        std::make_unique<
                            ZedGmslPointCloudSource
                        >(
                            camera,
                            serial,
                            configCameraFpsCombo
                                ->currentData()
                                .toInt(),
                            configCameraResolutionCombo
                                ->currentData()
                                .toString()
                                .toStdString(),
                            configCameraTimeoutSpin
                                ->value(),
                            configCameraAutoReconnectCheck
                                ->isChecked(),
                            configCameraReconnectIntervalSpin
                                ->value()
                        );

                    auto sharedRgbFrame =
                        source->getSharedRgbFrame();


                    auto rgbSource =
                        std::make_unique<
                            ZedGmslRgbFrameSource
                        >(
                            camera,
                            sharedRgbFrame
                        );


                    if (!rgbCameraWorker.setFrameSource(
                            camera,
                            std::move(rgbSource)
                        ))
                    {
                        qWarning(
                            "No se pudo instalar RGB ZED Camera %zu - Serial %u",
                            camera + 1,
                            serial
                        );

                        continue;
                    }


                    if (!vision3DWorker->setPointCloudSource(
                            camera,
                            std::move(source)
                        ))
                    {
                        qWarning(
                            "No se pudo instalar ZED Camera %zu - Serial %u",
                            camera + 1,
                            serial
                        );

                        rgbCameraWorker.clearFrameSource(
                            camera
                        );

                        continue;
                    }


                    ++usableCameraCount;


                    qInfo(
                        "Camera %zu lista para visión - Serial %u",
                        camera + 1,
                        serial
                    );
                }

                                /*
                * ========================================================
                * CÁMARAS TRASERAS RGB
                * ========================================================
                *
                * rearRgbCameraSerialNumbers[0] -> Cámara física 6
                * rearRgbCameraSerialNumbers[1] -> Cámara física 7
                *
                * Se utilizan solamente para RGB.
                * No generan nube de puntos.
                */
                for (std::size_t rearCamera = 0;
                    rearCamera < rearRgbCameraSerialNumbers.size();
                    ++rearCamera)
                {
                    /*
                    * Índice lógico dentro de las siete cámaras:
                    *
                    * 5 -> Cámara 6
                    * 6 -> Cámara 7
                    */
                    const std::size_t camera =
                        Vision3DProcessor::CAMERA_COUNT +
                        rearCamera;


                    const uint32_t serial =
                        rearRgbCameraSerialNumbers[
                            rearCamera
                        ];


                    /*
                    * Cámara trasera sin serial configurado.
                    */
                    if (serial == 0)
                    {
                        qWarning(
                            "Camera %zu trasera RGB sin serial ZED configurado",
                            camera + 1
                        );

                        continue;
                    }


                    /*
                    * Verificar que la ZED esté físicamente presente.
                    */
                    bool physicallyDetected =
                        false;


                    for (uint32_t detectedSerial :
                        detectedSerials)
                    {
                        if (detectedSerial == serial)
                        {
                            physicallyDetected =
                                true;

                            break;
                        }
                    }


                    if (!physicallyDetected)
                    {
                        qWarning(
                            "Camera %zu trasera RGB NO ENCONTRADA - Serial %u",
                            camera + 1,
                            serial
                        );

                        continue;
                    }


                    /*
                    * Abrir la ZED trasera solamente como fuente RGB.
                    */
                    auto rgbSource =
                        std::make_unique<
                            ZedGmslRgbOnlyFrameSource
                        >(
                            camera,
                            serial,
                            configCameraFpsCombo
                                ->currentData()
                                .toInt(),
                            configCameraResolutionCombo
                                ->currentData()
                                .toString()
                                .toStdString(),
                            configCameraTimeoutSpin
                                ->value(),
                            configCameraAutoReconnectCheck
                                ->isChecked(),
                            configCameraReconnectIntervalSpin
                                ->value()
                        );


                    if (!rgbCameraWorker.setFrameSource(
                            camera,
                            std::move(rgbSource)
                        ))
                    {
                        qWarning(
                            "No se pudo instalar RGB ZED trasera Camera %zu - Serial %u",
                            camera + 1,
                            serial
                        );

                        continue;
                    }


                    qInfo(
                        "Camera %zu trasera RGB lista - Serial %u",
                        camera + 1,
                        serial
                    );
                }        
                /*
                * Arrancar si existe al menos
                * una cámara utilizable.
                */
                if (usableCameraCount > 0)
                {
                    if (!vision3DWorker->start())
                    {
                        qWarning(
                            "No se pudo iniciar Vision3DWorker "
                            "con cámaras ZED GMSL2"
                        );
                    }
                    else
                    {
                        qInfo(
                            "Vision3DWorker iniciado con %zu cámara(s)",
                            usableCameraCount
                        );


                        /*
                        * ========================================================
                        * INICIAR DETECTOR YOLO TENSORRT
                        * ========================================================
                        */
                                                if (tensorRtConfigValid)
                        {
                            const int formatValue =
                                configYoloFormatCombo
                                    ->currentData()
                                    .toInt();


                            const auto outputFormat =
                                static_cast<
                                    TensorRtTasselDetector::
                                        ModelOutputFormat
                                >(
                                    formatValue
                                );


                            const QByteArray enginePathUtf8 =
                                enginePath.toUtf8();


                            if (!yoloInferenceWorker.initialize(
                                enginePathUtf8.constData(),
                                outputFormat,
                                static_cast<float>(
                                    configYoloConfidenceSpin->value()
                                ),
                                static_cast<float>(
                                    configYoloNmsSpin->value()
                                )
                            ))
                            {
                                qWarning(
                                    "TensorRT: no se pudo inicializar "
                                    "el detector de panojas"
                                );

                                addLogMessage(
                                    "TensorRT: no se pudo inicializar "
                                    "el detector de panojas"
                                );
                            }
                            else if (!yoloInferenceWorker.start())
                            {
                                qWarning(
                                    "TensorRT: no se pudo iniciar "
                                    "YoloInferenceWorker"
                                );

                                addLogMessage(
                                    "TensorRT: no se pudo iniciar "
                                    "YoloInferenceWorker"
                                );
                            }
                            else
                            {
                                qInfo(
                                    "TensorRT: detector de panojas iniciado"
                                );

                                addLogMessage(
                                    "TensorRT: detector de panojas iniciado"
                                );
                            }
                        }
                    }
                }
                else
                
                {
                    qWarning(
                        "No hay cámaras ZED habilitadas y disponibles"
                    );

                    addLogMessage(
                        "Visión 3D: no hay cámaras ZED habilitadas y disponibles"
                    );
                }


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

    generalPageLayout->addWidget(
        visionSourceFrame
    );

        /*
     * ========================================================
     * DETECTOR DE PANOJAS - TENSORRT
     * ========================================================
     */
    QFrame *yoloConfigFrame =
        new QFrame();

    yoloConfigFrame->setFrameShape(
        QFrame::StyledPanel
    );


    QVBoxLayout *yoloConfigLayout =
        new QVBoxLayout(
            yoloConfigFrame
        );


    QLabel *yoloConfigTitle =
        new QLabel(
            "DETECTOR DE PANOJAS"
        );

    yoloConfigTitle->setAlignment(
        Qt::AlignCenter
    );

    yoloConfigTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    yoloConfigLayout->addWidget(
        yoloConfigTitle
    );


    /*
     * Formato de salida YOLO.
     */
    QHBoxLayout *yoloFormatLayout =
        new QHBoxLayout();

    QLabel *yoloFormatLabel =
        new QLabel(
            "Formato del modelo:"
        );

    configYoloFormatCombo =
        new QComboBox();

    configYoloFormatCombo->addItem(
        "AUTO",
        static_cast<int>(
            TensorRtTasselDetector::
                ModelOutputFormat::Auto
        )
    );

    configYoloFormatCombo->addItem(
        "YOLO MODERNO RAW",
        static_cast<int>(
            TensorRtTasselDetector::
                ModelOutputFormat::ModernRaw
        )
    );

    configYoloFormatCombo->addItem(
        "YOLOv5 RAW",
        static_cast<int>(
            TensorRtTasselDetector::
                ModelOutputFormat::YoloV5Raw
        )
    );

    configYoloFormatCombo->addItem(
        "END-TO-END NMS",
        static_cast<int>(
            TensorRtTasselDetector::
                ModelOutputFormat::EndToEndNms
        )
    );


    yoloFormatLayout->addWidget(
        yoloFormatLabel
    );

    yoloFormatLayout->addWidget(
        configYoloFormatCombo
    );

    yoloFormatLayout->addStretch();

    yoloConfigLayout->addLayout(
        yoloFormatLayout
    );


    /*
     * Ruta del engine TensorRT.
     */
    QHBoxLayout *yoloEngineLayout =
        new QHBoxLayout();

    QLabel *yoloEngineLabel =
        new QLabel(
            "Engine TensorRT:"
        );

    configYoloEngineEdit =
        new QLineEdit();

    configYoloEngineEdit->setPlaceholderText(
        "/ruta/al/modelo.engine"
    );


    yoloEngineLayout->addWidget(
        yoloEngineLabel
    );

    yoloEngineLayout->addWidget(
        configYoloEngineEdit,
        1
    );

    yoloConfigLayout->addLayout(
        yoloEngineLayout
    );

        /*
     * Confianza mínima.
     */
    QHBoxLayout *yoloConfidenceLayout =
        new QHBoxLayout();

    QLabel *yoloConfidenceLabel =
        new QLabel(
            "Confianza mínima:"
        );

    configYoloConfidenceSpin =
        new QDoubleSpinBox();

    configYoloConfidenceSpin->setRange(
        0.0,
        1.0
    );

    configYoloConfidenceSpin->setDecimals(
        2
    );

    configYoloConfidenceSpin->setSingleStep(
        0.05
    );

    configYoloConfidenceSpin->setValue(
        0.25
    );


    yoloConfidenceLayout->addWidget(
        yoloConfidenceLabel
    );

    yoloConfidenceLayout->addWidget(
        configYoloConfidenceSpin
    );

    yoloConfidenceLayout->addStretch();

    yoloConfigLayout->addLayout(
        yoloConfidenceLayout
    );


    /*
     * Umbral IoU para NMS.
     */
    QHBoxLayout *yoloNmsLayout =
        new QHBoxLayout();

    QLabel *yoloNmsLabel =
        new QLabel(
            "NMS IoU:"
        );

    configYoloNmsSpin =
        new QDoubleSpinBox();

    configYoloNmsSpin->setRange(
        0.0,
        1.0
    );

    configYoloNmsSpin->setDecimals(
        2
    );

    configYoloNmsSpin->setSingleStep(
        0.05
    );

    configYoloNmsSpin->setValue(
        0.45
    );


    yoloNmsLayout->addWidget(
        yoloNmsLabel
    );

    yoloNmsLayout->addWidget(
        configYoloNmsSpin
    );

    yoloNmsLayout->addStretch();

    yoloConfigLayout->addLayout(
        yoloNmsLayout
    );


    generalPageLayout->addWidget(
        yoloConfigFrame
    );

    generalPageLayout->addStretch();

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

    for (std::size_t camera = 0;
        camera < Vision3DProcessor::CAMERA_COUNT;
        ++camera)
    {
        configVisionCameraCombo->addItem(
            QString("CÁMARA %1")
                .arg(camera + 1),
            static_cast<int>(camera)
        );
    }

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
    * Número de serie ZED.
    */
    QHBoxLayout *cameraSerialLayout =
        new QHBoxLayout();

    QLabel *cameraSerialLabel =
        new QLabel(
            "Serial ZED:"
        );

    configVisionCameraSerial =
        new QSpinBox();

    configVisionCameraSerial->setRange(
        0,
        999999999
    );

    configVisionCameraSerial->setSpecialValueText(
        "NO CONFIGURADO"
    );

    cameraSerialLayout->addWidget(
        cameraSerialLabel
    );

    cameraSerialLayout->addWidget(
        configVisionCameraSerial
    );

    cameraSerialLayout->addStretch();

    visionCameraMainLayout->addLayout(
        cameraSerialLayout
    );

    /*
    * Estado de detección de la cámara seleccionada.
    */
    configVisionCameraStatusLabel =
        new QLabel(
            "Estado ZED: NO DETECTADA"
        );

    configHagieImuLabel =
        new QLabel(
            "IMU Hagie: NO DISPONIBLE"
        );

    configVisionCameraImuLabel =
        new QLabel(
            "IMU cámara: NO DISPONIBLE"
        );

    configVisionCameraMountingErrorLabel =
        new QLabel(
            "Error montaje: NO DISPONIBLE"
        );

    configHagieImuLabel->setStyleSheet(
        "font-weight: bold;"
    );

    configVisionCameraImuLabel->setStyleSheet(
        "font-weight: bold;"
    );

    configVisionCameraMountingErrorLabel->setStyleSheet(
        "font-weight: bold;"
    );

    configVisionCameraStatusLabel->setStyleSheet(
        "font-weight: bold;"
    );


    /*
    * Botón de detección automática.
    */
    configVisionDetectButton =
        new QPushButton(
            "DETECTAR CÁMARAS ZED"
        );

    visionCameraMainLayout->addWidget(
        configVisionCameraStatusLabel
    );

   
    visionCameraMainLayout->addWidget(
        configHagieImuLabel
    );

    visionCameraMainLayout->addWidget(
        configVisionCameraImuLabel
    );

    visionCameraMainLayout->addWidget(
        configVisionCameraMountingErrorLabel
    );

   
    visionCameraMainLayout->addWidget(
        configVisionDetectButton
    );


    /*
    * Seriales ZED detectados pero todavía
    * no asignados a ninguna cámara lógica.
    */
    QHBoxLayout *detectedSerialLayout =
        new QHBoxLayout();

    QLabel *detectedSerialLabel =
        new QLabel(
            "ZED nueva detectada:"
        );

    configVisionDetectedSerialCombo =
        new QComboBox();

    configVisionDetectedSerialCombo->setEnabled(
        false
    );

    detectedSerialLayout->addWidget(
        detectedSerialLabel
    );

    detectedSerialLayout->addWidget(
        configVisionDetectedSerialCombo
    );

    detectedSerialLayout->addStretch();

    visionCameraMainLayout->addLayout(
        detectedSerialLayout
    );


    /*
    * Asignar la ZED seleccionada a la
    * cámara lógica actualmente seleccionada.
    */
    configVisionAssignDetectedButton =
        new QPushButton(
            "ASIGNAR A CÁMARA SELECCIONADA"
        );

    configVisionAssignDetectedButton->setEnabled(
        false
    );

    visionCameraMainLayout->addWidget(
        configVisionAssignDetectedButton
    );

    /*
    * ========================================================
    * DETECCIÓN AUTOMÁTICA DE CÁMARAS ZED
    * ========================================================
    */
   
    connect(
        configVisionDetectButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            refreshZedCameraDetection();
        }
    );

        connect(
            configVisionAssignDetectedButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                if (configVisionDetectedSerialCombo == nullptr)
                {
                    return;
                }


                if (configVisionDetectedSerialCombo->count() == 0)
                {
                    return;
                }


                uint32_t newSerial =
                    static_cast<uint32_t>(
                        configVisionDetectedSerialCombo
                            ->currentData()
                            .toULongLong()
                    );


                if (newSerial == 0)
                {
                    return;
                }


                /*
                * Asignar a la cámara lógica seleccionada.
                */
                visionCameraSerialNumbers[
                    currentVisionCamera
                ] =
                    newSerial;


                /*
                * Reflejarlo inmediatamente
                * en el campo de la GUI.
                */
                configVisionCameraSerial->setValue(
                    static_cast<int>(
                        newSerial
                    )
                );


                /*
                * Guardar configuración.
                */
                saveConfiguration();


                /*
                * Mostrar estado actualizado.
                */
                configVisionCameraStatusLabel->setText(
                    QString(
                        "Estado ZED: ASIGNADA - Serial %1"
                    ).arg(
                        newSerial
                    )
                );


                /*
                * Quitar esta cámara de la lista
                * de nuevas disponibles.
                */
                int currentIndex =
                    configVisionDetectedSerialCombo
                        ->currentIndex();


                if (currentIndex >= 0)
                {
                    configVisionDetectedSerialCombo
                        ->removeItem(
                            currentIndex
                        );
                }


                bool hasRemaining =
                    configVisionDetectedSerialCombo
                        ->count() > 0;


                configVisionDetectedSerialCombo
                    ->setEnabled(
                        hasRemaining
                    );


                configVisionAssignDetectedButton
                    ->setEnabled(
                        hasRemaining
                    );


                qInfo(
                    "ZED serial %u asignada a Camera %zu",
                    newSerial,
                    currentVisionCamera + 1
                );
            }
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
    camerasPageLayout->addWidget(
        visionCameraFrame
    );

    camerasPageLayout->addStretch();

    QGridLayout *bodyGrid =
        new QGridLayout();

    QGridLayout *regionGrid =
        new QGridLayout();


    for (std::size_t body = 0;
        body < HagieState::BODY_COUNT;
        ++body)
    {
        /*
        * ====================================================
        * PANEL CUERPO / ENCODER
        * ====================================================
        */
        QFrame *frame =
            new QFrame();

        frame->setFrameShape(
            QFrame::StyledPanel
        );

        QVBoxLayout *bodyLayout =
            new QVBoxLayout(
                frame
            );


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
        * ====================================================
        * PANEL REGIÓN VISIÓN 3D
        * ====================================================
        */
        QFrame *regionFrame =
            new QFrame();

        regionFrame->setFrameShape(
            QFrame::StyledPanel
        );

        QVBoxLayout *regionLayout =
            new QVBoxLayout(
                regionFrame
            );


        QLabel *regionBodyTitle =
            new QLabel(
                QString("CUERPO %1")
                    .arg(body + 1)
            );

        regionBodyTitle->setAlignment(
            Qt::AlignCenter
        );

        regionBodyTitle->setStyleSheet(
            "font-size: 16px;"
            "font-weight: bold;"
        );

        regionLayout->addWidget(
            regionBodyTitle
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

                 /*
         * ====================================================
         * REGIÓN DE VISIÓN 3D
         * ====================================================
         */

        QLabel *visionRegionTitle =
            new QLabel(
                "Región visión 3D"
            );

        visionRegionTitle->setStyleSheet(
            "font-weight: bold;"
        );


        /*
         * X mínimo.
         */
        QLabel *visionMinXLabel =
            new QLabel(
                "X mínimo (m)"
            );

        configVisionRegionMinX[body] =
            new QDoubleSpinBox();

        configVisionRegionMinX[body]
            ->setDecimals(2);

        configVisionRegionMinX[body]
            ->setRange(
                -20.0,
                20.0
            );

        configVisionRegionMinX[body]
            ->setSingleStep(
                0.10
            );


        /*
         * X máximo.
         */
        QLabel *visionMaxXLabel =
            new QLabel(
                "X máximo (m)"
            );

        configVisionRegionMaxX[body] =
            new QDoubleSpinBox();

        configVisionRegionMaxX[body]
            ->setDecimals(2);

        configVisionRegionMaxX[body]
            ->setRange(
                -20.0,
                20.0
            );

        configVisionRegionMaxX[body]
            ->setSingleStep(
                0.10
            );


        /*
         * Y mínimo.
         */
        QLabel *visionMinYLabel =
            new QLabel(
                "Y mínimo (m)"
            );

        configVisionRegionMinY[body] =
            new QDoubleSpinBox();

        configVisionRegionMinY[body]
            ->setDecimals(2);

        configVisionRegionMinY[body]
            ->setRange(
                -50.0,
                50.0
            );

        configVisionRegionMinY[body]
            ->setSingleStep(
                0.10
            );


        /*
         * Y máximo.
         */
        QLabel *visionMaxYLabel =
            new QLabel(
                "Y máximo (m)"
            );

        configVisionRegionMaxY[body] =
            new QDoubleSpinBox();

        configVisionRegionMaxY[body]
            ->setDecimals(2);

        configVisionRegionMaxY[body]
            ->setRange(
                -50.0,
                50.0
            );

        configVisionRegionMaxY[body]
            ->setSingleStep(
                0.10
            );


        /*
         * Z mínimo.
         */
        QLabel *visionMinZLabel =
            new QLabel(
                "Z mínimo (m)"
            );

        configVisionRegionMinZ[body] =
            new QDoubleSpinBox();

        configVisionRegionMinZ[body]
            ->setDecimals(2);

        configVisionRegionMinZ[body]
            ->setRange(
                -5.0,
                20.0
            );

        configVisionRegionMinZ[body]
            ->setSingleStep(
                0.10
            );


        /*
         * Z máximo.
         */
        QLabel *visionMaxZLabel =
            new QLabel(
                "Z máximo (m)"
            );

        configVisionRegionMaxZ[body] =
            new QDoubleSpinBox();

        configVisionRegionMaxZ[body]
            ->setDecimals(2);

        configVisionRegionMaxZ[body]
            ->setRange(
                -5.0,
                20.0
            );

        configVisionRegionMaxZ[body]
            ->setSingleStep(
                0.10
            );


        /*
         * Cantidad mínima de puntos.
         */
        QLabel *visionMinPointsLabel =
            new QLabel(
                "Puntos mínimos"
            );

        configVisionRegionMinPoints[body] =
            new QSpinBox();

        configVisionRegionMinPoints[body]
            ->setRange(
                1,
                100000
            );


        /*
         * Valores por defecto.
         *
         * Coinciden con Vision3DProcessor.
         */
        const double defaultMinX =
            -3.0 +
            static_cast<double>(body);

        const double defaultMaxX =
            -2.0 +
            static_cast<double>(body);


        configVisionRegionMinX[body]
            ->setValue(
                defaultMinX
            );

        configVisionRegionMaxX[body]
            ->setValue(
                defaultMaxX
            );

        configVisionRegionMinY[body]
            ->setValue(
                -10.0
            );

        configVisionRegionMaxY[body]
            ->setValue(
                10.0
            );

        configVisionRegionMinZ[body]
            ->setValue(
                0.0
            );

        configVisionRegionMaxZ[body]
            ->setValue(
                5.0
            );

        configVisionRegionMinPoints[body]
            ->setValue(
                1
            );


        /*
         * Agregar controles al panel REGIONES 3D.
         */
        regionLayout->addWidget(
            visionRegionTitle
        );

        regionLayout->addWidget(
            visionMinXLabel
        );

        regionLayout->addWidget(
            configVisionRegionMinX[body]
        );

        regionLayout->addWidget(
            visionMaxXLabel
        );

        regionLayout->addWidget(
            configVisionRegionMaxX[body]
        );

        regionLayout->addWidget(
            visionMinYLabel
        );

        regionLayout->addWidget(
            configVisionRegionMinY[body]
        );

        regionLayout->addWidget(
            visionMaxYLabel
        );

        regionLayout->addWidget(
            configVisionRegionMaxY[body]
        );

        regionLayout->addWidget(
            visionMinZLabel
        );

        regionLayout->addWidget(
            configVisionRegionMinZ[body]
        );

        regionLayout->addWidget(
            visionMaxZLabel
        );

        regionLayout->addWidget(
            configVisionRegionMaxZ[body]
        );

        regionLayout->addWidget(
            visionMinPointsLabel
        );

        regionLayout->addWidget(
            configVisionRegionMinPoints[body]
        );


        /*
         * Agregar cada panel a su grid correspondiente.
         */
        bodyGrid->addWidget(
            frame,
            body / 3,
            body % 3
        );

        regionGrid->addWidget(
            regionFrame,
            body / 3,
            body % 3
        );
    }


    bodiesPageLayout->addLayout(
        bodyGrid
    );

    bodiesPageLayout->addStretch();


    regionsPageLayout->addLayout(
        regionGrid
    );

    regionsPageLayout->addStretch();


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


    controlPageLayout->addWidget(
        controlFrame
    );

        /*
     * ========================================================
     * Gestión hidráulica
     * ========================================================
     */

    QFrame *hydraulicFrame =
        new QFrame();

    hydraulicFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QGridLayout *hydraulicLayout =
        new QGridLayout(hydraulicFrame);


    QLabel *hydraulicTitle =
        new QLabel(
            "GESTIÓN HIDRÁULICA"
        );

    hydraulicTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    hydraulicLayout->addWidget(
        hydraulicTitle,
        0,
        0,
        1,
        4
    );


    QLabel *hydraulicModeLabel =
        new QLabel(
            "Modo"
        );

    configHydraulicModeCombo =
        new QComboBox();

    configHydraulicModeCombo->addItem(
        "NORMAL / DESACTIVADA",
        0
    );

    configHydraulicModeCombo->addItem(
        "BÁSICA",
        1
    );


    QLabel *hydraulicThresholdLabel =
        new QLabel(
            "Umbral demanda fuerte"
        );

    configHydraulicHighThresholdSpin =
        new QSpinBox();

    configHydraulicHighThresholdSpin->setRange(
        1,
        1000
    );

    configHydraulicHighThresholdSpin->setValue(
        700
    );


    QLabel *hydraulicMaxBodiesLabel =
        new QLabel(
            "Máx. cuerpos alta demanda"
        );

    configHydraulicMaxBodiesSpin =
        new QSpinBox();

    configHydraulicMaxBodiesSpin->setRange(
        1,
        static_cast<int>(
            HagieState::BODY_COUNT
        )
    );

    configHydraulicMaxBodiesSpin->setValue(
        2
    );


    QLabel *hydraulicSecondaryLabel =
        new QLabel(
            "Demanda secundaria (%)"
        );

    configHydraulicSecondaryPercentSpin =
        new QSpinBox();

    configHydraulicSecondaryPercentSpin->setRange(
        0,
        100
    );

    configHydraulicSecondaryPercentSpin->setSuffix(
        " %"
    );

    configHydraulicSecondaryPercentSpin->setValue(
        40
    );


    hydraulicLayout->addWidget(
        hydraulicModeLabel,
        1,
        0
    );

    hydraulicLayout->addWidget(
        configHydraulicModeCombo,
        1,
        1
    );

    hydraulicLayout->addWidget(
        hydraulicThresholdLabel,
        1,
        2
    );

    hydraulicLayout->addWidget(
        configHydraulicHighThresholdSpin,
        1,
        3
    );


    hydraulicLayout->addWidget(
        hydraulicMaxBodiesLabel,
        2,
        0
    );

    hydraulicLayout->addWidget(
        configHydraulicMaxBodiesSpin,
        2,
        1
    );

    hydraulicLayout->addWidget(
        hydraulicSecondaryLabel,
        2,
        2
    );

    hydraulicLayout->addWidget(
        configHydraulicSecondaryPercentSpin,
        2,
        3
    );


    controlPageLayout->addWidget(
        hydraulicFrame
    );

        /*
     * ========================================================
     * Sintonía control de altura
     * ========================================================
     */

    QFrame *heightTuningFrame =
        new QFrame();

    heightTuningFrame->setFrameShape(
        QFrame::StyledPanel
    );

    QGridLayout *heightTuningLayout =
        new QGridLayout(
            heightTuningFrame
        );


    QLabel *heightTuningTitle =
        new QLabel(
            "SINTONÍA CONTROL DE ALTURA"
        );

    heightTuningTitle->setStyleSheet(
        "font-size: 16px;"
        "font-weight: bold;"
    );

    heightTuningLayout->addWidget(
        heightTuningTitle,
        0,
        0,
        1,
        4
    );


    /*
     * Kp
     */
    QLabel *heightKpLabel =
        new QLabel(
            "Kp"
        );

    configHeightKpSpin =
        new QDoubleSpinBox();

    configHeightKpSpin->setRange(
        0.0,
        100.0
    );

    configHeightKpSpin->setDecimals(
        2
    );

    configHeightKpSpin->setSingleStep(
        0.10
    );

    configHeightKpSpin->setValue(
        5.00
    );


    /*
     * Ki
     */
    QLabel *heightKiLabel =
        new QLabel(
            "Ki"
        );

    configHeightKiSpin =
        new QDoubleSpinBox();

    configHeightKiSpin->setRange(
        0.0,
        100.0
    );

    configHeightKiSpin->setDecimals(
        2
    );

    configHeightKiSpin->setSingleStep(
        0.10
    );

    configHeightKiSpin->setValue(
        0.00
    );


    /*
     * Kd
     */
    QLabel *heightKdLabel =
        new QLabel(
            "Kd"
        );

    configHeightKdSpin =
        new QDoubleSpinBox();

    configHeightKdSpin->setRange(
        0.0,
        100.0
    );

    configHeightKdSpin->setDecimals(
        2
    );

    configHeightKdSpin->setSingleStep(
        0.10
    );

    configHeightKdSpin->setValue(
        0.00
    );


    /*
     * Banda muerta
     */
    QLabel *heightDeadbandLabel =
        new QLabel(
            "Banda muerta"
        );

    configHeightDeadbandSpin =
        new QDoubleSpinBox();

    configHeightDeadbandSpin->setRange(
        0.0,
        500.0
    );

    configHeightDeadbandSpin->setDecimals(
        2
    );

    configHeightDeadbandSpin->setSingleStep(
        1.0
    );

    configHeightDeadbandSpin->setSuffix(
        " mm"
    );

    configHeightDeadbandSpin->setValue(
        10.00
    );


    /*
     * Fila Kp / Ki
     */
    heightTuningLayout->addWidget(
        heightKpLabel,
        1,
        0
    );

    heightTuningLayout->addWidget(
        configHeightKpSpin,
        1,
        1
    );

    heightTuningLayout->addWidget(
        heightKiLabel,
        1,
        2
    );

    heightTuningLayout->addWidget(
        configHeightKiSpin,
        1,
        3
    );


    /*
     * Fila Kd / banda muerta
     */
    heightTuningLayout->addWidget(
        heightKdLabel,
        2,
        0
    );

    heightTuningLayout->addWidget(
        configHeightKdSpin,
        2,
        1
    );

    heightTuningLayout->addWidget(
        heightDeadbandLabel,
        2,
        2
    );

    heightTuningLayout->addWidget(
        configHeightDeadbandSpin,
        2,
        3
    );


    controlPageLayout->addWidget(
        heightTuningFrame
    );

    controlPageLayout->addStretch();


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

    /*
    * ============================================================
    * SIMULACIÓN DEL LAZO DE ALTURA
    * ============================================================
    *
    * Solamente funciona cuando la fuente de visión está
    * en SIMULATION.
    *
    * Simulamos:
    *
    * visión 3D -> objetivo -> respuesta hidráulica -> encoder
    *
    * En modo real este bloque no modifica nada.
    */

    if (visionHeightSource != nullptr &&
        heightTargetController != nullptr &&
        visionHeightSource->getSourceMode() ==
            VisionHeightSource::SourceMode::SIMULATION)
    {
        for (std::size_t body = 0;
            body < HagieState::BODY_COUNT;
            ++body)
        {
            HagieState::BodyState bodyState =
                state->getBodyState(body);


            /*
            * Calcular objetivo exactamente mediante
            * HeightTargetController.
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
                continue;
            }


            /*
            * En simulación no dependemos de que STM32Worker
            * esté conectado para publicar el objetivo.
            */
            state->setBodyTarget(
                body,
                targetResult.target_mm
            );


            /*
            * Primera ejecución:
            * arrancar el encoder algo separado del objetivo
            * para poder observar cómo converge.
            */
            if (!simulatedEncoderInitialized[body])
            {
                simulatedEncoderHeightMm[body] =
                    static_cast<double>(
                        targetResult.target_mm
                    ) - 80.0;

                if (simulatedEncoderHeightMm[body] < 0.0)
                {
                    simulatedEncoderHeightMm[body] =
                        0.0;
                }

                simulatedEncoderInitialized[body] =
                    true;
            }


            const double target =
                static_cast<double>(
                    targetResult.target_mm
                );


            /*
            * Diferentes velocidades de respuesta por cuerpo.
            *
            * Esto nos permite comprobar que los seis históricos
            * son realmente independientes.
            */
            const double alpha =
                0.055 +
                static_cast<double>(body) *
                    0.008;


            const double error =
                target -
                simulatedEncoderHeightMm[body];


            /*
            * Modelo simple de respuesta hidráulica.
            */
            simulatedEncoderHeightMm[body] +=
                alpha * error;


            /*
            * Limitar físicamente la simulación.
            */
            if (simulatedEncoderHeightMm[body] < 0.0)
            {
                simulatedEncoderHeightMm[body] =
                    0.0;
            }

            if (simulatedEncoderHeightMm[body] > 2000.0)
            {
                simulatedEncoderHeightMm[body] =
                    2000.0;
            }


            /*
            * Publicarlo como si fuera la lectura
            * proveniente del encoder físico.
            */
            state->setBodyHeight(
                body,
                static_cast<uint16_t>(
                    simulatedEncoderHeightMm[body] + 0.5
                )
            );
        }
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

        if (heightTrendWidget != nullptr)
        {
            heightTrendWidget->addSample(
                static_cast<int>(body),
                static_cast<double>(
                    bodyState.target_mm
                ),
                static_cast<double>(
                    bodyState.height_mm
                )
            );
        }

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

        /*
     * ========================================================
     * RENDIMIENTO DE DESPANOJADO
     * ========================================================
     */

    const TasselCounter::State counterState =
        tasselCounter.getState();

    const TasselVerifier::State verifierState =
        tasselVerifier.getState();


    if (tasselDetectedLabel != nullptr)
    {
        tasselDetectedLabel->setText(
            QString(
                "Detectadas: %1"
            ).arg(
                counterState.front_count
            )
        );
    }


    if (tasselRemovedLabel != nullptr)
    {
        tasselRemovedLabel->setText(
            QString(
                "Removidas: %1"
            ).arg(
                verifierState.verified_removed
            )
        );
    }


    if (tasselRemainingLabel != nullptr)
    {
        tasselRemainingLabel->setText(
            QString(
                "Presentes: %1"
            ).arg(
                verifierState.verified_remaining
            )
        );
    }


    if (tasselEfficiencyLabel != nullptr)
    {
        const std::uint64_t verifiedTotal =
            verifierState.verified_removed +
            verifierState.verified_remaining;

        if (verifiedTotal > 0)
        {
            const double efficiency =
                100.0 *
                static_cast<double>(
                    verifierState.verified_removed
                ) /
                static_cast<double>(
                    verifiedTotal
                );

            tasselEfficiencyLabel->setText(
                QString(
                    "Efectividad: %1 %"
                ).arg(
                    efficiency,
                    0,
                    'f',
                    1
                )
            );

                        if (efficiency >= 90.0)
            {
                tasselEfficiencyLabel->setStyleSheet(
                    "font-size: 16px;"
                    "font-weight: bold;"
                    "color: #2E7D32;"
                );
            }
            else if (efficiency >= 75.0)
            {
                tasselEfficiencyLabel->setStyleSheet(
                    "font-size: 16px;"
                    "font-weight: bold;"
                    "color: #F9A825;"
                );
            }
            else
            {
                tasselEfficiencyLabel->setStyleSheet(
                    "font-size: 16px;"
                    "font-weight: bold;"
                    "color: #C62828;"
                );
            }
        }
        else
        {
            tasselEfficiencyLabel->setText(
                "Efectividad: -- %"
            );

            tasselEfficiencyLabel->setStyleSheet(
                "font-size: 16px;"
                "font-weight: bold;"
                "color: #616161;"
            );
        }
    }
    updateSystemStatus();
    updateFaultPage();
    updateTestPage();


    /*
    * ========================================================
    * DIAGNÓSTICO DE COMUNICACIONES EN TIEMPO REAL
    * ========================================================
    */
    HagieState::SystemState system =
        state->getSystemState();

    /*
    * ========================================================
    * LOG - CAMBIO DE ESTADO STM32
    * ========================================================
    */
    if (!logStm32StateInitialized)
    {
        logPreviousStm32Connected =
            system.stm32_connected;

        logStm32StateInitialized =
            true;
    }
    else if (system.stm32_connected !=
            logPreviousStm32Connected)
    {
        addLogMessage(
            system.stm32_connected
                ? "STM32 conectada"
                : "STM32 desconectada"
        );

        logPreviousStm32Connected =
            system.stm32_connected;
    }
    /*
    * ========================================================
    * LOG - CAMBIO DE ESTADO CAN / AXIOMATIC
    * ========================================================
    */
    if (!logCanStateInitialized)
    {
        logPreviousCanOk =
            system.can_ok;

        logCanStateInitialized =
            true;

        addLogMessage(
            system.can_ok
                ? "CAN / Axiomatic OK"
                : "CAN / Axiomatic FALLA"
        );
    }
    else if (system.can_ok !=
            logPreviousCanOk)
    {
        addLogMessage(
            system.can_ok
                ? "CAN / Axiomatic OK"
                : "CAN / Axiomatic FALLA"
        );

        logPreviousCanOk =
            system.can_ok;
    }


    /*
    * ========================================================
    * LOG - CAMBIO DE ESTADO VISIÓN 3D
    * ========================================================
    */
    if (!logVisionStateInitialized)
    {
        logPreviousVisionRunning =
            system.vision_running;

        logVisionStateInitialized =
            true;

        addLogMessage(
            system.vision_running
                ? "Visión 3D activa"
                : "Visión 3D detenida"
        );
    }
    else if (system.vision_running !=
            logPreviousVisionRunning)
    {
        addLogMessage(
            system.vision_running
                ? "Visión 3D activa"
                : "Visión 3D detenida"
        );

        logPreviousVisionRunning =
            system.vision_running;
    }

        /*
     * ========================================================
     * LOG - CAMBIO DE ESTADO IA
     * ========================================================
     */
    if (!logAiStateInitialized)
    {
        logPreviousAiRunning =
            system.ai_running;

        logAiStateInitialized =
            true;
    }
    else if (system.ai_running !=
            logPreviousAiRunning)
    {
        addLogMessage(
            system.ai_running
                ? "IA / detector de panojas activo"
                : "IA / detector de panojas detenido"
        );

        logPreviousAiRunning =
            system.ai_running;
    }

     /*
     * ========================================================
     * LOG - RESUMEN PERIÓDICO DE IA / DESPANOJADO
     * ========================================================
     */
    const QDateTime nowLogTime =
        QDateTime::currentDateTime();

    if (!logLastAiSummaryTime.isValid() ||
        logLastAiSummaryTime.msecsTo(nowLogTime) >= 5000)
    {
        const TasselCounter::State counterState =
            tasselCounter.getState();

        const TasselVerifier::State verifierState =
            tasselVerifier.getState();

        const std::uint64_t verifiedTotal =
            verifierState.verified_removed +
            verifierState.verified_remaining;

        QString efficiencyText =
            "--";

        if (verifiedTotal > 0)
        {
            const double efficiency =
                100.0 *
                static_cast<double>(
                    verifierState.verified_removed
                ) /
                static_cast<double>(
                    verifiedTotal
                );

            efficiencyText =
                QString::number(
                    efficiency,
                    'f',
                    1
                );
        }

        const QString aiSummary =
            QString(
                "IA: detectadas=%1 | removidas=%2 | "
                "presentes=%3 | pendientes=%4 | "
                "efectividad=%5 %"
            )
                .arg(counterState.front_count)
                .arg(verifierState.verified_removed)
                .arg(verifierState.verified_remaining)
                .arg(verifierState.pending)
                .arg(efficiencyText);

        if (aiSummary != logPreviousAiSummary)
        {
            addLogMessage(
                aiSummary
            );

            logPreviousAiSummary =
                aiSummary;
        }

        logLastAiSummaryTime =
            nowLogTime;
    }
    /*
     * ========================================================
     * LOG - RESUMEN PERIÓDICO DE VISIÓN 3D
     * ========================================================
     */
    if (!logLastVisionSummaryTime.isValid() ||
        logLastVisionSummaryTime.msecsTo(nowLogTime) >= 5000)
    {
        if (visionHeightSource != nullptr)
        {
            const VisionHeightSource::VisionResult visionResult =
                visionHeightSource->getResult();

            QStringList visionBodyTexts;

            for (std::size_t body = 0;
                 body < VisionHeightSource::BODY_COUNT;
                 ++body)
            {
                const VisionHeightSource::BodyVisionResult& bodyResult =
                    visionResult.bodies[body];

                if (bodyResult.valid)
                {
                    visionBodyTexts.append(
                        QString(
                            "C%1=%2 mm"
                        )
                            .arg(body + 1)
                            .arg(bodyResult.height_mm)
                    );
                }
                else
                {
                    visionBodyTexts.append(
                        QString(
                            "C%1=INVÁLIDA"
                        ).arg(body + 1)
                    );
                }
            }

            const QString visionSummary =
                QString(
                    "VISIÓN 3D: %1"
                ).arg(
                    visionBodyTexts.join(
                        " | "
                    )
                );

            if (visionSummary !=
                logPreviousVisionSummary)
            {
                addLogMessage(
                    visionSummary
                );

                logPreviousVisionSummary =
                    visionSummary;
            }
        }

        logLastVisionSummaryTime =
            nowLogTime;
    }

    if (communicationsStm32StatusLabel != nullptr)
    {
        communicationsStm32StatusLabel->setText(
            system.stm32_connected
                ? "Estado: CONECTADA"
                : "Estado: DESCONECTADA"
        );
    }


    if (communicationsStm32UptimeLabel != nullptr)
    {
        const uint32_t uptimeSeconds =
            system.stm32_uptime_ticks / 1000U;

        const uint32_t hours =
            uptimeSeconds / 3600U;

        const uint32_t minutes =
            (uptimeSeconds % 3600U) / 60U;

        const uint32_t seconds =
            uptimeSeconds % 60U;


        communicationsStm32UptimeLabel->setText(
            QString("Uptime: %1 h %2 min %3 s")
                .arg(hours)
                .arg(minutes)
                .arg(seconds)
        );
    }


    if (communicationsUartErrorsLabel != nullptr)
    {
        communicationsUartErrorsLabel->setText(
            QString("Errores UART: %1")
                .arg(system.uart_error_count)
        );
    }


    if (communicationsTxDroppedLabel != nullptr)
    {
        communicationsTxDroppedLabel->setText(
            QString("TX descartados: %1")
                .arg(system.jetson_tx_queue_dropped)
        );
    }


    if (communicationsCanStatusLabel != nullptr)
    {
        communicationsCanStatusLabel->setText(
            system.can_ok
                ? "Estado CAN: OK"
                : "Estado CAN: FALLA"
        );
    }


    if (communicationsAxiomaticModulesLabel != nullptr)
    {
        communicationsAxiomaticModulesLabel->setText(
            QString("Módulos detectados: %1")
                .arg(system.axiomatic_modules)
        );
    }


    if (communicationsCanDroppedLabel != nullptr)
    {
        communicationsCanDroppedLabel->setText(
            QString("RX descartados: %1")
                .arg(system.axiomatic_rx_dropped)
        );
    }


    if (communicationsImuStatusLabel != nullptr)
    {
        communicationsImuStatusLabel->setText(
            system.imu_valid
                ? "Estado: OK"
                : "Estado: NO DISPONIBLE"
        );
    }


    if (communicationsVisionStatusLabel != nullptr)
    {
        communicationsVisionStatusLabel->setText(
            system.vision_running
                ? "Estado: ACTIVA"
                : "Estado: DETENIDA"
        );
    }

    /*
    * ========================================================
    * IMU GENERAL HAGIE EN TIEMPO REAL
    * ========================================================
    */
    if (configHagieImuLabel != nullptr)
{
    PointCloudSource::CameraOrientation
        machineOrientation;


    if (vision3DWorker != nullptr)
    {
        machineOrientation =
            vision3DWorker->getMachineOrientation();
    }


    if (machineOrientation.valid)
    {
        configHagieImuLabel->setText(
            QString(
                "IMU Hagie: Roll %1° | Pitch %2°"
            )
                .arg(
                    machineOrientation.roll_deg,
                    0,
                    'f',
                    2
                )
                .arg(
                    machineOrientation.pitch_deg,
                    0,
                    'f',
                    2
                )
        );
    }
    else
    {
        configHagieImuLabel->setText(
            "IMU Hagie: NO DISPONIBLE"
        );
    }
}

    /*
     * ========================================================
     * DIAGNÓSTICO EN TIEMPO REAL DE IMU DE CÁMARA
     * ========================================================
     */
    if (vision3DWorker != nullptr &&
        currentVisionCamera <
            Vision3DProcessor::CAMERA_COUNT)
    {
        PointCloudSource::CameraOrientation
            cameraOrientation =
                vision3DWorker->getCameraOrientation(
                    currentVisionCamera
                );


        if (configVisionCameraImuLabel != nullptr)
        {
            if (cameraOrientation.valid)
            {
                configVisionCameraImuLabel->setText(
                    QString(
                        "IMU cámara: Roll %1° | Pitch %2°"
                    )
                        .arg(
                            cameraOrientation.roll_deg,
                            0,
                            'f',
                            2
                        )
                        .arg(
                            cameraOrientation.pitch_deg,
                            0,
                            'f',
                            2
                        )
                );
            }
            else
            {
                configVisionCameraImuLabel->setText(
                    "IMU cámara: NO DISPONIBLE"
                );
            }
        }


        if (configVisionCameraMountingErrorLabel != nullptr)
        {
            float rollOffsetDeg = 0.0f;
            float pitchOffsetDeg = 0.0f;


            if (vision3DWorker->getCameraMountingOffset(
                    currentVisionCamera,
                    rollOffsetDeg,
                    pitchOffsetDeg
                ))
            {
                configVisionCameraMountingErrorLabel->setText(
                    QString(
                        "Error montaje: Roll %1° | Pitch %2°"
                    )
                        .arg(
                            rollOffsetDeg,
                            0,
                            'f',
                            2
                        )
                        .arg(
                            pitchOffsetDeg,
                            0,
                            'f',
                            2
                        )
                );
            }
            else
            {
                configVisionCameraMountingErrorLabel->setText(
                    "Error montaje: NO DISPONIBLE"
                );
            }
        }
    }
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


            /*
            * El botón solamente se habilita
            * si existe realmente NO_MOVEMENT.
            *
            * Bit 0x04 = BODY_FAULT_NO_MOVEMENT.
            */
            if (clearNoMovementButtons[body] != nullptr)
            {
                const bool hasNoMovement =
                    (bodyState.faults & 0x04U) != 0;

                clearNoMovementButtons[body]
                    ->setEnabled(
                        hasNoMovement
                    );
            }

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

    visionCameraSerialNumbers[camera] =
        static_cast<uint32_t>(
            configVisionCameraSerial->value()
        );

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

    configVisionCameraSerial->setValue(
        static_cast<int>(
            visionCameraSerialNumbers[camera]
        )
    );

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

void MainWindow::applyVisionBodyRegions()
{
    if (vision3DProcessor == nullptr)
    {
        return;
    }

    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        Vision3DProcessor::BodyRegion region;

        region.min_x =
            static_cast<float>(
                configVisionRegionMinX[body]->value()
            );

        region.max_x =
            static_cast<float>(
                configVisionRegionMaxX[body]->value()
            );

        region.min_y =
            static_cast<float>(
                configVisionRegionMinY[body]->value()
            );

        region.max_y =
            static_cast<float>(
                configVisionRegionMaxY[body]->value()
            );

        region.min_z =
            static_cast<float>(
                configVisionRegionMinZ[body]->value()
            );

        region.max_z =
            static_cast<float>(
                configVisionRegionMaxZ[body]->value()
            );

        region.min_points =
            static_cast<std::size_t>(
                configVisionRegionMinPoints[body]->value()
            );

        vision3DProcessor->setBodyRegion(
            body,
            region
        );
    }
}
void MainWindow::updateTasselVerificationTiming()
{
    if (configTasselSpeedSpin == nullptr ||
        configTasselCameraDistanceSpin == nullptr ||
        configTasselTimingToleranceSpin == nullptr ||
        configTasselExpectedTimeLabel == nullptr ||
        configTasselVerificationWindowLabel == nullptr)
    {
        return;
    }

    const double speedKmh =
        configTasselSpeedSpin->value();

    const double distanceMm =
        static_cast<double>(
            configTasselCameraDistanceSpin->value()
        );

    const double tolerancePercent =
        configTasselTimingToleranceSpin->value();

    if (speedKmh <= 0.0)
    {
        configTasselExpectedTimeLabel->setText(
            "Tiempo estimado hasta cámara trasera: --- ms"
        );

        configTasselVerificationWindowLabel->setText(
            "Ventana de verificación: --- ms"
        );

        return;
    }

    /*
     * km/h -> mm/ms
     *
     * 1 km/h = 1000000 mm / 3600000 ms
     *        = 1 / 3.6 mm/ms
     */
    const double speedMmPerMs =
        speedKmh / 3.6;

    const double expectedTimeMs =
        distanceMm /
        speedMmPerMs;

    const double toleranceFactor =
        tolerancePercent /
        100.0;

    const double minDelayMs =
        expectedTimeMs *
        (1.0 - toleranceFactor);

    const double maxDelayMs =
        expectedTimeMs *
        (1.0 + toleranceFactor);

    tasselVerifier.setVerificationWindow(
        static_cast<std::uint64_t>(
            qRound(minDelayMs)
        ),
        static_cast<std::uint64_t>(
            qRound(maxDelayMs)
        )
    );    

    configTasselExpectedTimeLabel->setText(
        QString(
            "Tiempo estimado hasta cámara trasera: %1 ms"
        ).arg(
            qRound(expectedTimeMs)
        )
    );

    configTasselVerificationWindowLabel->setText(
        QString(
            "Ventana de verificación: %1 - %2 ms"
        )
        .arg(
            qRound(minDelayMs)
        )
        .arg(
            qRound(maxDelayMs)
        )
    );
}

QString MainWindow::configurationFilePath() const
{
    QString configDir =
        QStandardPaths::writableLocation(
            QStandardPaths::AppConfigLocation
        );

    QDir dir;

    if (!dir.mkpath(configDir))
    {
        qWarning()
            << "No se pudo crear el directorio de configuración:"
            << configDir;
    }

    return QDir(configDir).filePath(
        "hagie_config.ini"
    );
}


bool MainWindow::systemReadyForAuto() const
{
    if (state == nullptr)
    {
        return false;
    }

    const HagieState::SystemState system =
        state->getSystemState();

    /*
     * Condiciones generales necesarias para
     * permitir funcionamiento automático.
     */
    if (!system.stm32_connected)
    {
        return false;
    }

    if (!system.can_ok)
    {
        return false;
    }

    return true;
}


void MainWindow::saveConfiguration()
{
    QSettings settings(
        configurationFilePath(),
        QSettings::IniFormat
    );

        /*
     * ========================================================
     * Guardar el valor actualmente visible de cámara trasera
     * ========================================================
     *
     * El selector guarda el serial al cambiar de Cámara 6/7,
     * pero el usuario también puede editar el serial y pulsar
     * GUARDAR sin cambiar de cámara.
     */
    if (configRearRgbCameraSerial != nullptr &&
        currentRearRgbCamera <
            rearRgbCameraSerialNumbers.size())
    {
        rearRgbCameraSerialNumbers[
            currentRearRgbCamera
        ] =
            static_cast<uint32_t>(
                configRearRgbCameraSerial->value()
            );
    }

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

                settings.setValue(
            "vision_region_min_x",
            configVisionRegionMinX[body]->value()
        );

        settings.setValue(
            "vision_region_max_x",
            configVisionRegionMaxX[body]->value()
        );

        settings.setValue(
            "vision_region_min_y",
            configVisionRegionMinY[body]->value()
        );

        settings.setValue(
            "vision_region_max_y",
            configVisionRegionMaxY[body]->value()
        );

        settings.setValue(
            "vision_region_min_z",
            configVisionRegionMinZ[body]->value()
        );

        settings.setValue(
            "vision_region_max_z",
            configVisionRegionMaxZ[body]->value()
        );

        settings.setValue(
            "vision_region_min_points",
            configVisionRegionMinPoints[body]->value()
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
     * Gestión hidráulica
     * ========================================================
     */
    settings.beginGroup(
        "Hydraulic"
    );



    settings.setValue(
        "management_mode",
        configHydraulicModeCombo
            ->currentData()
            .toInt()
    );

    settings.setValue(
        "high_command_threshold",
        configHydraulicHighThresholdSpin
            ->value()
    );

    settings.setValue(
        "max_high_demand_bodies",
        configHydraulicMaxBodiesSpin
            ->value()
    );

    settings.setValue(
        "secondary_percent",
        configHydraulicSecondaryPercentSpin
            ->value()
    );

    settings.endGroup();


    settings.beginGroup("HeightControl");

    settings.setValue(
        "kp",
        configHeightKpSpin->value()
    );

    settings.setValue(
        "ki",
        configHeightKiSpin->value()
    );

    settings.setValue(
        "kd",
        configHeightKdSpin->value()
    );

    settings.setValue(
        "deadband_mm",
        configHeightDeadbandSpin->value()
    );

    settings.endGroup();

    

        /*
     * ========================================================
     * Parámetros generales de cámaras y procesamiento
     * ========================================================
     */
    settings.beginGroup(
        "CameraProcessing"
    );

    settings.setValue(
        "camera_fps",
        configCameraFpsCombo
            ->currentData()
            .toInt()
    );

    settings.setValue(
        "rgb_resolution",
        configCameraResolutionCombo
            ->currentData()
            .toString()
    );

    settings.setValue(
        "camera_timeout_ms",
        configCameraTimeoutSpin
            ->value()
    );

    settings.setValue(
        "camera_auto_reconnect",
        configCameraAutoReconnectCheck
            ->isChecked()
    );

    settings.setValue(
        "camera_reconnect_interval_ms",
        configCameraReconnectIntervalSpin
            ->value()
    );

    settings.setValue(
        "ai_frame_interval",
        configAiFrameIntervalSpin
            ->value()
    );

    settings.setValue(
        "vision_data_timeout_ms",
        configVisionDataTimeoutSpin
            ->value()
    );

    settings.endGroup();


    /*
     * ========================================================
     * Verificación de panojas
     * ========================================================
     */
    settings.beginGroup(
        "TasselVerification"
    );

    settings.setValue(
        "estimated_speed_kmh",
        configTasselSpeedSpin
            ->value()
    );

    settings.setValue(
        "camera_distance_mm",
        configTasselCameraDistanceSpin
            ->value()
    );

    settings.setValue(
        "timing_tolerance_percent",
        configTasselTimingToleranceSpin
            ->value()
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

        settings.setValue(
        "yolo_output_format",
        configYoloFormatCombo
            ->currentData()
            .toInt()
    );

    settings.setValue(
        "yolo_engine_path",
        configYoloEngineEdit
            ->text()
    );

        settings.setValue(
        "yolo_confidence_threshold",
        configYoloConfidenceSpin
            ->value()
    );

    settings.setValue(
        "yolo_nms_threshold",
        configYoloNmsSpin
            ->value()
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
    * Guardar las configuraciones de cámaras 3D.
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

        settings.setValue(
            "serial_number",
            static_cast<qulonglong>(
                visionCameraSerialNumbers[camera]
            )
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
     * ========================================================
     * Cámaras traseras RGB
     * ========================================================
     */
    for (std::size_t camera = 0;
         camera < rearRgbCameraSerialNumbers.size();
         ++camera)
    {
        QString group =
            QString("RearRgbCamera%1")
                .arg(camera + 6);

        settings.beginGroup(group);

        settings.setValue(
            "serial_number",
            static_cast<qulonglong>(
                rearRgbCameraSerialNumbers[camera]
            )
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
    applyVisionBodyRegions();
    settings.sync();
}

void MainWindow::loadConfiguration()
{
    QSettings settings(
        configurationFilePath(),
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

                const double defaultMinX =
            -3.0 +
            static_cast<double>(body);

        const double defaultMaxX =
            -2.0 +
            static_cast<double>(body);


        configVisionRegionMinX[body]->setValue(
            settings.value(
                "vision_region_min_x",
                defaultMinX
            ).toDouble()
        );

        configVisionRegionMaxX[body]->setValue(
            settings.value(
                "vision_region_max_x",
                defaultMaxX
            ).toDouble()
        );

        configVisionRegionMinY[body]->setValue(
            settings.value(
                "vision_region_min_y",
                -10.0
            ).toDouble()
        );

        configVisionRegionMaxY[body]->setValue(
            settings.value(
                "vision_region_max_y",
                10.0
            ).toDouble()
        );

        configVisionRegionMinZ[body]->setValue(
            settings.value(
                "vision_region_min_z",
                0.0
            ).toDouble()
        );

        configVisionRegionMaxZ[body]->setValue(
            settings.value(
                "vision_region_max_z",
                5.0
            ).toDouble()
        );

        configVisionRegionMinPoints[body]->setValue(
            settings.value(
                "vision_region_min_points",
                1
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

    applyVisionBodyRegions();


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
     * Gestión hidráulica
     * ========================================================
     */
    settings.beginGroup(
        "Hydraulic"
    );

    int hydraulicMode =
        settings.value(
            "management_mode",
            0
        ).toInt();

    int hydraulicModeIndex =
        configHydraulicModeCombo
            ->findData(
                hydraulicMode
            );

    if (hydraulicModeIndex >= 0)
    {
        configHydraulicModeCombo
            ->setCurrentIndex(
                hydraulicModeIndex
            );
    }

    configHydraulicHighThresholdSpin->setValue(
        settings.value(
            "high_command_threshold",
            700
        ).toInt()
    );

    configHydraulicMaxBodiesSpin->setValue(
        settings.value(
            "max_high_demand_bodies",
            2
        ).toInt()
    );

    configHydraulicSecondaryPercentSpin->setValue(
        settings.value(
            "secondary_percent",
            40
        ).toInt()
    );

    settings.endGroup();


    settings.beginGroup("HeightControl");

    configHeightKpSpin->setValue(
        settings.value(
            "kp",
            5.0
        ).toDouble()
    );

    configHeightKiSpin->setValue(
        settings.value(
            "ki",
            0.0
        ).toDouble()
    );

    configHeightKdSpin->setValue(
        settings.value(
            "kd",
            0.0
        ).toDouble()
    );

    configHeightDeadbandSpin->setValue(
        settings.value(
            "deadband_mm",
            10.0
        ).toDouble()
    );

    settings.endGroup();

        /*
     * ========================================================
     * Parámetros generales de cámaras y procesamiento
     * ========================================================
     */
    settings.beginGroup(
        "CameraProcessing"
    );

    int cameraFps =
        settings.value(
            "camera_fps",
            30
        ).toInt();

    QString rgbResolution =
        settings.value(
            "rgb_resolution",
            "HD720"
        ).toString();

    int cameraTimeoutMs =
        settings.value(
            "camera_timeout_ms",
            1000
        ).toInt();

    bool cameraAutoReconnect =
        settings.value(
            "camera_auto_reconnect",
            true
        ).toBool();

    int cameraReconnectIntervalMs =
        settings.value(
            "camera_reconnect_interval_ms",
            2000
        ).toInt();

    int aiFrameInterval =
        settings.value(
            "ai_frame_interval",
            1
        ).toInt();

    int visionDataTimeoutMs =
        settings.value(
            "vision_data_timeout_ms",
            500
        ).toInt();

    settings.endGroup();




    int cameraFpsIndex =
        configCameraFpsCombo
            ->findData(
                cameraFps
            );

    if (cameraFpsIndex >= 0)
    {
        configCameraFpsCombo
            ->setCurrentIndex(
                cameraFpsIndex
            );
    }


    int cameraResolutionIndex =
        configCameraResolutionCombo
            ->findData(
                rgbResolution
            );

    if (cameraResolutionIndex >= 0)
    {
        configCameraResolutionCombo
            ->setCurrentIndex(
                cameraResolutionIndex
            );
    }


    configCameraTimeoutSpin->setValue(
        cameraTimeoutMs
    );

    configCameraAutoReconnectCheck->setChecked(
        cameraAutoReconnect
    );

    configCameraReconnectIntervalSpin->setValue(
        cameraReconnectIntervalMs
    );

    configAiFrameIntervalSpin->setValue(
        aiFrameInterval
    );

    yoloInferenceWorker.setFrameInterval(
        static_cast<std::size_t>(
            configAiFrameIntervalSpin->value()
        )
    );

    configVisionDataTimeoutSpin->setValue(
        visionDataTimeoutMs
    );

        /*
     * ========================================================
     * Verificación de panojas
     * ========================================================
     */
    settings.beginGroup(
        "TasselVerification"
    );

    configTasselSpeedSpin->setValue(
        settings.value(
            "estimated_speed_kmh",
            6.0
        ).toDouble()
    );

    configTasselCameraDistanceSpin->setValue(
        settings.value(
            "camera_distance_mm",
            3000
        ).toInt()
    );

    configTasselTimingToleranceSpin->setValue(
        settings.value(
            "timing_tolerance_percent",
            30.0
        ).toDouble()
    );

    settings.endGroup();

    updateTasselVerificationTiming();

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

        int yoloOutputFormat =
        settings.value(
            "yolo_output_format",
            static_cast<int>(
                TensorRtTasselDetector::
                    ModelOutputFormat::Auto
            )
        ).toInt();


    QString yoloEnginePath =
        settings.value(
            "yolo_engine_path",
            ""
        ).toString();  
        
        
    double yoloConfidenceThreshold =
        settings.value(
            "yolo_confidence_threshold",
            0.25
        ).toDouble();

    double yoloNmsThreshold =
        settings.value(
            "yolo_nms_threshold",
            0.45
        ).toDouble();    

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

        int yoloFormatIndex =
        configYoloFormatCombo
            ->findData(
                yoloOutputFormat
            );


    if (yoloFormatIndex >= 0)
    {
        configYoloFormatCombo
            ->setCurrentIndex(
                yoloFormatIndex
            );
    }


    configYoloEngineEdit->setText(
        yoloEnginePath
    );

    configYoloConfidenceSpin->setValue(
        yoloConfidenceThreshold
    );

    configYoloNmsSpin->setValue(
        yoloNmsThreshold
    );

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

        visionCameraSerialNumbers[camera] =
            static_cast<uint32_t>(
                settings.value(
                    "serial_number",
                    0
                ).toULongLong()
            );    


        /*
        * Cuerpos atendidos.
        */
        for (std::size_t body = 0;
            body < HagieState::BODY_COUNT;
            ++body)
        {
           bool defaultBodyEnabled =
            false;

        if (camera == 0)
        {
            defaultBodyEnabled =
                (body == 0 || body == 1);
        }
        else if (camera == 1)
        {
            defaultBodyEnabled =
                (body == 1 || body == 2);
        }
        else if (camera == 2)
        {
            defaultBodyEnabled =
                (body == 2 || body == 3);
        }
        else if (camera == 3)
        {
            defaultBodyEnabled =
                (body == 3 || body == 4);
        }
        else if (camera == 4)
        {
            defaultBodyEnabled =
                (body == 4 || body == 5);
        }

        config.body_enabled[body] =
            settings.value(
                QString("body_%1")
                    .arg(body + 1),
                defaultBodyEnabled
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
     * Cámaras traseras RGB
     * ========================================================
     */
    for (std::size_t camera = 0;
         camera < rearRgbCameraSerialNumbers.size();
         ++camera)
    {
        QString group =
            QString("RearRgbCamera%1")
                .arg(camera + 6);

        settings.beginGroup(group);

        rearRgbCameraSerialNumbers[camera] =
            static_cast<uint32_t>(
                settings.value(
                    "serial_number",
                    0
                ).toULongLong()
            );

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
        if (configRearRgbCameraCombo != nullptr &&
        configRearRgbCameraSerial != nullptr)
    {
        int currentRearCamera =
            configRearRgbCameraCombo
                ->currentIndex();

        if (currentRearCamera >= 0 &&
            static_cast<std::size_t>(
                currentRearCamera
            ) <
                rearRgbCameraSerialNumbers.size())
        {
            currentRearRgbCamera =
                static_cast<std::size_t>(
                    currentRearCamera
                );

            configRearRgbCameraSerial->setValue(
                static_cast<int>(
                    rearRgbCameraSerialNumbers[
                        currentRearRgbCamera
                    ]
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
     * K 0x10
     * Modo de gestión hidráulica.
     */
    stm32Worker->setHydraulicManagementMode(
        static_cast<uint8_t>(
            configHydraulicModeCombo
                ->currentData()
                .toInt()
        )
    );

    /*
     * K 0x11
     * Umbral de alta demanda.
     */
    stm32Worker->setHydraulicHighCommandThreshold(
        static_cast<uint16_t>(
            configHydraulicHighThresholdSpin
                ->value()
        )
    );

    /*
     * K 0x12
     * Máximo de cuerpos con alta demanda simultánea.
     */
    stm32Worker->setHydraulicMaxHighDemandBodies(
        static_cast<uint8_t>(
            configHydraulicMaxBodiesSpin
                ->value()
        )
    );

    /*
     * K 0x13
     * Porcentaje para demandas secundarias.
     */
    stm32Worker->setHydraulicSecondaryPercent(
        static_cast<uint8_t>(
            configHydraulicSecondaryPercentSpin
                ->value()
        )
    );

    /*
     * K 0x14
     * Ganancia proporcional Kp.
     */
    stm32Worker->setHeightControlKp(
        static_cast<float>(
            configHeightKpSpin->value()
        )
    );


    /*
     * K 0x15
     * Ganancia integral Ki.
     */
    stm32Worker->setHeightControlKi(
        static_cast<float>(
            configHeightKiSpin->value()
        )
    );


    /*
     * K 0x16
     * Ganancia derivativa Kd.
     */
    stm32Worker->setHeightControlKd(
        static_cast<float>(
            configHeightKdSpin->value()
        )
    );


    /*
     * K 0x17
     * Banda muerta del control de altura.
     */
    stm32Worker->setHeightControlDeadband(
        static_cast<float>(
            configHeightDeadbandSpin->value()
        )
    );
    /*
    * Ya cargamos toda la configuración en runtimeConfig.
    * Ahora comenzar el envío secuencial K -> ACK -> K.
    */
    stm32Worker->beginConfigurationSync();
}