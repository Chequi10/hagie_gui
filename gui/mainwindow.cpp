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
#include <QStringList>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSettings>


MainWindow::MainWindow(
    HagieState *hagieState,
    STM32Worker *stm32Worker,
    QWidget *parent)
    : QMainWindow(parent),
      state(hagieState),
      stm32Worker(stm32Worker),
      centralStack(nullptr)
{
    setWindowTitle("Hagie Control");

    resize(
        1280,
        720
    );

    /*
     * Widget central con múltiples páginas.
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

    createMenus();
    createStatusBar();
    updateDashboard();
    updateFaultPage();


    /*
    * Actualización periódica de la interfaz.
    *
    * La GUI consulta HagieState cada 100 ms.
    * Los hilos UART, visión e IA nunca modifican
    * directamente los widgets Qt.
    */
    dashboardTimer =
        new QTimer(this);

    connect(
        dashboardTimer,
        &QTimer::timeout,
        this,
        &MainWindow::updateDashboard
    );

    dashboardTimer->start(100);
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

    mainCamera->setMinimumHeight(450);

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
    * Selector de intensidad del comando
    * para las pruebas manuales.
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
     * Grid de los 6 cuerpos.
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


        testHeightLabels[body] =
            new QLabel(
                "Altura: --- mm"
            );

        testValveLabels[body] =
            new QLabel(
                "Válvula: 0"
            );


            testFaultLabels[body] =
            new QLabel(
                "Estado: OK"
            );

            testEncoderStatusLabels[body] =
                new QLabel(
                    "Encoder: ---"
                );

            testEncoderDeltaLabels[body] =
                new QLabel(
                    "Cambio: 0 mm"
                );

            testEncoderStatusLabels[body]
                ->setAlignment(
                    Qt::AlignCenter
                );

            testEncoderDeltaLabels[body]
                ->setAlignment(
                    Qt::AlignCenter
                );

        testFaultLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        testFaultLabels[body]
            ->setStyleSheet(
                "font-weight: bold;"
            );


        testHeightLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );

        testValveLabels[body]
            ->setAlignment(
                Qt::AlignCenter
            );


        /*
         * Botones de movimiento.
         */
        QHBoxLayout *buttonLayout =
            new QHBoxLayout();


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

        /*
        * Alias locales para no tener que modificar
        * el resto de los connect().
        */
        QPushButton *downButton =
            testDownButtons[body];

        QPushButton *upButton =
            testUpButtons[body];


        /*
        * BAJAR mientras se mantiene presionado.
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
         * STOP
         */
        connect(
            stopButton,
            &QPushButton::clicked,
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
        * SUBIR mientras se mantiene presionado.
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


        buttonLayout->addWidget(
            downButton
        );

        buttonLayout->addWidget(
            stopButton
        );

        buttonLayout->addWidget(
            upButton
        );


        bodyLayout->addWidget(
            bodyTitle
        );

        bodyLayout->addWidget(
            testHeightLabels[body]
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
            buttonLayout
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
     * Botón global de seguridad.
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
            if (stm32Worker == nullptr)
            {
                return;
            }

            stm32Worker->stopAllValves();
        }
    );


    mainLayout->addWidget(
        stopAllButton
    );


    return page;
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

        bodyLayout->addWidget(
            configEncoderDirectionCombo[body]
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

            /*
            * ====================================================
            * K 0x01
            * Límites mínimo / máximo de cada cuerpo
            * ====================================================
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

                /*
                * Validación antes de enviar.
                */
                if (minHeight >= maxHeight)
                {
                    return;
                }

                stm32Worker->setBodyLimits(
                    static_cast<uint8_t>(body),
                    minHeight,
                    maxHeight
                );
            }


            /*
            * ====================================================
            * K 0x02
            * Umbral de comando para considerar movimiento
            * ====================================================
            */
            stm32Worker->setMoveCommandThreshold(
                static_cast<uint16_t>(
                    configMoveThresholdSpin->value()
                )
            );


            /*
            * ====================================================
            * K 0x03
            * Movimiento mínimo esperado
            * ====================================================
            */
            stm32Worker->setMinBodyMovement(
                static_cast<float>(
                    configMinMovementSpin->value()
                )
            );


            /*
            * ====================================================
            * K 0x04
            * Timeout NO_MOVEMENT
            * ====================================================
            */
            stm32Worker->setNoMovementTimeout(
                static_cast<uint32_t>(
                    configNoMovementTimeoutSpin->value()
                )
            );


            /*
            * ====================================================
            * K 0x05
            * Timeout de consigna AUTO
            * ====================================================
            */
            stm32Worker->setTargetTimeout(
                static_cast<uint32_t>(
                    configTargetTimeoutSpin->value()
                )
            );

            /*
            * ====================================================
            * K 0x06
            * Sentido de cada encoder
            * ====================================================
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
                        ? 0       // Normal
                        : 1;      // Invertido

                stm32Worker->setEncoderDirection(
                    static_cast<uint8_t>(body),
                    protocolDirection
                );
            }


            /*
            * ====================================================
            * K 0x07
            * Escala de cada encoder
            * ====================================================
            */
            for (std::size_t body = 0;
                body < HagieState::BODY_COUNT;
                ++body)
            {
                float scale =
                    static_cast<float>(
                        configEncoderScaleSpin[body]
                            ->value()
                    );

                stm32Worker->setEncoderScale(
                    static_cast<uint8_t>(body),
                    scale
                );
            }
        }
    );

    
    mainLayout->addWidget(
        saveButton
    );


    return page;
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
        + aiText
    );
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
void MainWindow::updateTestPage()
{
    if (state == nullptr)
    {
        return;
    }

    constexpr int32_t ENCODER_MOVEMENT_THRESHOLD_MM = 2;
    constexpr int64_t ENCODER_STILL_TIMEOUT_MS = 300;

    auto now =
        std::chrono::steady_clock::now();

    for (std::size_t body = 0;
         body < HagieState::BODY_COUNT;
         ++body)
    {
        HagieState::BodyState bodyState =
            state->getBodyState(body);


        /*
         * ========================================================
         * DIAGNÓSTICO DEL ENCODER
         * ========================================================
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

            testEncoderStatusLabels[body]->setText(
                "Encoder: QUIETO"
            );

            testEncoderStatusLabels[body]->setStyleSheet(
                "font-weight: bold;"
                "color: gray;"
            );

            testEncoderDeltaLabels[body]->setText(
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

            if (delta >= ENCODER_MOVEMENT_THRESHOLD_MM)
            {
                testEncoderDirection[body] = 1;

                testLastEncoderMovementTime[body] =
                    now;

                testEncoderStatusLabels[body]->setText(
                    "Encoder: SUBIENDO"
                );

                testEncoderStatusLabels[body]->setStyleSheet(
                    "font-weight: bold;"
                    "color: green;"
                );

                testEncoderDeltaLabels[body]->setText(
                    QString("Cambio: +%1 mm")
                        .arg(delta)
                );

                testPreviousHeight[body] =
                    bodyState.height_mm;
            }
            else if (delta <= -ENCODER_MOVEMENT_THRESHOLD_MM)
            {
                testEncoderDirection[body] = -1;

                testLastEncoderMovementTime[body] =
                    now;

                testEncoderStatusLabels[body]->setText(
                    "Encoder: BAJANDO"
                );

                testEncoderStatusLabels[body]->setStyleSheet(
                    "font-weight: bold;"
                    "color: green;"
                );

                testEncoderDeltaLabels[body]->setText(
                    QString("Cambio: %1 mm")
                        .arg(delta)
                );

                testPreviousHeight[body] =
                    bodyState.height_mm;
            }
            else
            {
                auto elapsed =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                            now -
                            testLastEncoderMovementTime[body]
                        ).count();

                if (elapsed >= ENCODER_STILL_TIMEOUT_MS)
                {
                    testEncoderDirection[body] = 0;

                    testEncoderStatusLabels[body]->setText(
                        "Encoder: QUIETO"
                    );

                    testEncoderStatusLabels[body]->setStyleSheet(
                        "font-weight: bold;"
                        "color: gray;"
                    );

                    testEncoderDeltaLabels[body]->setText(
                        "Cambio: 0 mm"
                    );
                }
            }
        }


        /*
         * ========================================================
         * TODO ESTO ES LA LÓGICA QUE YA FUNCIONABA
         * ========================================================
         */

        testHeightLabels[body]->setText(
            QString("Altura: %1 mm")
                .arg(bodyState.height_mm)
        );

        testValveLabels[body]->setText(
            QString("Válvula: %1")
                .arg(bodyState.valve_command)
        );

        if (bodyState.faults == 0)
        {
            testFaultLabels[body]->setText(
                "Estado: OK"
            );

            testFaultLabels[body]->setStyleSheet(
                "font-weight: bold;"
                "color: green;"
            );

            testBodyFrames[body]->setStyleSheet(
                ""
            );

            /*
             * Sin fallas:
             * permitir movimiento manual.
             */
            testUpButtons[body]->setEnabled(true);
            testDownButtons[body]->setEnabled(true);
        }
        else
        {
            QStringList faults;

            if ((bodyState.faults & 0x01U) != 0)
            {
                faults << "ENCODER_TIMEOUT";
            }

            if ((bodyState.faults & 0x02U) != 0)
            {
                faults << "ENCODER_RANGE";
            }

            if ((bodyState.faults & 0x04U) != 0)
            {
                faults << "NO_MOVEMENT";
            }

            if ((bodyState.faults & 0x08U) != 0)
            {
                faults << "MIN_LIMIT";
            }

            if ((bodyState.faults & 0x10U) != 0)
            {
                faults << "MAX_LIMIT";
            }

            if ((bodyState.faults & 0x20U) != 0)
            {
                faults << "TARGET_TIMEOUT";
            }

            if ((bodyState.faults & 0x40U) != 0)
            {
                faults << "VALVE_ERROR";
            }

            testFaultLabels[body]->setText(
                "FALLA: "
                + faults.join(" | ")
            );

            testFaultLabels[body]->setStyleSheet(
                "font-weight: bold;"
                "color: red;"
            );

            testBodyFrames[body]->setStyleSheet(
                "QFrame {"
                "border: 2px solid red;"
                "}"
            );

            testUpButtons[body]->setEnabled(false);
            testDownButtons[body]->setEnabled(false);
        }
    }
}