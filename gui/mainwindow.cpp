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

    QVBoxLayout *layout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "PANEL DE TEST"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    layout->addWidget(title);


    QPushButton *valves =
        new QPushButton(
            "Test de válvulas"
        );

     

    QPushButton *encoders =
        new QPushButton(
            "Test de encoders"
        );

    QPushButton *stm32 =
        new QPushButton(
            "Test comunicación STM32"
        );

    QPushButton *imu =
        new QPushButton(
            "Test IMU"
        );

    QPushButton *cameras =
        new QPushButton(
            "Test cámaras"
        );


    layout->addWidget(valves);
    layout->addWidget(encoders);
    layout->addWidget(stm32);
    layout->addWidget(imu);
    layout->addWidget(cameras);
    

    layout->addStretch();


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

    QVBoxLayout *layout =
        new QVBoxLayout(page);


    QLabel *title =
        new QLabel(
            "CONFIGURACIÓN DEL SISTEMA"
        );

    title->setAlignment(
        Qt::AlignCenter
    );

    title->setStyleSheet(
        "font-size: 22px;"
        "font-weight: bold;"
    );

    layout->addWidget(title);


    QPushButton *machine =
        new QPushButton(
            "Configuración de máquina"
        );

    QPushButton *bodies =
        new QPushButton(
            "Configuración de cuerpos"
        );

    QPushButton *encoders =
        new QPushButton(
            "Configuración de encoders"
        );

    QPushButton *cameras =
        new QPushButton(
            "Configuración de cámaras"
        );

    QPushButton *ai =
        new QPushButton(
            "Configuración de IA"
        );

    QPushButton *stm32 =
        new QPushButton(
            "Configuración STM32"
        );

    QPushButton *can =
        new QPushButton(
            "CAN / Axiomatic"
        );


    layout->addWidget(machine);
    layout->addWidget(bodies);
    layout->addWidget(encoders);
    layout->addWidget(cameras);
    layout->addWidget(ai);
    layout->addWidget(stm32);
    layout->addWidget(can);

    layout->addStretch();


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