#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <array>
#include <chrono>


#include "core/hagie_state.h"


class QStackedWidget;
class QLabel;
class QTimer;
class STM32Worker;
class QFrame;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QComboBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(
        HagieState *hagieState,
        STM32Worker *stm32Worker,
        QWidget *parent = nullptr
    );

    ~MainWindow();

private:
    HagieState *state;

    STM32Worker *stm32Worker;

    QStackedWidget *centralStack;

    QTimer *dashboardTimer;

    QLabel *systemStatusLabel;

    QLabel *systemFaultsLabel = nullptr;

    int testValveCommand = 300;

    std::array<QFrame *, HagieState::BODY_COUNT>
    testBodyFrames {};

    std::array<QLabel *, HagieState::BODY_COUNT>
    testFaultLabels {};

    
    std::array<QLabel *, HagieState::BODY_COUNT>
        bodyFaultDetailLabels {};

    std::array<QLabel *, HagieState::BODY_COUNT>
    testHeightLabels {};

    std::array<QLabel *, HagieState::BODY_COUNT>
    testValveLabels {};

    std::array<QPushButton *, HagieState::BODY_COUNT>
    testUpButtons {};

    std::array<QPushButton *, HagieState::BODY_COUNT>
    testDownButtons {};

    /*
    * ========================================================
    * Diagnóstico de encoders en Panel de Test
    * ========================================================
    */

    std::array<QLabel *, HagieState::BODY_COUNT>
        testEncoderStatusLabels {};

    std::array<QLabel *, HagieState::BODY_COUNT>
        testEncoderDeltaLabels {};

    /*
    * Última altura observada de cada encoder.
    */
    std::array<uint16_t, HagieState::BODY_COUNT>
        testPreviousHeight {};

    /*
    * Indica si ya tenemos una primera muestra válida.
    */
    std::array<bool, HagieState::BODY_COUNT>
        testEncoderInitialized {};

        /*para que no quede siemrpe quieto*/

    std::array<std::chrono::steady_clock::time_point, HagieState::BODY_COUNT>
        testLastEncoderMovementTime {};

    std::array<int8_t, HagieState::BODY_COUNT>
        testEncoderDirection {};


     /*Para configuracion*/  

    std::array<QSpinBox *, HagieState::BODY_COUNT>
        configMinHeightSpin {};

    std::array<QSpinBox *, HagieState::BODY_COUNT>
        configMaxHeightSpin {};

    std::array<QDoubleSpinBox *, HagieState::BODY_COUNT>
        configEncoderScaleSpin {};

    std::array<QComboBox *, HagieState::BODY_COUNT>
        configEncoderDirectionCombo {};  
        
    QSpinBox *configMoveThresholdSpin = nullptr;

    QDoubleSpinBox *configMinMovementSpin = nullptr;

    QSpinBox *configNoMovementTimeoutSpin = nullptr;

    QSpinBox *configTargetTimeoutSpin = nullptr;

       

    void updateTestPage();

    void updateFaultPage();

    std::array<QLabel *, HagieState::BODY_COUNT> heightLabels {};
    std::array<QLabel *, HagieState::BODY_COUNT> targetLabels {};
    std::array<QLabel *, HagieState::BODY_COUNT> modeLabels {};
    std::array<QLabel *, HagieState::BODY_COUNT> valveLabels {};
    std::array<QLabel *, HagieState::BODY_COUNT> faultLabels {};

    void updateDashboard();

    void updateSystemStatus();

    QWidget *createDashboardPage();
    QWidget *createCamerasPage();
    QWidget *createFaultsPage();
    QWidget *createTestsPage();
    QWidget *createConfigurationPage();

    void createMenus();
    void createStatusBar();

    void saveConfiguration();
    void loadConfiguration();
    void syncConfigurationToWorker();
};

#endif