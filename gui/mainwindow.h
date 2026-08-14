#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <array>


#include "core/hagie_state.h"


class QStackedWidget;
class QLabel;
class QTimer;
class STM32Worker;
class QFrame;
class QPushButton;

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

    int testValveCommand = 100;

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
};

#endif