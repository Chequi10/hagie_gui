#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QLabel>

#include <array>
#include <chrono>
#include <cstdint>
#include <QCheckBox>
#include <vector>
#include <QPlainTextEdit>


#include "core/hagie_state.h"

#include "vision/vision_3d_processor.h"
#include "vision/rgb_camera_worker.h"

#include "vision/rgb_camera_worker.h"
#include "vision/simulated_rgb_frame_source.h"


class QStackedWidget;
class QTimer;
class STM32Worker;
class HeightTargetController;
class QFrame;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QComboBox;
class VisionHeightSource;
class Vision3DWorker;


class MainWindow : public QMainWindow
{
    Q_OBJECT


public:

    explicit MainWindow(
        HagieState *hagieState,
        STM32Worker *stm32Worker,
        HeightTargetController *heightTargetController,
        VisionHeightSource *visionHeightSource,
        Vision3DProcessor *vision3DProcessor,
        Vision3DWorker *vision3DWorker,
        QWidget *parent = nullptr
    );


    ~MainWindow();


private:

    // ========================================================
    // ESTADO CENTRAL
    // ========================================================

    HagieState *state;

    STM32Worker *stm32Worker;

    VisionHeightSource *visionHeightSource;

    HeightTargetController *heightTargetController;

    Vision3DProcessor *vision3DProcessor;

    Vision3DWorker *vision3DWorker;

    


    // ========================================================
    // GUI GENERAL
    // ========================================================

    QStackedWidget *centralStack = nullptr;

    QTimer *dashboardTimer = nullptr;


    QLabel *systemStatusLabel = nullptr;

    QLabel *systemFaultsLabel = nullptr;

    QLabel *configSyncLabel = nullptr;


    // ========================================================
    // DASHBOARD
    // ========================================================

    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > heightLabels {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > targetLabels {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > modeLabels {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > valveLabels {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > faultLabels {};


    // ========================================================
    // FALLAS
    // ========================================================

    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > bodyFaultDetailLabels {};

    std::array<
        QPushButton *,
        HagieState::BODY_COUNT
    > clearNoMovementButtons {};


    // ========================================================
    // PANEL DE TEST
    // ========================================================

    int testValveCommand = 300;


    std::array<
        QFrame *,
        HagieState::BODY_COUNT
    > testBodyFrames {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testFaultLabels {};


    /*
     * Altura REAL del cuerpo.
     *
     * Proviene del encoder STM32.
     */
    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testHeightLabels {};


    /*
     * ========================================================
     * VISIÓN 3D
     * ========================================================
     *
     * Altura detectada por cámaras / nube de puntos.
     */
    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testVisionHeightLabels {};


    /*
     * ========================================================
     * OBJETIVO CALCULADO DESDE VISIÓN 3D
     * ========================================================
     *
     * Resultado de:
     *
     * VisionHeightSource
     *          ↓
     * HeightTargetController
     *
     * Por ahora SOLO se muestra.
     *
     * NO se transmite automáticamente a STM32.
     */
    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testVisionTargetLabels {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testValveLabels {};


    std::array<
        QPushButton *,
        HagieState::BODY_COUNT
    > testUpButtons {};


    std::array<
        QPushButton *,
        HagieState::BODY_COUNT
    > testDownButtons {};


    // ========================================================
    // DIAGNÓSTICO DE ENCODERS
    // ========================================================

    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testEncoderStatusLabels {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testEncoderDeltaLabels {};


    /*
     * Última altura observada de cada encoder.
     */
    std::array<
        uint16_t,
        HagieState::BODY_COUNT
    > testPreviousHeight {};


    /*
     * Indica si ya tenemos una primera muestra válida.
     */
    std::array<
        bool,
        HagieState::BODY_COUNT
    > testEncoderInitialized {};


    /*
     * Momento del último movimiento observado.
     */
    std::array<
        std::chrono::steady_clock::time_point,
        HagieState::BODY_COUNT
    > testLastEncoderMovementTime {};


    /*
     * -1 = bajando
     *  0 = quieto
     * +1 = subiendo
     */
    std::array<
        int8_t,
        HagieState::BODY_COUNT
    > testEncoderDirection {};


    // ========================================================
    // CONFIGURACION VISION 3D - CAMARAS
    // ========================================================

    QComboBox *configVisionCameraCombo =
        nullptr;

    QCheckBox *configVisionCameraEnabled =
        nullptr;

    std::array<
        QCheckBox *,
        HagieState::BODY_COUNT
    > configVisionCameraBodyChecks {};


    QDoubleSpinBox *configVisionCameraPositionX =
        nullptr;

    QDoubleSpinBox *configVisionCameraPositionY =
        nullptr;

    QDoubleSpinBox *configVisionCameraPositionZ =
        nullptr;

    QDoubleSpinBox *configVisionCameraHeight =
        nullptr;

    QDoubleSpinBox *configVisionCameraRoll =
        nullptr;

    QDoubleSpinBox *configVisionCameraPitch =
        nullptr;

    QPushButton *configVisionDetectButton =
        nullptr;

    QLabel *configVisionCameraStatusLabel =
        nullptr;

    QLabel *configHagieImuLabel =
        nullptr;

    QLabel *configVisionCameraImuLabel =
        nullptr;

    QLabel *configVisionCameraMountingErrorLabel =
        nullptr;    

    QLabel *communicationsStm32StatusLabel =
        nullptr;

    QLabel *communicationsStm32UptimeLabel =
        nullptr;

    QLabel *communicationsUartErrorsLabel =
        nullptr;

    QLabel *communicationsTxDroppedLabel =
        nullptr;

    QLabel *communicationsCanStatusLabel =
        nullptr;

    QLabel *communicationsAxiomaticModulesLabel =
        nullptr;

    QLabel *communicationsCanDroppedLabel =
        nullptr;

    QLabel *communicationsImuStatusLabel =
        nullptr;

    QLabel *communicationsVisionStatusLabel =
        nullptr;

    QComboBox *configVisionDetectedSerialCombo =
        nullptr;

    QPushButton *configVisionAssignDetectedButton =
        nullptr;

    std::array<
        uint32_t,
        Vision3DProcessor::CAMERA_COUNT
        > visionCameraSerialNumbers {};

    QSpinBox *configVisionCameraSerial =
        nullptr;


    /*
    * Seriales ZED detectados físicamente.
    *
    * Se actualizan mediante refreshZedCameraDetection()
    * y se utilizan desde la lógica de seguridad
    * sin consultar continuamente el SDK.
    */
    std::vector<uint32_t>
        detectedZedSerialNumbers;

        std::array<
    Vision3DProcessor::CameraConfig,
        Vision3DProcessor::CAMERA_COUNT
    > visionCameraConfigs {};

    /*
    * Cámara actualmente mostrada
    * en los widgets de configuración.
    */
    std::size_t currentVisionCamera =
        0;

    bool isBodyCoveredByActiveCamera(
        std::size_t body
    ) const;




    

    void loadVisionCameraIntoWidgets(
        std::size_t camera
    );

    void saveVisionCameraFromWidgets(
        std::size_t camera
    );


    // ========================================================
    // CONFIGURACIÓN POR CUERPO
    // ========================================================

    std::array<
        QSpinBox *,
        HagieState::BODY_COUNT
    > configMinHeightSpin {};


    std::array<
        QSpinBox *,
        HagieState::BODY_COUNT
    > configMaxHeightSpin {};


    std::array<
        QDoubleSpinBox *,
        HagieState::BODY_COUNT
    > configEncoderScaleSpin {};


    std::array<
        QSpinBox *,
        HagieState::BODY_COUNT
    > configVisionOffsetSpin {};

    std::array<
    QDoubleSpinBox *,
    HagieState::BODY_COUNT
    > configVisionRegionMinX {};


    std::array<
        QDoubleSpinBox *,
        HagieState::BODY_COUNT
    > configVisionRegionMaxX {};


    std::array<
        QDoubleSpinBox *,
        HagieState::BODY_COUNT
    > configVisionRegionMinY {};


    std::array<
        QDoubleSpinBox *,
        HagieState::BODY_COUNT
    > configVisionRegionMaxY {};


    std::array<
        QDoubleSpinBox *,
        HagieState::BODY_COUNT
    > configVisionRegionMinZ {};


    std::array<
        QDoubleSpinBox *,
        HagieState::BODY_COUNT
    > configVisionRegionMaxZ {};


    std::array<
        QSpinBox *,
        HagieState::BODY_COUNT
    > configVisionRegionMinPoints {};


    std::array<
        QComboBox *,
        HagieState::BODY_COUNT
    > configEncoderDirectionCombo {};


    // ========================================================
    // CONFIGURACIÓN GLOBAL
    // ========================================================

    QSpinBox *configMoveThresholdSpin =
        nullptr;


    QDoubleSpinBox *configMinMovementSpin =
        nullptr;


    QSpinBox *configNoMovementTimeoutSpin =
        nullptr;


    QSpinBox *configTargetTimeoutSpin =
    nullptr;

    QPlainTextEdit *logsTextEdit =
        nullptr;

    QPushButton *logsClearButton =
        nullptr;

    QPushButton *logsSaveButton =
        nullptr;

    QPushButton *logsPauseButton =
        nullptr;

    bool logsPaused =
        false;

    bool logStm32StateInitialized =
        false;

    bool logPreviousStm32Connected =
        false;

    bool logCanStateInitialized =
        false;

    bool logPreviousCanOk =
        false;

    bool logVisionStateInitialized =
    false;

    bool logPreviousVisionRunning =
        false;    

    /*
    * Fuente utilizada para obtener
    * las mediciones de visión 3D.
    *
    * SIMULACIÓN / CÁMARAS 3D
    */
    QComboBox *configVisionSourceCombo = nullptr;


    // ========================================================
    // CONTROL AUTO DE PRUEBA
    // ========================================================

    QTimer *testAutoTimer =
        nullptr;


    std::array<
        QSpinBox *,
        HagieState::BODY_COUNT
    > testTargetHeightSpin {};


    std::array<
        QLabel *,
        HagieState::BODY_COUNT
    > testModeLabels {};


    std::array<
        QPushButton *,
        HagieState::BODY_COUNT
    > testManualButtons {};


    std::array<
        QPushButton *,
        HagieState::BODY_COUNT
    > testAutoButtons {};

    std::array<
        QPushButton *,
        HagieState::BODY_COUNT
    > testVisionAutoButtons {};


    std::array<
        bool,
        HagieState::BODY_COUNT
    > testAutoEnabled {};

    std::array<
        bool,
        HagieState::BODY_COUNT
    > testVisionAutoEnabled {};

    // ========================================================
    // CÁMARAS RGB
    // ========================================================

    RgbCameraWorker rgbCameraWorker;

    QLabel *mainCameraLabel =
        nullptr;

    std::size_t selectedRgbCamera =
        0;
    // ========================================================
    // ACTUALIZACIÓN GUI
    // ========================================================

    void updateDashboard();

    void updateSystemStatus();

    void updateFaultPage();

    void updateTestPage();

    void addLogMessage(
        const QString& message
    );
    void updateCameraPage();




    // ========================================================
    // CREACIÓN DE PÁGINAS
    // ========================================================

    QWidget *createDashboardPage();

    QWidget *createCamerasPage();

    QWidget *createFaultsPage();

    QWidget *createTestsPage();

    QWidget *createConfigurationPage();

    QWidget *createCommunicationsPage();

    QWidget *createLogsPage();



    // ========================================================
    // MENÚS / STATUS
    // ========================================================

    void createMenus();

    void createStatusBar();


    // ========================================================
    // CONFIGURACIÓN
    // ========================================================

    void saveConfiguration();

    void loadConfiguration();

    void applyVisionBodyRegions();

    void syncConfigurationToWorker();
    /*
    * Detecta las cámaras ZED conectadas,
    * compara sus seriales con la configuración
    * y actualiza la GUI.
    */
    void refreshZedCameraDetection();
};


#endif