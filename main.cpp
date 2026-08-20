#include <QApplication>


#include "gui/mainwindow.h"
#include "core/hagie_state.h"
#include "core/height_target_controller.h"
#include "stm32/stm32_worker.h"
#include "vision/vision_height_source.h"
#include "vision/vision_3d_processor.h"
#include "vision/vision_3d_worker.h"



int main(
    int argc,
    char *argv[])
{
    QApplication app(
        argc,
        argv
    );


    /*
     * ========================================================
     * ESTADO CENTRAL
     * ========================================================
     *
     * Estado compartido por:
     *
     * - GUI
     * - STM32
     * - Visión 3D
     * - IA
     */
    HagieState hagieState;


    /*
     * ========================================================
     * CÁLCULO DE OBJETIVOS DE ALTURA
     * ========================================================
     *
     * Este módulo convierte:
     *
     * altura detectada por visión
     *
     *          ↓
     *
     * altura objetivo del cuerpo
     *
     * Por ahora:
     *
     * offset = 0 mm
     *
     * por lo tanto:
     *
     * objetivo 3D = altura visión
     *
     * IMPORTANTE:
     *
     * Este objeto NO mueve válvulas.
     * Tampoco transmite todavía a STM32.
     */
    HeightTargetController
        heightTargetController;


    /*
     * ========================================================
     * COMUNICACIÓN STM32
     * ========================================================
     */
    STM32Worker stm32Worker(
        &hagieState,
        "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF525649898367224551-if02",
        115200
    );


    if (!stm32Worker.start())
    {
        qWarning(
            "No se pudo iniciar la comunicación con STM32"
        );
    }


    /*
    * ========================================================
    * PROCESADOR DE VISIÓN 3D
    * ========================================================
    *
    * Procesa las nubes de puntos de las cámaras 3D
    * y obtiene las alturas correspondientes a los cuerpos.
    *
    * Se crea una única instancia para toda la aplicación.
    */
    Vision3DProcessor vision3DProcessor;

    /*
     * ========================================================
     * VISIÓN 3D
     * ========================================================
     *
     * Actualmente VisionHeightSource utiliza
     * datos simulados.
     *
     * Después será reemplazado internamente por:
     *
     * cámaras ZED
     *      ↓
     * nube de puntos
     *      ↓
     * cálculo geométrico
     *      ↓
     * altura de cultivo
     *
     * VisionHeightSource publica sus resultados
     * en HagieState.
     */
    VisionHeightSource visionHeightSource(
        &hagieState
    );

    /*
    * ========================================================
    * WORKER DE VISIÓN 3D
    * ========================================================
    *
    * Coordina:
    *
    * - adquisición de nubes de puntos;
    * - procesamiento de cada cámara;
    * - fusión multicámara;
    * - publicación en VisionHeightSource.
    *
    * NO controla válvulas.
    * NO envía comandos a STM32.
    */
    Vision3DWorker vision3DWorker(
        &hagieState,
        &vision3DProcessor,
        &visionHeightSource
    );


    if (!visionHeightSource.start())
    {
        qWarning(
            "No se pudo iniciar VisionHeightSource"
        );
    }


    /*
    * ========================================================
    * VENTANA PRINCIPAL
    * ========================================================
    *
    * La GUI recibe:
    *
    * - estado central;
    * - comunicación STM32;
    * - controlador de objetivos 3D;
    * - fuente de visión 3D.
    */
    MainWindow window(
        &hagieState,
        &stm32Worker,
        &heightTargetController,
        &visionHeightSource,
        &vision3DProcessor
    );


    window.show();


    /*
     * ========================================================
     * LOOP PRINCIPAL Qt
     * ========================================================
     */
    int result =
        app.exec();

    vision3DWorker.stop();
    /*
     * ========================================================
     * CIERRE ORDENADO
     * ========================================================
     */

    /*
     * Primero detener visión.
     */
    visionHeightSource.stop();


    /*
     * Después detener STM32.
     *
     * STM32Worker intenta dejar las válvulas
     * detenidas antes de cerrar.
     */
    stm32Worker.stop();


    return result;
}