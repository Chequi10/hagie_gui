#include <QApplication>

#include "gui/mainwindow.h"
#include "core/hagie_state.h"
#include "stm32/stm32_worker.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    /*
     * Estado central de toda la aplicación Hagie.
     *
     * Este objeto será compartido por:
     * - GUI
     * - UART / STM32
     * - Visión
     * - IA
     */
    HagieState hagieState;

    /*
    * Comunicación STM32.
    */
    STM32Worker stm32Worker(
        &hagieState,
        "/dev/ttyACM0",
        115200
    );

    if (!stm32Worker.start())
    {
        qWarning("No se pudo iniciar la comunicación con STM32");
    }


  

    /*
     * Crear ventana principal.
     *
     * Le entregamos una referencia al estado central.
     */
    MainWindow window(&hagieState);
    

    window.show();

    int result = app.exec();

    /*
    * Detener correctamente el hilo STM32
    * al cerrar la aplicación.
    */
    stm32Worker.stop();

    return result;
}