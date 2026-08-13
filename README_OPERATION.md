# Funcionamiento General - Ubuntu / Jetson Hagie

## 1. Objetivo

Este documento describe el funcionamiento general del software
ejecutado en Ubuntu/Jetson para el sistema de control Hagie.

Ubuntu/Jetson constituye el nivel de control de alto nivel del sistema.

Sus responsabilidades principales son:

- recibir las consignas generadas por la IA;
- mantener las alturas objetivo de los 6 cuerpos;
- comunicarse con la STM32;
- enviar heartbeat;
- enviar consignas de altura;
- enviar comandos manuales cuando corresponda;
- recibir encoders;
- recibir estado de válvulas;
- recibir diagnóstico;
- recibir estado general de STM32;
- recibir información de IMU;
- supervisar la comunicación.


---

# 2. Arquitectura general

La arquitectura principal es:

```text
Cámaras / IA / nube 3D
          |
          v
       Jetson
       Ubuntu
          |
          | USB / Serial
          | protocolo propio
          v
        STM32
          |
          +---- Encoders
          |
          +---- CAN
          |
          +---- IMU
          |
          +---- Axiomatic
                    |
                    v
                 Válvulas
```


---

# 3. División de responsabilidades

## Jetson / Ubuntu

La Jetson realiza las tareas de alto nivel:

```text
visión
IA
nube de puntos 3D
cálculo de alturas objetivo
supervisión
interfaz de usuario
registro de datos
```

La Jetson decide:

```text
qué altura queremos para cada cuerpo
```


## STM32

La STM32 realiza las tareas críticas de tiempo real:

```text
lectura de encoders
control de altura
control de válvulas
CAN
IMU
watchdogs
detección de fallas
parada segura
```

La Jetson no debe ser la única barrera de seguridad.


---

# 4. Cantidad de cuerpos

El sistema controla:

```cpp
BODY_COUNT = 6
```

Correspondencia:

```text
body 0 = cuerpo 1
body 1 = cuerpo 2
body 2 = cuerpo 3
body 3 = cuerpo 4
body 4 = cuerpo 5
body 5 = cuerpo 6
```

Internamente se utiliza numeración desde cero.


---

# 5. Archivos principales

La aplicación Ubuntu se divide principalmente en:

```text
main.cpp
stm32canbusif.h
stm32canbusif.cpp
protocol.h
protocol.cpp
height_control.h
height_control.cpp
valve_test.h
valve_test.cpp
```

Cada archivo tiene una responsabilidad específica.


---

# 6. main.cpp

`main.cpp` es actualmente el punto principal de ejecución.

Sus responsabilidades incluyen:

```text
crear la interfaz STM32
crear height_control
registrar callbacks
arrancar comunicación
enviar heartbeat
enviar consignas activas
detener cuerpos cuando vence una consigna
ejecutar pruebas lógicas durante desarrollo
```

En la versión definitiva, parte de las pruebas temporales deberán
eliminarse o separarse del programa principal.


---

# 7. stm32canbus_serialif

La clase:

```cpp
stm32canbus_serialif
```

encapsula la comunicación entre Ubuntu y STM32.

Se crea indicando:

```text
dispositivo serie
baudrate
```

Ejemplo:

```cpp
stm32canbus_serialif stm32(
    "/dev/ttyACM0",
    115200
);
```

En el programa actual el dispositivo se recibe como argumento.


---

# 8. Ejecución

Ejemplo:

```bash
./hagie /dev/ttyACM0
```

El programa abre:

```text
/dev/ttyACM0
```

a:

```text
115200 baud
```


---

# 9. Boost.Asio

La comunicación serie utiliza:

```cpp
boost::asio
```

Principalmente:

```cpp
boost::asio::io_context
boost::asio::serial_port
```

Esto permite realizar recepción serie asíncrona sin bloquear el hilo
principal de la aplicación.


---

# 10. Hilo de comunicación

`stm32canbus_serialif` posee:

```cpp
std::thread comm_thread;
```

y:

```cpp
std::atomic<bool> running;
```

Cuando se llama:

```cpp
stm32.start();
```

se inicia la comunicación y el hilo encargado de ejecutar el
`io_context` de Boost.Asio.


---

# 11. Recepción asíncrona

La recepción serie utiliza un buffer:

```cpp
std::array<uint8_t, BUFSIZE> rx_buffer;
```

con:

```cpp
BUFSIZE = 64
```

El flujo conceptual es:

```text
STM32
  |
  | USB / Serial
  v
Boost.Asio
  |
  v
rx_buffer
  |
  v
read_handler()
  |
  v
packet_decoder
  |
  v
handle_packet()
  |
  v
callback correspondiente
```


---

# 12. Protocolo

La aplicación utiliza un protocolo binario propio.

Las clases principales son:

```cpp
protocol::packet_decoder
protocol::packet_encoder
```

`stm32canbus_serialif` hereda privadamente de ambas.


## Objetivo

El protocolo permite transportar mensajes binarios entre:

```text
Ubuntu <-> STM32
```

con mecanismos de sincronización y validación.


---

# 13. Estructura general del protocolo

El protocolo utiliza una cabecera de sincronización:

```text
'P' 'K' 'T' '!'
```

seguida por:

```text
LENGTH
PAYLOAD
CRC16
TERMINADOR
```

Conceptualmente:

```text
+-----+-----+-----+-----+--------+---------+-------+------------+
|  P  |  K  |  T  |  !  | LENGTH | PAYLOAD | CRC16 | TERMINADOR |
+-----+-----+-----+-----+--------+---------+-------+------------+
```


---

# 14. CRC

El protocolo utiliza:

```text
CRC16
```

para detectar corrupción de paquetes.

El valor inicial utilizado por el protocolo es:

```text
0xFFFF
```

Un paquete con CRC incorrecto no debe procesarse como comando válido.


---

# 15. Decoder

Los bytes recibidos desde Boost.Asio se entregan al:

```cpp
protocol::packet_decoder
```

El decoder reconstruye los paquetes.

Cuando existe un paquete válido llama:

```cpp
handle_packet(
    const uint8_t* payload,
    std::size_t n
);
```

La interfaz determina entonces el opcode recibido.


---

# 16. Encoder

Para transmitir hacia STM32 se utiliza:

```cpp
protocol::packet_encoder
```

La clase genera el paquete completo y finalmente utiliza:

```cpp
send_impl()
```

para transmitirlo por el puerto serie.


---

# 17. Comandos Ubuntu -> STM32

Actualmente se utilizan principalmente:

```text
'A' heartbeat
'B' comando directo de válvula
'C' detener todas las válvulas
'D' altura objetivo
'J' reconocimiento de falla NO_MOVEMENT
```


---

# 18. OPCODE 'A' - Heartbeat

Ubuntu envía periódicamente:

```cpp
stm32.send_heartbeat();
```

Esto informa a la STM32 que la comunicación con Jetson continúa activa.

La STM32 posee su propio watchdog y puede detener el sistema si deja de
recibir comunicación válida.


---

# 19. Heartbeat periódico

Actualmente `main.cpp` ejecuta aproximadamente cada:

```text
100 ms
```

un ciclo donde se envía:

```cpp
stm32.send_heartbeat();
```

Además se mide cuánto tiempo pasó desde el ciclo anterior.

Durante desarrollo existe una advertencia si el ciclo se atrasa más de:

```text
150 ms
```

Esto permite detectar bloqueos o retrasos importantes del hilo principal.


---

# 20. OPCODE 'B' - Comando directo de válvula

La función:

```cpp
set_valve_command(
    uint8_t body,
    int16_t command
);
```

envía una orden directa a un cuerpo.

Rango:

```text
-1000 .. +1000
```

Conceptualmente:

```text
-1000 = bajar máximo
    0 = detener
+1000 = subir máximo
```

Cuando STM32 recibe `'B'`, ese cuerpo pasa a:

```text
MANUAL
```


---

# 21. OPCODE 'C' - Stop global

Ubuntu dispone de:

```cpp
stop_all_valves();
```

Esto envía:

```text
OPCODE 'C'
```

La STM32 detiene los seis cuerpos.


---

# 22. OPCODE 'D' - Altura objetivo

La función:

```cpp
set_target_height(
    uint8_t body,
    uint16_t height_mm
);
```

envía una consigna automática de altura para un cuerpo.

Payload conceptual:

```text
'D'
body
height MSB
height LSB
```

Cuando la STM32 acepta la consigna:

```text
cuerpo -> AUTO
```


---

# 23. Ubuntu no calcula directamente la válvula en AUTO

Este concepto es importante.

En modo AUTO Ubuntu envía:

```text
ALTURA OBJETIVO
```

No envía continuamente:

```text
SUBIR / BAJAR
```

El control de tiempo real se realiza en STM32.

Por ejemplo:

```text
IA calcula 520 mm
       |
       v
Ubuntu manda:
cuerpo 2 -> 520 mm
       |
       v
STM32 compara:
520 mm objetivo
vs
altura encoder
       |
       v
STM32 calcula válvula
```


---

# 24. height_control

La clase:

```cpp
height_control
```

almacena las consignas de altura generadas por el nivel superior.

En el futuro esas consignas serán actualizadas principalmente por:

```text
IA
nube 3D
procesamiento de cámaras
```


---

# 25. Flujo futuro de la IA

El flujo previsto es:

```text
Cámaras ZED
     |
     v
nube 3D
     |
     v
detección de cultivo / panojas
     |
     v
cálculo de altura deseada
     |
     v
height_control
     |
     v
main.cpp
     |
     v
set_target_height()
     |
     v
STM32
```


---

# 26. Consignas independientes

Cada cuerpo posee su propia consigna.

Conceptualmente:

```text
target cuerpo 1
target cuerpo 2
target cuerpo 3
target cuerpo 4
target cuerpo 5
target cuerpo 6
```

Esto permite controlar los seis cuerpos de forma independiente.


---

# 27. Vigencia de consignas

Ubuntu verifica:

```cpp
heightControl.has_target(body)
```

para determinar si la consigna de ese cuerpo sigue activa.


## Si está activa

Ubuntu envía:

```cpp
stm32.set_target_height(
    body,
    heightControl.get_target(body)
);
```


## Si venció

Si anteriormente estaba activa y ahora dejó de estarlo:

```cpp
stm32.set_valve_command(
    body,
    0
);
```

Esto detiene ese cuerpo y lo coloca en MANUAL en la STM32.


---

# 28. targetWasActive

`main.cpp` mantiene:

```cpp
std::array<bool, height_control::BODY_COUNT>
    targetWasActive{};
```

Su objetivo es detectar la transición:

```text
ACTIVA -> VENCIDA
```

para no enviar continuamente comandos de parada innecesarios.


---

# 29. Doble protección de timeout

Existen dos niveles.

## Ubuntu

`height_control` controla si una consigna continúa siendo válida.

Si vence:

```text
Ubuntu manda command = 0
```


## STM32

La STM32 posee:

```text
BODY_FAULT_TARGET_TIMEOUT
```

Si Ubuntu falla y deja de refrescar una consigna AUTO:

```text
STM32 detiene ese cuerpo
```

Por lo tanto la seguridad no depende solamente del programa Ubuntu.


---

# 30. Estados recibidos desde STM32

`stm32canbus_serialif` define estructuras para representar los estados
recibidos:

```cpp
encoder_state
valve_state
diagnostic_state
stm32_state
imu_state
```


---

# 31. encoder_state

Contiene:

```cpp
std::array<uint16_t, BODY_COUNT>
    height_mm;
```

Representa las alturas de los seis cuerpos medidas por STM32.


---

# 32. Callback de encoders

Se registra mediante:

```cpp
set_encoder_callback();
```

Flujo:

```text
STM32
 |
 | 'E'
 v
stm32canbus_serialif
 |
 v
encoder_callback
 |
 v
aplicación
```


---

# 33. valve_state

Contiene:

```cpp
std::array<int16_t, BODY_COUNT>
    command;
```

Representa el comando que STM32 está aplicando a cada cuerpo.


---

# 34. Callback de válvulas

Se registra mediante:

```cpp
set_valve_callback();
```

Permite conocer desde Ubuntu el estado de comando de los seis cuerpos.


---

# 35. diagnostic_state

Contiene información de diagnóstico como:

```text
axiomatic_rx_dropped
uart_error_count
jetson_tx_queue_dropped
jetson_tx_dma_errors
jetson_connection_ok
axiomatic_modules
system_faults
body_faults[0..5]
```

Este estado es fundamental para supervisar la salud del sistema.


---

# 36. Callback de diagnóstico

Se registra mediante:

```cpp
set_diagnostic_callback();
```

Actualmente `main.cpp` imprime esta información en la terminal durante
desarrollo.


---

# 37. stm32_state

Contiene información general de STM32:

```text
jetson_connection_ok
uptime_ticks
encoder_count
body_count
axiomatic_modules
```

Permite comprobar que la configuración informada por STM32 coincide con
la esperada por Ubuntu.


---

# 38. imu_state

Contiene:

```text
valid

roll_deg
pitch_deg
gravity_deg

gyro_roll_dps
gyro_pitch_dps
gyro_yaw_dps

accel_x_mps2
accel_y_mps2
accel_z_mps2
```


---

# 39. Callback IMU

Se registra mediante:

```cpp
set_imu_callback();
```

Ubuntu recibe así la información de IMU previamente procesada/recibida
por la STM32.


---

# 40. Paquetes STM32 -> Ubuntu

Actualmente se utilizan principalmente:

```text
'E' encoders
'F' válvulas
'G' diagnóstico
'H' estado STM32
'I' IMU
```


---

# 41. OPCODE 'E'

Representa:

```text
estado de encoders
```

Contiene las alturas de los seis cuerpos.


---

# 42. OPCODE 'F'

Representa:

```text
estado de válvulas
```

Contiene los comandos de los seis cuerpos.


---

# 43. OPCODE 'G'

Representa:

```text
diagnóstico
```

Incluye:

```text
contadores
errores
system_faults
body_faults
```


---

# 44. OPCODE 'H'

Representa:

```text
estado general STM32
```

Incluye información de funcionamiento y configuración.


---

# 45. OPCODE 'I'

Representa:

```text
estado IMU
```


---

# 46. OPCODE 'J'

Ubuntu utiliza `'J'` para solicitar el reconocimiento de una falla
`NO_MOVEMENT` de un cuerpo.

La función correspondiente es:

```cpp
clear_body_fault(body);
```

La STM32 borra la falla correspondiente y mantiene el cuerpo detenido.


---

# 47. Fallas

La descripción detallada del sistema de fallas se encuentra en:

```text
README_FAULTS.md
```

Las fallas por cuerpo actualmente probadas son:

```text
0x04 NO_MOVEMENT
0x20 TARGET_TIMEOUT
```


---

# 48. NO_MOVEMENT

Si STM32 ordena movimiento pero el encoder no registra suficiente
desplazamiento:

```text
BODY_FAULT_NO_MOVEMENT
```

Ubuntu recibe el bit mediante el paquete `'G'`.

La falla permanece activa hasta ser reconocida.


---

# 49. TARGET_TIMEOUT

Si un cuerpo está en AUTO y deja de recibir consignas:

```text
BODY_FAULT_TARGET_TIMEOUT
```

STM32 detiene ese cuerpo.

Cuando vuelve a llegar una consigna válida, esta falla puede recuperarse
automáticamente.


---

# 50. Seguridad por capas

El sistema está diseñado con varias capas:

```text
IA
 |
 v
height_control
 |
 | vigencia de consigna
 v
main.cpp
 |
 | heartbeat + targets
 v
stm32canbus_serialif
 |
 | protocolo + CRC
 v
STM32
 |
 | watchdog
 | TARGET_TIMEOUT
 | NO_MOVEMENT
 v
control de altura
 |
 v
setBodyValveCommand()
 |
 v
CAN
 |
 v
Axiomatic
 |
 v
válvula
```


---

# 51. Pérdida completa de Ubuntu

Si Ubuntu se bloquea:

```text
deja de enviar paquetes
```

La STM32 posee un watchdog independiente.

Por lo tanto:

```text
Ubuntu bloqueado
      |
      v
STM32 detecta timeout
      |
      v
detiene cuerpos
```

La seguridad crítica no depende de que Linux continúe funcionando.


---

# 52. Pérdida de una sola consigna

También puede ocurrir que:

```text
Ubuntu sigue funcionando
heartbeat sigue funcionando
pero un cuerpo deja de recibir target
```

En ese caso la STM32 puede detectar:

```text
TARGET_TIMEOUT
```

solamente en ese cuerpo.


---

# 53. Modo MANUAL y AUTO

Resumen:

```text
OPCODE B
   |
   v
MANUAL
   |
   v
Ubuntu manda directamente command
```

y:

```text
OPCODE D
   |
   v
AUTO
   |
   v
Ubuntu manda altura
   |
   v
STM32 calcula command
```


---

# 54. Parada individual

Para detener un cuerpo:

```cpp
stm32.set_valve_command(
    body,
    0
);
```

Esto también hace que STM32 coloque ese cuerpo en MANUAL.


---

# 55. Parada global

Para detener todos los cuerpos:

```cpp
stm32.stop_all_valves();
```

Esto utiliza:

```text
OPCODE 'C'
```


---

# 56. Arranque de comunicación

La secuencia básica es:

```cpp
stm32canbus_serialif stm32(...);

registrar callbacks

stm32.start();
```

Después de `start()` comienza la recepción asíncrona.


---

# 57. Detención de comunicación

La clase dispone de:

```cpp
stop();
```

y:

```cpp
is_running();
```

para administrar el estado del hilo de comunicación.


---

# 58. Manejo de excepciones

`main.cpp` utiliza:

```cpp
try
{
    ...
}
catch (const std::exception& e)
{
    ...
}
```

Los errores de apertura o funcionamiento del puerto serie pueden así
informarse al usuario.


---

# 59. Flujo completo de una consigna

Ejemplo: IA quiere cuerpo 4 a 550 mm.

```text
1. IA calcula 550 mm
        |
        v
2. height_control guarda target
        |
        v
3. main.cpp detecta target activo
        |
        v
4. set_target_height(3, 550)
        |
        v
5. packet_encoder
        |
        v
6. protocolo serie
        |
        v
7. USB
        |
        v
8. STM32 recibe 'D'
        |
        v
9. cuerpo 4 pasa a AUTO
        |
        v
10. STM32 compara target con encoder
        |
        v
11. calcula comando
        |
        v
12. CAN -> Axiomatic
        |
        v
13. válvula mueve cuerpo
        |
        v
14. encoder mide nueva altura
        |
        v
15. STM32 envía 'E'
        |
        v
16. Ubuntu recibe encoder_state
```


---

# 60. Flujo de telemetría

```text
STM32
 |
 +-- E -> encoders
 |
 +-- F -> válvulas
 |
 +-- G -> diagnóstico
 |
 +-- H -> estado STM32
 |
 +-- I -> IMU
 |
 v
stm32canbus_serialif
 |
 v
callbacks
 |
 v
aplicación Ubuntu
```


---

# 61. Frecuencias actuales

Durante el desarrollo:

```text
loop principal Ubuntu:
aprox. 100 ms / 10 Hz

heartbeat:
aprox. cada 100 ms

refresco de targets activos:
aprox. cada 100 ms
```

Estos valores pueden modificarse posteriormente según las pruebas del
sistema completo.


---

# 62. Pruebas lógicas

Como todavía no se dispone de todo el hardware final, se realizaron
pruebas lógicas desde Ubuntu.

Entre ellas:

```text
comunicación serie

heartbeat

recepción de encoders simulados/provisorios

recepción de válvulas

diagnóstico

estado STM32

IMU

NO_MOVEMENT

clear_body_fault

TARGET_TIMEOUT

recuperación TARGET_TIMEOUT
```


---

# 63. valve_test

Los archivos:

```text
valve_test.h
valve_test.cpp
```

se utilizan durante desarrollo para generar consignas de prueba.

Estas funciones permiten probar la cadena:

```text
Ubuntu
 ->
STM32
 ->
control
 ->
diagnóstico
```

sin disponer todavía de la IA definitiva.


---

# 64. Alturas de prueba

Los valores utilizados actualmente en pruebas no deben interpretarse
como calibraciones definitivas de la Hagie.

Ejemplos como:

```text
200 mm
500 mm
900 mm
```

son valores utilizados durante desarrollo.

Los límites y recorridos reales deberán obtenerse de la máquina.


---

# 65. Código temporal de pruebas

Antes de utilizar el software en la máquina deben revisarse y eliminarse
las pruebas automáticas que puedan generar:

```text
targets artificiales
fallas artificiales
clear automático
comandos de válvula de prueba
```

El programa de producción no debe iniciar movimientos únicamente por
código de prueba.


---

# 66. Elementos pendientes

Entre los elementos que todavía deben integrarse o validarse con
hardware real se encuentran:

```text
IA definitiva

nube 3D

alturas reales

calibración de encoders

límites físicos

Axiomatic real

válvulas reales

IMU real

ajuste hidráulico

control proporcional definitivo

frecuencias definitivas

diagnóstico completo de hardware
```


---

# 67. Reglas para futuras modificaciones

Al modificar el software Ubuntu se deben mantener estas reglas:

1. El heartbeat no debe bloquearse por procesamiento pesado.
2. Las consignas activas deben refrescarse periódicamente.
3. Una consigna vencida debe provocar una condición segura.
4. La parada siempre debe poder enviarse.
5. Las fallas se deben interpretar como máscaras de bits.
6. No asumir que una falla individual representa una falla global.
7. No eliminar las protecciones equivalentes de STM32.
8. No colocar control crítico únicamente en Linux.
9. Mantener compatibilidad de protocolo entre Ubuntu y STM32.
10. Modificar ambos lados si cambia la estructura de un paquete.


---

# 68. Compatibilidad de protocolo

STM32 y Ubuntu deben utilizar exactamente la misma definición de:

```text
opcodes
longitudes
orden de bytes
tipos de datos
CRC
estructura de payload
```

Si se modifica un paquete en STM32 también debe revisarse:

```text
stm32canbusif.cpp
```

en Ubuntu, y viceversa.


---

# 69. Orden de bytes

Los valores multibyte enviados por el protocolo deben reconstruirse en
el mismo orden en ambos lados.

Ejemplo conceptual para `uint16_t`:

```text
MSB
LSB
```

No cambiar este criterio en un solo extremo del protocolo.


---

# 70. Filosofía general

La arquitectura puede resumirse así:

```text
IA decide la altura.

Ubuntu administra las consignas y la comunicación.

STM32 realiza el control de tiempo real.

Encoder informa la posición real.

Axiomatic controla eléctricamente las válvulas.

Los watchdogs y fallas llevan el sistema a un estado seguro.
```


---

# 71. Documentación relacionada

Consultar:

```text
README_FAULTS.md
```

para la descripción detallada de:

```text
fallas por cuerpo
fallas globales
bits
recuperación
NO_MOVEMENT
TARGET_TIMEOUT
clear_body_fault
```

También consultar la documentación equivalente del firmware STM32:

```text
README_OPERATION.md
README_FAULTS.md
```

dentro del proyecto STM32.