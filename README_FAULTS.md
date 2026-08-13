# Sistema de Fallas - Ubuntu / Jetson Hagie

## 1. Objetivo

Este documento describe cómo el software ejecutado en Ubuntu/Jetson
recibe, interpreta y administra la información de fallas proveniente
de la STM32.

La filosofía general es:

```text
STM32
  |
  | detecta fallas de tiempo real
  |
  v
Ubuntu / Jetson
  |
  | recibe diagnóstico
  |
  v
supervisión / registro / interfaz
```

Las protecciones críticas se ejecutan en la STM32.

Ubuntu no debe ser la única barrera de seguridad del sistema.


---

# 2. Tipos de fallas

La STM32 informa dos grupos de fallas:

```text
system_faults
```

para fallas globales del sistema, y:

```text
body_faults[0..5]
```

para fallas individuales de los seis cuerpos.


---

# 3. Correspondencia de cuerpos

En el protocolo se utiliza numeración desde cero:

```text
body 0 = cuerpo 1
body 1 = cuerpo 2
body 2 = cuerpo 3
body 3 = cuerpo 4
body 4 = cuerpo 5
body 5 = cuerpo 6
```

Esto debe tenerse en cuenta al mostrar información al operador.


---

# 4. Recepción del diagnóstico

Ubuntu recibe periódicamente desde la STM32 un paquete de diagnóstico.

Actualmente corresponde al:

```text
OPCODE 'G'
```

El paquete contiene información general de comunicación y diagnóstico,
incluyendo:

```text
axiomatic_rx_dropped
uart_error_count
jetson_tx_queue_dropped
jetson_tx_dma_errors
jetson_connection_ok
axiomatic_modules
system_faults
body_faults[0]
body_faults[1]
body_faults[2]
body_faults[3]
body_faults[4]
body_faults[5]
```


---

# 5. Estructura diagnostic_state

La interfaz serie de Ubuntu representa esta información mediante:

```cpp
stm32canbus_serialif::diagnostic_state
```

Entre sus campos se encuentran:

```cpp
uint32_t system_faults;

std::array<uint32_t, BODY_COUNT>
    body_faults;
```

De esta manera Ubuntu conserva separadas:

```text
fallas globales
```

y:

```text
fallas individuales por cuerpo
```


---

# 6. Callback de diagnóstico

La aplicación puede recibir los diagnósticos utilizando:

```cpp
set_diagnostic_callback()
```

Ejemplo conceptual:

```cpp
stm32.set_diagnostic_callback(
    [](const stm32canbus_serialif::diagnostic_state& state)
    {
        // state.system_faults
        // state.body_faults[0..5]
    }
);
```

El callback se ejecuta cuando se recibe y decodifica correctamente
un nuevo paquete de diagnóstico desde la STM32.


---

# 7. Fallas individuales por cuerpo

Los bits definidos actualmente son:

| Bit | Hex | Falla | Estado |
|---:|---:|---|---|
| 0 | 0x01 | BODY_FAULT_ENCODER_TIMEOUT | Pendiente |
| 1 | 0x02 | BODY_FAULT_ENCODER_RANGE | Pendiente |
| 2 | 0x04 | BODY_FAULT_NO_MOVEMENT | IMPLEMENTADA Y PROBADA |
| 3 | 0x08 | BODY_FAULT_MIN_LIMIT | Pendiente |
| 4 | 0x10 | BODY_FAULT_MAX_LIMIT | Pendiente |
| 5 | 0x20 | BODY_FAULT_TARGET_TIMEOUT | IMPLEMENTADA Y PROBADA |
| 6 | 0x40 | BODY_FAULT_VALVE_ERROR | Pendiente |


---

# 8. BODY_FAULT_NO_MOVEMENT

Valor:

```text
0x04
```

Estado:

```text
IMPLEMENTADA Y PROBADA
```

Esta falla es detectada por la STM32.

Significa:

```text
se ordenó mover un cuerpo
+
el encoder no registró desplazamiento suficiente
+
transcurrió el tiempo de supervisión
```

La STM32 detiene automáticamente el cuerpo afectado.


## Ejemplo recibido por Ubuntu

```text
body_faults =
[
    0x04,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
]
```

Significa:

```text
cuerpo 1 -> NO_MOVEMENT
cuerpos 2..6 -> sin falla
```


---

# 9. NO_MOVEMENT es una falla enclavada

`NO_MOVEMENT` no se borra automáticamente.

Esto es intencional.

Si el cuerpo recibió una orden de movimiento y físicamente no respondió,
puede existir un problema como:

```text
encoder
válvula
hidráulica
mecanismo trabado
cableado
controlador Axiomatic
```

Por lo tanto el sistema requiere un reconocimiento antes de volver a
permitir movimiento.


---

# 10. Reconocimiento de NO_MOVEMENT

Ubuntu dispone de una función para solicitar a la STM32 el borrado de
la falla:

```cpp
clear_body_fault(body);
```

El cuerpo utiliza numeración:

```text
0..5
```

Ejemplo:

```cpp
stm32.clear_body_fault(0);
```

solicita reconocer la falla del:

```text
cuerpo 1
```


---

# 11. Protocolo de clear_body_fault

La función envía:

```text
OPCODE 'J'
```

con:

```text
[0] = 'J'
[1] = cuerpo
```

La STM32 recibe el comando y borra:

```text
BODY_FAULT_NO_MOVEMENT
```

del cuerpo indicado.


---

# 12. Seguridad después del reconocimiento

Reconocer la falla NO significa ordenar movimiento.

Después de borrar `NO_MOVEMENT`, la STM32 deja el cuerpo:

```text
MANUAL
+
command = 0
```

Por lo tanto:

```text
clear_body_fault()
```

no debe provocar movimiento por sí mismo.


---

# 13. Prueba realizada de NO_MOVEMENT

Durante las pruebas lógicas se comprobó la secuencia:

```text
0x00
 |
 v
se genera NO_MOVEMENT
 |
 v
0x04
 |
 v
Ubuntu envía clear_body_fault()
 |
 v
STM32 reconoce la falla
 |
 v
0x00
```

La transición fue observada desde Ubuntu mediante el paquete de
diagnóstico.


---

# 14. BODY_FAULT_TARGET_TIMEOUT

Valor:

```text
0x20
```

Estado:

```text
IMPLEMENTADA Y PROBADA
```

Esta falla también es detectada por la STM32.


## Significado

Un cuerpo estaba funcionando en:

```text
AUTO
```

pero dejó de recibir nuevas consignas de altura desde Ubuntu/Jetson.


---

# 15. TARGET_TIMEOUT es individual

Cada cuerpo posee su propia supervisión de consigna.

Por ejemplo:

```text
Cuerpo 1 -> recibe D
Cuerpo 2 -> recibe D
Cuerpo 3 -> NO recibe D
Cuerpo 4 -> recibe D
Cuerpo 5 -> recibe D
Cuerpo 6 -> recibe D
```

La comunicación general puede seguir funcionando correctamente.

Después del timeout solamente el cuerpo 3 debe generar:

```text
BODY_FAULT_TARGET_TIMEOUT
```


---

# 16. Ejemplo de TARGET_TIMEOUT

Ubuntu podría recibir:

```text
body_faults =
[
    0x00,
    0x00,
    0x20,
    0x00,
    0x00,
    0x00
]
```

Esto significa:

```text
cuerpo 3 -> TARGET_TIMEOUT
resto -> sin falla
```


---

# 17. Recuperación de TARGET_TIMEOUT

A diferencia de `NO_MOVEMENT`, `TARGET_TIMEOUT` posee recuperación
automática.

Cuando Ubuntu vuelve a enviar una consigna de altura válida:

```cpp
set_target_height(body, height_mm);
```

la STM32 puede borrar:

```text
BODY_FAULT_TARGET_TIMEOUT
```

y volver a colocar ese cuerpo en AUTO.


## Secuencia

```text
consignas D
   |
   v
AUTO
   |
   v
dejan de llegar consignas
   |
   v
TARGET_TIMEOUT = 0x20
   |
   v
cuerpo detenido
   |
   v
vuelve una D válida
   |
   v
0x00
   |
   v
AUTO
```


---

# 18. Diferencia entre 0x04 y 0x20

## 0x04 - NO_MOVEMENT

La consigna puede estar llegando correctamente.

El problema es:

```text
se ordena movimiento
pero no existe desplazamiento suficiente
```


## 0x20 - TARGET_TIMEOUT

El problema es:

```text
dejaron de llegar consignas AUTO
para ese cuerpo
```

No implica necesariamente una falla del encoder o de la válvula.


---

# 19. Varias fallas simultáneas

Los valores son máscaras de bits.

Por lo tanto un cuerpo puede contener varias fallas simultáneamente.

Ejemplo:

```text
0x24
```

equivale a:

```text
0x20 TARGET_TIMEOUT
+
0x04 NO_MOVEMENT
```

Por esta razón NO se debe comprobar una falla utilizando solamente:

```cpp
fault == 0x04
```

cuando puedan coexistir varios bits.

La comprobación correcta conceptualmente es:

```cpp
if ((fault & 0x04) != 0)
{
    // NO_MOVEMENT activo
}
```


---

# 20. Fallas globales

La STM32 también transmite:

```cpp
system_faults
```

Los bits definidos actualmente son:

| Bit | Hex | Falla |
|---:|---:|---|
| 0 | 0x01 | SYSTEM_FAULT_JETSON_TIMEOUT |
| 1 | 0x02 | SYSTEM_FAULT_IMU_TIMEOUT |
| 2 | 0x04 | SYSTEM_FAULT_UART_RX |
| 3 | 0x08 | SYSTEM_FAULT_UART_TX |
| 4 | 0x10 | SYSTEM_FAULT_CAN |


---

# 21. SYSTEM_FAULT_JETSON_TIMEOUT

Valor:

```text
0x01
```

Es una falla GLOBAL.

Significa que la STM32 dejó de recibir comunicación válida desde
Ubuntu/Jetson durante el tiempo permitido.

Ante esta condición la STM32 lleva los actuadores a un estado seguro.


---

# 22. Diferencia entre JETSON_TIMEOUT y TARGET_TIMEOUT

Esta diferencia es fundamental.


## JETSON_TIMEOUT

```text
No llegan paquetes válidos desde Jetson
```

Afecta al sistema global.


## TARGET_TIMEOUT

```text
La comunicación sigue funcionando,
pero un cuerpo dejó de recibir su consigna AUTO.
```

Afecta solamente a ese cuerpo.


---

# 23. SYSTEM_FAULT_IMU_TIMEOUT

Valor:

```text
0x02
```

Indica que la STM32 considera que los datos de IMU dejaron de ser
válidos por falta de actualización.


---

# 24. Fallas UART

Actualmente están reservados:

```text
0x04 -> SYSTEM_FAULT_UART_RX
0x08 -> SYSTEM_FAULT_UART_TX
```

Permiten informar problemas asociados a la comunicación serie.


---

# 25. SYSTEM_FAULT_CAN

Valor:

```text
0x10
```

Permite informar una condición global relacionada con la comunicación
CAN de la STM32.


---

# 26. Fallas pendientes

Las siguientes fallas poseen bits definidos pero todavía no tienen su
implementación definitiva:

```text
BODY_FAULT_ENCODER_TIMEOUT
BODY_FAULT_ENCODER_RANGE
BODY_FAULT_MIN_LIMIT
BODY_FAULT_MAX_LIMIT
BODY_FAULT_VALVE_ERROR
```

Ubuntu puede reservar desde ahora esos valores, pero no debe asumir
que esas protecciones ya están funcionando.


---

# 27. ENCODER_TIMEOUT pendiente

Valor reservado:

```text
0x01
```

No debe confundirse con `NO_MOVEMENT`.

Su implementación futura estará relacionada con la detección de una
pérdida o condición anormal del encoder.

Actualmente `NO_MOVEMENT` ya proporciona una protección importante:

```text
si se ordena movimiento
+
no se detecta desplazamiento
->
se detiene el cuerpo
```


---

# 28. ENCODER_RANGE pendiente

Valor reservado:

```text
0x02
```

Se implementará cuando se conozcan las calibraciones y recorridos
físicos reales de los seis cuerpos.


---

# 29. MIN_LIMIT y MAX_LIMIT pendientes

Valores:

```text
MIN_LIMIT = 0x08
MAX_LIMIT = 0x10
```

Se utilizarán para proteger los extremos físicos de recorrido.

Los límites definitivos deben obtenerse de la máquina real.


---

# 30. VALVE_ERROR pendiente

Valor:

```text
0x40
```

Se utilizará cuando esté definido el diagnóstico real proveniente de
los módulos Axiomatic y de las válvulas.


---

# 31. Responsabilidad de Ubuntu

Ubuntu debe utilizar las fallas recibidas principalmente para:

```text
mostrar estado al operador
registrar eventos
generar alarmas
decidir estrategias de alto nivel
solicitar reconocimientos cuando corresponda
```

Pero la parada crítica de seguridad no debe depender exclusivamente
de Ubuntu.


---

# 32. Responsabilidad de STM32

La STM32 es responsable de ejecutar las protecciones de tiempo real.

Ejemplos:

```text
NO_MOVEMENT
TARGET_TIMEOUT
JETSON_TIMEOUT
```

Esto permite que el sistema llegue a un estado seguro incluso si
Ubuntu se bloquea o pierde comunicación.


---

# 33. Heartbeat

Ubuntu envía periódicamente:

```cpp
send_heartbeat();
```

mediante:

```text
OPCODE 'A'
```

Esto mantiene activa la comunicación general con la STM32.

El heartbeat y las consignas de altura son conceptos independientes.


---

# 34. Heartbeat no reemplaza las consignas

Puede ocurrir:

```text
Heartbeat -> OK
Heartbeat -> OK
Heartbeat -> OK
```

pero que un cuerpo no reciba:

```text
'D'
```

En ese caso:

```text
JETSON_TIMEOUT = NO
TARGET_TIMEOUT = SI
```

para el cuerpo afectado.


---

# 35. Envío de consignas

Ubuntu utiliza:

```cpp
set_target_height(
    body,
    height_mm
);
```

para enviar la altura deseada de cada cuerpo.

Mientras una consigna se considere activa, debe refrescarse
periódicamente.


---

# 36. height_control

Ubuntu utiliza el objeto:

```cpp
height_control
```

para almacenar las consignas producidas por el nivel superior.

Conceptualmente:

```text
IA / nube 3D
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

# 37. Vencimiento de una consigna en Ubuntu

Ubuntu también controla la vigencia de sus propias consignas mediante:

```cpp
has_target(body)
```

Cuando una consigna deja de estar activa, Ubuntu puede enviar:

```cpp
set_valve_command(body, 0);
```

para detener ese cuerpo.

Esto constituye una protección adicional al `TARGET_TIMEOUT` de la
STM32.


---

# 38. Capas de protección

El diseño posee varias capas:

```text
IA
 |
 v
height_control
 |
 | vigencia de consigna
 v
Ubuntu
 |
 | protocolo
 v
STM32
 |
 | TARGET_TIMEOUT
 | NO_MOVEMENT
 | JETSON_TIMEOUT
 v
setBodyValveCommand()
 |
 | barrera final
 v
Axiomatic
 |
 v
válvula
```

La intención es que una única falla de software no deje un cuerpo
moviendo indefinidamente.


---

# 39. Estado seguro

Desde el punto de vista de Ubuntu, la orden segura básica es:

```cpp
set_valve_command(body, 0);
```

Para detener todos los cuerpos existe:

```cpp
stop_all_valves();
```

correspondiente al:

```text
OPCODE 'C'
```


---

# 40. Regla importante

Una orden de parada:

```text
command = 0
```

debe permanecer permitida incluso cuando existen fallas.

Las fallas pueden bloquear movimiento, pero nunca deben bloquear la
posibilidad de detener.


---

# 41. Diagnóstico mostrado durante desarrollo

Durante las pruebas, Ubuntu imprime información similar a:

```text
DIAGNOSTICO:
dropped=0
uart_errors=0
tx_queue_dropped=0
tx_dma_errors=0
jetson=1
modulos=3
system_faults=0x0
body_faults=[0x0,0x0,0x0,0x0,0x0,0x0]
```

Esto permite observar directamente los cambios de estado.


---

# 42. Ejemplo NO_MOVEMENT

```text
body_faults=[0x4,0x0,0x0,0x0,0x0,0x0]
```

Significa:

```text
cuerpo 1
NO_MOVEMENT
```


---

# 43. Ejemplo TARGET_TIMEOUT

```text
body_faults=[0x20,0x0,0x0,0x0,0x0,0x0]
```

Significa:

```text
cuerpo 1
TARGET_TIMEOUT
```


---

# 44. Ejemplo sin fallas

```text
system_faults=0x0
body_faults=[0x0,0x0,0x0,0x0,0x0,0x0]
```

Significa que en ese instante no existen bits de falla reportados.


---

# 45. Pruebas realizadas

Se realizaron pruebas lógicas sin disponer todavía de todo el hardware
final.

Entre las pruebas realizadas se encuentran:

```text
recepción de diagnóstico

visualización de body_faults

generación de NO_MOVEMENT

NO_MOVEMENT = 0x04

clear_body_fault()

transición 0x04 -> 0x00

generación de TARGET_TIMEOUT

TARGET_TIMEOUT = 0x20

recuperación 0x20 -> 0x00

supervisión individual por cuerpo
```


---

# 46. Pruebas futuras con hardware real

Cuando el sistema esté instalado en la Hagie será necesario comprobar:

```text
NO_MOVEMENT con hidráulica real

encoder desconectado

válvula desconectada

módulo Axiomatic desconectado

CAN interrumpido

USB Jetson-STM32 desconectado

IMU desconectada

límites físicos

recorridos reales

fallas simultáneas

recuperación después de una falla
```


---

# 47. Resumen de estado actual

## Fallas por cuerpo listas

```text
0x04 BODY_FAULT_NO_MOVEMENT
0x20 BODY_FAULT_TARGET_TIMEOUT
```


## Fallas por cuerpo pendientes

```text
0x01 BODY_FAULT_ENCODER_TIMEOUT
0x02 BODY_FAULT_ENCODER_RANGE
0x08 BODY_FAULT_MIN_LIMIT
0x10 BODY_FAULT_MAX_LIMIT
0x40 BODY_FAULT_VALVE_ERROR
```


## Fallas globales definidas

```text
0x01 SYSTEM_FAULT_JETSON_TIMEOUT
0x02 SYSTEM_FAULT_IMU_TIMEOUT
0x04 SYSTEM_FAULT_UART_RX
0x08 SYSTEM_FAULT_UART_TX
0x10 SYSTEM_FAULT_CAN
```


---

# 48. Filosofía general

```text
Jetson:
decide, supervisa, registra y muestra.

STM32:
controla en tiempo real y protege.

Axiomatic:
ejecuta las órdenes eléctricas hacia las válvulas.

Encoder:
realimenta la posición física.
```

La seguridad crítica debe permanecer lo más cerca posible del
controlador de tiempo real.


---

# 49. Documentación relacionada

Consultar también:

```text
README_OPERATION.md
```

para la descripción general del funcionamiento del software
Ubuntu/Jetson y su comunicación con STM32.