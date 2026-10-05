# Documentación del desarrollo: control de velocidad de un motor DC con PID en Arduino Uno

**Asignatura:** Control Inteligente, octavo semestre  
**Proyecto:** `Motor_encoder/motor_encoder` (PlatformIO)  
**Archivo principal:** `src/main.cpp`  
**Período documentado:** septiembre de 2026

> Este documento reúne, en orden cronológico y con su justificación técnica, todo el proceso de desarrollo del firmware: los requisitos de cada etapa, las decisiones de diseño, los problemas encontrados y cómo se diagnosticaron, y el estado final del sistema. Está pensado como material base para redactar un informe formal. El código fuente final se incluye completo en el Anexo A.

## 1. Introducción y objetivo

El objetivo del proyecto es controlar la velocidad de un motor de corriente continua mediante modulación por ancho de pulso (PWM), primero en lazo abierto y luego en lazo cerrado con un controlador PID. La unidad de control es un Arduino Uno, la etapa de potencia un puente H L298N y el transductor de velocidad un sensor óptico de ranura tipo herradura. La interacción con el sistema se hace por el puerto serie: desde el monitor serie se envían comandos de texto (porcentaje de PWM, velocidad deseada, constantes del controlador) y el Arduino responde con líneas de confirmación y reportes periódicos. En la etapa final esas líneas las lee una aplicación en Python que el estudiante desarrolla en paralelo, junto con un simulador del sistema.

El desarrollo fue incremental. En la primera etapa se implementó el control de velocidad en lazo abierto, con el porcentaje de PWM escrito desde el monitor serie. En la segunda se incorporó el sensor FC-03 para medir las revoluciones por minuto. En la tercera se cerró el lazo con un PID cuyo setpoint y constantes se ajustan en caliente por comandos. En las etapas siguientes se refinó la medición, con mayor frecuencia de muestreo y rechazo de ruido, y se añadieron el fondo de escala real del motor y una prealimentación (feedforward) que compensa la zona muerta.

## 2. Entorno de desarrollo

El proyecto se desarrolló en Visual Studio Code con la extensión PlatformIO IDE (versión 3.3.4). El archivo `platformio.ini` define un único entorno, `uno`, con la plataforma `atmelavr`, la placa `uno` y el framework `arduino`. En la compilación, PlatformIO reportó la plataforma Atmel AVR 5.1.0, el paquete `framework-arduino-avr` 5.2.0, `tool-avrdude` 6.3.0 y el compilador `toolchain-atmelavr` 7.3.0 (GCC 7.3). El microcontrolador es un ATmega328P a 16 MHz, con 2 KB de RAM y 31,5 KB de memoria flash disponibles para el programa.

A la configuración inicial se le añadieron dos opciones del monitor serie. `monitor_speed = 9600` hace que el monitor abra a la misma velocidad que usa el firmware, y `monitor_echo = yes` hace que muestre los caracteres a medida que se escriben, porque por defecto el monitor de PlatformIO no tiene eco local. También se creó el archivo `.vscode/tasks.json` con tareas que invocan PlatformIO por su ruta absoluta; el motivo se explica en la sección 4.5.

## 3. Hardware del sistema

### 3.1 Componentes

El sistema está formado por un Arduino Uno, un módulo de puente H basado en el circuito integrado L298N, un motor DC pequeño, un paquete de baterías que entrega alrededor de 6,5 V y un módulo sensor óptico de velocidad tipo herradura de la serie MH-Sensor (modelo FC-03 o equivalente), con un disco ranurado (encoder) montado en el eje del motor.

El módulo FC-03 tiene un emisor infrarrojo y un fototransistor enfrentados a ambos lados de una ranura. Cuando una ranura del disco pasa entre ellos, la luz atraviesa; cuando pasa una zona opaca, se interrumpe. La señal del fototransistor entra a un comparador LM393 cuyo umbral se ajusta con un potenciómetro, y la salida del comparador es la señal digital `D0`. El módulo ofrece también una salida analógica, `A0`, que no se usa en este proyecto, y dos LED: uno de alimentación y otro que refleja el estado de `D0`. Se asume un disco de 20 ranuras, el valor más común en estos kits, pero el número es configurable por comando.

### 3.2 Conexiones

| Origen | Destino | Función |
|---|---|---|
| L298N `ENA` | Arduino `D3` | Señal PWM de habilitación (velocidad) |
| L298N `IN1` | Arduino `D7` | Sentido de giro |
| L298N `IN2` | Arduino `D8` | Sentido de giro |
| FC-03 `D0` | Arduino `D2` | Pulsos del encoder (interrupción externa INT0) |
| FC-03 `VCC` | Arduino `5V` | Alimentación del sensor |
| FC-03 `GND` | Arduino `GND` | Referencia común |
| Motor | L298N `OUT1` / `OUT2` | Salida del canal A del puente |
| Baterías (+) | L298N, entrada de 12 V | Alimentación de potencia |
| Baterías (−) | L298N `GND`, unido al `GND` del Arduino | Referencia común |

La elección de `D2` para el sensor no es arbitraria. En el Arduino Uno solo los pines 2 y 3 admiten interrupciones externas (INT0 e INT1), y el pin 3 ya lo ocupa la señal PWM de `ENA`. Contar los pulsos por interrupción, y no leyendo el pin periódicamente, garantiza que no se pierdan pulsos mientras el programa hace otra cosa, por ejemplo imprimir por el puerto serie.

La salida PWM del pin 3 la genera el Timer2 del ATmega328P a unos 490 Hz, mientras que `millis()` y `micros()` usan el Timer0, así que la señal al motor y la base de tiempo de la medición no interfieren entre sí. Una frecuencia de 490 Hz produce un silbido audible en el motor. Se puede subir a unos 31 kHz cambiando el preescalador del Timer2 (`TCCR2B = (TCCR2B & 0b11111000) | 0x01`), a costa de que `tone()` deje de funcionar en el pin 11. Esta modificación no se aplicó.

### 3.3 Consideraciones sobre el L298N y la alimentación

Para controlar la velocidad con PWM hay que retirar el jumper del pin `ENA` del módulo. Si se deja puesto, `ENA` queda fijo en alto y el motor gira siempre a velocidad máxima, sin importar la señal del Arduino. Además, las tierras del Arduino, del L298N y de las baterías deben estar unidas: sin esa referencia común, las señales lógicas del Arduino no tienen un cero compartido con el L298N y el puente no las interpreta correctamente.

El módulo tiene otro jumper que habilita un regulador lineal 7805 interno para generar los 5 V de la lógica a partir de la entrada de potencia. Ese regulador necesita del orden de 7 V en la entrada para entregar 5 V estables. Con las baterías del proyecto y el jumper puesto se midieron 6,1 V en la entrada de 12 V y 4,7 V en la salida de 5 V. Aunque está por debajo del valor nominal, 4,7 V queda muy por encima del umbral lógico de las entradas del L298N, y en la práctica el módulo funcionó.

Los transistores bipolares del L298N provocan una caída de tensión de unos 2 a 3 V entre la alimentación y el motor. Con una batería de 6,5 V, el motor recibe bastante menos tensión que conectado directamente, y necesita un porcentaje mínimo de PWM para vencer la fricción estática. En la etapa final se midió esa zona muerta: el motor arranca con un 19 % de PWM y, una vez en movimiento, sigue girando hasta el 15 %. Con el PWM al 100 % alcanza unas 600 RPM.

## 4. Desarrollo por etapas

### 4.1 Etapa 1: control de PWM en lazo abierto desde el monitor serie

El primer requisito fue un programa que variara el PWM para controlar la velocidad del motor, con el porcentaje deseado escrito desde el monitor serie, usando `ENA` en el pin 3 e `IN1` e `IN2` en los pines 7 y 8. El programa se escribió sobre la plantilla vacía que PlatformIO genera en `src/main.cpp`.

El usuario escribe un número y el firmware lo interpreta como porcentaje de PWM. Se aceptan valores entre −100 y 100. El signo indica el sentido de giro: con valores positivos `IN1` va en alto e `IN2` en bajo, y con negativos al revés. La magnitud se convierte al ciclo útil de 8 bits que espera `analogWrite()`:

$$
\text{pwm} = \operatorname{round}\left(\frac{|p| \cdot 255}{100}\right)
$$

Con 0 % ambas entradas quedan en bajo y el motor queda libre, sin freno. Además de los números se definieron los comandos `s` (detener) y `?` (ayuda), y cada cambio se confirma con una línea que muestra el porcentaje, el valor de 8 bits y el sentido.

Para leer el puerto serie se descartaron `Serial.readString()` y la clase `String`, que fragmentan la escasa RAM del Uno. En su lugar, los caracteres se acumulan en un búfer de tamaño fijo y la línea se interpreta con `strtod()`, comprobando mediante el puntero de fin que realmente se leyó un número. Así se evita el comportamiento de `toInt()`, que devuelve 0 ante una entrada no numérica y podría detener o arrancar el motor por error. El rango también se valida antes de aplicar el valor. Esta primera versión ocupaba el 10,9 % de la RAM y el 18,5 % de la flash.

### 4.2 Problema: la entrada serie se partía por dígitos

La primera versión procesaba la línea al recibir el fin de línea, pero también cuando pasaban 60 ms sin llegar caracteres. La intención era que funcionara aunque el monitor del IDE de Arduino estuviera configurado sin fin de línea. En la práctica, el valor se aplicaba antes de presionar Enter: al escribir 10, el motor pasaba primero a 1 % y después a 0 %.

La causa es que el monitor serie de PlatformIO envía cada tecla en el momento en que se presiona. Entre el 1 y el 0 pasaban más de 60 ms, así que el temporizador procesaba la línea incompleta y el 0 llegaba después como una línea aparte. La solución fue eliminar el procesamiento por tiempo: ahora la línea se interpreta únicamente al recibir `\n` o `\r`, y como solo se procesa si el búfer no está vacío, la secuencia CR+LF cuenta como un único Enter. En consecuencia, en el monitor del IDE de Arduino hay que seleccionar "Nueva línea" o "Ambos NL & CR". En esta corrección se añadió también `monitor_echo = yes` a `platformio.ini`. La versión corregida ocupaba el 10,7 % de la RAM y el 18,0 % de la flash.

### 4.3 Diagnóstico del hardware: el motor no giraba

Con el programa cargado, el motor no se movía. Conectado directamente a las baterías funcionaba bien, y cambiar al canal B del puente (`ENB`, `IN3`, `IN4`) no cambió nada. El usuario midió unos 6,5 V en la bornera de 12 V y observó continuidad entre `OUT1` y `OUT2`, y también entre `OUT3` y `OUT4`.

Lo primero fue aclarar que esa continuidad es normal y no dice nada del puente H: el multímetro mide la resistencia del bobinado del motor, de pocos ohmios, esté o no alimentado el L298N. Como fallaban ambos canales, la sospecha recayó en algo común a todo el integrado, en particular la alimentación lógica, porque el regulador 7805 del módulo necesita alrededor de 7 V. Las mediciones siguientes, con el jumper del regulador puesto, dieron 4,7 V en el pin de 5 V y 6,1 V en la entrada de 12 V. Se concluyó que 4,7 V bastaban para la lógica y que el problema estaba en otra parte, así que se propuso comprobar la tierra común entre el Arduino y el L298N y medir en voltios DC, con el programa en marcha, las tensiones de `IN1`, `IN2` y `ENA` y después la de `OUT1` a `OUT2`.

Antes de hacer esas mediciones, el usuario encontró la causa real: estaba confundiendo las borneras de `OUT1`/`OUT2` con las de `OUT3`/`OUT4`, es decir, el motor no estaba conectado a las salidas del canal que se estaba excitando. Una vez corregida la conexión, el sistema funcionó.

### 4.4 Etapa 2: medición de RPM con el sensor FC-03

Con el motor funcionando, se incorporó el sensor óptico para medir la velocidad. El disco debe atravesar la herradura por el centro, sin rozar ninguna de las paredes. El potenciómetro del comparador se ajusta con el motor girando lento, por ejemplo al 15 %, hasta que el LED de `D0` parpadee claramente con cada ranura. Si ese LED queda fijo, encendido o apagado, el umbral está mal ajustado y no se cuenta ningún pulso.

En el firmware, `attachInterrupt()` asocia el flanco de bajada de `D2` a una rutina de servicio de interrupción (ISR) que solo incrementa un contador. La ISR se mantuvo deliberadamente mínima, sin llamadas a `Serial` ni cálculos. El comparador LM393 no tiene histéresis y oscila en el borde de cada ranura; para descartar esos rebotes, la ISR ignora cualquier flanco que llegue menos de 250 µs después del último válido. Ese margen admite hasta 4000 pulsos por segundo, es decir 12 000 RPM con un disco de 20 ranuras, muy por encima de la velocidad del motor.

El contador es de 32 bits y el ATmega328P trabaja con palabras de 8 bits, así que leerlo requiere varias instrucciones. Si la interrupción ocurriera a mitad de la copia, el valor leído sería inconsistente, y si ocurriera entre leer y poner a cero, se perdería un pulso. Por eso la lectura y la puesta a cero se hacen con las interrupciones deshabilitadas (`noInterrupts()` e `interrupts()`). Las variables compartidas con la ISR se declaran `volatile` para que el compilador no las guarde en registros.

En esta versión la velocidad se calculaba cada 500 ms a partir de los pulsos contados en la ventana. Se usaba el tiempo realmente transcurrido y no el nominal, para que un retraso del lazo principal no sesgara el resultado:

$$
\text{RPM} = \frac{N_{\text{pulsos}}}{N_{\text{ranuras}}} \cdot \frac{60\,000}{\Delta t_{\text{ms}}}
$$

Con 20 ranuras y una ventana de 500 ms, cada pulso equivale a 6 RPM, que era la resolución de la medición. Se añadieron los comandos `r<N>`, para declarar el número de ranuras del disco, y `m`, para activar o desactivar el reporte periódico. Esta versión ocupaba el 11,8 % de la RAM y el 22,1 % de la flash.

### 4.5 Problemas del entorno: carga del firmware y botones de PlatformIO

En esta etapa aparecieron dos problemas ajenos al código. El primero fue que la carga fallaba con el error `avrdude: ser_open(): can't set com-state for "\.\COM2"`, aunque la compilación terminaba correctamente. PlatformIO elegía por su cuenta el puerto COM2, que resultó ser un puerto virtual del programa *HHD Software Bridged Serial Port*, igual que COM1. Los demás puertos del sistema (COM4, COM5, COM7 y COM8) eran puertos serie sobre Bluetooth. En la lista de dispositivos USB de Windows no aparecía ningún convertidor USB-serie: ni CH340 (`VID_1A86`), ni FTDI (`VID_0403`), ni un Arduino original (`VID_2341`). Tampoco había dispositivos con error de controlador, lo que descartaba un problema de drivers y apuntaba a la conexión física, típicamente un cable USB que solo sirve para cargar. Se recomendó probar otro cable y otro puerto, y fijar `upload_port` y `monitor_port` en `platformio.ini` una vez identificado el puerto correcto, para que la autodetección no volviera a elegir COM2.

El segundo problema fue que no aparecían los botones de PlatformIO para compilar, cargar y abrir el monitor. La extensión estaba instalada, pero en VS Code estaba abierta la carpeta `Motor_encoder`, que solo contiene la subcarpeta `motor_encoder`, donde está `platformio.ini`. PlatformIO solo reconoce un proyecto cuando `platformio.ini` está en la raíz de una carpeta del espacio de trabajo. Como los botones siguieron sin aparecer después de abrir la subcarpeta, y el comando `pio` no estaba en el PATH del sistema, se creó `.vscode/tasks.json` con tareas que llaman al ejecutable por su ruta absoluta (`${userHome}/.platformio/penv/Scripts/platformio.exe`): `PIO: Build`, que es la tarea de compilación por defecto (`Ctrl+Shift+B`), `PIO: Upload`, `PIO: Monitor`, `PIO: Upload y Monitor` y `PIO: Listar puertos`. Así se pudo trabajar sin depender de la extensión. Más adelante el usuario informó que el problema estaba resuelto.

### 4.6 Etapa 3: control PID de la velocidad

La tercera etapa cerró el lazo. Los requisitos fueron fijar la velocidad deseada con el comando `rpm` seguido de un valor entre −4000 y 4000; poder cambiar las constantes del controlador, con Kp entre 1 y 10, Ki entre 0 y 200 y Kd entre 0 y 2; actuar únicamente sobre el PWM; y que el comando `pwm` con un valor de 0 a 100 fijara ese PWM y desactivara el PID hasta recibir de nuevo `rpm`.

El sistema quedó con dos modos. En el modo manual el PWM lo fija el usuario con `pwm`, o con un número solo, que se mantuvo como atajo compatible con la etapa 1. En el modo PID el PWM lo calcula el controlador para seguir el setpoint. Aunque se pidió `pwm` de 0 a 100, se aceptaron valores de −100 a 100 para conservar el giro inverso que ya funcionaba; con valores de 0 a 100 el comportamiento es exactamente el pedido.

El sensor FC-03 tiene un solo canal óptico: cuenta pulsos, pero no distingue el sentido de giro. Por eso el controlador regula la magnitud de la velocidad, comparando |SP| con las RPM medidas, y el sentido lo impone el signo del setpoint directamente sobre `IN1` e `IN2`. Un control con signo real necesitaría un encoder en cuadratura, de dos canales.

Varias decisiones de diseño de esta etapa se mantienen en la versión final. La primera es normalizar el error a puntos porcentuales del fondo de escala antes de multiplicarlo por las constantes. Con el error en RPM, un Kp de 1 daría un 1000 % de PWM ante un error de 1000 RPM, y las constantes tendrían que ser del orden de 0,01. Con la normalización, Kp = 3 significa "3 % de PWM por cada 1 % de error de fondo de escala", y los rangos pedidos (1–10, 0–200 y 0–2) resultan útiles. En esta etapa el fondo de escala era una constante de 4000 RPM.

La segunda decisión es calcular el término derivativo sobre la medición y no sobre el error, lo que elimina el pico de salida (*derivative kick*) que aparece cada vez que el setpoint cambia de golpe. La tercera es un anti-windup por integración condicional: el integrador solo acumula cuando la salida no está saturada, o cuando el error empuja la salida de vuelta al rango útil. Sin él, al pedir una velocidad inalcanzable el integrador crecería sin límite y el motor tardaría mucho en responder al bajar el setpoint. Además, el integrador se reinicia al pasar del modo manual al modo PID y al cambiar Ki, porque lo acumulado ya no corresponde a la nueva situación.

La medición también cambió. El lazo de control pasó a ejecutarse cada 50 ms (20 Hz) y el reporte cada 500 ms. Con conteo simple, una ventana de 50 ms daría una resolución de 60 RPM por pulso, demasiado gruesa para derivar. Por eso la velocidad pasó a medirse de flanco a flanco: la ISR guarda la marca de tiempo (`micros()`) del último flanco válido, y el cálculo usa el intervalo entre el último flanco de la ventana actual y el de la anterior, lo que elimina el error de cuantización de contar pulsos en un intervalo fijo. Si durante 300 ms no llega ningún pulso, se considera que el motor está detenido y la velocidad pasa a cero. El primer pulso después de una detención solo fija la referencia de tiempo, porque todavía no hay un intervalo válido que medir.

Por último, la escritura sobre el puente H se separó de la impresión. La función `aplicarSalida()` escribe los pines sin imprimir nada, porque el PID la llama muchas veces por segundo y cualquier `Serial.print` ahí saturaría el monitor. Las confirmaciones las imprime `imprimirEstado()` y el reporte periódico `reportar()`, que distingue el modo con los prefijos `[PID]` y `[MAN]`. El intérprete de comandos se reescribió para separar el comando, formado por las letras iniciales pasadas a minúsculas, del argumento numérico, de modo que `rpm 800`, `rpm800` y `rpm=800` se aceptan por igual. Los valores iniciales de las constantes fueron Kp = 3, Ki = 40 y Kd = 0, es decir un PI puro para empezar a sintonizar. Esta versión ocupaba el 15,7 % de la RAM y el 30,2 % de la flash.

### 4.7 Problema: el comando `rpm` respondía "Ranuras no válidas"

Al probar el PID, cada comando `rpm` respondía con el mensaje "Ranuras no validas (1 a 1000). Ej: r20". Ese texto no existía en el código fuente vigente, sino en la versión de la etapa 2. Al revisar el binario compilado, `firmware.elf`, se comprobó que sí contenía los textos nuevos del PID. El Arduino, en cambio, seguía ejecutando el firmware anterior.

En esa versión el intérprete miraba solo la primera letra del comando. Al escribir `rpm 1500`, veía la `r`, lo interpretaba como el comando de ranuras y convertía el resto con `atol("pm 1500")`, que devuelve 0 porque empieza con letras. Como 0 queda fuera del rango de 1 a 1000, aparecía el mensaje. La solución fue cargar el firmware nuevo, cuyo intérprete compara el token completo y distingue `rpm` de `r`. Para saber qué versión está cargada basta con mirar la cabecera de la ayuda: la versión con PID muestra `=== Control de motor DC: PWM manual + PID de RPM ===`.

### 4.8 Aumento de la frecuencia de muestreo y de reporte

Después se duplicaron la frecuencia de lectura del sensor y la de impresión. El período del lazo de medición y control (`PERIODO_PID_MS`) pasó de 50 ms a 25 ms, es decir de 20 Hz a 40 Hz, y el del reporte (`PERIODO_REPORTE_MS`) de 500 ms a 250 ms, de 2 Hz a 4 Hz.

Como el PID se ejecuta en el mismo ciclo que la lectura del sensor, también pasó a 40 Hz. Kp y Ki mantienen su efecto porque el cálculo usa el Δt real, pero el derivativo no: el mismo ruido de medición, dividido entre la mitad de tiempo, pesa el doble. La columna `pulsos` del reporte pasó a contar los pulsos de 250 ms y muestra la mitad que antes; las RPM no cambian. Con 20 ranuras, por debajo de unas 120 RPM no llega un pulso en cada ventana de 25 ms, y en esas ventanas se conserva la última medición. En cuanto al puerto serie, a 9600 baudios cuatro líneas de reporte por segundo, de unos 68 caracteres cada una, ocupan cerca del 28 % de la capacidad del canal. Si en el futuro hiciera falta más, habría que subir a 115 200 baudios tanto en el firmware como en `monitor_speed`.

### 4.9 Filtrado del ruido del sensor

En la medición de velocidad se identificaron picos aislados, con saltos a 1400, 2000 y 3200 RPM, atribuidos al ruido que la conmutación del PWM induce en la línea del sensor. A partir de un análisis hecho con el simulador que el estudiante desarrolla en paralelo, se propuso añadir condensadores en el hardware y dos filtros en el firmware. Los filtros se aplicaron tal como se propusieron.

El primer filtro está en la ISR. Un pico inducido por el PWM dura pocos microsegundos, mientras que una ranura real mantiene la línea en bajo durante cientos de microsegundos. Por eso, al detectar un flanco de bajada, la ISR espera 8 µs (`delayMicroseconds(8)`) y vuelve a leer el pin directamente del registro del puerto (`PIND & _BV(PD2)`, que corresponde al pin 2 del Uno). Si la línea ya volvió a alto, el flanco era ruido y se descarta; si no, se aplica el filtro de rebote de 250 µs. Como la ISR lee `PD2` directamente, se añadió un `static_assert(PIN_SENSOR == 2, ...)`: si alguien cambia el pin del sensor, el programa deja de compilar hasta que se corrija esa lectura. El costo es de unos 8 µs por flanco. A 4000 RPM con 20 ranuras llegan unos 1300 flancos por segundo, alrededor del 1 % de la CPU, y `millis()` no pierde precisión porque 8 µs es mucho menos que el período de desbordamiento del Timer0.

Este filtro tiene un límite conocido. Rechaza los picos hacia abajo cuando la línea está en alto, que es el caso más frecuente, porque en alto la línea solo la sostiene la resistencia de pull-up de la salida en colector abierto del LM393. Un pico hacia arriba mientras la línea está en bajo termina en un flanco de bajada con la línea todavía en bajo, y ese no se rechaza. Es poco probable, porque en bajo el transistor del LM393 fija la línea con baja impedancia.

El segundo filtro fue una mediana de las tres últimas mediciones crudas, que impide que un pico aislado llegue al PID. Se calculaba con la expresión `max(min(a,b), min(max(a,b), c))`, y el historial se ponía a cero cuando el motor se daba por detenido, para no mezclar mediciones viejas al volver a arrancar. Con un período de 25 ms, un cambio brusco de velocidad aparecía en la mediana dos muestras después, unos 50 ms. Esta versión ocupaba el 16,3 % de la RAM y el 31,2 % de la flash.

### 4.10 Etapa 4: fondo de escala real, prealimentación y mediana de cinco

La última etapa introdujo tres cambios con una restricción explícita: no modificar el formato de las líneas de reporte y de confirmación, los comandos existentes, los rangos de Kp, Ki y Kd, la velocidad en baudios ni el período de reporte, porque una aplicación en Python lee el puerto serie y depende de ellos.

El primer cambio fue convertir el fondo de escala de constante (`RPM_MAX = 4000`) a variable (`rpmMax`), con un valor inicial de 600 RPM, la velocidad real del motor con el PWM al 100 %. La variable se usa en la normalización del error, en el límite del comando `rpm` (que pasa a aceptar valores entre −rpmMax y +rpmMax), en la conversión del error a RPM del reporte y en el término derivativo, que debe usar la misma escala que el error para no quedar descalibrado respecto a P e I. El nuevo comando `max <v>`, con valores entre 10 y 4000, permite cambiarla sin recargar el programa. Al cambiarla se reinicia el estado del PID, porque el error normalizado cambia de escala.

El segundo cambio fue una prealimentación (feedforward). En modo PID la salida pasa a ser u = ff + P + I + D, donde ff es el PWM que, según una recta entre la zona muerta y el fondo de escala, ya debería producir la velocidad pedida. Así el PID solo corrige lo que el modelo no acierta, en vez de construir todo el esfuerzo desde el integrador y atravesar la zona muerta acumulando error. El parámetro `pwmMin` arranca en 15 %, el valor medido de la zona muerta, y se cambia con el comando `zm <v>` (0 a 90). El comando `ff`, sin argumento, activa o desactiva la prealimentación, que arranca activada. El anti-windup evalúa la saturación con el ff incluido, porque el ff ya ocupa parte del rango de PWM y, si no se contara, el integrador seguiría acumulando con la salida real ya en 100 %. El caso SP = 0 se resuelve antes de calcular el ff, así que con setpoint cero el motor se sigue deteniendo; de lo contrario, el ff mantendría un 15 % de PWM aplicado.

El tercer cambio fue sustituir la mediana de tres por una de cinco, que rechaza hasta dos valores atípicos entre las últimas cinco muestras, sean consecutivos o no. Se implementó con la función `medianaHist()`, que ordena por inserción una copia del historial y devuelve el elemento central. El historial se sigue poniendo a cero cuando el motor se da por detenido.

Hubo que cuidar un posible conflicto entre el comando `m` y el nuevo `max`. El intérprete ya separaba la palabra completa del comando y la comparaba de forma exacta con `strcmp`, así que `max` y `zm` nunca se confunden con `m`; se dejó en el código un comentario que lo explica. En la ayuda se añadieron los tres comandos nuevos, la línea de `rpm` pasó a indicar el rango "(-max a max)" y se agregó una línea con los valores actuales (`max=600.0  zm=15.0  ff=activada`) debajo de la línea de constantes, que quedó igual. La versión final ocupa el 17,6 % de la RAM (361 bytes) y el 33,7 % de la flash (10 866 bytes).

## 5. Descripción técnica del firmware final

### 5.1 Arquitectura y temporización

El programa no usa `delay()` ni esperas bloqueantes en el lazo principal. En cada vuelta de `loop()` se leen los caracteres disponibles del puerto serie y, cuando se completa una línea, se pasa a `procesarLinea()`. Además se revisan dos temporizadores basados en `millis()`. Cada 25 ms se actualiza la medición de velocidad (`actualizarMedicion()`) y, si el modo PID está activo, se ejecuta el controlador (`controlPid()`). Cada 250 ms se imprime el reporte (`reportar()`). El conteo de pulsos ocurre de forma asíncrona en la ISR `contarPulso()`. El Δt que usan el integrador y el derivativo es el tiempo realmente transcurrido entre ejecuciones y no el período nominal, de modo que un retraso ocasional del lazo no altera el cálculo.

Las funciones principales son `contarPulso()` (la ISR: filtro de picos, filtro de rebote y conteo), `actualizarMedicion()` (lectura atómica, cálculo flanco a flanco y mediana), `medianaHist()` (mediana de cinco), `controlPid()` (prealimentación, PID y anti-windup), `reiniciarPid()` (limpia el integrador y el estado del derivativo), `aplicarSalida()` (escribe el sentido y el PWM en el puente H, sin imprimir), `imprimirEstado()` (líneas de confirmación), `reportar()` (reporte periódico), `mostrarAyuda()` y `procesarLinea()` (intérprete de comandos).

### 5.2 Medición de velocidad

La medición pasa por cuatro filtros sucesivos. En la ISR, un flanco de bajada solo se acepta si la línea sigue en bajo 8 µs después (rechazo de picos) y si han pasado al menos 250 µs desde el último flanco válido (rechazo de rebotes). Cada 25 ms se leen de forma atómica el número de pulsos acumulados y la marca de tiempo del último flanco, y se calcula la velocidad cruda sobre el intervalo exacto entre flancos:

$$
\text{RPM}_{\text{cruda}} = \frac{60 \times 10^{6} \cdot N_{\text{pulsos}}}{\Delta t_{\mu s} \cdot N_{\text{ranuras}}}
$$

Aquí Δt es el tiempo entre el último flanco de la ventana actual y el último flanco procesado antes. La velocidad cruda entra en un historial de cinco posiciones, y la velocidad que usa el control es la mediana de ese historial. Si pasan 300 ms sin pulsos, la velocidad se fija en cero, el historial se vacía y la medición siguiente espera un nuevo flanco de referencia.

### 5.3 Ley de control

Si |SP| < 1 RPM, el controlador aplica 0 % de PWM, limpia el integrador y termina. En otro caso calcula, en este orden:

$$
ff = \operatorname{sat}_{[0,100]}\left(pwm_{min} + |SP| \cdot \frac{100 - pwm_{min}}{rpm_{max}}\right) \quad (ff = 0 \text{ si la prealimentación está desactivada})
$$

$$
e_{\%} = \frac{(|SP| - \text{RPM}) \cdot 100}{rpm_{max}}
$$

$$
P = K_p \, e_{\%} \qquad
D = -K_d \, \frac{(\text{RPM}_k - \text{RPM}_{k-1}) \cdot 100 / rpm_{max}}{\Delta t} \qquad
I_{\text{tent}} = I + K_i \, e_{\%} \, \Delta t
$$

El integrador adopta el valor tentativo solo si la salida tentativa, u_tent = ff + P + I_tent + D, no está saturada, o si lo está pero el error tiene el signo que la saca de la saturación (u_tent > 100 con e < 0, o u_tent < 0 con e > 0). La salida final es

$$
u = \operatorname{sat}_{[0,100]}\left(ff + P + I + D\right)
$$

y se aplica con el signo del setpoint: positivo, sentido "horario" (`IN1` en alto); negativo, "antihorario" (`IN2` en alto). La correspondencia con el sentido físico depende de cómo esté conectado el motor. Con los valores por defecto (pwmMin = 15 % y rpmMax = 600), la prealimentación vale 57,5 % para un setpoint de 300 RPM y 100 % para 600 RPM.

### 5.4 Interfaz serie

La comunicación es a 9600 baudios y cada comando debe terminar con fin de línea. El intérprete ignora los espacios iniciales, separa las letras del comando sin distinguir mayúsculas de minúsculas y admite un espacio, `=` o `:` entre el comando y el valor.

| Comando | Rango | Efecto |
|---|---|---|
| `rpm <v>` | −rpmMax a +rpmMax | Fija el setpoint y activa el PID |
| `pwm <v>` | −100 a 100 | Fija el PWM y desactiva el PID |
| `<v>` (solo el número) | −100 a 100 | Atajo de `pwm <v>` |
| `kp <v>` | 1 a 10 | Constante proporcional |
| `ki <v>` | 0 a 200 | Constante integral (reinicia el integrador) |
| `kd <v>` | 0 a 2 | Constante derivativa |
| `max <v>` | 10 a 4000 | RPM del motor con PWM al 100 % (reinicia el PID) |
| `zm <v>` | 0 a 90 | Zona muerta: PWM mínimo de la prealimentación |
| `ff` | — | Activa o desactiva la prealimentación |
| `s` o `stop` | — | Detiene el motor y desactiva el PID |
| `r <v>` | 1 a 1000 | Ranuras por vuelta del disco |
| `m` | — | Activa o desactiva el reporte periódico |
| `?` | — | Muestra la ayuda |

El reporte periódico tiene dos formatos según el modo. Los valores de estos ejemplos son ilustrativos:

```
[PID] SP: 300  RPM: 298.7  PWM: 58.1 %  err: 1.3  pulsos: 25
[MAN] RPM: 245.3  PWM: 50.0 %  pulsos: 20
```

En el reporte, el setpoint y el PWM aparecen en valor absoluto, `err` es el error en RPM y `pulsos` es el número de pulsos contados desde el reporte anterior. Las confirmaciones de `rpm`, `pwm` y `s` siguen el formato `<prefijo>  SP: <sp> RPM  PWM: <pwm> %  sentido: <horario|antihorario|detenido>`. El tramo del SP solo aparece en modo PID y el PWM se imprime con signo. Los prefijos son `[PID] activado.`, `[MAN] PID desactivado.` y `[MAN] detenido, PID desactivado.`. El PWM que muestra la confirmación de `rpm` es el que había al recibir el comando, antes del primer ciclo del PID.

Las demás respuestas son `Kp = 3.00`, `Ki = 40.00` y `Kd = 0.00` (dos decimales), `max = 600.0` y `zm = 15.0` (un decimal), `Ranuras por vuelta: 20`, `Reporte: activado` o `desactivado` y `Prealimentacion: activada` o `desactivada`. Los errores se informan como `Fuera de rango: <parámetro> debe ir de <mín> a <máx>`, `Falta el valor numerico en: <línea>` y `Comando no reconocido: <línea>`. La ayuda termina con dos líneas de estado: `Kp=3.00  Ki=40.00  Kd=0.00  ranuras=20  modo=MANUAL` y `max=600.0  zm=15.0  ff=activada`.

### 5.5 Parámetros y valores por defecto

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `kp`, `ki`, `kd` | 3; 40; 0 | Constantes del PID (PI puro al inicio) |
| `rpmMax` | 600 RPM | Fondo de escala: velocidad con PWM al 100 % |
| `pwmMin` | 15 % | Zona muerta que usa la prealimentación |
| `ffActivo` | activada | Estado de la prealimentación |
| `ranurasPorVuelta` | 20 | Ranuras del disco encoder |
| `PERIODO_PID_MS` | 25 ms | Período de medición y control (40 Hz) |
| `PERIODO_REPORTE_MS` | 250 ms | Período del reporte (4 Hz) |
| `MIN_PULSO_US` | 250 µs | Separación mínima entre flancos válidos |
| Filtro de picos en la ISR | 8 µs | Tiempo de confirmación del flanco |
| `TIEMPO_SIN_PULSOS_US` | 300 ms | Tiempo sin pulsos para dar el motor por detenido |
| `N_MEDIANA` | 5 | Tamaño del historial de la mediana |
| `BAUDIOS` | 9600 | Velocidad del puerto serie |
| `LARGO_BUFFER` | 24 | Longitud del búfer de línea (23 caracteres útiles) |

### 5.6 Uso de recursos

La tabla resume la evolución del uso de memoria según el informe de compilación de PlatformIO.

| Versión | RAM | Flash |
|---|---|---|
| Etapa 1: PWM en lazo abierto | 10,9 % | 18,5 % |
| Etapa 1 corregida (sin procesamiento por tiempo) | 10,7 % | 18,0 % |
| Etapa 2: medición de RPM | 11,8 % | 22,1 % |
| Etapa 3: PID (igual tras duplicar el muestreo) | 15,7 % | 30,2 % |
| Filtros de ruido (ISR y mediana de tres) | 16,3 % | 31,2 % |
| Etapa 4: rpmMax, prealimentación y mediana de cinco | 17,6 % (361 B) | 33,7 % (10 866 B) |

## 6. Verificación realizada

Cada cambio se compiló con PlatformIO, sin errores ni advertencias, antes de darse por terminado. En la etapa final se hicieron además tres comprobaciones específicas. Primero, se comparó el conjunto de todas las cadenas `F("...")` del programa antes y después de los cambios: la única que desapareció fue la línea de ayuda de `rpm` con el rango antiguo "(-4000 a 4000)", lo que confirma que no cambió ninguna de las líneas de reporte ni de confirmación de las que depende la aplicación en Python. Segundo, se verificó que no quedara ninguna referencia a la constante `RPM_MAX`. Tercero, como no había un compilador de C++ para PC, se tradujeron fielmente a C# la función `medianaHist()` y la línea de la prealimentación, y se probaron desde PowerShell. La mediana acertó en los 127 casos probados: las 120 permutaciones de cinco valores distintos y siete casos particulares (rearranque desde cero con una, dos y tres mediciones reales, un pico aislado, dos picos separados, dos picos consecutivos y cinco valores iguales). La prealimentación dio los valores esperados: 57,5 % para 300 RPM, 100 % para 600 RPM, 100 % (saturada) para 900 RPM y 50 % para 300 RPM con zona muerta nula.

El hardware se verificó por pasos a lo largo del desarrollo: el motor, conectado directamente a la batería; el L298N, midiendo las tensiones de potencia y de lógica; y el sensor, ajustando el potenciómetro hasta ver parpadear el LED de `D0`. La respuesta del PID con el motor real, es decir la sintonía de las constantes, queda como trabajo experimental del estudiante.

## 7. Limitaciones conocidas y trabajo futuro

El sensor de un canal no detecta el sentido de giro, así que el PID controla la magnitud de la velocidad y supone que el sentido real coincide con el signo del setpoint; para medir el sentido haría falta un encoder en cuadratura. La batería de 6,5 V y la caída del L298N limitan la velocidad máxima a unas 600 RPM. Si se baja `max` por debajo del setpoint vigente, el setpoint no se recorta: la prealimentación queda en 100 % y el anti-windup evita que el integrador se dispare.

Al pasar el fondo de escala de 4000 a 600 RPM, las mismas constantes actúan unas 6,7 veces más fuerte (4000/600) sobre el mismo error en RPM. Cualquier sintonía anterior queda más agresiva, y como la prealimentación aporta la mayor parte del PWM, lo esperable es que se necesiten Kp y Ki más bajos. Cambiar `zm` o activar y desactivar `ff` con el motor en marcha no reinicia el integrador, así que produce un escalón en el PWM que el integrador absorbe con el tiempo. Para comparar la respuesta con y sin prealimentación conviene partir siempre del reposo (`s` y luego `rpm`).

La mediana de cinco retrasa la medición: un escalón de velocidad aparece tres muestras después, unos 75 ms, y al arrancar desde el reposo hacen falta tres mediciones reales para que la velocidad deje de marcar cero. A baja velocidad las muestras llegan más espaciadas y el retraso crece. El derivativo sigue siendo sensible al ruido y conviene introducirlo con valores pequeños de Kd. El filtro de 8 µs no rechaza los picos positivos que ocurren con la línea en bajo. Quedan como mejoras posibles los condensadores de desacoplo en la línea del sensor y en la alimentación del motor, subir el puerto serie a 115 200 baudios si hace falta más información por segundo, subir la frecuencia del PWM para eliminar el silbido audible, fijar `upload_port` en `platformio.ini` para evitar la autodetección de puertos virtuales, y automatizar la identificación de `rpmMax` y `pwmMin`.

## 8. Conclusiones y lecciones del proceso

Del proceso quedan varias lecciones útiles para el informe. Una prueba de continuidad en las salidas del puente H mide el bobinado del motor y no el funcionamiento del puente; para diagnosticar hay que medir tensiones con el circuito en marcha, y antes de sospechar de los componentes conviene revisar la identificación de las borneras. Ante un comportamiento que no corresponde al código, lo primero es confirmar qué firmware está cargado en la placa: el mensaje "Ranuras no válidas" no era un error del programa nuevo, sino la prueba de que seguía cargado el anterior. Y como el monitor serie de PlatformIO envía cada tecla por separado, cualquier interpretación de comandos basada en tiempo es frágil; el fin de línea es el único delimitador fiable.

En cuanto al control, normalizar el error sobre el fondo de escala deja las constantes adimensionales y en rangos manejables. La prealimentación basada en un modelo lineal con zona muerta libera al integrador de sostener el punto de operación, de modo que el PID pasa a corregir desviaciones en lugar de construir toda la salida. El anti-windup y el derivativo sobre la medición son necesarios en un sistema con saturación y cambios bruscos de setpoint. Por último, la calidad de la medición condiciona la del control: medir de flanco a flanco, filtrar los picos en la ISR y aplicar una mediana hacen que la señal que llega al PID sea utilizable a 40 Hz.

## Anexo A. Código fuente final (`src/main.cpp`)

```cpp
#include <Arduino.h>
#include <ctype.h>

/*
 * Control de velocidad de un motor DC: PWM manual + PID de RPM
 * Arduino Uno + puente H L298N + sensor optico de ranura (FC-03 / MH-Sensor)
 *
 * Conexiones:
 *   L298N  ENA -> D3   (salida PWM, ~490 Hz por Timer2)
 *   L298N  IN1 -> D7
 *   L298N  IN2 -> D8
 *   FC-03  D0  -> D2   (INT0: la cuenta de pulsos se hace por interrupcion)
 *   FC-03  VCC -> 5V del Arduino
 *   FC-03  GND -> GND del Arduino
 *   Todos los GND (Arduino, L298N y bateria) unidos entre si.
 *
 * Comandos del monitor serial (9600 baudios), siempre con Enter al final:
 *   rpm 300    -> setpoint de 300 RPM y ACTIVA el PID (-max a max)
 *   pwm 60     -> fija 60 % de PWM y DESACTIVA el PID (-100 a 100)
 *   60         -> atajo equivalente a "pwm 60"
 *   kp 3.5     -> constante proporcional (1 a 10)
 *   ki 40      -> constante integral (0 a 200)
 *   kd 0.5     -> constante derivativa (0 a 2)
 *   max 600    -> RPM reales del motor con PWM al 100 % (10 a 4000)
 *   zm 15      -> zona muerta: PWM minimo de la prealimentacion (0 a 90)
 *   ff         -> activar / desactivar la prealimentacion (arranca activada)
 *   s          -> detener el motor (tambien desactiva el PID)
 *   r 20       -> ranuras por vuelta del disco encoder
 *   m          -> activar / desactivar el reporte periodico
 *   ?          -> mostrar la ayuda
 *
 * El signo indica el sentido de giro, tanto en "rpm" como en "pwm". El sensor
 * de ranura no distingue el sentido, asi que el PID regula la MAGNITUD de la
 * velocidad y el sentido lo impone el signo del setpoint.
 */

const uint8_t PIN_ENA = 3;
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;
const uint8_t PIN_SENSOR = 2;
// contarPulso() lee el pin directo como PD2 por velocidad: si mueves el
// sensor de pin, hay que ajustar tambien esa lectura.
static_assert(PIN_SENSOR == 2, "contarPulso() lee PD2 directamente");

const unsigned long BAUDIOS = 9600;

// Velocidad real del motor con el PWM al 100 %. Es el fondo de escala del
// sistema: normaliza el error del PID (asi las constantes quedan en los rangos
// kp 1-10, ki 0-200, kd 0-2), limita el setpoint y escala la prealimentacion.
// Se cambia en caliente con el comando max.
float rpmMax = 600.0f;
const float RPMMAX_MIN = 10.0f, RPMMAX_MAX = 4000.0f;

// Prealimentacion (feedforward): ff = pwmMin + |SP| * (100 - pwmMin) / rpmMax.
// pwmMin es la zona muerta medida: el motor arranca con 19 % y se sigue
// moviendo hasta 15 %. Se cambia con el comando zm; ff la activa/desactiva.
float pwmMin = 15.0f;
const float ZM_MIN = 0.0f, ZM_MAX = 90.0f;
bool ffActivo = true;

const unsigned long PERIODO_PID_MS = 25;       // lectura del sensor y PID: 40 Hz
const unsigned long PERIODO_REPORTE_MS = 250;  // impresion por serial: 4 Hz

// Ranuras (o aspas) del disco encoder. Los discos tipicos traen 20; cambialo
// aqui o en caliente con el comando r.
uint16_t ranurasPorVuelta = 20;

// Ancho minimo entre flancos validos. Filtra los rebotes del comparador LM393
// en el borde de cada ranura. 250 us admiten hasta 4000 pulsos/s (12000 RPM
// con un disco de 20 ranuras), de sobra para este motor.
const unsigned long MIN_PULSO_US = 250;

// Sin pulsos durante este tiempo se considera el motor detenido.
const unsigned long TIEMPO_SIN_PULSOS_US = 300000UL;

// La linea se arma caracter por caracter y se procesa unicamente al recibir el
// fin de linea: el monitor serial envia cada tecla por separado, asi que
// cualquier procesado por tiempo partiria "10" en un 1 y luego un 0.
const uint8_t LARGO_BUFFER = 24;
char buffer[LARGO_BUFFER];
uint8_t indiceBuffer = 0;

// --- Estado del control ---
bool pidActivo = false;
float setpointRpm = 0.0f;       // con signo: negativo = sentido inverso
float salidaPorcentaje = 0.0f;  // PWM aplicado, con signo
float rpmMedida = 0.0f;

// Constantes del PID. Valores de arranque conservadores: PI puro, sin
// derivativo, que es lo mas estable para empezar a sintonizar.
float kp = 3.0f;
float ki = 40.0f;
float kd = 0.0f;

const float KP_MIN = 1.0f, KP_MAX = 10.0f;
const float KI_MIN = 0.0f, KI_MAX = 200.0f;
const float KD_MIN = 0.0f, KD_MAX = 2.0f;

float integral = 0.0f;
float rpmPrevia = 0.0f;
float errorActualPct = 0.0f;

bool reporteActivo = true;
unsigned long ultimoControlMs = 0;
unsigned long ultimoReporteMs = 0;
unsigned long pulsosAcumulados = 0;

// Referencia temporal de la medicion: instante del ultimo flanco procesado.
unsigned long anclaUs = 0;
bool anclaValida = false;

// Ultimas 5 medidas crudas, para la mediana que filtra picos aislados.
const uint8_t N_MEDIANA = 5;
float hist[N_MEDIANA] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Compartidas con la ISR: volatile para que el compilador no las cachee en un
// registro y las relea de memoria en cada acceso.
volatile unsigned long contadorPulsos = 0;
volatile unsigned long tUltimoPulsoUs = 0;

void mostrarAyuda();
void procesarLinea(char *linea);
void aplicarSalida(float porcentaje);
void actualizarMedicion();
float medianaHist();
void controlPid(float dtS);
void reiniciarPid();
void imprimirEstado(const __FlashStringHelper *prefijo);
void reportar();
void contarPulso();

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_SENSOR, INPUT);  // el modulo ya entrega un nivel digital firme

  Serial.begin(BAUDIOS);
  aplicarSalida(0.0f);  // arrancar siempre con el motor detenido

  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), contarPulso, FALLING);
  ultimoControlMs = millis();
  ultimoReporteMs = ultimoControlMs;

  mostrarAyuda();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (indiceBuffer > 0) {  // ignora el segundo caracter de un CR+LF
        buffer[indiceBuffer] = '\0';
        procesarLinea(buffer);
        indiceBuffer = 0;
      }
    } else if (indiceBuffer < LARGO_BUFFER - 1) {
      buffer[indiceBuffer++] = c;
    }
    // Si la trama excede el buffer se ignoran los caracteres sobrantes.
  }

  unsigned long ahoraMs = millis();

  if (ahoraMs - ultimoControlMs >= PERIODO_PID_MS) {
    // dt real, no el nominal: si el loop se retrasa, el integral y el
    // derivativo se calculan sobre el tiempo que de verdad transcurrio.
    float dtS = (float)(ahoraMs - ultimoControlMs) / 1000.0f;
    ultimoControlMs = ahoraMs;

    actualizarMedicion();
    if (pidActivo) {
      controlPid(dtS);
    }
  }

  if (ahoraMs - ultimoReporteMs >= PERIODO_REPORTE_MS) {
    ultimoReporteMs = ahoraMs;
    reportar();
  }
}

// ISR: lo mas corta posible. Descarta picos de ruido y rebotes, y suma.
void contarPulso() {
  // Un pico inducido por el PWM dura pocos microsegundos; una ranura real
  // mantiene la linea en bajo cientos. Si tras unos us volvio a alto, era ruido.
  delayMicroseconds(8);
  if (PIND & _BV(PD2)) {  // D2 = PD2 en el Uno
    return;
  }
  unsigned long ahoraUs = micros();
  if (ahoraUs - tUltimoPulsoUs < MIN_PULSO_US) {
    return;
  }
  tUltimoPulsoUs = ahoraUs;
  contadorPulsos++;
}

void actualizarMedicion() {
  unsigned long pulsos;
  unsigned long tUltimo;

  // Lectura atomica: son variables de 4 bytes en un micro de 8 bits, asi que
  // sin esto una interrupcion podria colarse a mitad de la copia.
  noInterrupts();
  pulsos = contadorPulsos;
  contadorPulsos = 0;
  tUltimo = tUltimoPulsoUs;
  interrupts();

  pulsosAcumulados += pulsos;

  if (pulsos > 0) {
    if (!anclaValida) {
      // Primer pulso tras un arranque: solo se fija la referencia, todavia no
      // hay un intervalo valido que medir.
      anclaUs = tUltimo;
      anclaValida = true;
    } else {
      unsigned long dtUs = tUltimo - anclaUs;
      anclaUs = tUltimo;
      if (dtUs > 0) {
        // Se mide de flanco a flanco, no de borde a borde de la ventana: asi
        // no hay error de cuantizacion por contar pulsos en un intervalo fijo,
        // que a 25 ms serian saltos de 120 RPM.
        float cruda = (60000000.0f * (float)pulsos) /
                      ((float)dtUs * (float)ranurasPorVuelta);
        // Desplaza el historial y agrega la medida nueva al final.
        for (uint8_t i = 0; i < N_MEDIANA - 1; i++) {
          hist[i] = hist[i + 1];
        }
        hist[N_MEDIANA - 1] = cruda;
        // Mediana de 5: hasta dos picos entre las ultimas 5 nunca pasan al PID.
        rpmMedida = medianaHist();
      }
    }
  } else if (micros() - anclaUs > TIEMPO_SIN_PULSOS_US) {
    rpmMedida = 0.0f;
    for (uint8_t i = 0; i < N_MEDIANA; i++) {
      hist[i] = 0.0f;  // sin historia vieja al rearrancar
    }
    anclaValida = false;
  }
}

// Mediana del historial: ordena una copia por insercion (son solo 5 valores)
// y devuelve el valor central.
float medianaHist() {
  float orden[N_MEDIANA];
  for (uint8_t i = 0; i < N_MEDIANA; i++) {
    float v = hist[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && orden[j] > v) {
      orden[j + 1] = orden[j];
      j--;
    }
    orden[j + 1] = v;
  }
  return orden[N_MEDIANA / 2];
}

void controlPid(float dtS) {
  float spAbs = fabs(setpointRpm);

  if (spAbs < 1.0f) {  // setpoint cero: parar y limpiar el integrador
    integral = 0.0f;
    errorActualPct = 0.0f;
    rpmPrevia = rpmMedida;
    aplicarSalida(0.0f);
    return;
  }

  // Prealimentacion: el PWM que, segun la recta entre la zona muerta y
  // rpmMax, ya deberia dar la velocidad pedida. Asi el PID solo corrige lo que
  // el modelo no acierta, en vez de construir todo el esfuerzo desde el
  // integrador y atravesar la zona muerta a base de acumular error.
  float ff = 0.0f;
  if (ffActivo) {
    ff = constrain(pwmMin + spAbs * (100.0f - pwmMin) / rpmMax, 0.0f, 100.0f);
  }

  // Error normalizado a puntos porcentuales del fondo de escala: con esto las
  // constantes viven en los rangos pedidos en vez de tener que ser diminutas
  // para compensar errores de cientos de RPM.
  float errorPct = (spAbs - rpmMedida) * 100.0f / rpmMax;
  errorActualPct = errorPct;

  float p = kp * errorPct;

  // Derivativo sobre la medicion, no sobre el error: evita el salto brusco de
  // salida cada vez que se cambia el setpoint.
  float d = -kd * ((rpmMedida - rpmPrevia) * 100.0f / rpmMax) / dtS;

  float integralTent = integral + ki * errorPct * dtS;
  float salidaTent = ff + p + integralTent + d;

  // Anti-windup: el integrador solo acumula si la salida no esta saturada, o
  // si el error ya empuja de vuelta hacia el rango util. Sin esto, arrancar
  // con el motor frenado cargaria el integrador hasta valores enormes y el
  // motor no obedeceria al bajar el setpoint. La saturacion se evalua con el
  // ff incluido: el ff ya ocupa parte del rango, y sin contarlo el integrador
  // seguiria cargandose con la salida real ya clavada en 100 %.
  bool satAlta = (salidaTent > 100.0f);
  bool satBaja = (salidaTent < 0.0f);
  if ((!satAlta && !satBaja) ||
      (satAlta && errorPct < 0.0f) ||
      (satBaja && errorPct > 0.0f)) {
    integral = integralTent;
  }

  float salida = constrain(ff + p + integral + d, 0.0f, 100.0f);
  rpmPrevia = rpmMedida;

  // El PID regula la magnitud; el sentido lo decide el signo del setpoint.
  aplicarSalida(setpointRpm >= 0.0f ? salida : -salida);
}

void reiniciarPid() {
  integral = 0.0f;
  rpmPrevia = rpmMedida;
  errorActualPct = 0.0f;
}

// Escribe en el puente H sin imprimir nada: la llama el PID 40 veces por
// segundo y cualquier Serial aqui inundaria el monitor.
void aplicarSalida(float porcentaje) {
  porcentaje = constrain(porcentaje, -100.0f, 100.0f);
  salidaPorcentaje = porcentaje;

  if (porcentaje > 0.0f) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (porcentaje < 0.0f) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    // Ambas entradas en bajo: el motor queda libre (sin freno).
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  uint8_t pwm = (uint8_t)(fabs(porcentaje) * 255.0f / 100.0f + 0.5f);
  analogWrite(PIN_ENA, pwm);
}

void imprimirEstado(const __FlashStringHelper *prefijo) {
  Serial.print(prefijo);
  if (pidActivo) {
    Serial.print(F("  SP: "));
    Serial.print(setpointRpm, 0);
    Serial.print(F(" RPM"));
  }
  Serial.print(F("  PWM: "));
  Serial.print(salidaPorcentaje, 1);
  Serial.print(F(" %  sentido: "));
  if (salidaPorcentaje > 0.0f) {
    Serial.println(F("horario"));
  } else if (salidaPorcentaje < 0.0f) {
    Serial.println(F("antihorario"));
  } else {
    Serial.println(F("detenido"));
  }
}

void reportar() {
  unsigned long pulsos = pulsosAcumulados;
  pulsosAcumulados = 0;

  if (!reporteActivo) {
    return;
  }

  if (pidActivo) {
    Serial.print(F("[PID] SP: "));
    Serial.print(fabs(setpointRpm), 0);
    Serial.print(F("  RPM: "));
    Serial.print(rpmMedida, 1);
    Serial.print(F("  PWM: "));
    Serial.print(fabs(salidaPorcentaje), 1);
    Serial.print(F(" %  err: "));
    Serial.print(errorActualPct * rpmMax / 100.0f, 1);
    Serial.print(F("  pulsos: "));
    Serial.println(pulsos);
  } else {
    Serial.print(F("[MAN] RPM: "));
    Serial.print(rpmMedida, 1);
    Serial.print(F("  PWM: "));
    Serial.print(fabs(salidaPorcentaje), 1);
    Serial.print(F(" %  pulsos: "));
    Serial.println(pulsos);
  }
}

void mostrarAyuda() {
  Serial.println();
  Serial.println(F("=== Control de motor DC: PWM manual + PID de RPM ==="));
  Serial.println(F("  rpm <v>  -> setpoint en RPM y activa el PID (-max a max)"));
  Serial.println(F("  pwm <v>  -> PWM fijo y desactiva el PID (-100 a 100)"));
  Serial.println(F("  <v>      -> atajo de 'pwm <v>'"));
  Serial.println(F("  kp <v>   -> proporcional (1 a 10)"));
  Serial.println(F("  ki <v>   -> integral (0 a 200)"));
  Serial.println(F("  kd <v>   -> derivativo (0 a 2)"));
  Serial.println(F("  max <v>  -> RPM del motor con PWM al 100 % (10 a 4000)"));
  Serial.println(F("  zm <v>   -> zona muerta, PWM minimo del ff (0 a 90)"));
  Serial.println(F("  ff       -> activar/desactivar la prealimentacion"));
  Serial.println(F("  s        -> detener"));
  Serial.println(F("  r <v>    -> ranuras por vuelta del disco"));
  Serial.println(F("  m        -> activar/desactivar el reporte"));
  Serial.println(F("  ?        -> esta ayuda"));
  Serial.print(F("Kp="));
  Serial.print(kp, 2);
  Serial.print(F("  Ki="));
  Serial.print(ki, 2);
  Serial.print(F("  Kd="));
  Serial.print(kd, 2);
  Serial.print(F("  ranuras="));
  Serial.print(ranurasPorVuelta);
  Serial.print(F("  modo="));
  Serial.println(pidActivo ? F("PID") : F("MANUAL"));
  Serial.print(F("max="));
  Serial.print(rpmMax, 1);
  Serial.print(F("  zm="));
  Serial.print(pwmMin, 1);
  Serial.print(F("  ff="));
  Serial.println(ffActivo ? F("activada") : F("desactivada"));
  Serial.println();
}

// Informa un valor fuera de rango de forma uniforme.
static void errorRango(const __FlashStringHelper *que, float minimo, float maximo) {
  Serial.print(F("Fuera de rango: "));
  Serial.print(que);
  Serial.print(F(" debe ir de "));
  Serial.print(minimo, 2);
  Serial.print(F(" a "));
  Serial.println(maximo, 2);
}

void procesarLinea(char *linea) {
  while (*linea == ' ' || *linea == '\t') {
    linea++;
  }

  if (*linea == '\0') {
    return;
  }

  if (*linea == '?') {
    mostrarAyuda();
    return;
  }

  // Se separa el comando (letras) del argumento (numero), de modo que "rpm800",
  // "rpm 800" y "rpm=800" se acepten por igual.
  char comando[8];
  uint8_t n = 0;
  while (isalpha((unsigned char)*linea) && n < sizeof(comando) - 1) {
    comando[n++] = (char)tolower((unsigned char)*linea);
    linea++;
  }
  comando[n] = '\0';

  while (*linea == ' ' || *linea == '\t' || *linea == '=' || *linea == ':') {
    linea++;
  }

  char *fin = NULL;
  float valor = strtod(linea, &fin);
  bool hayNumero = (fin != linea);

  // Comandos sin argumento.
  if (strcmp(comando, "s") == 0 || strcmp(comando, "stop") == 0) {
    pidActivo = false;
    setpointRpm = 0.0f;
    reiniciarPid();
    aplicarSalida(0.0f);
    imprimirEstado(F("[MAN] detenido, PID desactivado."));
    return;
  }

  // strcmp exacto: "max" y "zm" no caen aqui aunque contengan una m.
  if (strcmp(comando, "m") == 0) {
    reporteActivo = !reporteActivo;
    Serial.print(F("Reporte: "));
    Serial.println(reporteActivo ? F("activado") : F("desactivado"));
    return;
  }

  if (strcmp(comando, "ff") == 0) {
    ffActivo = !ffActivo;
    Serial.print(F("Prealimentacion: "));
    Serial.println(ffActivo ? F("activada") : F("desactivada"));
    return;
  }

  // De aqui en adelante todos los comandos necesitan un valor.
  if (!hayNumero) {
    Serial.print(F("Falta el valor numerico en: "));
    Serial.println(buffer);
    return;
  }

  if (strcmp(comando, "rpm") == 0) {
    if (valor < -rpmMax || valor > rpmMax) {
      errorRango(F("el setpoint"), -rpmMax, rpmMax);
      return;
    }
    bool veniaDeManual = !pidActivo;
    setpointRpm = valor;
    pidActivo = true;
    if (veniaDeManual) {
      reiniciarPid();  // arranque limpio del integrador al entrar al PID
    }
    imprimirEstado(F("[PID] activado."));
    return;
  }

  // n == 0 significa que la linea era solo un numero: atajo de "pwm <v>".
  if (strcmp(comando, "pwm") == 0 || n == 0) {
    if (valor < -100.0f || valor > 100.0f) {
      errorRango(F("el PWM"), -100.0f, 100.0f);
      return;
    }
    pidActivo = false;
    setpointRpm = 0.0f;
    reiniciarPid();
    aplicarSalida(valor);
    imprimirEstado(F("[MAN] PID desactivado."));
    return;
  }

  if (strcmp(comando, "kp") == 0) {
    if (valor < KP_MIN || valor > KP_MAX) {
      errorRango(F("Kp"), KP_MIN, KP_MAX);
      return;
    }
    kp = valor;
    Serial.print(F("Kp = "));
    Serial.println(kp, 2);
    return;
  }

  if (strcmp(comando, "ki") == 0) {
    if (valor < KI_MIN || valor > KI_MAX) {
      errorRango(F("Ki"), KI_MIN, KI_MAX);
      return;
    }
    ki = valor;
    integral = 0.0f;  // lo acumulado ya no corresponde a la nueva Ki
    Serial.print(F("Ki = "));
    Serial.println(ki, 2);
    return;
  }

  if (strcmp(comando, "kd") == 0) {
    if (valor < KD_MIN || valor > KD_MAX) {
      errorRango(F("Kd"), KD_MIN, KD_MAX);
      return;
    }
    kd = valor;
    Serial.print(F("Kd = "));
    Serial.println(kd, 2);
    return;
  }

  if (strcmp(comando, "max") == 0) {
    if (valor < RPMMAX_MIN || valor > RPMMAX_MAX) {
      errorRango(F("max"), RPMMAX_MIN, RPMMAX_MAX);
      return;
    }
    rpmMax = valor;
    reiniciarPid();  // el error normalizado cambio de escala: integrador a cero
    Serial.print(F("max = "));
    Serial.println(rpmMax, 1);
    return;
  }

  if (strcmp(comando, "zm") == 0) {
    if (valor < ZM_MIN || valor > ZM_MAX) {
      errorRango(F("zm"), ZM_MIN, ZM_MAX);
      return;
    }
    pwmMin = valor;
    Serial.print(F("zm = "));
    Serial.println(pwmMin, 1);
    return;
  }

  if (strcmp(comando, "r") == 0) {
    if (valor < 1.0f || valor > 1000.0f) {
      errorRango(F("las ranuras"), 1.0f, 1000.0f);
      return;
    }
    ranurasPorVuelta = (uint16_t)valor;
    Serial.print(F("Ranuras por vuelta: "));
    Serial.println(ranurasPorVuelta);
    return;
  }

  Serial.print(F("Comando no reconocido: "));
  Serial.println(buffer);
}
```

## Anexo B. Configuración de PlatformIO (`platformio.ini`)

```ini
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 9600
monitor_echo = yes
```

## Anexo C. Tareas de VS Code (`.vscode/tasks.json`)

```jsonc
{
    // Tareas de PlatformIO invocadas por ruta absoluta, para no depender de que
    // la extension registre sus botones ni de tener "pio" en el PATH.
    "version": "2.0.0",
    "tasks": [
        {
            "label": "PIO: Build",
            "type": "shell",
            "command": "${userHome}/.platformio/penv/Scripts/platformio.exe",
            "args": ["run"],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": "$gcc",
            "presentation": {
                "reveal": "always",
                "panel": "dedicated"
            }
        },
        {
            "label": "PIO: Upload",
            "type": "shell",
            "command": "${userHome}/.platformio/penv/Scripts/platformio.exe",
            "args": ["run", "--target", "upload"],
            "group": "build",
            "problemMatcher": "$gcc",
            "presentation": {
                "reveal": "always",
                "panel": "dedicated"
            }
        },
        {
            "label": "PIO: Monitor",
            "type": "shell",
            "command": "${userHome}/.platformio/penv/Scripts/platformio.exe",
            "args": ["device", "monitor"],
            "problemMatcher": [],
            "presentation": {
                "reveal": "always",
                "panel": "dedicated"
            }
        },
        {
            "label": "PIO: Upload y Monitor",
            "type": "shell",
            "command": "${userHome}/.platformio/penv/Scripts/platformio.exe",
            "args": ["run", "--target", "upload", "--target", "monitor"],
            "group": "build",
            "problemMatcher": "$gcc",
            "presentation": {
                "reveal": "always",
                "panel": "dedicated"
            }
        },
        {
            "label": "PIO: Listar puertos",
            "type": "shell",
            "command": "${userHome}/.platformio/penv/Scripts/platformio.exe",
            "args": ["device", "list"],
            "problemMatcher": [],
            "presentation": {
                "reveal": "always",
                "panel": "dedicated"
            }
        }
    ]
}
```
