# Documentación técnica del simulador de control PID y de la interfaz de control del motor DC físico

**Asignatura:** Control Inteligente, octavo semestre
**Proyecto:** `Simulador_motor` (archivo principal `simulador.py`)
**Periodo documentado:** del 9 al 11 de septiembre de 2026, con actualizaciones el 1 de octubre de 2026 (sección 6.12) y el 2 de octubre de 2026 (sección 6.13, control difuso)
**Propósito de este documento:** reunir, en forma de texto continuo, todo el contexto técnico, las decisiones de diseño, los diagnósticos, las mediciones y los resultados obtenidos durante el desarrollo, como base para redactar más adelante un informe formal.

---

## 1. Alcance y procedencia de los datos

Este documento recoge el trabajo realizado sobre un simulador de control PID de un motor de corriente directa que ya existía al inicio del periodo documentado, y su extensión para gobernar un motor real mediante un Arduino, un puente H y un sensor óptico de ranura. Se describe primero el simulador tal como se encontró, después el hardware y el firmware del motor físico, y a continuación la cronología completa de los cambios, con el problema que motivó cada uno, el análisis realizado, la solución adoptada y la forma en que se verificó.

Para que el informe final pueda distinguir con claridad qué es un resultado comprobado y qué no, conviene tener presentes tres tipos de datos que aparecen a lo largo del texto. Los primeros son **mediciones automáticas**: valores obtenidos ejecutando el propio código del simulador en el equipo de desarrollo (tiempos de arranque, métricas de las respuestas simuladas, resultados de las pruebas). Los segundos son **observaciones del ensayo real**: lo que mostraban las capturas de pantalla de la gráfica del motor físico y los datos que reportó el usuario (por ejemplo, el umbral de la zona muerta). Los terceros son **estimaciones y supuestos**: valores razonados pero no medidos directamente, como la constante de tiempo mecánica del motor o el voltaje efectivo en bornes. Cada vez que un dato pertenece a esta última categoría, el texto lo indica.

## 2. Descripción general del sistema

El sistema tiene dos partes que conviven en una misma aplicación de escritorio. La primera es un **simulador** de un lazo cerrado de velocidad: un controlador PID calcula el voltaje de armadura de un motor DC modelado con sus ecuaciones eléctrica y mecánica, y la aplicación muestra en tiempo real el diagrama de bloques, las señales del lazo, un modelo 3D del motor girando y la dinámica interna de la planta. La segunda es una **interfaz de control de un motor físico**: la misma aplicación se conecta por el puerto serie a un Arduino Uno que solo mide la velocidad y aplica el PWM que recibe, mientras el PID de velocidad se calcula en la propia aplicación. Esta muestra la velocidad medida por el encoder y el PWM aplicado, dibuja la respuesta RPM contra el tiempo con el setpoint como referencia y el PWM debajo, y permite configurar el setpoint, las ganancias, la prealimentación, la zona muerta, las RPM máximas y los límites sin tocar el código del microcontrolador. Hasta la sección 6.11 el PID corría en el firmware; la sección 6.12 describe el traslado a la aplicación. Desde la sección 6.13, el lazo cerrado del motor real puede calcularlo, en lugar del PID, un controlador difuso (Fuzzy) tipo Mamdani, cuya vista muestra en vivo la fuzzificación, la base de reglas, la inferencia y la defuzzificación.

La aplicación se organiza en tres vistas seleccionables desde la cabecera: **"Control PID"** y **"Dinamica del Motor"**, que corresponden al simulador, y **"Control de Motor Fisico"**, que corresponde al motor real. Dentro de esta última, una vez conectada la placa, dos botones (**"Control PID"** y **"Control Fuzzy"**) eligen qué controlador cierra el lazo y qué disposición de la vista se muestra.

## 3. Entorno de desarrollo y ejecución

La aplicación está escrita en Python y se ejecuta dentro de un entorno virtual propio del proyecto (`.venv`). Las versiones instaladas son Python 3.14.3, PyQt5 5.15.11 sobre Qt 5.15.2, pyqtgraph 0.14.0 y numpy 2.5.2. El archivo `requirements.txt` declara las dependencias mínimas (`PyQt5>=5.15`, `pyqtgraph>=0.13`, `numpy>=1.24`). El archivo `ejecutar.bat` lanza el simulador dentro del entorno virtual y, si este no existe, lo crea e instala las dependencias antes de arrancar.

La comunicación serie usa el módulo `PyQt5.QtSerialPort`, que viene incluido en la distribución oficial de PyQt5, por lo que no fue necesario añadir ninguna dependencia nueva (en particular, no se usa `pyserial`). Si en otra instalación ese módulo faltara, la vista del motor físico lo indica y el resto del simulador sigue funcionando.

Todo el código está en un único archivo, `simulador.py`, que al inicio del periodo tenía 2104 líneas, 3218 al cierre de la sección 6.11, 3636 tras la sección 6.12 y 4927 tras la sección 6.13. El controlador difuso se implementó directamente con numpy, sin bibliotecas de lógica difusa (por ejemplo, no se usa `scikit-fuzzy`), de modo que tampoco añadió dependencias.

## 4. El simulador de partida

### 4.1 Modelo de la planta

El motor se modela como un motor DC controlado por armadura, con dos variables de estado: la corriente de armadura $i_a$ y la velocidad angular $\omega$. Las ecuaciones son la de la malla eléctrica y la del equilibrio mecánico del rotor:

$$L_a \frac{di_a}{dt} = V_a - R_a\, i_a - K_e\, \omega$$

$$J \frac{d\omega}{dt} = K_t\, i_a - B\,\omega - T_L$$

donde $V_a$ es el voltaje de armadura (la salida del controlador), $R_a$ y $L_a$ la resistencia y la inductancia de armadura, $K_e$ la constante de fuerza contraelectromotriz, $K_t$ la constante de par, $J$ el momento de inercia, $B$ el coeficiente de fricción viscosa y $T_L$ el par de carga. El par del motor es $\tau_m = K_t\, i_a$ y la salida realimentada es $y(t) = \omega(t)$.

El modelo se integra con Runge-Kutta de cuarto orden, manteniendo $V_a$ constante durante cada periodo de muestreo del controlador (retenedor de orden cero). Como las constantes de tiempo eléctrica ($\tau_e = L_a/R_a$) y mecánica ($\tau_m = J/(B + K_t K_e/R_a)$) pueden ser mucho menores que el periodo de muestreo, cada periodo se subdivide en $n = \lceil T_s / (0.15 \min(\tau_e, \tau_m)) \rceil$ subpasos, con un tope de 400. Si la integración diverge, la simulación se detiene y se muestra un aviso.

### 4.2 Controlador PID

El controlador es un PID en paralelo, $u = P + I + D$, con las siguientes características. La acción proporcional es $P = K_p\, e$. La acción integral acumula $K_i\, e\, T_s$ en cada muestra, con **anti-windup por integración condicional**: si la salida está saturada y el error empuja hacia la misma saturación, el integrador se congela. La acción derivativa se calcula, por defecto, **sobre la medición** ($-dy/dt$) en lugar de sobre el error, para evitar el salto de la salida al cambiar la referencia, y se filtra con un pasa-bajas de primer orden de constante $T_f = K_d/N$. La salida se limita a $\pm V_{a,\max}$. El anti-windup, la derivada sobre la medición y el parámetro $N$ se pueden modificar desde la interfaz.

La referencia puede ser un escalón, una onda cuadrada simétrica o una senoidal, con amplitud en rad/s (la interfaz muestra también su equivalente en rpm) y periodo configurable.

### 4.3 Interfaz

La ventana tiene una cabecera con el título, el selector de vistas, un indicador de estado y los botones "Iniciar simulacion", "Detener" y "Reiniciar". La columna izquierda agrupa los parámetros de la planta, las condiciones iniciales, la referencia y los parámetros de simulación (periodo de muestreo, ventana de tiempo y velocidad de simulación). Los campos que definen el modelo se bloquean mientras la simulación está en marcha; las ganancias, la referencia y el par de carga se pueden ajustar en caliente.

La vista "Control PID" muestra el diagrama de bloques del lazo cerrado con el valor vivo de cada señal y tres gráficas: el error $e(t)$, la salida del controlador $u(t)$ con las líneas de saturación, y la salida del proceso $y(t)$ junto a la referencia. La vista "Dinamica del Motor" muestra un diagrama del modelo físico (circuito de armadura y balance de pares), las ecuaciones del modelo, las gráficas de la corriente de armadura y de la velocidad angular, y un panel con el balance de pares del rotor.

La columna derecha contiene un modelo 3D del motor, las señales en tiempo real y los controles de las ganancias del PID (deslizador sincronizado con un campo numérico). El modelo 3D se dibuja por software con QPainter, mediante proyección en perspectiva, sombreado plano por cara y el algoritmo del pintor, por lo que no depende de OpenGL ni del controlador de video. La cámara se gira arrastrando con el ratón, y un selector de "Giro visual" escala la rotación mostrada para que sea visible a velocidades altas sin afectar a la simulación.

### 4.4 Diseño del bucle de tiempo real

La aplicación es de un solo hilo, así que el diseño cuida que cada fotograma deje tiempo libre al bucle de eventos de Qt; de lo contrario Windows marcaría la ventana como "no responde". El bucle de simulación usa un temporizador de un solo disparo que se reprograma al terminar cada fotograma descontando lo que este costó (periodo nominal de 16 ms). Las curvas y la telemetría se refrescan como mucho cada 33 ms (unos 30 fotogramas por segundo) y la vista 3D cada 50 ms, mediante un limitador de repintado que funde en uno solo los pedidos que llegan dentro de ese intervalo. Cada curva se dibuja con 700 puntos como máximo y el número de integraciones RK4 por fotograma se acota a 6000: si el equipo no da abasto, se sacrifica tiempo simulado y nunca la respuesta de la interfaz. El suavizado de pyqtgraph está desactivado porque duplica el coste de rasterizar cada curva.

### 4.5 Valores por defecto originales

Al inicio del periodo, el simulador arrancaba con $V_{a,\max} = 24$ V, $R_a = 1\ \Omega$, $L_a = 0.010$ H, $K_t = K_e = 0.100$, $J = 0.010$ kg·m², $B = 0.005$ N·m·s, $T_L = 0$, referencia de onda cuadrada de ±100 rad/s con periodo de 4 s, $T_s = 1$ ms, ventana de 6 s, velocidad 1× y ganancias $K_p = 0.15$, $K_i = 2.0$, $K_d = 0.005$. Estos valores se sustituyeron al final del periodo por los del motor equivalente al real (sección 6.10).

## 5. Hardware y firmware del motor físico

### 5.1 Montaje

El motor físico es un motorreductor DC (el modelo exacto no se documentó) gobernado por un Arduino Uno a través de un puente H L298N, con un sensor óptico de ranura FC-03 (módulo con comparador LM393) que lee un disco encoder de 20 ranuras. Las conexiones son las siguientes.

| Señal | Pin del Arduino | Función |
|---|---|---|
| L298N ENA | D3 | Salida PWM (~490 Hz, Timer2) |
| L298N IN1 | D7 | Sentido de giro |
| L298N IN2 | D8 | Sentido de giro |
| FC-03 D0 | D2 (INT0) | Pulsos del encoder, por interrupción en flanco de bajada |
| FC-03 VCC / GND | 5 V / GND | Alimentación del sensor |

Todos los GND (Arduino, L298N y batería) están unidos. El voltaje de la fuente no se documentó; en la simulación se supuso un voltaje efectivo de 6 V en bornes con PWM al 100 %.

### 5.2 Firmware original

Esta sección describe el firmware tal como era durante las secciones 6.1 a 6.11. En la sección 6.12 se sustituyó por un sketch que solo mide y actúa.

El sketch original implementa control de velocidad con dos modos: PWM manual y PID de RPM. Se comunica a 9600 baudios con órdenes de una línea terminadas en Enter. El lazo de control corre cada 50 ms (20 Hz) usando el tiempo real transcurrido, y el reporte por serie se imprime cada 500 ms.

La velocidad se mide **de flanco a flanco**: en cada ventana de control se cuentan los pulsos recibidos y se toma el intervalo entre el último flanco de la ventana anterior y el último de la actual, de modo que

$$\text{rpm} = \frac{60\,000\,000 \cdot \text{pulsos}}{\Delta t_{\mu s} \cdot \text{ranuras}}$$

Este método evita el error de cuantización que tendría contar pulsos en una ventana fija (con 50 ms serían saltos de 60 rpm). La interrupción descarta flancos separados menos de 250 µs del anterior, para filtrar rebotes del comparador, y si pasan 300 ms sin pulsos el motor se da por detenido.

El PID del firmware trabaja con el error **normalizado** al fondo de escala `RPM_MAX = 4000`: $e_{\%} = (|SP| - \text{rpm}) \cdot 100 / \text{RPM\_MAX}$, para que las constantes vivan en rangos cómodos (Kp de 1 a 10, Ki de 0 a 200, Kd de 0 a 2). La derivada se toma sobre la medición, el integrador tiene anti-windup y la salida se limita de 0 a 100 %. El PID regula la magnitud de la velocidad, porque el sensor de ranura no distingue el sentido, y el signo del setpoint decide el sentido de giro. Cambiar Ki reinicia el integrador. Con salida cero, ambas entradas del puente quedan en bajo y el motor gira libre, sin freno. Los valores de arranque son Kp = 3, Ki = 40 y Kd = 0.

| Orden | Efecto |
|---|---|
| `rpm <v>` | Setpoint en RPM (−4000 a 4000) y activa el PID |
| `pwm <v>` | PWM fijo (−100 a 100) y desactiva el PID |
| `<v>` | Atajo de `pwm <v>` |
| `kp <v>`, `ki <v>`, `kd <v>` | Constantes del PID (1–10, 0–200, 0–2) |
| `s` | Detiene el motor y desactiva el PID |
| `r <v>` | Ranuras por vuelta del disco |
| `m` | Activa o desactiva el reporte periódico |
| `?` | Muestra la ayuda |

El reporte periódico tiene dos formatos, según el modo:

```
[PID] SP: 1500  RPM: 1493.2  PWM: 62.3 %  err: 6.8  pulsos: 245
[MAN] RPM: 0.0  PWM: 0.0 %  pulsos: 0
```

Además, cada orden de consigna produce una línea de confirmación que incluye el sentido (`sentido: horario`, `antihorario` o `detenido`).

## 6. Cronología del desarrollo

| Fecha | Fase | Motivo |
|---|---|---|
| 9 sep | 6.1 Vista "Control de Motor Fisico" | Conectar y controlar el motor real desde la aplicación |
| 10 sep | 6.2 Arranque lento | La aplicación tardaba ~26 s en aparecer |
| 10 sep | 6.3 Gráfica RPM vs tiempo | Visualizar el error y la respuesta al escalón |
| 10 sep | 6.4 Primer ensayo real | Diagnóstico de una respuesta que no seguía el setpoint |
| 10 sep | 6.5 Correcciones de hardware y firmware | Ruido en la medición |
| 10 sep | 6.6 Segundo ensayo real | Oscilación creciente con el setpoint |
| 10 sep | 6.7 Velocidad máxima y zona muerta | Escala real del motor |
| 10 sep | 6.8 Mejoras del firmware | Escala, prealimentación y filtrado |
| 11 sep | 6.9 Motor equivalente y sintonía | Simulación parecida al motor real |
| 11 sep | 6.10 Autoescalado y valores por defecto | Gráficas "enloquecidas" tras el asentamiento |
| 11 sep | 6.11 Historial completo y avance lento | Conservar el transitorio inicial y verlo formarse |
| 1 oct | 6.12 El lazo de control pasa a la aplicación | El firmware quedó solo como sensor y actuador |
| 2 oct | 6.13 Control difuso (Fuzzy) del motor real | Alternativa al PID que muestra paso a paso cómo funciona el control difuso |

### 6.1 Vista "Control de Motor Fisico"

El requisito fue añadir una tercera ventana, con el mismo estilo que las otras dos, dedicada al motor real. Debía conservar únicamente el modelo 3D del motor, mostrar en grande la velocidad angular en RPM junto con el porcentaje de PWM y los pulsos, y mantener los deslizadores de las constantes del PID. Todo ello debía aparecer solo después de elegir el puerto COM en un menú desplegable y pulsar "Conectar", y además se pidió un recuadro con el texto completo recibido y la información del puerto seleccionado.

Para la comunicación se creó la clase `EnlaceSerieMotor`, que encapsula un `QSerialPort`. Abre el puerto en formato 8N1 a los baudios elegidos y activa la línea DTR; eso reinicia la placa, igual que lo hace el monitor serie del IDE de Arduino, y deja el firmware en un estado conocido. La lectura llega por la señal `readyRead`, nunca de forma bloqueante, así que la ventana sigue respondiendo aunque la placa se quede muda. Como un reporte puede llegar partido entre dos lecturas, los bytes se acumulan en un buffer que se corta por líneas; si una trama supera 4096 caracteres sin fin de línea, se descarta. Si se desconecta el cable USB en caliente, el error del puerto cierra el enlace en lugar de dejar la interfaz esperando datos.

Cada línea se interpreta con expresiones regulares que extraen `RPM`, `PWM`, `pulsos`, `SP` y `err`, el modo (`[PID]` o `[MAN]`) y el sentido; "antihorario" se comprueba antes que "horario" porque contiene esa palabra. Como el reporte periódico imprime magnitudes sin signo, el sentido se toma de la última consigna enviada y de las líneas de confirmación.

La vista se construyó con estas partes. Arriba, una **tarjeta de conexión** con el desplegable de puertos (nombre y descripción del dispositivo), un botón ↻ para volver a buscar, el selector de baudios (9600 por defecto) y el botón "Conectar"; mientras no hay enlace, el resto de la página muestra solo un texto que explica lo que aparecerá al conectar. Una vez conectado, aparece el **panel del motor**. La columna izquierda contiene el modelo 3D, que gira integrando el ángulo a partir de las RPM informadas, con su propio selector de giro visual (1:10 por defecto). Debajo está la tira **"Estado del firmware"** (setpoint, modo, sentido y pulsos acumulados) y el **monitor del puerto serie**, que muestra a la izquierda la ficha del puerto (puerto, ubicación, descripción, fabricante, número de serie e identificadores USB) y a la derecha una consola con todo el texto que entra y sale (hasta 600 líneas), un campo para escribir órdenes a mano y los botones "Enviar", "Limpiar" y "Autodesplazar". La columna derecha contiene la **medición del encoder**: la velocidad angular en RPM en tipografía grande, el PWM aplicado y los pulsos por reporte. Debajo va la **consigna del motor**, con campos para el setpoint del PID, el PWM manual y las ranuras por vuelta, cada uno con su botón "Aplicar", más un botón "Detener el motor". Por último están las **ganancias del PID del firmware**, con deslizadores limitados a los rangos que acepta el sketch y valores iniciales iguales a los del firmware (Kp 3, Ki 40, Kd 0).

Varias decisiones responden a detalles del hardware. Como abrir el puerto reinicia la placa, las tres ganancias se envían 2.5 s después de conectar, cuando el firmware ya arrancó. Al arrastrar un deslizador se generan decenas de cambios por segundo, y a 9600 baudios cada orden tarda unos 5 ms en salir, así que los cambios se agrupan y solo se envía el último valor de cada ganancia cada 180 ms; un botón permite reenviar las tres después de un reinicio de la placa. Si pasan más de 2.5 s sin reportes (el firmware reporta cada 500 ms), el indicador pasa a "Conectado · sin datos". Por seguridad, al desconectar o cerrar la ventana se envía la orden `s` antes de soltar el puerto y se espera a que salga: cerrar el puerto no detiene el motor, porque el firmware conserva la última orden. Al entrar en esta vista, la simulación se pausa y se ocultan sus columnas y botones; la etiqueta de estado de la cabecera muestra el estado del enlace y se restituye al volver a las vistas del simulador.

La primera versión del diseño apilaba las lecturas, las consignas y los deslizadores en una sola columna y, con 930 px de alto, los deslizadores del PID quedaban fuera de la vista. El diseño se reorganizó en dos columnas de altura completa, con el monitor bajo la vista 3D, hasta que todo cupo sin desplazamiento.

La verificación se hizo sin placa real: se inyectaron líneas sintéticas del protocolo y se simuló un puerto. Se comprobó la interpretación de seis tipos de líneas, el reensamblado de una trama partida en tres lecturas, el descarte de tramas corruptas, el fallo controlado al abrir un puerto inexistente (COM99), la ficha de un puerto real del equipo y el texto exacto de las órdenes generadas (`rpm -1200`, `pwm 55.0`, `r 20`, `s`, `kp 4.250`, `ki 65.000`, `kd 0.350`, `m`). También se comprobaron el giro del modelo 3D, la aparición del panel solo al conectar y la pausa de la simulación al entrar en la vista.

### 6.2 Corrección del arranque lento

Tras añadir la vista, la aplicación parecía no arrancar. Se midió el tiempo de cada operación y la causa resultó ser la función `QSerialPortInfo.isBusy()`, que se usaba para mostrar si el puerto estaba libre u ocupado. En Windows esa función abre el puerto para averiguarlo, y en los puertos serie virtuales sobre Bluetooth del equipo (COM4 y COM8) cada apertura tardaba **12.8 s**, porque intenta contactar con el dispositivo emparejado. Como la ficha de los siete puertos se calculaba durante la construcción de la ventana, esta tardaba unos **26 s** en aparecer, aunque nunca se fuera a usar el motor real.

La corrección tuvo tres partes. Se eliminó la consulta de `isBusy()`, porque si el puerto está ocupado ya lo indica el error de apertura al conectar. La enumeración de puertos se aplazó hasta la primera vez que se entra en la vista (o hasta pulsar ↻). Y el aviso de fallo de apertura sugiere cerrar el monitor serie del IDE de Arduino, que es el caso más común de puerto ocupado.

| Medida | Antes | Después |
|---|---|---|
| Tiempo hasta ver la ventana | ~26 s | 1.9 s |
| Entrar en la vista del motor físico | — | 34 ms |
| Botón ↻ (volver a buscar puertos) | — | 11 ms |

Se añadió a las pruebas un control que falla si enumerar los puertos vuelve a tardar más de un segundo.

### 6.3 Gráfica RPM contra tiempo con el setpoint

El siguiente requisito fue una gráfica de RPM contra tiempo en la vista del motor físico, con una línea horizontal en el setpoint, para visualizar el error y obtener un dibujo como el de una respuesta clásica al escalón: un sobreimpulso **A** sobre la referencia, el tiempo de subida **t_r** en que la curva cruza por primera vez el setpoint (tramo **B**), un segundo pico menor **C** y el tiempo de establecimiento **t_ss**. Además, a medida que llegaran datos, la gráfica debía mostrarlos todos, "apretándolos".

Se añadió la tarjeta **"Respuesta del motor"** junto al modelo 3D, con la curva de RPM en verde y el setpoint en naranja, dentro de la paleta de la aplicación. El setpoint se dibuja **en escalones**: se registra solo cuando cambia, y así un cambio de consigna a mitad del ensayo se ve como un salto limpio en el instante exacto de la línea de confirmación. En modo manual no hay consigna y la línea se corta. El eje de tiempo no usa una ventana deslizante: va de 0 al último reporte, con un ancho mínimo de 10 s, así que la respuesta completa queda siempre a la vista y se comprime a medida que el ensayo se alarga. Un botón "↺ Reiniciar" borra la gráfica y pone el tiempo en cero, y la gráfica se reinicia también al conectar. En la tira de estado se añadió el recuadro **"ERROR SP − RPM"**, que muestra con signo el error calculado por el firmware cuando el PID está activo.

Para que el coste no creciera con la duración del ensayo, la curva se reduce a unos 2000 puntos con un **diezmado que conserva máximos y mínimos** (función `diezmar_picos`): cada tramo aporta su mínimo y su máximo en orden temporal, de modo que el pico del sobreimpulso nunca se pierde, cosa que sí puede ocurrir tomando una muestra de cada *k*. La gráfica se redibuja como mucho cada 100 ms y solo cuando está a la vista.

Las pruebas usaron una respuesta sintética de segundo orden subamortiguada y comprobaron la conservación de picos (50 000 muestras reducidas a menos de 2050 sin perder un pico aislado), los escalones del setpoint, el corte de la línea en modo manual, la compresión del eje (el mismo ancho pasa de mostrar 0–25 s a mostrar 0–60 s), el límite de refresco y el reinicio. Con 100 000 muestras, cada redibujo cuesta unos 24 ms. El arranque de la aplicación siguió en 1.97 s.

En ese momento se observó que el firmware reporta cada 500 ms, lo que da una curva de solo dos puntos por segundo. Se recomendó bajar el periodo de reporte a unos 100 ms y subir la velocidad del puerto a 115200 baudios, porque a 9600 baudios una línea `[PID]` de unos 66 caracteres ya tarda unos 69 ms en transmitirse.

### 6.4 Primer ensayo real y diagnóstico

La primera gráfica del motor real mostraba tres tramos con setpoint de 1000 rpm, uno de 200 rpm y uno de 500 rpm. Al aplicar el setpoint de 1000 aparecían picos de 1250 y 3200 rpm, tras los cuales la lectura caía y se quedaba entre 250 y 350 rpm, muy por debajo de la consigna; en otro tramo oscilaba entre 300 y 1000 rpm. Con setpoint de 200 rpm el motor no se movió durante unos 12 s. Con 500 rpm, durante un minuto, la lectura se mantuvo entre 200 y 400 rpm con picos aislados de 1400 a 2000 rpm.

El análisis identificó varias causas superpuestas. En primer lugar, un PID con acción integral no puede mantener un error constante durante decenas de segundos, salvo que la salida esté saturada, que Ki sea prácticamente cero o que la medición engañe al integrador. En segundo lugar, los picos aislados que duran una sola muestra no son físicamente compatibles con la inercia del motor: son **pulsos falsos** en la entrada del sensor, inducidos por la conmutación del PWM del L298N (490 Hz) o por el rebote del comparador. La magnitud encaja con esa hipótesis: a 300 rpm reales, con 20 ranuras, entran unos 5 pulsos por ventana de 50 ms, y si el PWM añade unos 25 flancos de ruido el firmware calcula unas 1800 rpm. El filtro de 250 µs no los detiene, porque esos flancos llegan cada 2 ms.

En tercer lugar, los picos interactúan de forma muy desfavorable con un Ki alto. Un pico de 1800 rpm con setpoint de 500 produce un error normalizado de −32.5 %, y con Ki = 40 ese único pico resta al integrador 40 × 32.5 × 0.05 = **65 puntos de PWM** en un solo paso de control. El integrador termina igualando al setpoint el promedio de las lecturas, y como los picos inflan ese promedio, la velocidad real queda por debajo. Además, con un reporte cada 500 ms y un control cada 50 ms, la gráfica muestra solo una de cada diez ventanas que ve el PID, así que es probable que hubiera muchos más picos de los visibles. En cuarto lugar, con las ganancias por defecto (Kp 3, Ki 40) el tiempo integral $T_i = K_p/K_i = 0.075$ s es más rápido que el propio motor, lo que favorece el sobreimpulso y la oscilación. Por último, con setpoint de 200 la acción proporcional aporta solo un 15 % de PWM, dentro de la zona muerta del motor.

Se recomendaron unos valores iniciales conservadores (Kp = 1, Ki = 5, Kd = 0; la derivada amplificaría los picos) y un procedimiento de comprobación:

- Una prueba en modo manual con PWM constante, para ver si los picos aparecen sin el PID.
- Revisar la columna PWM del monitor para descartar la saturación.
- Correcciones de hardware para reducir el ruido.
- Una identificación en lazo abierto para calcular las constantes con datos reales. A partir de un escalón de PWM se miden la ganancia $K$ (ΔRPM/ΔPWM) y la constante de tiempo $\tau$ (63 % del cambio). Con $K_n = K/40$ y una constante de lazo cerrado deseada $\lambda$ entre 0.3 y 0.5 s, se obtiene $K_p = \tau/(K_n \lambda)$ y $K_i = K_p/\tau$.

Hay que advertir que el valor Ki = 5 se calculó suponiendo, como hacía la normalización del firmware, un motor de unos 4000 rpm; esa suposición se corrigió más adelante (secciones 6.6 y 6.7).

### 6.5 Correcciones de hardware y firmware

Con los componentes disponibles (un condensador cerámico de 100 nF y uno electrolítico de 1 µF), y con el cable de D0 del sensor ya trenzado con su GND, se decidió lo siguiente. El **cerámico de 100 nF va soldado directamente entre los bornes del motor**, para frenar el ruido de las escobillas y de la conmutación en su origen. Tiene que ser un condensador no polarizado, porque el puente H invierte la polaridad del motor al cambiar el sentido de giro: un electrolítico en esa posición quedaría polarizado al revés y podría calentarse o reventar. El **electrolítico de 1 µF va entre VCC y GND del módulo FC-03**, respetando la polaridad, para desacoplar la alimentación del sensor, que comparte los 5 V y la tierra con el motor.

Ninguno de los dos se colocó en la salida D0. El módulo tiene una resistencia de pull-up, normalmente de 10 kΩ, y un condensador en paralelo forma con ella un filtro RC con $\tau = 1$ ms (100 nF) o $\tau = 10$ ms (1 µF). A 1000 rpm las ranuras pasan cada 3 ms y a 2000 rpm cada 1.5 ms, así que la señal no alcanzaría a subir y se perderían pulsos; para ese punto haría falta un condensador de 1 a 10 nF. También se recomendó llevar el GND del sensor directamente a un pin GND del Arduino, y no a la bornera del L298N, para que la corriente del motor no circule por la tierra del sensor, uniendo batería, puente y Arduino en un solo punto.

En el firmware se añadieron dos filtros. El primero rechaza los picos dentro de la propia interrupción: un pico inducido por el PWM dura unos pocos microsegundos, mientras que una ranura real mantiene la línea en bajo cientos de microsegundos, así que se espera 8 µs y se descarta el flanco si la línea ya volvió a alto:

```cpp
void contarPulso() {
  delayMicroseconds(8);
  if (PIND & _BV(PD2)) {      // D2 = PD2 en el Uno: si volvio a alto, era ruido
    return;
  }
  unsigned long ahoraUs = micros();
  if (ahoraUs - tUltimoPulsoUs < MIN_PULSO_US) {
    return;
  }
  tUltimoPulsoUs = ahoraUs;
  contadorPulsos++;
}
```

El segundo es una **mediana de las tres últimas medidas** de velocidad, que impide que un pico aislado llegue al PID a cambio de unos 50 ms de retardo:

```cpp
float cruda = (60000000.0f * (float)pulsos) /
              ((float)dtUs * (float)ranurasPorVuelta);
hist[0] = hist[1];
hist[1] = hist[2];
hist[2] = cruda;
rpmMedida = fmaxf(fminf(hist[0], hist[1]),
                  fminf(fmaxf(hist[0], hist[1]), hist[2]));
```

El historial de la mediana se pone a cero cuando el motor se da por detenido. El usuario informó haber aplicado todas estas correcciones.

### 6.6 Segundo ensayo real

La segunda gráfica, de unos 195 s, mostró una mejora clara. Con setpoints de 50 y 75 rpm la curva seguía la consigna limpiamente y sin error permanente. Con 10 rpm la lectura caía a cero a ratos. Desde 100 rpm aparecían picos hacia arriba, y con 250 rpm la lectura se movía entre 130 y 330 rpm con picos de hasta 500. Al bajar la consigna de 150 a 50 rpm, la velocidad tardó unos 12 s en llegar. Un detalle resultó muy revelador: con el mismo setpoint de 75 rpm, el tramo inicial del ensayo tenía picos y el tramo final estaba limpio, lo que indica un fenómeno intermitente y no una propiedad del controlador.

El análisis partió de un principio de la teoría de control lineal: un PI lineal responde igual, en proporción, a cualquier amplitud de consigna, así que si el comportamiento empeora al subir el setpoint hay una no linealidad o un ruido que es proporcional a la velocidad. Los picos iban siempre hacia arriba y medían entre 1.5 y 2 veces la velocidad base, que es la firma de pulsos contados de más: un pulso extra en una ventana con dos pulsos reales multiplica la lectura por 1.5, y en una ventana con uno la duplica. Como el error es un porcentaje de la velocidad, en rpm crece al subirla. A esto se suma la poca resolución del encoder: con 20 ranuras en el eje de salida, a 100 rpm llegan unos 33 pulsos por segundo, es decir, uno o dos por cada ciclo de 50 ms del PID, y cualquier irregularidad del disco aparece como ruido.

Las caídas a cero con 10 rpm tienen una explicación directa: a esa velocidad los pulsos llegan cada 300 ms, que es justo el tiempo sin pulsos a partir del cual el firmware da el motor por detenido. La lentitud de la bajada de 150 a 50 rpm se explicó por la **normalización**: el firmware divide el error entre 4000 rpm, pero el motor es mucho más lento, así que el error normalizado y, con él, la ganancia efectiva del lazo quedaban muy reducidos. Con Ki = 5 y un error de −2.25 %, el integrador se descargaba a unos 11 puntos de PWM por segundo, lo que explica los 12 s. En esa fase se planteó además la hipótesis de que con setpoints de 200 y 250 rpm el actuador estuviera saturado en torno a 150–180 rpm; el ensayo siguiente la **refutó** (sección 6.7).

Las mejoras propuestas, de mayor a menor impacto, fueron: poner la escala correcta (RPM_MAX igual a la velocidad máxima real), añadir una prealimentación con compensación de la zona muerta, mejorar la medición (mediana de cinco ventanas y, en teoría, un encoder en el eje del motor, antes de la reductora, o un encoder Hall de cuadratura) y mejorar el actuador (un TB6612FNG, con una caída de unos 0.5 V frente a los 2–3 V del L298N).

### 6.7 Velocidad máxima y zona muerta

Para fijar la escala se hizo un ensayo con el PWM al 100 % en modo manual. Con el motor estabilizado, entre 2.5 y 14.3 s, las lecturas variaron entre 320 y 1065 rpm; a partir de unas 48 muestras leídas de la gráfica, la mediana es de unas **650 rpm** y el promedio de unas 680. Se eligió **RPM_MAX = 600**, redondeando hacia abajo porque los picos inflan las lecturas; un error del 10 al 20 % en este valor no es crítico, porque solo reescala las ganancias y se absorbe al sintonizar. Para confirmarlo con más precisión se propuso promediar la columna `pulsos` del monitor con el motor a PWM máximo y calcular $\text{rpm} = 3 \cdot \text{pulsos}/T$, con $T$ el periodo de reporte en segundos (la separación de los puntos en esa gráfica sugería un reporte cada 0.25 s, aunque no se confirmó).

Este ensayo **corrigió** la conclusión anterior. Como el motor alcanza unas 600 rpm, con setpoints de 200 y 250 el PWM no estaba al 100 %. La curva quedaba por debajo de la consigna porque las lecturas falsas hacia arriba inflaban el promedio que iguala el integrador, no porque el actuador se saturase. **El límite práctico lo impone el sensor, no el L298N**: con el PWM constante la lectura variaba un ±40 %, algo imposible para la velocidad real de un motorreductor, lo que confirma pulsos perdidos o sobrantes cuya proporción crece con la velocidad. Se recomendó revisar que el disco estuviera centrado y perpendicular al eje, que no rozara la horquilla del sensor y que este estuviera bien fijo. El rango útil del controlador quedó entre unos **25 y 150 rpm**, por la calidad de la medición.

La zona muerta se midió directamente: el motor arranca con un 19 % de PWM y, una vez en movimiento, sigue girando al bajar hasta el 15 %. Se adoptó **pwmMin = 15 %**, el umbral con el motor ya en movimiento, para que la prealimentación nunca empuje de más y el PI aporte el extra necesario para vencer la fricción estática al arrancar. Como alternativa, se había propuesto estimarla por extrapolación lineal a partir de dos puntos de funcionamiento, $\text{PWM}_{\min} = P_1 - R_1 (P_2 - P_1)/(R_2 - R_1)$.

### 6.8 Mejoras del firmware

Las mejoras se especificaron en un texto para el asistente que edita el sketch en otra sesión de trabajo, con la condición de no alterar el protocolo del que depende la aplicación. La especificación final fue la siguiente:

```
Edita tú mismo el sketch con estos cambios:

1. RPM_MAX: pásalo de constante a variable rpmMax = 600 (es la velocidad real del
   motor con el PWM al 100 %). Que la usen la normalización del error, el límite
   del comando "rpm" y la conversión de "err" en el reporte. Agrega el comando
   "max <v>" (10 a 4000) para cambiarla sin recargar el sketch; al cambiarla,
   reinicia el integrador.

2. Prealimentación: en modo PID, u = ff + P + I + D, con
   ff = pwmMin + |SP| * (100 - pwmMin) / rpmMax, limitada a 0..100 %.
   - pwmMin arranca en 15 (zona muerta medida: el motor arranca con 19 % y se
     sigue moviendo hasta 15 %) y se cambia con el comando "zm <v>" (0 a 90).
   - "ff" sin argumento activa o desactiva la prealimentación; arranca activada.
   - El anti-windup debe evaluar la saturación con el ff incluido.
   - Con SP 0 el motor se sigue deteniendo como ahora.

3. Cambia la mediana de 3 por una de 5 y pon el historial a cero cuando el motor
   se da por detenido.

No cambies el formato de las líneas de reporte y confirmación, los comandos
existentes, los rangos de Kp/Ki/Kd, los baudios ni el período de reporte: una app
en Python lee el puerto serie y depende de ellos. Ojo con "m" frente a "max".
Agrega los comandos nuevos a mostrarAyuda().
```

La prealimentación se basa en un modelo lineal del motor por encima de la zona muerta: la velocidad crece linealmente desde 0 rpm en $\text{pwmMin}$ hasta $\text{rpmMax}$ con PWM al 100 %. Así, el término $ff$ aporta de antemano el PWM que el modelo estima necesario para el setpoint, y el PI solo corrige la diferencia. El comando `ff` permite comparar en el informe el PI solo frente al PI con prealimentación. La implementación la hizo ese otro asistente y su código final no se revisó en este registro; como se preservó el protocolo, la aplicación sigue siendo compatible.

Tras el cambio de escala (de 4000 a 600, un factor de unos 6.7), se recomendó empezar con **Kp = 1, Ki = 3, Kd = 0**. Se advirtió además que, al conectar, la aplicación envía al firmware los valores de sus deslizadores, que por defecto son Kp 3 y Ki 40; con la nueva escala esos valores resultan mucho más agresivos, así que conviene moverlos antes de enviar un setpoint. Los valores por defecto de esos deslizadores **no se cambiaron** en la aplicación.

### 6.9 Motor equivalente y sintonía en el simulador

El objetivo de esta fase fue obtener unos parámetros de simulación que se parecieran a las condiciones del motor real y produjeran una respuesta PID típica, como la de la figura de referencia. Esa figura, analizada sobre la imagen, tiene un sobreimpulso de aproximadamente el 52 % y una razón de decaimiento C/A de aproximadamente 0.29, cercana al criterio de un cuarto de decaimiento, que corresponde a un amortiguamiento $\zeta \approx 0.2$.

**Construcción del motor equivalente.** Se tomó un voltaje efectivo de 6 V con PWM al 100 % (supuesto), $R_a = 5\ \Omega$ y $L_a = 2$ mH, valores típicos de un motor pequeño ($\tau_e = 0.4$ ms). La zona muerta se representó como una fricción seca mediante el par de carga: si el motor deja de girar por debajo de $V_0 = 0.15 \cdot 6 = 0.9$ V, el par que la corriente correspondiente debe vencer es $T_L = K_t V_0 / R_a$. Las constantes $K_t = K_e$ se eligieron para que la velocidad a 6 V fuera de unas 600 rpm, a partir de la velocidad en régimen

$$\omega = \frac{K_t V_a / R_a - T_L}{K_t K_e / R_a + B}$$

lo que da $K_t = K_e = 0.08$ y $T_L = 0.0144$ N·m. La inercia se fijó para una constante de tiempo mecánica de unos 0.1 s, un valor supuesto y típico de un motorreductor pequeño con disco: $\tau_m = J R_a/(K_t K_e) \approx 781\,J$, de donde $J = 1.3 \times 10^{-4}$ kg·m². La fricción viscosa se dejó en $B = 1 \times 10^{-5}$ N·m·s. Si en el futuro se mide la constante de tiempo real, basta con ajustar $J = 0.00129 \cdot \tau$.

Este motor se comprobó con las propias clases del simulador. Con 6 V llega a **604 rpm** (medido en el real: unas 600). Con el 15 % de PWM no gira (0 rpm) y con el 19 % gira a 28.4 rpm, igual que la zona muerta observada. Su constante de tiempo mecánica resultó de 101 ms.

**Equivalencia de ganancias entre el firmware y el simulador.** El firmware trabaja en "% de PWM por % de fondo de escala" y el simulador en "voltios por rad/s". Como $e_{\%} = e_{\text{rpm}} \cdot 100/\text{rpmMax}$ y $u_{\%} = V_a \cdot 100/V_{a,\max}$, se obtiene

$$K_{\text{sim}} = K_{\text{Arduino}} \cdot \frac{V_{a,\max}}{\omega_{\max}} = 0.0949 \cdot K_{\text{Arduino}}$$

con $\omega_{\max} = 63.2$ rad/s. La relación vale igual para Kp, Ki y Kd, porque ambos controladores acumulan la integral como $K_i\, e\, \Delta t$.

**Búsqueda de ganancias.** Se simularon escalones de 100 rpm (10.47 rad/s, dentro del rango limpio del sensor real) con Kd = 0, igual que en el motor real, en una rejilla de 16 valores de Kp por 18 de Ki, con el periodo de muestreo del Arduino (Ts = 50 ms). Para cada respuesta se midieron el sobreimpulso, el primer cruce del setpoint, el tiempo de pico, el tiempo de establecimiento al 2 %, la razón de decaimiento, la fracción de tiempo saturado y el error final. Se descartaron las respuestas con giro hacia atrás (un artefacto del par de carga constante cuando el voltaje inicial queda por debajo de la zona muerta) y las que no alcanzaban el setpoint. De 288 candidatas resultaron 87 válidas. Se repitió la búsqueda con Ts = 5 ms para obtener una curva suave (306 candidatas, 128 válidas). Los resultados, medidos después sobre la propia interfaz, fueron los siguientes.

| Opción | Ts | Kp | Ki | Sobreimpulso | C/A | Establecimiento | Equivalente en el Arduino |
|---|---|---|---|---|---|---|---|
| A. Como la figura, fiel al Arduino | 50 ms | 0.030 | 5.60 | 48 % | 0.21 | 0.40 s | Kp 0.32 (fuera de rango), Ki 59 |
| A′. Como la figura, curva suave | 5 ms | 0.084 | 15.34 | 48 % | 0.23 | 0.38 s | Kp 0.89, Ki 162 |
| B. Bien sintonizado | 50 ms | 0.103 | 4.01 | 13 % | 0.01 | 0.10 s | Kp 1.08, Ki 42 |
| B′. Bien sintonizado, 5 ms | 5 ms | 0.493 | 30.0 | 12 % | 0.01 | 0.07 s | Kp 5.2, Ki 316 (fuera de rango) |
| C. Valores del Arduino en ese momento (Kp 1, Ki 3) | 50 ms | 0.095 | 0.28 | 0 % | — | 2.25 s | Kp 1, Ki 3 |

La opción C reprodujo en simulación la respuesta sobreamortiguada y lenta observada en el motor real con esos valores. La opción B indica que, en un sistema ideal, unas ganancias cercanas a Kp 1 y Ki 40 en unidades del firmware darían una respuesta rápida con poco sobreimpulso. La opción A reproduce la forma de la figura de referencia, pero su equivalente en el Arduino exige un Kp menor que el mínimo que acepta el firmware.

**Por qué toda la respuesta dura menos de medio segundo.** Para una planta de primer orden $G(s) = K/(\tau s + 1)$ con un controlador PI, la ecuación característica en tiempo continuo es $\tau s^2 + (1 + K K_p)s + K K_i = 0$, de donde $2\zeta\omega_n = (1 + K K_p)/\tau \geq 1/\tau$. La envolvente de la respuesta decae, como mínimo, como $e^{-t/(2\tau)}$, es decir, $e^{-5t}$ con $\tau = 0.1$ s, cualesquiera que sean las ganancias. En tiempo discreto, con retenedor de orden cero y Ts = 50 ms, el polo de la planta está en $a = e^{-T_s/\tau} = 0.61$ y el producto de los polos del lazo cerrado es $a - K(1-a)K_p \leq 0.61$, así que su módulo no supera 0.78 y la respuesta también decae en pocas muestras. La consecuencia práctica es que, con un motor tan rápido, cualquier respuesta PID dura unos cientos de milisegundos, y para verla hace falta una ventana corta y una velocidad de simulación reducida.

**Diferencias entre el simulador y el motor real.** El simulador no incluye el ruido ni el retardo de la medición (la mediana de cinco ventanas añade unos 100 ms), no modela la fricción estática del arranque al 19 %, puede aplicar voltaje negativo y por tanto frenar activamente (el Arduino solo entrega de 0 a 100 %), no incluye la prealimentación del firmware, y el par de carga que representa la zona muerta solo es válido con escalones positivos; con la onda cuadrada simétrica conviene usar $T_L = 0$. Por estas razones se recomendó, para el motor real, subir Ki poco a poco desde 3 hacia 40 y tomar como límite práctico el valor en que empiece a oscilar.

### 6.10 Autoescalado de las gráficas y valores por defecto

Con la configuración A (con Ts = 1 ms), el usuario observó que, pasado un tiempo, las gráficas del simulador "enloquecían": en torno a los 3.4–4.4 s el eje del error mostraba una escala de $10^{-9}$ y el de la salida del proceso se quedaba sin números, mientras la curva parecía oscilar con fuerza.

La simulación era correcta. La respuesta ya se había estabilizado y el error era de una milmillonésima de rad/s, pero el autoescalado del eje Y seguía el residuo del transitorio, que decae exponencialmente, de modo que una línea plana se veía como una gran oscilación. El único tope existente era un rango mínimo absoluto de $10^{-9}$, y si la simulación seguía, el eje habría llegado al ruido de redondeo (del orden de $10^{-15}$). Se reprodujo el fenómeno con los valores del usuario y se midió la altura del eje: 26 rad/s a 1 s, 0.0255 a 2 s, $3.2 \times 10^{-5}$ a 3 s y $1.7 \times 10^{-9}$ a 4.4 s, antes de saltar de golpe al rango fijo de ±1.

La corrección impone a cada gráfica un **rango visible mínimo del 20 % de la magnitud natural de su señal**: la amplitud de la consigna para el error, la salida y la velocidad angular, y la corriente de arranque $V_a/R_a$ para la corriente de armadura. La gráfica del motor físico usa la misma función, pero sin ese parámetro, así que conserva su comportamiento original. Tras la corrección, la altura del eje se mantuvo en 2.6 rad/s desde los 2 hasta los 10 s, con escalas legibles.

En el mismo cambio, los valores de esa configuración quedaron como **valores por defecto** del simulador.

| Planta | Valor | Referencia, simulación y PID | Valor |
|---|---|---|---|
| Voltaje de armadura máximo | 6.00 V | Tipo de señal | Escalón |
| Resistencia de armadura | 5.0000 Ω | Amplitud | 10.47 rad/s (100 rpm) |
| Inductancia de armadura | 0.002000 H | Periodo de muestreo | 1.000 ms |
| Constante de par Kt | 0.0800 N·m/A | Ventana inicial | 1.00 s |
| Constante FEM Ke | 0.0800 V·s/rad | Velocidad de simulación | 0.10× (desde la fase 6.11) |
| Momento de inercia J | 0.000130 kg·m² | Kp | 0.030 |
| Fricción viscosa B | 0.000010 N·m·s | Ki | 5.60 |
| Par de carga TL | 0.0144 N·m | Kd | 0.0000 |
| Condiciones iniciales | 0 A, 0 rad/s | Anti-windup / derivada sobre la medición / N | activos / activos / 20 |

Con Ts = 1 ms, estas ganancias (diseñadas para 50 ms) producen la versión en tiempo continuo de la respuesta: un pico de 15.19 rad/s a los 0.132 s, es decir, un sobreimpulso del 45 %, seguido de oscilaciones que se amortiguan.

### 6.11 Historial completo y avance lento de las gráficas

El último requisito fue que los primeros datos de las gráficas del simulador no se perdieran ni quedaran atrás, que se viera avanzar la gráfica y que todo ocurriera más lento.

La causa era estructural: el historial de la simulación era un buffer circular del tamaño de la ventana (con 1 s de ventana y 1 ms de muestreo, 1000 muestras), así que pasado un segundo el transitorio inicial se descartaba definitivamente. Se sustituyó por un **historial del ensayo completo con memoria acotada**. Guarda hasta 20 000 muestras y, cuando se llena, se queda con una de cada dos y duplica el paso de registro. De este modo la memoria no crece, la primera muestra nunca se descarta y el muestreo sigue uniforme, con una resolución que baja al alargarse el ensayo igual que la de la propia gráfica.

El eje de tiempo dejó de ser una ventana deslizante: arranca con el ancho de la ventana (el campo pasó a llamarse **"Ventana inicial"**) y, cuando la simulación la rebasa, se estira desde 0 hasta el instante actual. Así la curva se ve dibujarse de izquierda a derecha y después se comprime sin que el transitorio salga nunca de la gráfica. Las curvas se dibujan con el mismo diezmado que conserva máximos y mínimos; con el diezmado anterior de una muestra de cada *k*, el pico del sobreimpulso aparecía y desaparecía al irse comprimiendo la gráfica. A las curvas se les entregan copias de los datos, porque el historial reescribe su memoria al compactarse. Por último, la velocidad de simulación por defecto pasó a **0.1×**: un segundo simulado tarda diez segundos reales y el transitorio de unos 0.4 s se ve formarse en unos 4 s (el campo admite hasta 0.05×). La vista "Dinamica del Motor" se comporta de la misma manera.

La verificación dio los siguientes resultados. Un segundo real produjo 0.096 s simulados (0.10×). A los 0.3 s simulados, el eje iba de 0 a 1 s y la curva llegaba al instante actual. A los 5 s, el eje iba de 0 a 5 s con todo el transitorio visible. Tras 60 s simulados, el historial tenía 15 001 muestras (una de cada cuatro), seguía empezando en t = 0 y conservaba el pico exacto de 15.19 rad/s. Preparar el redibujo de las tres gráficas con el historial lleno costaba 0.5 ms.

### 6.12 El lazo de control pasa a la aplicación

El firmware se reescribió para que el Arduino hiciera solo de sensor y actuador: mide la velocidad, aplica el PWM que recibe y no guarda setpoint, ganancias ni escala. Con ese cambio la aplicación dejó de funcionar con la placa, porque enviaba órdenes que el sketch ya no reconoce (`rpm`, `kp`, `ki`, `kd`) y esperaba reportes con el formato `[PID] SP: … RPM: …`, que ya no llegan. El requisito fue trasladar a la aplicación todos los cálculos y toda la configuración (setpoint, ganancias, zona muerta, RPM máximas y límites), de modo que el Arduino reciba únicamente el PWM y entregue únicamente la velocidad.

**El sketch actual.** Cada 25 ms (40 Hz) mide la velocidad de flanco a flanco, con la mediana de cinco medidas de la sección 6.8, y envía una línea `<rpm>,<pwm>`: la velocidad en magnitud y el PWM aplicado con signo. Acepta tres órdenes: `pwm <v>` (de −100 a 100, el signo es el sentido; un número solo equivale a `pwm`), `s` (detener) y `r <v>` (ranuras por vuelta). Las órdenes no tienen respuesta, y una orden inválida contesta una única línea que empieza con `ERR`. El sketch se revisó como parte de este cambio, no se encontraron errores y no se modificó.

**El controlador en la aplicación.** Se añadió la clase `ControlMotorReal`, que reproduce el esquema que antes tenía el firmware y reutiliza el mismo `ControladorPID` del simulador. Tiene tres modos: parado, PWM fijo (lazo abierto) y PID (lazo cerrado). En modo PID, con cada medida calcula el error normalizado $e_\% = (|SP| - \text{rpm}) \cdot 100 / \text{rpmMax}$, la prealimentación $ff = \text{pwmMin} + |SP|\,(100 - \text{pwmMin})/\text{rpmMax}$ y la salida $u = ff + P + I + D$, limitada de 0 al PWM máximo. Para que el anti-windup evalúe la saturación con el ff incluido, como pedía la especificación de la sección 6.8, los límites del PID se desplazan en $-ff$: el PID puede entregar entre $-ff$ y $\text{PWM}_{\max} - ff$. El signo del setpoint decide el sentido y el lazo regula la magnitud, igual que antes, y las ganancias conservan sus unidades (% de PWM por % de fondo de escala). Hay tres diferencias deliberadas con el firmware anterior. Cambiar $K_i$ ya no reinicia el integrador: este acumula $K_i\,e\,\Delta t$, así que el cambio no produce saltos. Un setpoint nuevo con el lazo cerrado y el mismo sentido conserva el integrador, de modo que el escalón parte del estado actual; al cerrar el lazo o invertir el sentido, el integrador se reinicia. Y cambiar las RPM máximas solo borra la memoria de la derivada, para que la muestra siguiente no vea un salto ficticio.

**El lazo de tiempo real.** El PID se ejecuta al llegar cada medida, no con un temporizador propio, así que el control queda sincronizado con el muestreo de la placa. El $\Delta t$ es el periodo nominal configurado (25 ms) y no el tiempo entre llegadas: el USB entrega las líneas a ráfagas, pero en el reloj del Arduino están separadas 25 ms. Si llegan varias medidas juntas, se calcula un PWM por cada una, para no alterar la cuenta del integrador, pero solo se envía el último. El PWM se envía con una décima de resolución y solo cuando cambia. Para detectar órdenes perdidas se usa el eco: si durante cuatro medidas seguidas la placa informa un PWM distinto del último enviado, se reenvía. Las etiquetas, la consola y la gráfica se repintan a su propio ritmo (cada 50 y 100 ms), fuera del camino medida → PWM, y el cálculo de cada medida tarda en promedio 0.05 ms.

**Arranque y seguridad.** Abrir el puerto reinicia la placa, así que el lazo no se cierra hasta recibir la primera medida; la primera línea tras abrir se descarta, por si el puerto se abrió a mitad de una. En ese momento se envían las ranuras por vuelta, que el sketch olvida al reiniciarse, y el PWM vigente. Esto sustituye a la espera fija de 2.5 s de la versión anterior. Si la placa deja de enviar medidas, a los 0.5 s el indicador pasa a "Conectado · sin datos" y al segundo la aplicación detiene el motor (`s`): sin medidas el PID no puede corregir y la placa conservaría el último PWM. Cuando las medidas vuelven, se repite el arranque, pero el motor sigue parado hasta una orden nueva. Al conectar, al desconectar y al cerrar la ventana el lazo queda abierto y el motor sin tensión.

**Interfaz.** La columna derecha de la vista física se reorganizó en cuatro tarjetas. La de medición muestra la velocidad, el PWM aplicado y las medidas por segundo. La de mando tiene el setpoint (lazo cerrado), el PWM fijo (lazo abierto) y el botón de detener. La del controlador PID tiene los deslizadores de Kp, Ki y Kd, el aporte de FF, P, I y D en la última muestra, las casillas de prealimentación, anti-windup y derivada sobre la medición, y el filtro N. La de escala agrupa las RPM máximas, la zona muerta, el PWM máximo, el periodo de muestreo y las ranuras por vuelta. Todo se aplica de inmediato y sin tráfico por el puerto, salvo las ranuras. Los valores por defecto son los recomendados tras el cambio de escala de la sección 6.8: Kp = 1, Ki = 3, Kd = 0, RPM máximas 600, zona muerta 15 % y un setpoint de 100 rpm, dentro del rango limpio del sensor. Los rangos ya no los impone el firmware: Kp admite valores desde 0, lo que permite probar la opción A de la sección 6.9 (Kp = 0.32), y el setpoint queda limitado a ± las RPM máximas. Bajo la gráfica de RPM se añadió la del PWM aplicado, con el mismo eje de tiempo, para ver la saturación de un vistazo, y el setpoint pasó a dibujarse en gris discontinuo, como la referencia del simulador. Como ahora llegan 40 medidas por segundo, el historial de la gráfica usa la misma memoria acotada que el de la simulación. La tira de estado muestra el setpoint, el error, el modo, el sentido y el PWM calculado; los pulsos desaparecen porque el sketch ya no los informa.

El monitor serie conserva la consola, pero las órdenes de setpoint, ganancias y escala las atiende ahora la aplicación: `rpm`, `pwm` (o un número solo), `s`, `kp`, `ki`, `kd`, `ff`, `max`, `zm`, `pmax`, `ts` y `r` mueven el mismo control de la vista, de modo que la interfaz y el lazo nunca discrepan. La orden `?` muestra la lista, y cualquier otra se envía tal cual al Arduino. El tráfico continuo (unas 40 líneas por segundo en cada sentido) solo se muestra con la casilla "Ver tramas", y se vuelca en bloques para no repintar la consola con cada línea.

**Verificación.** El controlador se probó en lazo cerrado contra el motor equivalente del apéndice B, muestreado cada 25 ms. Con los valores por defecto, un escalón a 150 rpm llega al setpoint con un pico de 160 rpm y error final nulo, y alcanza el 90 % en 0.08 s con prealimentación frente a 1.2 s sin ella. Un setpoint negativo produce PWM negativo, la salida nunca supera el PWM máximo y el anti-windup mantiene el integrador acotado: con un PWM máximo del 50 % y un setpoint inalcanzable, al bajar la consigna a 150 rpm el motor responde en la muestra siguiente, mientras que sin anti-windup el integrador llega al 642 % y a los 3 s el motor todavía no ha bajado. La interfaz se probó con un Arduino emulado que habla el protocolo del sketch sobre un puerto falso, con 39 comprobaciones: el arranque, el seguimiento de setpoints positivos y negativos, las órdenes de la consola, el reenvío por eco, la parada por silencio y su recuperación, la respuesta `ERR`, la traza, la gráfica y la desconexión. También se comprobó que el simulador no cambió: el escalón por defecto sigue dando el pico de 15.19 rad/s de la sección 6.10.

### 6.13 Control difuso (Fuzzy) del motor real

El requisito fue añadir, dentro de la vista del motor físico, un apartado de control difuso que se aplicara al motor de la misma forma que el PID y que dejara ver con claridad cómo funciona: las variables lingüísticas, las funciones de membresía triangulares y sus áreas de solapamiento, las reglas difusas, la inferencia y la defuzzificación. Las variables lingüísticas venían fijadas: **negativo grande (NG), negativo pequeño (NP), cero (Z), positivo pequeño (PP) y positivo grande (PG)** para las entradas, y **potencia cero (PC), potencia baja (PB), potencia media baja (PMB), potencia media (PM), potencia media alta (PMA) y potencia alta (PA)** para la salida. Las gráficas del PID tenían que conservarse, y lo existente no debía modificarse más allá de lo necesario para integrar el nuevo controlador.

**Elección de las entradas.** El controlador tiene dos entradas, ambas con los cinco conjuntos NG…PG: el **error** $e = |SP| - \text{rpm}$, en rpm, y el **error acumulado** $\int e = \sum e \cdot T_s$, en rpm·s. La alternativa habitual en los textos, el error y su cambio $\Delta e$, se descartó por la naturaleza de la salida. La salida pedida es una **potencia absoluta** (de cero a alta), no un incremento de potencia. Con las entradas $(e, \Delta e)$, en régimen ($e = 0$, $\Delta e = 0$) el controlador daría siempre la misma potencia, de modo que el motor solo alcanzaría el setpoint que corresponde a esa potencia y en cualquier otro quedaría un error permanente. Por ejemplo, si el estado de reposo diera potencia media (60 %), un setpoint de 100 rpm, que pide unos 29 % de PWM, se estabilizaría muy por encima de la consigna. El error acumulado es la memoria que encuentra la potencia que pide cada setpoint, de modo que el controlador resultante es un **PI difuso** en forma de posición. Es coherente con el PID del motor real, que en la práctica trabaja como PI (Kd = 0).

**Funciones de membresía.** Cada entrada se normaliza con su rango (los parámetros $e_{\max}$ y $\int e_{\max}$) y se recorta a $[-1, 1]$. Fuera de ese intervalo se toma el extremo, así que NG y PG actúan como conjuntos de hombro. Los cinco triángulos están centrados en −1, −0.5, 0, 0.5 y 1, con semibase 0.5: cada uno se solapa a la mitad con sus vecinos, de forma que un valor pertenece como mucho a dos conjuntos y sus grados de pertenencia suman 1 (partición de Ruspini). Las áreas de solapamiento son los triángulos $\min(\mu_k, \mu_{k+1})$, con vértice de pertenencia 0.5 en el punto medio entre centros. La salida tiene seis triángulos de semibase 20 %, centrados en 0, 20, 40, 60, 80 y 100 % de PWM. Los de los extremos son triángulos completos (de −20 a 20 % y de 80 a 120 %), porque con medios triángulos el centroide nunca llegaría a 0 ni a 100 %: con PA activo en solitario daría 93.3 %. El universo de salida se discretiza en 561 puntos, de −20 a 120 %, y el PWM resultante se limita después al rango del actuador (de 0 al PWM máximo).

**Base de reglas.** Las 25 reglas tienen la forma "SI $e$ es A Y $\int e$ es B ENTONCES la potencia es C" y se numeran de R1 a R25 recorriendo la tabla por filas:

| $e$ \ $\int e$ | NG | NP | Z | PP | PG |
|---|---|---|---|---|---|
| **NG** | PC | PC | PC | PB | PMB |
| **NP** | PC | PB | PMB | PM | PMA |
| **Z** | PB | PMB | PM | PMA | PA |
| **PP** | PMB | PM | PMA | PA | PA |
| **PG** | PMA | PA | PA | PA | PA |

Cerca de $e = $ Z, cada paso de cualquiera de las dos entradas sube o baja un conjunto de salida, lo que da una respuesta suave alrededor del setpoint. En las filas de error grande (NG y PG) el desplazamiento es de tres conjuntos en lugar de dos, así que la acción es más enérgica: con el motor muy por debajo del setpoint la potencia va casi al máximo, y muy por encima se corta, casi sin tener en cuenta el acumulado. Esta no linealidad, más agresiva lejos del setpoint y más suave cerca de él, es la que distingue al controlador de un PI lineal. La fila Z va de PB a PA, de modo que con error nulo el controlador puede sostener cualquier potencia entre 20 y 100 %. Por debajo del 20 %, el motor estaría cerca de su zona muerta (15 %).

**Inferencia y defuzzificación.** Se usa el método de Mamdani. La fuerza de cada regla es $w = \min(\mu_e, \mu_{\int e})$ (conectivo Y = mínimo). La implicación recorta el conjunto de salida de cada regla a la altura $w$ (mínimo). Si varias reglas llevan al mismo conjunto, se toma la mayor fuerza, y la agregación es la unión (máximo) de todos los conjuntos recortados. La defuzzificación es por centroide, $u^* = \sum \mu(x)\,x / \sum \mu(x)$, y ese valor es el PWM que se envía a la placa. Como los triángulos de entrada se solapan a la mitad, en cada muestra disparan como mucho cuatro reglas.

**Anti-windup y precarga.** El error acumulado se limita a $\pm \int e_{\max}$, que es un anti-windup natural. Además, como en el PID, se aplica integración condicional, desactivable con su casilla: no se acumula mientras el error está fuera de su universo ($|e| \ge e_{\max}$) o la salida ya está en su tope (a menos de 0.5 % del PWM máximo con error positivo, o de 0 % con error negativo). Para que un escalón no tenga que esperar a que el acumulado viaje desde cero hasta la potencia necesaria, se añadió una **precarga**, también desactivable. Al aplicar un setpoint, el error acumulado arranca en el valor que, con error nulo, hace que el controlador dé el PWM que predice la prealimentación del PID: $ff = \text{zona muerta} + |SP| \cdot (100 - \text{zona muerta}) / \text{rpmMax}$. Con error nulo solo trabaja la fila Z de la tabla, cuya salida crece de forma monótona con el acumulado, así que ese valor se halla por bisección (30 iteraciones). Si el lazo difuso ya estaba cerrado y el setpoint cambia en el mismo sentido, el acumulado se desplaza lo que cambia $ff$, y se conserva lo aprendido sobre la diferencia entre el modelo y el motor. Al cerrar el lazo o invertir el sentido se empieza de nuevo. La precarga no suma nada a la salida: el difuso entrega siempre la potencia completa y la prealimentación solo fija el estado inicial del acumulado.

**Integración en el lazo del motor real.** La clase `ControladorDifuso` implementa las cuatro etapas, y `ControlMotorReal` ganó un cuarto modo, FUZZY, junto a parado, PWM fijo y PID, además del atributo `ley`, que indica qué controlador cierra el lazo. Todo lo demás de la sección 6.12 se aplica igual: el difuso calcula un PWM con cada medida que llega, con $\Delta t$ igual al periodo nominal de 25 ms, el signo del setpoint decide el sentido, y valen el reenvío por eco, la parada por silencio y los límites. Para el PID se extrajo la prealimentación a un método común, sin cambiar su comportamiento. Se puede pasar de un controlador a otro con el lazo cerrado y el motor en marcha, **sin salto**: si entra el difuso, su acumulado se precarga para que dé el PWM que se estaba aplicando, y si entra el PID, su integrador se ajusta a ese PWM menos el $ff$. Esto permite comparar los dos controladores sobre la misma gráfica. El cálculo difuso de cada medida tarda en promedio 0.046 ms, del mismo orden que el PID (0.05 ms).

**Interfaz.** Al conectar con la placa aparecen, sobre el panel del motor, los dos botones "Control PID" y "Control Fuzzy", con el mismo estilo que el selector de vistas de la cabecera y una línea que resume el controlador activo. Con "Control PID" la vista queda como en la sección 6.12. Con "Control Fuzzy", la columna derecha sustituye la tarjeta del PID por la del controlador difuso. Esta tiene los deslizadores de $e_{\max}$ (10–600 rpm, por defecto 200) y $\int e_{\max}$ (5–600 rpm·s, por defecto 100); un rango menor hace al controlador más enérgico, como subir Kp o Ki. También tiene los valores de $e$, $\int e$, $u^*$ y el número de reglas activas en la última muestra, y las casillas de precarga y de anti-windup. Las tarjetas de medición, mando y escala son comunes a los dos modos. La zona izquierda pasa a ser una columna desplazable que sigue el orden del cálculo:

1. **Respuesta del motor y fuzzificación.** La gráfica de RPM contra el tiempo, con el setpoint y el PWM debajo, se conserva junto a la tarjeta de fuzzificación. Esta dibuja los cinco triángulos de cada entrada, rellenos con su color, con las áreas de solapamiento rayadas y el cruce de pertenencia 0.5 marcado. Una línea vertical indica el valor actual de la entrada, y los puntos donde corta a los triángulos se rotulan con sus grados ($\mu_{PP} = 0.62$, por ejemplo). Debajo aparece el vector de pertenencia completo, que es el resultado de la fuzzificación, y se avisa cuando la entrada se sale de su rango.
2. **Base de reglas, inferencia y defuzzificación.** La tabla de 5 × 5 reglas ilumina las que disparan con una intensidad proporcional a su fuerza $w$ y marca con borde grueso la más fuerte; debajo se escriben las reglas activas ("R14 SI e=PP (0.62) Y ∫e=Z (0.80) → PMA, w = 0.62"). El botón "Ver en texto" abre la lista de las 25 reglas con los nombres completos de los conjuntos. La gráfica de inferencia muestra los seis conjuntos de salida y, rellenos, los consecuentes recortados a su fuerza. La de agregación y defuzzificación muestra la unión de los recortes, la línea del centroide $u^*$ con el centro de gravedad del área, el PWM enviado y, si el PWM máximo es menor que 100 %, su límite.
3. **Superficie de control y motor 3D.** Como gráfica adicional se añadió la superficie de control: un mapa de colores de $u^*(e, \int e)$ en todo el dominio, calculado una sola vez (61 × 61 puntos, unos 60 ms). Tiene rejilla en los centros de los conjuntos, el consecuente de la regla que dispara sola en cada cruce, y el estado actual con una estela de los últimos tres segundos. Relaciona la tabla de reglas con el comportamiento continuo del controlador: entre los cruces, la inferencia y el centroide interpolan.
4. **Estado del lazo y monitor serie**, igual que en la vista del PID.

La vista 3D, la gráfica de respuesta, el estado del lazo y el monitor son los mismos objetos en los dos modos: al cambiar de controlador se mueven de una disposición a la otra, en lugar de duplicarse, así que siguen recibiendo los datos sin cambios. Los dibujos del difuso se pintan a mano con QPainter, como los diagramas del simulador. Se refrescan como mucho cada 66 ms (constante `MS_GRAFICA_DIFUSA`) y solo cuando su vista es la elegida, de modo que no cargan el camino medida → PWM. Mientras el difuso no está cerrando el lazo, las gráficas muestran los conjuntos y un aviso de "sin datos". El monitor serie atiende cuatro órdenes nuevas: `pid` y `fuzzy` eligen el controlador, y `re <v>` y `ra <v>` fijan $e_{\max}$ y $\int e_{\max}$. El indicador MODO del estado muestra FUZZY cuando el lazo lo cierra el difuso.

**Verificación.** Se comprobó primero el controlador aislado. Con error nulo, la potencia va de 20 % (acumulado en −1) a 100 % (acumulado en +1) y es monótona en las 201 posiciones probadas. Con error máximo y acumulado nulo da 100 %, y con error mínimo, 0 %. La superficie precalculada coincide con la inferencia directa. Después se probó en lazo cerrado contra un modelo de primer orden del motor, muestreado cada 25 ms: 600 rpm a PWM 100 %, zona muerta del 15 % y constante de tiempo de 0.25 s (**supuesta**, distinta del motor equivalente del apéndice B). La secuencia de setpoints fue 300 → 100 → 450 rpm, con los valores por defecto de ambos controladores. Con el modelo exacto, el difuso terminó en 298.9, 98.9 y 450.2 rpm, frente a 298.5, 100.0 y 450.0 rpm del PID, con picos de 317 y 322 rpm en el primer escalón. Con un motor un 15 % más débil que el modelo usado por la precarga, terminó en 298.7, 99.5 y 450.2 rpm con un pico de 302 rpm: el error acumulado corrige el desajuste del modelo. La interfaz se probó con la placa simulada inyectando medidas de un motor virtual. Se comprobaron las dos disposiciones de la vista, el cambio de controlador en caliente en los dos sentidos, el setpoint negativo, el PWM manual, la parada, los cambios de rango y de casillas en caliente, el diálogo de reglas, las órdenes de la consola, y la desconexión y reconexión estando en el modo difuso, sin excepciones. Se revisaron capturas de cada estado y se comprobó que la vista del PID y el simulador no cambiaron. **El controlador difuso no se ha ensayado todavía con el motor real.**

## 7. Arquitectura final del software

`simulador.py` se organiza en bloques bien delimitados. Al inicio están las constantes del presupuesto de tiempo, la paleta de colores y la hoja de estilo de la interfaz; después, el modelo, el controlador, los componentes de interfaz reutilizables, las vistas dibujadas a mano, el enlace serie y la ventana principal.

| Componente | Línea | Función |
|---|---|---|
| Constantes de tiempo y dibujo | 78–98 | Presupuesto del bucle, límites de las gráficas y vigilancia del enlace |
| `ParametrosMotor` | 237 | Parámetros físicos del motor |
| `MotorDC` | 259 | Integración RK4 del modelo con subpasos |
| `ControladorPID` | 327 | PID paralelo con derivada filtrada y anti-windup |
| `ControladorDifuso` | 393 | Controlador difuso Mamdani: conjuntos, reglas, inferencia, centroide, precarga y superficie |
| `ControlMotorReal` | 594 | Lazo del motor real: modos (PID o FUZZY), error normalizado, prealimentación, límites y cambio de controlador |
| `Historial` | 778 | Ensayo completo con compactación por mitades (campos configurables) |
| `Tarjeta`, `Rejilla`, `Metrica`, `MetricaGrande`, `ControlGanancia` | 864–968 | Componentes de interfaz |
| `RepintadoLimitado` | 1031 | Limitador de frecuencia de repintado |
| `DiagramaBloques` | 1072 | Diagrama del lazo con valores vivos |
| `DiagramaDinamica` | 1225 | Diagrama del modelo físico del motor |
| `VistaMotor3D` | 1511 | Motor 3D por software (QPainter) |
| `COLORES_E`, `COLORES_U`, `color_potencia`, `leyenda_terminos` | 1704–1717 | Colores de los conjuntos difusos y leyendas |
| `LienzoDifuso` | 1724 | Base de los dibujos del difuso: fuentes, texto y eje de pertenencia |
| `GraficaEntradaDifusa` | 1783 | Fuzzificación de una entrada con sus triángulos y solapamientos |
| `TablaReglas` | 1914 | Base de reglas 5 × 5 y reglas activas |
| `GraficaSalidaDifusa` | 2041 | Inferencia o agregación y defuzzificación sobre el universo de salida |
| `SuperficieControl` | 2181 | Mapa $u^*(e, \int e)$ con el estado actual |
| `EnlaceSerieMotor` | 2306 | Enlace serie con el Arduino e interpretación de las medidas |
| `crear_grafica`, `grafica_base`, `curva` | 2499–2555 | Fábrica de gráficas con el estilo de la aplicación |
| `diezmar_picos`, `escalones` | 2569–2597 | Diezmado con picos y línea escalonada del setpoint |
| `SimuladorPID` | 2611 | Ventana principal: las tres vistas, la simulación y el control del motor real (PID o Fuzzy) |
| `main` | 4910 | Punto de entrada |

| Constante | Valor | Significado |
|---|---|---|
| `MS_SIMULACION` | 16 ms | Periodo del bucle de simulación |
| `MS_GRAFICAS` | 33 ms | Refresco de curvas y telemetría |
| `MS_MOTOR_3D` | 50 ms | Refresco de la vista 3D |
| `PUNTOS_CURVA` | 700 | Puntos dibujados por curva del simulador |
| `CAPACIDAD_HISTORIAL` | 20 000 | Muestras guardadas del ensayo simulado |
| `SUBPASOS_FOTOGRAMA` | 6000 | Tope de integraciones RK4 por fotograma |
| `MS_GRAFICA_FISICA` | 100 ms | Refresco máximo de la gráfica del motor real |
| `PUNTOS_GRAFICA_FISICA` | 2000 | Puntos dibujados de la curva del motor real |
| `T_MIN_GRAFICA_FISICA` | 10 s | Ancho mínimo del eje de tiempo del motor real |
| `MS_GRAFICA_DIFUSA` | 66 ms | Refresco máximo de los dibujos del controlador difuso |
| `MS_ESPERA_PLACA` | 4000 ms | Tiempo de arranque de la placa antes de marcarla "sin datos" |
| `MS_SIN_DATOS` | 500 ms | Silencio de la placa que se marca como "sin datos" |
| `MS_PARADA_SEGURIDAD` | 1000 ms | Silencio tras el que la aplicación detiene el motor |
| `DESFASE_REENVIO` | 4 | Medidas con el eco del PWM distinto de lo enviado antes de reenviarlo |

Dentro de `SimuladorPID`, los métodos se agrupan en la construcción de la interfaz (cabecera, selector de vistas, columnas y las tres páginas, incluidas las tarjetas de conexión, motor, gráficas, estado, columna de control y monitor de la vista física), las conexiones de señales, los ajustes en caliente, el control de la simulación (iniciar, detener, reiniciar), el motor real (enumeración y conexión, la barra de elección del controlador, la disposición del modo difuso con sus tarjetas y la recolocación de las tarjetas comunes, el lazo medida → PID o difuso → PWM, configuración y mando, el cambio de controlador en caliente, el refresco de los dibujos del difuso, monitor y órdenes de la consola, gráfica, animación y vigilancia del enlace), el núcleo de simulación (referencia, paso y fotograma) y el refresco de las vistas (autoescalado, curvas y telemetría).

## 8. Protocolo de comunicación entre la aplicación y el firmware

Desde la sección 6.12 el protocolo es mínimo, porque toda la lógica de control está en la aplicación. La placa envía cada 25 ms una línea `<rpm>,<pwm>` (por ejemplo, `312.5,41.7`) con la velocidad medida, en magnitud, y el PWM que está aplicando, con signo; ambos llevan un decimal. La aplicación separa los dos campos por la coma. Cualquier línea que no contenga exactamente dos números finitos se muestra como texto en el monitor, como ocurre con las respuestas `ERR …`.

La aplicación envía `pwm %.1f` con el PWM calculado, solo cuando cambia o cuando el eco indica que la placa no lo aplicó. Envía también `s` para detener y `r %d` con las ranuras por vuelta, al arrancar la placa y cada vez que cambian. Cada orden termina en fin de línea, porque el firmware procesa la línea solo al recibirlo. A 9600 baudios, una medida ocupa unos 12 caracteres y una orden de PWM unos 10, de modo que a 40 Hz cada sentido usa aproximadamente la mitad del ancho de banda.

Las órdenes del protocolo anterior (`rpm`, `kp`, `ki`, `kd`, `max`, `zm`, `ff`, `m` y `?`), descritas en las secciones 5.2 y 6.8, ya no existen en el firmware. Si se escriben en el monitor, las atiende la propia aplicación (sección 6.12). El controlador difuso de la sección 6.13 no cambió el protocolo: la placa no distingue qué controlador calculó el PWM que recibe.

## 9. Verificación y pruebas

La verificación se apoyó en programas de prueba escritos durante el desarrollo, que ejecutan el código real de la aplicación sin intervención manual y guardan capturas de pantalla para la revisión visual. Estos programas se guardaron en carpetas temporales de la sesión de trabajo y **no forman parte de la carpeta del proyecto**, así que para conservarlos habría que copiarlos.

La prueba de la interfaz recorre las tres vistas, finge un Arduino conectado, inyecta reportes, comprueba las lecturas mostradas, el giro del modelo 3D y el texto de las órdenes enviadas, y verifica que al entrar en la vista física se detiene la simulación. La prueba del enlace serie comprueba el reensamblado de tramas partidas, el descarte de tramas sin fin de línea, la apertura fallida de un puerto inexistente, que la ficha de los puertos no los sondee y se calcule en menos de un segundo, y la desconexión sin enlace. La prueba de la gráfica del motor físico comprueba el diezmado con conservación de picos, los escalones del setpoint, el corte en modo manual, la compresión del eje, el límite de refresco, el coste con 100 000 muestras y el reinicio. La prueba de avance del simulador comprueba la velocidad 0.1× en tiempo real, el avance de la curva dentro de la ventana inicial, el estiramiento del eje, la conservación del inicio y del pico tras las compactaciones y el coste del redibujo. Además hubo programas específicos para reproducir el problema del autoescalado antes y después de corregirlo, para buscar las ganancias del motor equivalente y para capturar la interfaz con cada configuración. Para la sección 6.12 se añadieron una prueba del lazo `ControlMotorReal` contra el motor equivalente y una prueba de la vista física con un Arduino emulado sobre un puerto falso, descritas en esa sección; igual que las anteriores, quedaron fuera de la carpeta del proyecto. Para la sección 6.13 se añadieron una prueba del controlador difuso aislado y en lazo cerrado contra un modelo de primer orden del motor, comparado con el PID, y una prueba de la interfaz con la placa simulada que recorre los dos modos y guarda capturas de pantalla; también quedaron fuera de la carpeta del proyecto. Al cierre del periodo, todas las pruebas pasan.

## 10. Resultados consolidados

| Aspecto | Resultado | Tipo de dato |
|---|---|---|
| Arranque de la aplicación | 1.9–2.0 s (antes de corregir `isBusy()`: ~26 s) | Medido |
| Entrar en la vista del motor físico | 34–39 ms | Medido |
| Redibujo de la gráfica física con 100 000 muestras | ~20–26 ms | Medido |
| Velocidad máxima del motor real (PWM 100 %) | ~600 rpm (mediana ~650, lecturas de 320 a 1065) | Observado y estimado |
| Zona muerta del motor real | Arranca con 19 %, se mantiene hasta 15 % | Reportado por el usuario |
| Variación de la lectura a velocidad máxima | ±40 % con PWM constante | Observado |
| Rango útil del controlador real | ~25 a 150 rpm (limitado por la medición) | Estimado a partir de los ensayos |
| Límite inferior de medición | ~10–15 rpm (pulsos cada 300 ms = tiempo sin pulsos) | Deducido del firmware y observado |
| Motor equivalente simulado | 604 rpm a 6 V, zona muerta en 15 %, τ = 101 ms | Medido en simulación (τ supuesto) |
| Conversión de ganancias | K_sim = 0.0949 · K_Arduino | Deducido |
| Respuesta tipo figura (simulada) | 48 % de sobreimpulso, C/A 0.21–0.23 | Medido en simulación |
| Respuesta bien sintonizada (simulada) | 13 % de sobreimpulso, 0.10 s | Medido en simulación |
| Medidas del motor real | 40 por segundo (una cada 25 ms) | Diseño del sketch; 39.5–40 Hz con la placa emulada |
| Cálculo medida → PWM en la aplicación | 0.05 ms de media, unos 1 ms en el peor caso | Medido con la placa emulada |
| Escalón a 150 rpm con el lazo en la aplicación (simulado, 25 ms) | Pico de 160 rpm, error final nulo, 90 % en 0.08 s con ff y 1.2 s sin ff | Medido en simulación |
| Cálculo medida → PWM del controlador difuso | 0.046 ms de media | Medido |
| Superficie de control del difuso (61 × 61) | ~60 ms, una sola vez | Medido |
| Setpoints 300 → 100 → 450 rpm, difuso frente a PID (modelo de primer orden, τ = 0.25 s) | Difuso: 298.9 / 98.9 / 450.2 rpm, pico 317 rpm; PID: 298.5 / 100.0 / 450.0 rpm, pico 322 rpm | Medido en simulación (τ supuesto) |
| Igual, con el motor un 15 % más débil que el modelo | Difuso: 298.7 / 99.5 / 450.2 rpm, pico 302 rpm | Medido en simulación (τ supuesto) |

## 11. Limitaciones conocidas

La constante de tiempo mecánica del motor real no se midió; el valor de 0.1 s es un supuesto y, si se mide, cambia la inercia equivalente y la sintonía calculada. Tampoco se documentaron el modelo exacto del motorreductor ni el voltaje de la fuente. La velocidad máxima es una estimación a partir de lecturas muy ruidosas. Las ganancias usadas en cada ensayo real no se registraron de forma sistemática. El lazo de la sección 6.12 se verificó con una placa emulada, no con el motor real, y el controlador difuso de la sección 6.13 solo se ha probado en simulación. Sus rangos por defecto ($e_{\max}$ = 200 rpm, $\int e_{\max}$ = 100 rpm·s) se eligieron para que su ganancia local equivalga aproximadamente a la del PID por defecto, no se sintonizaron sobre el motor. Con error nulo, la potencia mínima que puede sostener es del 20 %, así que los setpoints por debajo de unas 35 rpm se mantendrían con un pequeño error negativo. La precarga depende de que la zona muerta y las RPM máximas configuradas se parezcan a las reales; si no, el error acumulado corrige la diferencia, pero más despacio.

En el lado de la medición, el encoder de 20 ranuras en el eje de salida da muy poca resolución a baja velocidad, y la lectura se degrada al subir la velocidad. Con el lazo en la aplicación, el control depende del PC: el USB y el planificador de Windows añaden un retardo variable, normalmente de pocos milisegundos, entre la medida y la aplicación del PWM. El periodo de muestreo de la aplicación es nominal y tiene que coincidir con el del sketch (25 ms); la tarjeta de medición muestra las medidas por segundo para comprobarlo. Si la aplicación se bloquea o termina sin pasar por el cierre normal de la ventana, la placa conserva el último PWM, porque el sketch no tiene un temporizador de vigilancia de órdenes: la parada por silencio solo cubre el caso contrario, que la placa deje de enviar medidas. Durante un silencio de la placa, la gráfica une con una recta la última medida anterior y la primera posterior. El simulador idealiza la planta en los aspectos descritos en la sección 6.9.

## 12. Trabajo futuro

Para el informe y para mejorar el sistema, quedan identificadas estas líneas:

- Medir la constante de tiempo real con un escalón de PWM en modo manual y actualizar la inercia equivalente.
- Calcular y marcar automáticamente sobre las gráficas el sobreimpulso, el tiempo de subida y el tiempo de establecimiento.
- Incorporar al simulador el retardo de la medición y un actuador limitado de 0 a 100 %, para acercarlo al comportamiento real.
- Exportar los ensayos reales a un archivo de datos para procesarlos en el informe.
- Agregar al sketch un temporizador de vigilancia que detenga el motor si no recibe ninguna orden durante unos 0.5 s, y hacer que la aplicación reenvíe el PWM periódicamente aunque no cambie.
- Guardar la configuración del lazo (ganancias, escala y zona muerta) entre sesiones de la aplicación.
- Ensayar el controlador difuso con el motor real, sintonizar $e_{\max}$ y $\int e_{\max}$, y comparar con el PID el sobreimpulso, el tiempo de establecimiento y el rechazo a perturbaciones, aprovechando el cambio de controlador en caliente.
- Permitir editar la base de reglas y el solapamiento de los triángulos desde la interfaz, y ofrecer otros métodos de defuzzificación (media de centros, bisectriz) para compararlos.
- En hardware, mejorar la resolución del encoder (disco en el eje del motor o encoder Hall de cuadratura) y usar un puente con menor caída de tensión.

## Apéndice A. Fórmulas de referencia

Medición de velocidad en el firmware:

$$\text{rpm} = \frac{60 \times 10^{6} \cdot \text{pulsos}}{\Delta t_{\mu s} \cdot \text{ranuras}}$$

Error normalizado del firmware:

$$e_{\%} = \frac{(|SP| - \text{rpm}) \cdot 100}{\text{rpmMax}}$$

Prealimentación con compensación de la zona muerta:

$$ff = \text{pwmMin} + |SP| \cdot \frac{100 - \text{pwmMin}}{\text{rpmMax}}$$

Sintonía IMC de un PI para una planta de primer orden con ganancia normalizada $K_n$, constante de tiempo $\tau$ y constante de lazo cerrado deseada $\lambda$:

$$K_p = \frac{\tau}{K_n\,\lambda}, \qquad K_i = \frac{K_p}{\tau}$$

Zona muerta por extrapolación lineal de dos puntos $(P_1, R_1)$ y $(P_2, R_2)$:

$$\text{PWM}_{\min} = P_1 - R_1 \frac{P_2 - P_1}{R_2 - R_1}$$

Velocidad media a partir de los pulsos por reporte (20 ranuras, periodo de reporte $T$ en segundos):

$$\text{rpm} = \frac{3 \cdot \text{pulsos}}{T}$$

Equivalencia de ganancias entre el firmware y el simulador:

$$K_{\text{sim}} = K_{\text{Arduino}} \cdot \frac{V_{a,\max}}{\omega_{\max}}$$

Constante de tiempo mecánica e inercia equivalente del motor simulado:

$$\tau_m = \frac{J\,R_a}{K_t K_e + R_a B} \;\;\Rightarrow\;\; J \approx 0.00129 \cdot \tau$$

Par de carga que representa la zona muerta:

$$T_L = \frac{K_t \cdot V_0}{R_a}, \qquad V_0 = \text{pwmMin} \cdot V_{a,\max}$$

Controlador difuso (sección 6.13). Entradas normalizadas y recortadas:

$$e_n = \operatorname{sat}_{[-1,1]}\!\left(\frac{|SP| - \text{rpm}}{e_{\max}}\right), \qquad a_n = \operatorname{sat}_{[-1,1]}\!\left(\frac{\sum e \, T_s}{\int e_{\max}}\right)$$

Función de membresía triangular de centro $c$ y semibase $b$ ($b = 0.5$ en las entradas y $b = 20\,\%$ en la salida):

$$\mu(x) = \max\!\left(0,\; 1 - \frac{|x - c|}{b}\right)$$

Fuerza de la regla $(i, j)$, activación de cada conjunto de salida $k$ y agregación (Mamdani):

$$w_{ij} = \min\big(\mu_i(e_n), \mu_j(a_n)\big), \qquad \alpha_k = \max_{R(i,j) = k} w_{ij}, \qquad \mu_{\text{ag}}(x) = \max_k \min\big(\alpha_k, \mu_k(x)\big)$$

Defuzzificación por centroide y PWM aplicado:

$$u^* = \frac{\sum_x \mu_{\text{ag}}(x)\, x}{\sum_x \mu_{\text{ag}}(x)}, \qquad \text{PWM} = \min\big(\max(u^*, 0),\, \text{PWM}_{\max}\big)$$

Ganancias locales equivalentes cerca de $e = 0$ (un conjunto de salida, 20 %, por cada media unidad normalizada de entrada):

$$K_{p,\text{eq}} \approx \frac{40}{e_{\max}}\ \frac{\%}{\text{rpm}}, \qquad K_{i,\text{eq}} \approx \frac{40}{\int e_{\max}}\ \frac{\%}{\text{rpm}\cdot\text{s}}$$

## Apéndice B. Parámetros del motor equivalente

| Parámetro | Símbolo | Valor | Origen |
|---|---|---|---|
| Voltaje efectivo a PWM 100 % | $V_{a,\max}$ | 6 V | Supuesto |
| Resistencia de armadura | $R_a$ | 5 Ω | Típico de un motor pequeño |
| Inductancia de armadura | $L_a$ | 2 mH | Típico de un motor pequeño |
| Constantes de par y FEM | $K_t = K_e$ | 0.08 | Ajustadas a ~600 rpm a 6 V |
| Momento de inercia | $J$ | 1.3 × 10⁻⁴ kg·m² | Ajustado a τ ≈ 0.1 s (supuesto) |
| Fricción viscosa | $B$ | 1 × 10⁻⁵ N·m·s | Casi despreciable |
| Par de carga (zona muerta) | $T_L$ | 0.0144 N·m | Zona muerta medida del 15 % |
| Ganancia de la planta sobre la zona muerta | $K$ | ≈ 12.4 (rad/s)/V | Deducida |
| Velocidad a 6 V | — | 604 rpm | Medida en simulación |
| Constante de tiempo mecánica | $\tau_m$ | 101 ms | Medida en simulación |
