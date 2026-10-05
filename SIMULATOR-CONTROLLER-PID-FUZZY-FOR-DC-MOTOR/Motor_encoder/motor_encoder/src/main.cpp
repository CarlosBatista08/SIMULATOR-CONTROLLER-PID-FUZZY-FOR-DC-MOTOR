#include <Arduino.h>
#include <ctype.h>

/*
 * Sensor de velocidad + actuador PWM de un motor DC
 * Arduino Uno + puente H L298N + sensor optico de ranura (FC-03 / MH-Sensor)
 *
 * El Arduino solo mide y actua: todo el calculo de control (PID, filtros,
 * prealimentacion, sintonia) vive en el programa de Python.
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
 * Comandos por serial (9600 baudios), cada uno terminado en salto de linea:
 *   pwm 60   -> fija 60 % de PWM (-100 a 100); el signo es el sentido de giro
 *   60       -> atajo equivalente a "pwm 60"
 *   s        -> detener el motor
 *   r 20     -> ranuras por vuelta del disco encoder
 *
 * Salida por serial, una linea cada PERIODO_REPORTE_MS:
 *   <rpm>,<pwm>
 *   rpm = velocidad medida (magnitud, el sensor no distingue el sentido)
 *   pwm = PWM aplicado en ese momento, con signo
 *
 * Los comandos no generan respuesta (para no ensuciar el flujo de datos). Un
 * comando invalido responde una unica linea que empieza con "ERR".
 */

const uint8_t PIN_ENA = 3;
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;
const uint8_t PIN_SENSOR = 2;
// contarPulso() lee el pin directo como PD2 por velocidad: si mueves el
// sensor de pin, hay que ajustar tambien esa lectura.
static_assert(PIN_SENSOR == 2, "contarPulso() lee PD2 directamente");

const unsigned long BAUDIOS = 9600;

const unsigned long PERIODO_MEDICION_MS = 25;  // lectura del sensor: 40 Hz
const unsigned long PERIODO_REPORTE_MS = 25;   // envio por serial: 40 Hz

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
// fin de linea.
const uint8_t LARGO_BUFFER = 24;
char buffer[LARGO_BUFFER];
uint8_t indiceBuffer = 0;

float salidaPorcentaje = 0.0f;  // PWM aplicado, con signo
float rpmMedida = 0.0f;

unsigned long ultimaMedicionMs = 0;
unsigned long ultimoReporteMs = 0;

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

void procesarLinea(char *linea);
void aplicarSalida(float porcentaje);
void actualizarMedicion();
float medianaHist();
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
  ultimaMedicionMs = millis();
  ultimoReporteMs = ultimaMedicionMs;
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

  if (ahoraMs - ultimaMedicionMs >= PERIODO_MEDICION_MS) {
    ultimaMedicionMs = ahoraMs;
    actualizarMedicion();
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
        // Mediana de 5: hasta dos picos entre las ultimas 5 se descartan.
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

// Escribe en el puente H sin imprimir nada.
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

// Formato "<rpm>,<pwm>", compacto para caber en 9600 baudios a 40 Hz.
void reportar() {
  Serial.print(rpmMedida, 1);
  Serial.print(',');
  Serial.println(salidaPorcentaje, 1);
}

void procesarLinea(char *linea) {
  while (*linea == ' ' || *linea == '\t') {
    linea++;
  }

  if (*linea == '\0') {
    return;
  }

  // Se separa el comando (letras) del argumento (numero), de modo que "pwm60",
  // "pwm 60" y "pwm=60" se acepten por igual.
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

  if (strcmp(comando, "s") == 0 || strcmp(comando, "stop") == 0) {
    aplicarSalida(0.0f);
    return;
  }

  // De aqui en adelante todos los comandos necesitan un valor.
  if (!hayNumero) {
    Serial.println(F("ERR falta valor"));
    return;
  }

  // n == 0 significa que la linea era solo un numero: atajo de "pwm <v>".
  if (strcmp(comando, "pwm") == 0 || n == 0) {
    if (valor < -100.0f || valor > 100.0f) {
      Serial.println(F("ERR pwm fuera de rango (-100 a 100)"));
      return;
    }
    aplicarSalida(valor);
    return;
  }

  if (strcmp(comando, "r") == 0) {
    if (valor < 1.0f || valor > 1000.0f) {
      Serial.println(F("ERR ranuras fuera de rango (1 a 1000)"));
      return;
    }
    ranurasPorVuelta = (uint16_t)valor;
    return;
  }

  Serial.println(F("ERR comando desconocido"));
}
