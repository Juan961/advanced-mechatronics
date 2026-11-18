#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// =========================================================================
// Brazo pick and place con reorientación — ESP32 + PCA9685
// Comandos por serial (115200, terminados en '\n'):
//   P x,y,phi   -> ciclo completo con la pose de la pieza (la envía la cámara)
//   B/S/E/W/H/G + ángulo (-90..90)  -> mover un eje (modo manual)
//   C x,y,z,pitch,roll,garra        -> cinemática inversa directa
//   V + ms      -> velocidad (ms por grado)
//   0           -> todos los ejes al centro
//   !           -> PARADA (detiene el movimiento y bloquea nuevos comandos)
//   R           -> rearmar después de una parada
//   ?           -> reporte de estado
//   M + vel (-100..100) -> motor DC (banda): % de velocidad, el signo da
//                          el sentido, M0 lo detiene
//   D + duty (-255..255) -> PWM crudo directo al puente H (sin rampa ni
//                          duty mínimo), para pruebas. D0 lo detiene
//   K + ángulo  -> ángulo de garra cerrada usado en el ciclo (misma escala que G;
//                  abierta = 0, cerrada = -40 por defecto)
//   Z + mm      -> altura de la punta al agarrar en el ciclo (más alto = baja menos)
// Respuestas para la interfaz: líneas que empiezan con
//   OK, ERR, PUNTO, PARADA, ESTADO
// =========================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50

const uint8_t base     = 0; // MG996R
const uint8_t shoulder = 1; // TD8120
const uint8_t elbow    = 2; // MG996R
const uint8_t wrist    = 3; // MG90S  (W: roll)
const uint8_t hand     = 4; // MG90S  (H: pitch)
const uint8_t gripper  = 15; // MG90S

const uint8_t NUM_SERVOS = 6;
const uint8_t servoChannels[NUM_SERVOS] = { base, shoulder, elbow, wrist, hand, gripper };
const char servoLetters[NUM_SERVOS]     = { 'B', 'S', 'E', 'W', 'H', 'G' };

#define ANGLE_MIN -90
#define ANGLE_MAX  90

// Sentido de la garra: con true, el ángulo se invierte antes de mandarlo al
// servo. Los ángulos de la garra (G, K, GARRA_ABIERTA, garraCerrada, home)
// están todos en la misma escala que el comando G.
const bool GARRA_INVERTIDA = true;
// --- Ángulos de la garra (escala del comando G) ---
const int GARRA_ABIERTA = 0;     // abierta (también al arrancar y en home)
const int GARRA_HOME    = GARRA_ABIERTA;
// true: la garra abre/cierra de un solo golpe (sin la rampa de 1° por paso)
const bool GARRA_DIRECTA = true;

// Pose de arranque (ángulos lógicos)
const int homeAngles[NUM_SERVOS] = { 0, 45, 0, 0, 0, GARRA_HOME };
int currentAngles[NUM_SERVOS]    = { 0, 45, 0, 0, 0, GARRA_HOME };

int velocidadMovimiento = 15;   // ms por grado (igual que en la simulación)

// --- Calibración de pulsos ---
#define US_MIN_STD   500
#define US_MAX_STD   2500
#define US_MIN_MG996 600
#define US_MAX_MG996 2400

// --- Cinemática ---
const float L0 = 110.0;
const float L1 = 130.0;
const float L2 = 130.0;
const float L3 = 110.0;
const float OFFSET_Q0 = 0.0;
const float OFFSET_Q1 = 90.0;
const float OFFSET_Q2 = 90.0;
const int OFFSET_FISICO_BASE = 30;   // ángulo que hay que sumarle al servo para que la base quede derecha (x del robot)
// Ángulo máximo que se le manda al servo de la base (después de sumar el
// offset). 100 -> pulsos de 500..2500 us, el rango completo del MG996R.
// Con el offset de +30, la base llega hasta 70° lógicos hacia la izquierda.
// Si el servo zumba o se traba en el extremo, bajarlo a 95.
const int BASE_FISICO_MAX = 100;

// =========================================================================
// --- PARÁMETROS DEL CICLO (deben coincidir con el controlador de Simulink) ---
// Marco del robot, mm y grados
// =========================================================================
// Destino (donde se suelta la pieza), en polar alrededor de la base:
//   ANG_D: giro de la base al soltar (+ = izquierda). Máximo 70 por el
//          offset de la base (ver BASE_FISICO_MAX).
//   R_D:   distancia desde el centro de la base [mm]
const float R_D = 200.0, ANG_D = 70.0;
const float XD = R_D * cos(ANG_D * PI / 180.0);
const float YD = R_D * sin(ANG_D * PI / 180.0);
const float PHI_D = -60.0;      // orientación en la plantilla (solo si USAR_ROLL)
const float Z_ALTO = 100.0;     // altura de tránsito de la punta
const float Z_APROX_MIN = 70.0; // altura mínima sobre la pieza si está lejos (ver ejecutarCiclo)
float zToma = 70.0;             // altura de la punta al agarrar (se ajusta en vivo con Z; más alto = baja menos)
const float Z_DEP  = 60.0;      // altura de la punta al soltar (más alto = suelta desde más arriba)
const float PITCH_CICLO = -90.0;
// Garra cerrada (escala del comando G). Se ajusta en vivo con K.
int garraCerrada = 80;
const bool  USAR_ROLL = false;  // false: el ciclo no gira la muñeca (roll = 0 al tomar y al soltar)
const float ROLL0  = 0.0;       // roll que alinea los dedos con la pieza (phi = 0)
const float S_ROLL = 1.0;       // -1 si la reorientación sale para el lado contrario
const unsigned long PAUSA_MS = 500;

// =========================================================================
// --- MOTOR DC CON PUENTE H BTS7960 (IBT-2) ---
//   RPWM -> PWM sentido adelante, LPWM -> PWM sentido atrás
//   R_EN / L_EN -> habilitación (en alto para que el puente funcione)
//   VCC del módulo a 3.3 V o 5 V del ESP32, GND común con el ESP32
// =========================================================================
const uint8_t MOTOR_RPWM = 25;
const uint8_t MOTOR_LPWM = 26;
const uint8_t MOTOR_R_EN = 27;
const uint8_t MOTOR_L_EN = 33;
const uint32_t MOTOR_PWM_FREQ = 20000;  // Hz (el BTS7960 aguanta hasta 25 kHz; 20 kHz no se escucha)
const uint8_t  MOTOR_PWM_BITS = 8;      // duty 0..255
const uint8_t  MOTOR_CANAL_R = 0;       // canales LEDC, solo para core ESP32 2.x
const uint8_t  MOTOR_CANAL_L = 1;
const int MOTOR_PWM_MIN = 80;           // duty mínimo con el que el motor arranca
const unsigned long MOTOR_RAMPA_MS = 10; // ms por cada 1 % de cambio (0->100 % en 1 s)
const bool DETENER_BANDA_EN_CICLO = true; // parar la banda mientras el brazo toma la pieza
const bool FRENAR_SIN_RAMPA = true;       // M0 detiene al instante (freno del puente H)

int motorObjetivo = 0;    // % pedido (-100..100)
int motorActual   = 0;    // % aplicado (va siguiendo la rampa)
unsigned long motorUltimoPaso = 0;
int motorDutyDirecto = 0; // duty crudo del comando D (-255..255), 0 = sin uso

// --- Estado ---
bool parada = false;
unsigned long piezasProcesadas = 0;
const char* estadoActual = "REPOSO";

// =========================================================================
// Bajo nivel
// =========================================================================
void moveServo(uint8_t channel, int angle) {
  if (channel == base) {
    angle += OFFSET_FISICO_BASE;
    angle = constrain(angle, -BASE_FISICO_MAX, BASE_FISICO_MAX);
  } else {
    if (channel == gripper && GARRA_INVERTIDA) angle = -angle;
    angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);
  }

  unsigned int us;
  if (channel == base || channel == elbow) {
    us = map(angle, ANGLE_MIN, ANGLE_MAX, US_MIN_MG996, US_MAX_MG996);
  } else {
    us = map(angle, ANGLE_MIN, ANGLE_MAX, US_MIN_STD, US_MAX_STD);
  }
  int pulse = (int)(us * SERVO_FREQ * 4096.0 / 1000000.0);
  pwm.setPWM(channel, 0, pulse);
}

// =========================================================================
// Motor DC
// =========================================================================
// Escribe el duty de los dos lados del puente (nunca los dos a la vez)
void motorEscribirPWM(uint32_t dutyR, uint32_t dutyL) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_RPWM, dutyR);
  ledcWrite(MOTOR_LPWM, dutyL);
#else
  ledcWrite(MOTOR_CANAL_R, dutyR);
  ledcWrite(MOTOR_CANAL_L, dutyL);
#endif
}

void motorIniciar() {
  pinMode(MOTOR_R_EN, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  digitalWrite(MOTOR_R_EN, LOW);      // deshabilitado mientras se configura el PWM
  digitalWrite(MOTOR_L_EN, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttach(MOTOR_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
  ledcSetup(MOTOR_CANAL_R, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcSetup(MOTOR_CANAL_L, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttachPin(MOTOR_RPWM, MOTOR_CANAL_R);
  ledcAttachPin(MOTOR_LPWM, MOTOR_CANAL_L);
#endif
  motorEscribirPWM(0, 0);
  digitalWrite(MOTOR_R_EN, HIGH);
  digitalWrite(MOTOR_L_EN, HIGH);
}

// Aplica un % (-100..100) directo al puente H
//   > 0: PWM en RPWM, < 0: PWM en LPWM, 0: los dos en bajo (freno)
void motorAplicar(int pct) {
  pct = constrain(pct, -100, 100);
  if (pct == 0) {
    motorEscribirPWM(0, 0);
    return;
  }
  uint32_t duty = map(abs(pct), 1, 100, MOTOR_PWM_MIN, (1 << MOTOR_PWM_BITS) - 1);
  if (pct > 0) motorEscribirPWM(duty, 0);
  else         motorEscribirPWM(0, duty);
}

// Avanza la rampa hacia motorObjetivo; se llama seguido, no bloquea
void actualizarMotor() {
  if (motorActual == motorObjetivo) return;
  unsigned long ahora = millis();
  if (ahora - motorUltimoPaso < MOTOR_RAMPA_MS) return;
  motorUltimoPaso = ahora;
  motorActual += (motorObjetivo > motorActual) ? 1 : -1;
  motorAplicar(motorActual);
}

// Aplica un duty crudo (-255..255) directo al puente H, sin rampa ni duty mínimo.
// Cancela la rampa del comando M para que no lo sobrescriba.
void motorAplicarDutyDirecto(int duty) {
  duty = constrain(duty, -255, 255);
  motorObjetivo = 0;
  motorActual = 0;
  motorDutyDirecto = duty;
  if (duty > 0)      motorEscribirPWM(duty, 0);
  else if (duty < 0) motorEscribirPWM(0, -duty);
  else               motorEscribirPWM(0, 0);
}

// Detiene el motor sin rampa (para la parada de emergencia)
void motorDetenerYa() {
  motorObjetivo = 0;
  motorActual = 0;
  motorDutyDirecto = 0;
  motorAplicar(0);
}

// Detiene con rampa y espera a que llegue a 0
void motorDetenerSuave() {
  if (motorDutyDirecto != 0) motorAplicarDutyDirecto(0);
  motorObjetivo = 0;
  while (motorActual != 0) { actualizarMotor(); delay(1); }
}

// Revisa si llegó un '!' por serial mientras el brazo se mueve
void revisarParada() {
  actualizarMotor();
  if (Serial.available() && Serial.peek() == '!') {
    Serial.read();
    parada = true;
    estadoActual = "PARADA";
    motorDetenerYa();
    Serial.println(F("PARADA"));
  }
}

// Pausa que se puede interrumpir con '!'
void esperar(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms && !parada) {
    revisarParada();
    delay(5);
  }
}

// 1 grado por paso en cada eje, todos a la vez.
// La garra (con GARRA_DIRECTA) no se escalona: va a su objetivo en un solo
// movimiento, al inicio; el resto de los ejes sigue con la rampa.
void moverSuavemente(const int objetivos[NUM_SERVOS]) {
  const uint8_t G = NUM_SERVOS - 1;   // índice de la garra
  if (GARRA_DIRECTA && currentAngles[G] != objetivos[G] && !parada) {
    currentAngles[G] = objetivos[G];
    moveServo(servoChannels[G], currentAngles[G]);
  }
  bool moviendose = true;
  while (moviendose && !parada) {
    moviendose = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      if (currentAngles[i] < objetivos[i])      { currentAngles[i]++; moviendose = true; }
      else if (currentAngles[i] > objetivos[i]) { currentAngles[i]--; moviendose = true; }
      moveServo(servoChannels[i], currentAngles[i]);
    }
    if (moviendose) {
      revisarParada();
      delay(velocidadMovimiento);
    }
  }
}

void moverAHome() {
  moverSuavemente(homeAngles);
}

void moveAll(int angle) {
  int obj[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) obj[i] = angle;
  moverSuavemente(obj);
}

float envolver(float a) {            // lleva un ángulo a (-180, 180]
  a = fmod(a + 180.0, 360.0);
  if (a < 0) a += 360.0;
  return a - 180.0;
}

// =========================================================================
// Cinemática inversa (misma que en la simulación)
// =========================================================================
bool ikSilencioso = false;   // true: calcularIK no imprime los ERR (para probar alturas)

bool calcularIK(float x, float y, float z, float pitch, float roll, int garra,
                int objetivos[NUM_SERVOS]) {
  float pr = pitch * PI / 180.0;
  float r_punta  = sqrt(x * x + y * y);
  float r_muneca = r_punta - L3 * cos(pr);
  float z_muneca = z - L3 * sin(pr);
  float q0 = atan2(y, x);
  float z_rel = z_muneca - L0;
  float D = sqrt(r_muneca * r_muneca + z_rel * z_rel);

  if (D > (L1 + L2) || D < fabs(L1 - L2)) {
    if (!ikSilencioso) Serial.println(F("ERR coordenada inalcanzable"));
    return false;
  }

  float c2 = constrain((D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2), -1.0, 1.0);
  float q2 = acos(c2);
  float q1 = atan2(z_rel, r_muneca) + atan2(L2 * sin(q2), L1 + L2 * c2);

  float q0d = q0 * 180.0 / PI;
  float q1d = q1 * 180.0 / PI;
  float q2d = q2 * 180.0 / PI;
  float q3d = pitch - (q1d - q2d);

  float calc[5] = { q0d - OFFSET_Q0, q1d - OFFSET_Q1, q2d - OFFSET_Q2, roll, q3d };
  const char* nombres[5] = { "Base", "Hombro", "Codo", "Roll(W)", "Pitch(H)" };
  for (uint8_t i = 0; i < 5; i++) {
    // La base se revisa con su offset y su rango propio: antes se recortaba
    // en silencio y la base se quedaba corta sin avisar.
    float minimo = ANGLE_MIN, maximo = ANGLE_MAX;
    if (i == 0) {
      minimo = -BASE_FISICO_MAX - OFFSET_FISICO_BASE - 0.5;  // 0.5: redondeo de float
      maximo =  BASE_FISICO_MAX - OFFSET_FISICO_BASE + 0.5;
    }
    if (calc[i] < minimo || calc[i] > maximo) {
      if (ikSilencioso) return false;
      Serial.print(F("ERR fuera de rango: "));
      Serial.print(nombres[i]);
      Serial.print(F(" = "));
      Serial.println(calc[i]);
      return false;
    }
  }
  for (uint8_t i = 0; i < 5; i++) objetivos[i] = (int)calc[i];
  objetivos[5] = garra;
  return true;
}

bool moverA(float x, float y, float z, float pitch, float roll, int garra) {
  int obj[NUM_SERVOS];
  if (!calcularIK(x, y, z, pitch, roll, garra, obj)) return false;
  moverSuavemente(obj);
  return !parada;
}

// =========================================================================
// Ciclo pick and place a partir de la pose de la pieza
// =========================================================================
struct Punto { float x, y, z, roll; int garra; };

bool ejecutarCiclo(float xp, float yp, float phi_p) {
  // Reorientación: lo que pide la plantilla menos lo que ya gira la base
  float base_t = atan2(yp, xp) * 180.0 / PI;
  float base_d = atan2(YD, XD) * 180.0 / PI;
  float giro = envolver(S_ROLL * ((PHI_D - phi_p) - (base_d - base_t)));

  // Pinza simétrica: tomar la pieza con la muñeca girada 180° es el mismo agarre
  float rt = 0, rd = 0;
  bool ok = !USAR_ROLL;          // sin roll: la muñeca queda en 0 todo el ciclo
  if (!USAR_ROLL) giro = 0;
  const float opciones[3] = { 0.0, -180.0, 180.0 };
  for (uint8_t k = 0; k < 3 && !ok; k++) {
    float cand = envolver(phi_p + ROLL0 + opciones[k]);
    if (fabs(cand) <= 90.0 && fabs(cand + giro) <= 90.0) {
      rt = cand; rd = cand + giro; ok = true;
    }
  }
  if (!ok) {
    Serial.println(F("ERR no hay roll valido para esta orientacion"));
    return false;
  }

  // Altura de aproximación sobre la pieza: Z_ALTO si alcanza; si la pieza está
  // lejos (con pitch -90 a 100 mm solo llega a ~213 mm), se baja de a 5 mm
  // hasta Z_APROX_MIN (a 70 mm llega a ~239 mm, a 50 mm a ~250 mm).
  float zAprox = Z_ALTO;
  {
    int tmp[NUM_SERVOS];
    ikSilencioso = true;
    while (zAprox > max(Z_APROX_MIN, zToma + 20.0f) &&
           !calcularIK(xp, yp, zAprox, PITCH_CICLO, rt, GARRA_ABIERTA, tmp)) {
      zAprox -= 5.0;
    }
    if (zAprox < max(Z_APROX_MIN, zToma + 20.0f)) zAprox = max(Z_APROX_MIN, zToma + 20.0f);
    ikSilencioso = false;
  }
  if (zAprox < Z_ALTO) {
    Serial.print(F("OK pieza lejos, aproximacion a z=")); Serial.println(zAprox);
  }

  Punto ciclo[8] = {
    { xp, yp, zAprox, rt, GARRA_ABIERTA },   // 1 encima de la pieza
    { xp, yp, zToma , rt, GARRA_ABIERTA },   // 2 baja
    { xp, yp, zToma , rt, garraCerrada  },   // 3 agarra
    { xp, yp, zAprox, rt, garraCerrada  },   // 4 sube
    { XD, YD, Z_ALTO, rd, garraCerrada  },   // 5 va al lateral y reorienta
    { XD, YD, Z_DEP,  rd, garraCerrada  },   // 6 baja
    { XD, YD, Z_DEP,  rd, GARRA_ABIERTA },   // 7 suelta
    { XD, YD, Z_ALTO, rd, GARRA_ABIERTA }    // 8 sube
  };

  // La pieza tiene que quedarse quieta mientras el brazo la toma
  // Antes de tomar la pieza, comprobar que el destino es alcanzable (si no,
  // el brazo quedaría con la pieza en la garra a mitad del ciclo)
  {
    int tmp[NUM_SERVOS];
    if (!calcularIK(XD, YD, Z_ALTO, PITCH_CICLO, rd, garraCerrada, tmp) ||
        !calcularIK(XD, YD, Z_DEP,  PITCH_CICLO, rd, garraCerrada, tmp)) {
      Serial.println(F("ERR destino inalcanzable, revisar R_D / ANG_D / Z_DEP"));
      return false;
    }
  }

  int motorAntes = motorObjetivo;
  if (DETENER_BANDA_EN_CICLO) motorDetenerSuave();

  estadoActual = "CICLO";
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("PUNTO ")); Serial.print(i + 1); Serial.println(F("/8"));
    if (!moverA(ciclo[i].x, ciclo[i].y, ciclo[i].z, PITCH_CICLO, ciclo[i].roll, ciclo[i].garra)) {
      if (!parada) Serial.println(F("ERR ciclo abortado"));
      estadoActual = parada ? "PARADA" : "REPOSO";
      return false;
    }
    esperar(PAUSA_MS);
    if (parada) return false;
  }

  moverAHome();
  if (parada) return false;
  piezasProcesadas++;
  estadoActual = "REPOSO";
  if (DETENER_BANDA_EN_CICLO) motorObjetivo = motorAntes;   // reanuda la banda
  Serial.print(F("OK ciclo completo, piezas=")); Serial.println(piezasProcesadas);
  return true;
}

// =========================================================================
// Comandos
// =========================================================================
void reportarEstado() {
  Serial.print(F("ESTADO ")); Serial.print(estadoActual);
  Serial.print(F(" piezas=")); Serial.print(piezasProcesadas);
  Serial.print(F(" angulos="));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(currentAngles[i]);
    if (i < NUM_SERVOS - 1) Serial.print(',');
  }
  Serial.print(F(" motor=")); Serial.print(motorActual);
  Serial.print('/'); Serial.print(motorObjetivo);
  Serial.print(F(" duty=")); Serial.print(motorDutyDirecto);
  Serial.print(F(" zToma=")); Serial.print(zToma);
  Serial.print(F(" garra=")); Serial.print(garraCerrada);
  Serial.println();
}

bool esEntero(const String &s) {
  if (s.length() == 0) return false;
  unsigned int i0 = (s[0] == '-' || s[0] == '+') ? 1 : 0;
  if (i0 >= s.length()) return false;
  for (unsigned int i = i0; i < s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char letra = toupper(cmd[0]);
  String resto = cmd.substring(1);
  resto.trim();

  // Parada y rearme se atienden siempre
  if (letra == '!') {
    parada = true; estadoActual = "PARADA";
    motorDetenerYa();
    Serial.println(F("PARADA"));
    return;
  }
  if (letra == 'R') {
    parada = false; estadoActual = "REPOSO";
    Serial.println(F("OK rearmado"));
    return;
  }
  if (letra == '?') { reportarEstado(); return; }

  if (parada) {
    Serial.println(F("ERR en parada, enviar R para rearmar"));
    return;
  }

  if (letra == 'P') {
    float x, y, phi;
    if (sscanf(resto.c_str(), "%f,%f,%f", &x, &y, &phi) == 3) {
      Serial.print(F("OK pose recibida x=")); Serial.print(x);
      Serial.print(F(" y=")); Serial.print(y);
      Serial.print(F(" phi=")); Serial.println(phi);
      ejecutarCiclo(x, y, phi);
    } else {
      Serial.println(F("ERR formato, usar: P x,y,phi"));
    }
    return;
  }

  if (letra == 'C') {
    float x, y, z, p, r; int g;
    if (sscanf(resto.c_str(), "%f,%f,%f,%f,%f,%d", &x, &y, &z, &p, &r, &g) == 6) {
      if (moverA(x, y, z, p, r, g)) Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR formato, usar: C x,y,z,pitch,roll,garra"));
    }
    return;
  }

  if (letra == 'V') {
    if (esEntero(resto)) {
      velocidadMovimiento = max(0, (int)resto.toInt());
      Serial.print(F("OK velocidad=")); Serial.println(velocidadMovimiento);
    }
    return;
  }

  if (letra == 'M') {
    if (!esEntero(resto)) { Serial.println(F("ERR formato, usar: M -100..100")); return; }
    int v = resto.toInt();
    if (v < -100 || v > 100) { Serial.println(F("ERR fuera de -100..100")); return; }
    if (motorDutyDirecto != 0) motorAplicarDutyDirecto(0);  // sale del modo D
    // M0 frena al instante (sin rampa): la cámara lo manda al ver una pieza y
    // cada ms de más la saca del encuadre. La rampa queda solo para acelerar.
    if (v == 0 && FRENAR_SIN_RAMPA) {
      motorDetenerYa();
      Serial.println(F("OK motor=0"));
      return;
    }
    motorObjetivo = v;
    Serial.print(F("OK motor=")); Serial.println(motorObjetivo);
    return;
  }

  if (letra == 'D') {
    if (!esEntero(resto)) { Serial.println(F("ERR formato, usar: D -255..255")); return; }
    int d = resto.toInt();
    if (d < -255 || d > 255) { Serial.println(F("ERR fuera de -255..255")); return; }
    motorAplicarDutyDirecto(d);
    Serial.print(F("OK duty=")); Serial.println(motorDutyDirecto);
    return;
  }

  if (letra == 'K') {
    if (!esEntero(resto)) { Serial.println(F("ERR formato, usar: K angulo")); return; }
    int g = resto.toInt();
    if (g == GARRA_ABIERTA || g < ANGLE_MIN || g > ANGLE_MAX) {
      Serial.println(F("ERR debe estar en -90..90 y ser distinto de la garra abierta"));
      return;
    }
    garraCerrada = g;
    Serial.print(F("OK garra cerrada=")); Serial.println(garraCerrada);
    return;
  }

  if (letra == 'Z') {
    if (!esEntero(resto)) { Serial.println(F("ERR formato, usar: Z mm")); return; }
    int z = resto.toInt();
    if (z < 0 || z > Z_ALTO - 10) {
      Serial.print(F("ERR debe estar entre 0 y ")); Serial.println((int)(Z_ALTO - 10));
      return;
    }
    zToma = z;
    Serial.print(F("OK altura de agarre=")); Serial.println(zToma);
    return;
  }

  if (letra == '0') {
    moveAll(0);
    Serial.println(F("OK centro"));
    return;
  }

  // Modo manual: un eje
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (servoLetters[i] == letra) {
      if (!esEntero(resto)) { Serial.println(F("ERR angulo invalido")); return; }
      int ang = resto.toInt();
      if (ang < ANGLE_MIN || ang > ANGLE_MAX) { Serial.println(F("ERR fuera de -90..90")); return; }
      int obj[NUM_SERVOS];
      for (uint8_t j = 0; j < NUM_SERVOS; j++) obj[j] = currentAngles[j];
      obj[i] = ang;
      estadoActual = "MANUAL";
      moverSuavemente(obj);
      if (!parada) { estadoActual = "REPOSO"; Serial.println(F("OK")); }
      return;
    }
  }

  Serial.print(F("ERR comando desconocido: ")); Serial.println(letra);
}

// =========================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  motorIniciar();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) moveServo(servoChannels[i], homeAngles[i]);
  delay(1000);
  Serial.println(F("OK listo"));
}

void loop() {
  actualizarMotor();
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}
