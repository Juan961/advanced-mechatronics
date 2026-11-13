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
const uint8_t gripper  = 5; // MG90S

const uint8_t NUM_SERVOS = 6;
const uint8_t servoChannels[NUM_SERVOS] = { base, shoulder, elbow, wrist, hand, gripper };
const char servoLetters[NUM_SERVOS]     = { 'B', 'S', 'E', 'W', 'H', 'G' };

#define ANGLE_MIN -90
#define ANGLE_MAX  90

// Pose de arranque (ángulos lógicos). Revisar la garra: en el ciclo
// abierta = -40 y cerrada = 0.
const int homeAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0, 90 };
int currentAngles[NUM_SERVOS]    = { 0, 0, 0, 0, 0, 90 };

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
const int OFFSET_FISICO_BASE = 20;

// =========================================================================
// --- PARÁMETROS DEL CICLO (deben coincidir con el controlador de Simulink) ---
// Marco del robot, mm y grados
// =========================================================================
const float XD = 85.0,  YD = -147.0, PHI_D = -60.0;  // destino en la plantilla
const float Z_ALTO = 100.0;     // altura de tránsito de la punta
const float Z_TOMA = 30.0;      // altura de la punta al agarrar
const float Z_DEP  = 35.0;      // altura de la punta al soltar
const float PITCH_CICLO = -90.0;
const int   GARRA_ABIERTA = -40;
const int   GARRA_CERRADA = 0;
const float ROLL0  = 0.0;       // roll que alinea los dedos con la pieza (phi = 0)
const float S_ROLL = 1.0;       // -1 si la reorientación sale para el lado contrario
const unsigned long PAUSA_MS = 500;

// --- Estado ---
bool parada = false;
unsigned long piezasProcesadas = 0;
const char* estadoActual = "REPOSO";

// =========================================================================
// Bajo nivel
// =========================================================================
void moveServo(uint8_t channel, int angle) {
  if (channel == base) angle += OFFSET_FISICO_BASE;
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);

  unsigned int us;
  if (channel == base || channel == elbow) {
    us = map(angle, ANGLE_MIN, ANGLE_MAX, US_MIN_MG996, US_MAX_MG996);
  } else {
    us = map(angle, ANGLE_MIN, ANGLE_MAX, US_MIN_STD, US_MAX_STD);
  }
  int pulse = (int)(us * SERVO_FREQ * 4096.0 / 1000000.0);
  pwm.setPWM(channel, 0, pulse);
}

// Revisa si llegó un '!' por serial mientras el brazo se mueve
void revisarParada() {
  if (Serial.available() && Serial.peek() == '!') {
    Serial.read();
    parada = true;
    estadoActual = "PARADA";
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

// 1 grado por paso en cada eje, todos a la vez
void moverSuavemente(const int objetivos[NUM_SERVOS]) {
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
    Serial.println(F("ERR coordenada inalcanzable"));
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
    if (calc[i] < ANGLE_MIN || calc[i] > ANGLE_MAX) {
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
  bool ok = false;
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

  Punto ciclo[8] = {
    { xp, yp, Z_ALTO, rt, GARRA_ABIERTA },   // 1 encima de la pieza
    { xp, yp, Z_TOMA, rt, GARRA_ABIERTA },   // 2 baja
    { xp, yp, Z_TOMA, rt, GARRA_CERRADA },   // 3 agarra
    { xp, yp, Z_ALTO, rt, GARRA_CERRADA },   // 4 sube
    { XD, YD, Z_ALTO, rd, GARRA_CERRADA },   // 5 va al lateral y reorienta
    { XD, YD, Z_DEP,  rd, GARRA_CERRADA },   // 6 baja
    { XD, YD, Z_DEP,  rd, GARRA_ABIERTA },   // 7 suelta
    { XD, YD, Z_ALTO, rd, GARRA_ABIERTA }    // 8 sube
  };

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

  for (uint8_t i = 0; i < NUM_SERVOS; i++) moveServo(servoChannels[i], homeAngles[i]);
  delay(1000);
  Serial.println(F("OK listo"));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}
