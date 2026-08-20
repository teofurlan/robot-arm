// =======================================================
//  ARDUINO MEGA 2560 — 3 steppers + 2 servos + HOME VIRTUAL
//  Modo ANGULOS ABSOLUTOS (control por herramienta, brazo MTDS v1)
// =======================================================
//  QUE CAMBIO Y POR QUE
//  --------------------
//  Antes el Mega recibia VELOCIDADES (<vx1,vy1,vx2,vy2,pot,reset>) y movia
//  un motor por cada eje del joystick. Ahora la ESP32 resuelve el
//  movimiento de la herramienta y manda ANGULOS ABSOLUTOS de junta:
//
//        <T1,T2,T3,Roll,Pinza>      angulos en grados, pinza 0..255
//        <H>                        pedido de home
//
//  El Mega solo tiene que llevar cada eje a su angulo. Eso lo hace mucho
//  mas simple y saca de aca toda la logica de control.
//
//  ⚠️ CAMBIO DE SEGURIDAD IMPORTANTE. Con velocidades, el watchdog ponia
//  las velocidades en cero y el brazo se detenia. Con angulos absolutos,
//  poner el objetivo en cero seria una orden de VOLVER AL ORIGEN a toda
//  velocidad — exactamente lo contrario de detenerse. Por eso ahora el
//  watchdog fija el objetivo = posicion actual: el brazo se queda quieto
//  donde esta.
//
//  ⚠️ CONVENCION DEL HOME: al terminar el home, los tres contadores de
//  pasos quedan en 0, o sea que la pose de home ES el angulo 0 de las tres
//  juntas. La ESP32 tambien pone su estado en cero al pedir el home, asi
//  que los dos lados quedan de acuerdo.
//  TODO de calibracion: ajustar los offsets del home para que ese cero
//  coincida con el cero del CAD (ver docs/CONTROL_V1.md).
// =======================================================
#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>

// ===== RAMPS 1.6 PINS - STEPPERS =====
#define X_STEP 54
#define X_DIR 55
#define X_EN 38
#define Y_STEP 60
#define Y_DIR 61
#define Y_EN 56
#define Z_STEP 46
#define Z_DIR 48
#define Z_EN 62

// ===== PIN SENSOR =====
#define X_MIN_PIN 3
#define Y_MIN_PIN 14

// ===== SERVO PINS =====
const int PIN_PINZA = 11;
const int PIN_SERVO2 = 6;

// ===== LIMITES DE RECORRIDO, EN PASOS =====
//  Mismos valores que tenia el firmware de velocidades. Con 17.77 pasos por
//  grado equivalen a: X +-10.02 deg, Y -59.99..+10.02 deg, Z +-281.4 deg.
//  La ESP32 ya recorta los angulos con estos mismos limites (T_MIN/T_MAX de
//  geom_mtds.h); aca se vuelven a aplicar como red de seguridad, porque el
//  Mega es el ultimo que toca los motores.
const long LIM_X = 178;
const long LIM_Y_DER = 178;
const long LIM_Y_IZQ = -1066;
const long LIM_Z_MAX = 5000;
const long LIM_Z_MIN = -5000;

// ===== CONVERSION =====
//  Confirmada en el firmware anterior y en la documentacion del proyecto.
const float STEPS_PER_DEGREE = 17.77f;

// ⚠️ TODO — MAPEO MOTOR -> JUNTA. Esta ASUMIDO como X->J1 (giro del cardan),
//  Y->J2 (inclinacion del cardan), Z->J3 (codo), por el orden en que estaban
//  los motores en el firmware de velocidades y porque los rangos calzan con
//  "la inclinacion lateral tiene recorrido estricto". HAY QUE CONFIRMARLO EN
//  EL BRAZO antes de confiar en los limites: si esta cruzado, los topes
//  protegen la junta equivocada, que es peor que no tenerlos.

// ===== ESTADO =====
long posX = 0, posY = 0, posZ = 0;          // pasos actuales
long tgtX = 0, tgtY = 0, tgtZ = 0;          // pasos objetivo
unsigned long lastStepX = 0, lastStepY = 0, lastStepZ = 0;
unsigned long lastCmdTime = 0;
bool haciendoHome = false;
bool linkUp = false;

float wristAngle = 0.0f;                     // grados de roll, -90..90
int   clampValue = 0;                        // 0..255 del potenciometro

const unsigned long WATCHDOG_TIMEOUT = 500;
//  Intervalo minimo entre pasos. La ESP32 limita cada junta a 0.9 deg por
//  ciclo de 20 ms = ~16 pasos, o sea ~800 pasos/s. 400 us da ~2500 pasos/s:
//  sobra para seguir el objetivo sin quedarse atras.
const unsigned long MIN_STEP_INTERVAL = 400;

Servo clamp;
Servo wrist;

// ===== PROTOTIPOS =====
void stepTowards(int stepPin, int dirPin, long &pos, long target,
                 unsigned long &lastStep, long minLimit, long maxLimit);
void ejecutarHomeVirtual();
long angleToSteps(float deg, long minLimit, long maxLimit);
void procesarTrama(const char *data);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_EN, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_EN, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT); pinMode(Z_EN, OUTPUT);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);

  digitalWrite(X_EN, LOW);
  digitalWrite(Y_EN, LOW);
  digitalWrite(Z_EN, LOW);

  clamp.attach(PIN_PINZA);
  wrist.attach(PIN_SERVO2);
  clamp.write(0);
  wrist.write(90);                 // 90 = roll 0

  Serial.println(F("Mega iniciado - modo angulos absolutos <T1,T2,T3,Roll,Pinza>"));
}

void loop()
{
  // ---------- 1. RECEPCION ----------
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    int a = input.indexOf('<');
    int b = input.indexOf('>');
    if (a != -1 && b != -1 && b > a) {
      String data = input.substring(a + 1, b);
      if (data == "H" || data == "h") {
        lastCmdTime = millis();
        linkUp = true;
        ejecutarHomeVirtual();
      } else {
        procesarTrama(data.c_str());
      }
    }
  }

  // ---------- 2. WATCHDOG ----------
  //  Sin comandos nuevos: quedarse QUIETO donde esta (no volver al origen).
  if (linkUp && millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
    tgtX = posX; tgtY = posY; tgtZ = posZ;
    if (linkUp) {
      Serial.println(F("WATCHDOG: sin comandos, manteniendo posicion"));
      linkUp = false;
    }
  }

  // ---------- 3. MOVIMIENTO ----------
  if (!haciendoHome) {
    stepTowards(X_STEP, X_DIR, posX, tgtX, lastStepX, -LIM_X, LIM_X);
    stepTowards(Y_STEP, Y_DIR, posY, tgtY, lastStepY, LIM_Y_IZQ, LIM_Y_DER);
    stepTowards(Z_STEP, Z_DIR, posZ, tgtZ, lastStepZ, LIM_Z_MIN, LIM_Z_MAX);
  }
}

// ---------------------------------------------------------------------
//  Parseo de "<T1,T2,T3,Roll,Pinza>".
//  Se usa strtod y NO sscanf con %f: en AVR, sscanf solo soporta %f si se
//  enlaza a proposito la version de scanf con punto flotante, asi que
//  hacerlo con sscanf falla en silencio en el Mega.
// ---------------------------------------------------------------------
void procesarTrama(const char *data)
{
  float v[5];
  const char *p = data;
  for (int i = 0; i < 5; ++i) {
    char *end = NULL;
    v[i] = (float)strtod(p, &end);
    if (end == p) return;                 // trama corrupta: se descarta entera
    p = end;
    while (*p == ' ') ++p;
    if (i < 4) {
      if (*p != ',') return;
      ++p;
    }
  }

  tgtX = angleToSteps(v[0], -LIM_X, LIM_X);
  tgtY = angleToSteps(v[1], LIM_Y_IZQ, LIM_Y_DER);
  tgtZ = angleToSteps(v[2], LIM_Z_MIN, LIM_Z_MAX);

  wristAngle = constrain(v[3], -90.0f, 90.0f);
  clampValue = constrain((int)v[4], 0, 255);

  lastCmdTime = millis();
  linkUp = true;

  if (!haciendoHome) {
    // El SG5010 tiene ~180 grados utiles: roll 0 cae en el centro (90).
    wrist.write((int)(90.0f + wristAngle));
    clamp.write(map(clampValue, 0, 255, 0, 180));
  }
}

long angleToSteps(float deg, long minLimit, long maxLimit)
{
  long s = (long)lround((double)deg * (double)STEPS_PER_DEGREE);
  if (s < minLimit) s = minLimit;
  if (s > maxLimit) s = maxLimit;
  return s;
}

// ---------------------------------------------------------------------
//  Un paso hacia el objetivo, respetando el intervalo minimo y los topes.
//  Como la ESP32 manda incrementos chicos a 50 Hz, avanzar cada eje de
//  forma independiente a paso fijo ya sale practicamente coordinado: no
//  hace falta MultiStepper ni sumar dependencias.
// ---------------------------------------------------------------------
void stepTowards(int stepPin, int dirPin, long &pos, long target,
                 unsigned long &lastStep, long minLimit, long maxLimit)
{
  if (target < minLimit) target = minLimit;
  if (target > maxLimit) target = maxLimit;
  if (pos == target) return;
  if (micros() - lastStep < MIN_STEP_INTERVAL) return;

  const bool up = (target > pos);
  if (up && pos >= maxLimit) return;
  if (!up && pos <= minLimit) return;

  digitalWrite(dirPin, up ? HIGH : LOW);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(stepPin, LOW);

  pos += up ? 1 : -1;
  lastStep = micros();
}

// ---------------------------------------------------------------------
//  HOME VIRTUAL. Al terminar, los tres contadores quedan en 0: la pose de
//  home ES el angulo cero de las tres juntas.
// ---------------------------------------------------------------------
void ejecutarHomeVirtual()
{
  haciendoHome = true;
  Serial.println(F("HOME: iniciando..."));

  // ---- eje X ----
  digitalWrite(X_DIR, LOW);
  while (digitalRead(X_MIN_PIN) == HIGH) {
    digitalWrite(X_STEP, HIGH); delayMicroseconds(1000);
    digitalWrite(X_STEP, LOW);  delayMicroseconds(1000);
  }
  delay(200);
  Serial.println(F("HOME X: sensor tocado, moviendo al offset..."));
  digitalWrite(X_DIR, HIGH);
  for (int i = 0; i < 178; i++) {
    digitalWrite(X_STEP, HIGH); delayMicroseconds(1200);
    digitalWrite(X_STEP, LOW);  delayMicroseconds(1200);
  }

  // ---- eje Y ----
  Serial.println(F("HOME Y: buscando sensor..."));
  digitalWrite(Y_DIR, LOW);
  while (digitalRead(Y_MIN_PIN) == HIGH) {
    digitalWrite(Y_STEP, HIGH); delayMicroseconds(1000);
    digitalWrite(Y_STEP, LOW);  delayMicroseconds(1000);
  }
  digitalWrite(Y_DIR, HIGH);
  for (int i = 0; i < 200; i++) {
    digitalWrite(Y_STEP, HIGH); delayMicroseconds(1200);
    digitalWrite(Y_STEP, LOW);  delayMicroseconds(1200);
  }

  // El eje Z (codo) no tiene fin de carrera: se toma su posicion actual
  // como cero. TODO: agregar endstop al codo o un cero mecanico marcado.
  posX = 0; posY = 0; posZ = 0;
  tgtX = 0; tgtY = 0; tgtZ = 0;

  wristAngle = 0.0f;
  wrist.write(90);

  while (Serial1.available() > 0) { Serial1.read(); }
  lastCmdTime = millis();
  haciendoHome = false;
  Serial.println(F("HOME finalizado - las tres juntas en angulo 0"));
}
