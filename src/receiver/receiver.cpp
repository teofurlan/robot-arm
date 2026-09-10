// =======================================================
//  RECEIVER (ESP32) — control por herramienta del brazo MTDS v1
// =======================================================
//  QUE CAMBIO Y POR QUE
//  --------------------
//  Antes este archivo era un puente tonto: recibia el paquete ESP-NOW del
//  control remoto y reenviaba los valores crudos de joystick al Mega, que
//  movia un motor por cada eje. El operador tenia que pensar en motores.
//
//  Ahora la ESP32 hace el trabajo: los joysticks comandan el movimiento de
//  la PUNTA DE LA HERRAMIENTA, y ctrl_mtds.h reparte ese movimiento entre
//  J1, J2 y J3 con el jacobiano del brazo. Al Mega se le mandan angulos
//  ABSOLUTOS de junta, no velocidades.
//
//  ⚠️ ESTE FIRMWARE NO IMPLEMENTA RCM, A PROPOSITO.
//  Con 3 juntas moviendo la recta del vastago y 2 restricciones que impone
//  un punto de pivote fijo, quedan 1 grado de libertad controlable y hacen
//  falta 3. Es imposible por conteo de grados de libertad, no por falta de
//  codigo. El punto de pivote se va a mover; es esperable y correcto para
//  este hardware. Ver docs/CONTROL_V1.md.
//
//  Costo medido del calculo: 0.11 us por ciclo en x86. Incluso 50x mas
//  lento en la ESP32 son ~5 us, o sea 0.03% del presupuesto de 20 ms.
// =======================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <stdio.h>

#include "geom_mtds.h"
#include "ctrl_mtds.h"

HardwareSerial MegaSerial(2); // UART2 (RX=16, TX=17)

// =======================================================
// DATA PACKET — sin cambios respecto del controller
// =======================================================
#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t upperJoyX; // -200 a 200
  int16_t upperJoyY; // -200 a 200
  int16_t lowerJoyX; // -200 a 200
  int16_t lowerJoyY; // -200 a 200
  uint8_t pot;       // 0 a 255
  uint8_t reset;     // 0 o 1
} DataPacket;
#pragma pack(pop)
static_assert(sizeof(DataPacket) == 10, "Error Critico: El struct no mide 10 bytes");

// ---- estado compartido con el callback de ESP-NOW ----
static volatile DataPacket rxPacket = {0, 0, 0, 0, 0, 0};
static volatile uint32_t   lastRxMs = 0;
static portMUX_TYPE        rxMux    = portMUX_INITIALIZER_UNLOCKED;

// ---- lazo de control ----
static const uint32_t LOOP_MS = 20;          // 50 Hz, igual que el envio del control
static const uint32_t WATCHDOG_MS = 500;     // sin paquetes -> parar
static const float    DT = (float)LOOP_MS / 1000.0f;

static CtrlState ctrl;
static CtrlMode  mode = CTRL_TOOL;           // CTRL_TOOL o CTRL_WORLD
static bool      prevReset = false;
static uint32_t  nextLoopMs = 0;
static uint32_t  dbgCount = 0;

// =======================================================
// CALLBACK ESP-NOW — solo copia; el trabajo se hace en el loop
// =======================================================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  (void)mac;
  if (len != (int)sizeof(DataPacket)) return;
  portENTER_CRITICAL(&rxMux);
  memcpy((void *)&rxPacket, incomingData, sizeof(DataPacket));
  lastRxMs = millis();
  portEXIT_CRITICAL(&rxMux);
}

// =======================================================
// SETUP
// =======================================================
void setup()
{
  Serial.begin(115200);
  MegaSerial.begin(115200, SERIAL_8N1, 16, 17);

  ctrl_init(ctrl);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  nextLoopMs = millis();
  float tip[3];
  ctrl_tip(0.0f, 0.0f, 0.0f, tip);
  Serial.println("ESP-NOW iniciado. Control por herramienta activo (sin RCM).");
  Serial.printf("Modo: %s | punta en reposo: X=%.1f Y=%.1f Z=%.1f mm\n",
                mode == CTRL_TOOL ? "marco de la herramienta" : "marco del robot",
                tip[0], tip[1], tip[2]);
}

// =======================================================
// LOOP — lazo de control de tiempo fijo
// =======================================================
void loop()
{
  const uint32_t now = millis();
  if ((int32_t)(now - nextLoopMs) < 0) return;   // resta con signo: sobrevive al wrap de millis()
  nextLoopMs += LOOP_MS;
  // si nos atrasamos mas de un ciclo entero, resincronizar en vez de acumular deuda
  if ((int32_t)(now - nextLoopMs) > (int32_t)LOOP_MS) nextLoopMs = now + LOOP_MS;

  // ---- copia atomica del ultimo paquete ----
  DataPacket p;
  uint32_t rxMs;
  portENTER_CRITICAL(&rxMux);
  memcpy(&p, (const void *)&rxPacket, sizeof(DataPacket));
  rxMs = lastRxMs;
  portEXIT_CRITICAL(&rxMux);

  const bool stale = (rxMs == 0) || ((uint32_t)(now - rxMs) > WATCHDOG_MS);

  float a = 0.0f, b = 0.0f, c = 0.0f, roll = 0.0f, grip = ctrl.grip;
  if (!stale) {
    ctrl_from_packet(p.upperJoyX, p.upperJoyY, p.lowerJoyX, p.lowerJoyY, p.pot,
                     a, b, c, roll, grip);
  }
  // Si el enlace se cayo, el brazo SE QUEDA DONDE ESTA. Dos cosas hacen falta
  // para eso, y las dos importan:
  //  1) NO mandar un objetivo cero. Con angulos absolutos, cero no significa
  //     "pare": significa "volve al origen", y a toda velocidad.
  //  2) Cortar el comando con ctrl_stop() y no solo ponerlo en cero. El
  //     filtro de suavizado tarda ~0,6 s en decaer, y en ese tiempo el brazo
  //     seguiria moviendose por inercia. ctrl_stop() lo corta en un ciclo.
  if (stale) ctrl_stop(ctrl);

  // ---- boton de reset (los dos joysticks a la vez) -> home ----
  const bool resetNow = (!stale && p.reset != 0);
  if (resetNow && !prevReset) {
    ctrl_init(ctrl);                 // el estado interno vuelve a cero...
    MegaSerial.print("<H>\n");       // ...y el Mega hace el home fisico
    Serial.println("RESET -> home solicitado; estado de control puesto en cero");
  }
  prevReset = resetNow;

  // ---- un paso de control ----
  ctrl_step(ctrl, mode, a, b, c, roll, grip, DT);

  // ---- trama de angulos absolutos al Mega ----
  char frame[64];
  ctrl_frame(ctrl, frame, sizeof frame);
  MegaSerial.print(frame);

  // ---- debug, 4 veces por segundo para no saturar el puerto ----
  if (++dbgCount % 12 == 0) {
    Serial.printf("T1:%.2f T2:%.2f T3:%.2f Roll:%.1f Pinza:%d%s%s%s\n",
                  ctrl.t1 * 57.2957795f, ctrl.t2 * 57.2957795f, ctrl.t3 * 57.2957795f,
                  ctrl.roll, (int)(ctrl.grip * 255.0f),
                  stale ? " [SIN ENLACE]" : "",
                  ctrl.limited ? " [TOPE]" : "",
                  ctrl.singular ? " [SINGULARIDAD]" : "");
  }
}
