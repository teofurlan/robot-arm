/**
 * ==============================================================
 *  Remote Control for MTDs Robot - ESP32 Joystick Controller
 ==============================================================
 *  Dispositive: ESP32
 * Description:
 * This code implements a remote control system for a robot using an ESP32 microcontroller.
 * It reads input from two joysticks and a potentiometer, constructs a data packet, and sends it to the robot using ESP-NOW wireless communication.
 *
 =============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "Joystick.h"

// --- CONFIGURACIÓN LED RGB (Cátodo Común) ---
const int pinR = 18;
const int pinG = 19;
const int pinB = 17;

// Receiver MAC Address (replace with your robot's MAC address)
// uint8_t receiverAddress[] = {0xA4, 0xF0, 0x0F, 0x69, 0xBB, 0xBC};
uint8_t receiverAddress[] = {0x84, 0x1F, 0xE8, 0x26, 0x5A, 0x04};

// Potentiometer pin definition
const int POT_PIN = 32;
// Joystick Buttons
const int UP_JOY_BUTTON = 22;  // Button on upper joystick
const int LOW_JOY_BUTTON = 21; // Button on lower joystick
// Joystick definitions (connected to ADC pins)
Joystick upperJoystick(35, 34);
Joystick lowerJoystick(39, 36);

// Timing
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 20; // 20 ms → 50 Hz

// =======================================================
// DATA PACKET STRUCTURE
// =======================================================

/**
 * Data packet definition to be sent to the robot. The struct is packed to ensure no padding bytes are added, guaranteeing a fixed size of 5 bytes.
 *
 * The struct uses __attribute__((packed)) to enforce this guarantee.
 */

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t joy1X;
  int16_t joy1Y;
  int16_t joy2X;
  int16_t joy2Y;
  uint8_t pot;   // potentiometer value (0-180)
  uint8_t reset; // reset state (0 or 1)
} DataPacket;
#pragma pack(pop)
// Static assertion to ensure the struct size is exactly 10 bytes (4 int16_t + 2 uint8_t)
static_assert(sizeof(DataPacket) == 10, "DataPacket struct size must be exactly 10 bytes");

esp_now_peer_info_t peerInfo;

// =======================================================
// FUNCTIONS
// =======================================================

// =======================================================
// FUNCIÓN PARA CONTROLAR EL LED
// =======================================================
void actualizarLED(int r, int g, int b) {
  // Forzamos que los otros pines se apaguen ANTES de encender el nuevo
  analogWrite(pinR, r);
  analogWrite(pinG, g);
  analogWrite(pinB, b);
}


// =======================================================
// CALLBACK DE ENVÍO: Aquí detectamos la conexión real
// =======================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nStatus de entrega: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("ÉXITO");
    actualizarLED(0, 0, 255); // Azul
  } else {
    Serial.println("FALLO");
    actualizarLED(255, 0, 0); // Rojo
  }
}


/**
 * sendData() - Reads the current state of the joysticks and potentiometer, constructs a data packet, and sends it to the robot using ESP-NOW.
 */

void sendData()
{
  // Joystick raw readings
  JoyPosition lowerJoystickPos = lowerJoystick.readPosition();
  JoyPosition upperJoystickPos = upperJoystick.readPosition();

  // Potentiometer reading
  // int rawPot = analogRead(POT_PIN);
  // int potValue = map(rawPot, 0, 4095, 0, 180); // Map directly to 0-180 range

  static int lastValidPot = -1; // Store the last valid potentiometer value to implement a simple noise filter
  const int POT_THRESHOLD = 3;  // Tolerance: only update if the change is greater than this threshold to avoid noise
  // Read raw potentiometer value and map it to 0-180 range
  int rawPot = analogRead(POT_PIN);
  int currentPot = map(rawPot, 0, 4095, 0, 180);
  // Check if the change in potentiometer value exceeds the threshold
  if (lastValidPot == -1 || abs(currentPot - lastValidPot) >= POT_THRESHOLD)
  {
    lastValidPot = currentPot;
  }
  // Check if both joystick buttons are pressed for reset state
  bool resetState = ((digitalRead(UP_JOY_BUTTON) == LOW) && (digitalRead(LOW_JOY_BUTTON) == LOW)); 
  // Create data packet
  DataPacket packet;
  packet.joy1X = (int16_t)upperJoystickPos.x;
  packet.joy1Y = (int16_t)upperJoystickPos.y;
  packet.joy2X = (int16_t)lowerJoystickPos.x;
  packet.joy2Y = (int16_t)lowerJoystickPos.y;
  packet.pot = (uint8_t)lastValidPot;
  packet.reset = (uint8_t)resetState;
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&packet, sizeof(packet));

  if (result == ESP_OK)
  {
    Serial.printf("Enviando -> vx1:%d vy1:%d vx2:%d vy2:%d POT:%d RESET:%d\n", upperJoystickPos.x, upperJoystickPos.y, lowerJoystickPos.x, lowerJoystickPos.y, lastValidPot, resetState);
  }
  else
  {
    Serial.printf("Error sending data: %d\n", result);
  }
}

// SETUP
void setup()
{
  // Begin Serial communication for debugging and having feedback
  Serial.begin(115200);
  // Configuración de pines LED
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);
  
  // Color inicial (Naranja/Amarillo) indicando "Iniciando..."
  actualizarLED(0, 255, 0);

  // Configure button pins with pull-up resistors
  pinMode(UP_JOY_BUTTON, INPUT_PULLUP);
  pinMode(LOW_JOY_BUTTON, INPUT_PULLUP);

  // Configure ADC for potentiometer reading
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("Calibrando centro, no muevas los joysticks...");
  delay(500);
  // Calibrate joysticks to set the center position
  upperJoystick.calibrate();
  lowerJoystick.calibrate();
  Serial.println("Calibración completa!");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error inicializando ESP-NOW");
    actualizarLED(255, 0, 0);
    return;
  };

 // Forzamos al compilador a aceptar el formato de la función
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0; // use current Wi-Fi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Fallo al emparejar con el receptor");
    return;
  }
  Serial.println("ESP-NOW Iniciado y emparejado.");
  actualizarLED(0, 0, 255);
}

// MAIN LOOP
void loop()
{
  // Send data at the defined interval
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL)
  {
    lastSend = now;
    sendData();
  }
}
