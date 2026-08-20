#ifndef WIFI_STUB_H
#define WIFI_STUB_H
#include "Arduino.h"
#define WIFI_STA 1
struct WiFiStub { void mode(int){} }; extern WiFiStub WiFi;
class HardwareSerial : public SerialStub { public: HardwareSerial(int){} };
#endif
