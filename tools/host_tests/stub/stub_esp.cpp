#include "WiFi.h"
#include "esp_now.h"
WiFiStub WiFi;
esp_now_recv_cb_t g_recvCb = nullptr;
