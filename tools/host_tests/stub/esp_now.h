#ifndef ESPNOW_STUB_H
#define ESPNOW_STUB_H
#include <cstdint>
#define ESP_OK 0
typedef int esp_err_t;
inline esp_err_t esp_now_init(){ return ESP_OK; }
typedef void (*esp_now_recv_cb_t)(const uint8_t*, const uint8_t*, int);
extern esp_now_recv_cb_t g_recvCb;
inline void esp_now_register_recv_cb(esp_now_recv_cb_t cb){ g_recvCb = cb; }
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*){}
inline void portEXIT_CRITICAL(portMUX_TYPE*){}
#endif
