#ifndef ESP_NOW_ROVER_H
#define ESP_NOW_ROVER_H

#include "esp_now.h"
#include "esp_wifi.h"

// Base station MAC address (you'll need to update this with actual base MAC)
extern uint8_t base_mac[6];

// RTCM data structure (same as base station)
typedef struct {
    uint8_t data[256];
    uint16_t length;
    uint32_t timestamp;
} rtcm_packet_t;

// External queue handle
extern QueueHandle_t rtcm_queue;

// Function prototypes
esp_err_t esp_now_rover_init(void);
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

#endif // ESP_NOW_ROVER_H