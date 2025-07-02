#include "esp_now_rover.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ESP_NOW_ROVER";

// Base station MAC address - UPDATE THIS WITH ACTUAL BASE MAC
uint8_t base_mac[6] = {0x24, 0x6F, 0x28, 0xDD, 0xEE, 0xFF};

static uint32_t packets_received = 0;

// ESP-NOW receive callback
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len <= 0 || len > 256) {
        ESP_LOGW(TAG, "Invalid RTCM packet length: %d", len);
        return;
    }
    
    // Verify sender MAC address
    if (memcmp(recv_info->src_addr, base_mac, 6) != 0) {
        ESP_LOGW(TAG, "RTCM packet from unknown sender");
        return;
    }
    
    packets_received++;
    ESP_LOGD(TAG, "RTCM packet received, length: %d, total: %lu", len, packets_received);
    
    // Send RTCM data to queue for UART forwarding
    if (xQueueSend(rtcm_queue, data, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "RTCM queue full, dropping packet");
    }
}

esp_err_t esp_now_rover_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW for rover...");
    
    // Initialize WiFi in STA mode for ESP-NOW
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Set WiFi channel (must match base station)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    // Add base station as peer
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, base_mac, 6);
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;  // No encryption for simplicity
    
    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add base station peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    ESP_LOGI(TAG, "Base station MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             base_mac[0], base_mac[1], base_mac[2], 
             base_mac[3], base_mac[4], base_mac[5]);
    
    return ESP_OK;
}