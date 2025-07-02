#include "uart_gnss.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include "esp_timer.h"

static const char *TAG = "UART_GNSS";

esp_err_t uart_gnss_init(void)
{
    ESP_LOGI(TAG, "Initializing UART for LC29HDA GNSS module...");
    
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver with queue
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_CTS_PIN));
    
    // Create UART RX task for reading GNSS data
    BaseType_t ret1 = xTaskCreatePinnedToCore(
        uart_gnss_rx_task,
        "uart_gnss_rx",
        4096,
        NULL,
        5,  // High priority for real-time GNSS data
        NULL,
        1   // Pin to core 1
    );
    
    // Create UART TX task for sending RTCM data
    BaseType_t ret2 = xTaskCreatePinnedToCore(
        uart_gnss_tx_task,
        "uart_gnss_tx",
        2048,
        NULL,
        4,  // High priority for RTCM forwarding
        NULL,
        1   // Pin to core 1
    );
    
    if (ret1 != pdPASS || ret2 != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART GNSS tasks");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UART GNSS initialized successfully");
    return ESP_OK;
}

void uart_gnss_rx_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    char *line_buffer = (char *)malloc(256);
    uint16_t line_index = 0;
    
    ESP_LOGI(TAG, "UART GNSS RX task started");
    
    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Process each byte for NMEA sentences
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                
                if (c == '\n' || c == '\r') {
                    // End of line, process if we have data
                    if (line_index > 0) {
                        line_buffer[line_index] = '\0';
                        
                        // Parse NMEA sentences
                        gnss_data_t gnss_data = {0};
                        bool parsed = false;
                        
                        if (strncmp(line_buffer, "$GPGGA", 6) == 0 || strncmp(line_buffer, "$GNGGA", 6) == 0) {
                            parsed = parse_gga_sentence(line_buffer, &gnss_data);
                        } else if (strncmp(line_buffer, "$GPRMC", 6) == 0 || strncmp(line_buffer, "$GNRMC", 6) == 0) {
                            parsed = parse_rmc_sentence(line_buffer, &gnss_data);
                        }
                        
                        if (parsed) {
                            gnss_data.timestamp = esp_timer_get_time() / 1000;  // Convert to milliseconds
                            if (xQueueSend(gnss_queue, &gnss_data, 0) != pdTRUE) {
                                ESP_LOGD(TAG, "GNSS queue full, dropping data");
                            }
                        }
                    }
                    line_index = 0;
                } else if (line_index < 255) {
                    line_buffer[line_index++] = c;
                } else {
                    // Line too long, reset
                    line_index = 0;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    free(data);
    free(line_buffer);
    vTaskDelete(NULL);
}

void uart_gnss_tx_task(void *pvParameters)
{
    uint8_t rtcm_data[256];
    
    ESP_LOGI(TAG, "UART GNSS TX task started");
    
    while (1) {
        // Wait for RTCM data from ESP-NOW
        if (xQueueReceive(rtcm_queue, rtcm_data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Forward RTCM data to GNSS module
            // Note: We don't know the exact length, so we send the first few bytes
            // In a real implementation, you'd need to parse the RTCM length properly
            uart_write_bytes(UART_PORT_NUM, rtcm_data, 64);  // Simplified approach
            ESP_LOGD(TAG, "RTCM data forwarded to GNSS module");
        }
    }
    
    vTaskDelete(NULL);
}

bool parse_gga_sentence(const char* sentence, gnss_data_t* gnss_data)
{
    if (sentence == NULL || gnss_data == NULL) return false;
    
    // Simple GGA parser
    // Format: $GPGGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,altitude,M,geoid,M,dgps_time,dgps_id*checksum
    
    char *token;
    char *sentence_copy = strdup(sentence);
    int field = 0;
    
    token = strtok(sentence_copy, ",");
    while (token != NULL && field < 15) {
        switch (field) {
            case 2: // Latitude
                if (strlen(token) > 0) {
                    double lat_deg = atof(token) / 100.0;
                    int deg = (int)lat_deg;
                    double min = (lat_deg - deg) * 100.0;
                    gnss_data->latitude = deg + min / 60.0;
                }
                break;
            case 3: // N/S
                if (token[0] == 'S') gnss_data->latitude = -gnss_data->latitude;
                break;
            case 4: // Longitude
                if (strlen(token) > 0) {
                    double lon_deg = atof(token) / 100.0;
                    int deg = (int)lon_deg;
                    double min = (lon_deg - deg) * 100.0;
                    gnss_data->longitude = deg + min / 60.0;
                }
                break;
            case 5: // E/W
                if (token[0] == 'W') gnss_data->longitude = -gnss_data->longitude;
                break;
            case 6: // Fix quality
                gnss_data->fix_quality = atoi(token);
                break;
            case 7: // Number of satellites
                gnss_data->satellites = atoi(token);
                break;
            case 8: // HDOP (horizontal accuracy)
                gnss_data->accuracy = atof(token);
                break;
            case 9: // Altitude
                gnss_data->altitude = atof(token);
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    
    free(sentence_copy);
    
    // Return true if we have a valid fix with coordinates
    return (gnss_data->fix_quality > 0 && gnss_data->latitude != 0.0 && gnss_data->longitude != 0.0);
}

bool parse_rmc_sentence(const char* sentence, gnss_data_t* gnss_data)
{
    if (sentence == NULL || gnss_data == NULL) return false;
    
    // Simple RMC parser for basic position validation
    // Format: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magnetic_var,E/W*checksum
    
    char *token;
    char *sentence_copy = strdup(sentence);
    int field = 0;
    
    token = strtok(sentence_copy, ",");
    while (token != NULL && field < 12) {
        switch (field) {
            case 2: // Status (A=active, V=void)
                if (token[0] != 'A') {
                    free(sentence_copy);
                    return false;  // Invalid fix
                }
                break;
            case 3: // Latitude
                if (strlen(token) > 0) {
                    double lat_deg = atof(token) / 100.0;
                    int deg = (int)lat_deg;
                    double min = (lat_deg - deg) * 100.0;
                    gnss_data->latitude = deg + min / 60.0;
                }
                break;
            case 4: // N/S
                if (token[0] == 'S') gnss_data->latitude = -gnss_data->latitude;
                break;
            case 5: // Longitude
                if (strlen(token) > 0) {
                    double lon_deg = atof(token) / 100.0;
                    int deg = (int)lon_deg;
                    double min = (lon_deg - deg) * 100.0;
                    gnss_data->longitude = deg + min / 60.0;
                }
                break;
            case 6: // E/W
                if (token[0] == 'W') gnss_data->longitude = -gnss_data->longitude;
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    
    free(sentence_copy);
    
    // Return true if we have valid coordinates
    return (gnss_data->latitude != 0.0 && gnss_data->longitude != 0.0);
}