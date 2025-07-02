#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_now_rover.h"
#include "uart_gnss.h"
#include "imu.h"
#include "motor_control.h"
#include "navigation.h"

static const char *TAG = "RTK_ROVER";

// Global queues for inter-task communication
QueueHandle_t rtcm_queue;
QueueHandle_t gnss_queue;
QueueHandle_t imu_queue;
QueueHandle_t nav_queue;

// Hardcoded waypoints (latitude, longitude in decimal degrees)
waypoint_t mission_waypoints[] = {
    {44.787197, 20.457273, 0.5},  // Beograd - Terazije
    {44.787500, 20.457500, 0.5},  // 50m north-east
    {44.787800, 20.457800, 0.5},  // Another 50m north-east
    {44.787197, 20.457273, 0.5}   // Return to start
};



void app_main(void)
{
    ESP_LOGI(TAG, "RTK Rover Starting...");

    num_waypoints = sizeof(mission_waypoints) / sizeof(waypoint_t);
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create queues
    rtcm_queue = xQueueCreate(10, 256);         // RTCM data from base station
    gnss_queue = xQueueCreate(5, sizeof(gnss_data_t));  // GNSS position data
    imu_queue = xQueueCreate(10, sizeof(imu_data_t));    // IMU orientation data
    nav_queue = xQueueCreate(5, sizeof(nav_command_t));  // Navigation commands
    
    if (rtcm_queue == NULL || gnss_queue == NULL || imu_queue == NULL || nav_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

    // Initialize components
    ESP_ERROR_CHECK(esp_now_rover_init());
    ESP_ERROR_CHECK(uart_gnss_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(motor_control_init());
    
    // Initialize navigation with waypoints
    ESP_ERROR_CHECK(navigation_init(mission_waypoints, num_waypoints));

    ESP_LOGI(TAG, "All components initialized successfully");
    ESP_LOGI(TAG, "Rover ready for autonomous navigation with %d waypoints", num_waypoints);
    
    // Wait a bit for all systems to stabilize
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Start navigation
    navigation_start();
    
    // Main loop - monitor system status
    while (1) {
        rover_status_t status = get_rover_status();
        
        ESP_LOGI(TAG, "Rover Status - WP: %d/%d, Dist: %.2fm, Heading: %.1f°, Speed: %.1f", 
                 status.current_waypoint + 1, status.total_waypoints,
                 status.distance_to_target, status.current_heading, status.current_speed);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}