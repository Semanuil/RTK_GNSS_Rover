#include "navigation.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"

static const char *TAG = "NAVIGATION";

// Global navigation variables
waypoint_t *waypoints = NULL;
int num_waypoints = 0;
int current_waypoint_index = 0;
nav_state_t navigation_state = NAV_STATE_IDLE;

// Current rover position and status
static gnss_data_t current_position = {0};
static imu_data_t current_imu = {0};
static rover_status_t rover_status = {0};

// PID controllers
static pid_controller_t heading_pid = {KP_HEADING, KI_HEADING, KD_HEADING, 0, 0, 0};
static pid_controller_t distance_pid = {KP_DISTANCE, KI_DISTANCE, KD_DISTANCE, 0, 0, 0};

// Navigation task handle
static TaskHandle_t nav_task_handle = NULL;

esp_err_t navigation_init(waypoint_t *wp_list, int wp_count)
{
    if (wp_list == NULL || wp_count <= 0) {
        ESP_LOGE(TAG, "Invalid waypoint list");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing navigation with %d waypoints", wp_count);
    
    waypoints = wp_list;
    num_waypoints = wp_count;
    current_waypoint_index = 0;
    navigation_state = NAV_STATE_IDLE;
    
    // Initialize rover status
    rover_status.state = NAV_STATE_IDLE;
    rover_status.current_waypoint = 0;
    rover_status.total_waypoints = wp_count;
    
    // Print waypoint list
    for (int i = 0; i < wp_count; i++) {
        ESP_LOGI(TAG, "Waypoint %d: %.6f, %.6f (tolerance: %.1fm)", 
                 i + 1, waypoints[i].latitude, waypoints[i].longitude, waypoints[i].tolerance);
    }
    
    return ESP_OK;
}

esp_err_t navigation_start(void)
{
    if (waypoints == NULL || num_waypoints == 0) {
        ESP_LOGE(TAG, "Navigation not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting autonomous navigation");
    
    // Reset navigation state
    current_waypoint_index = 0;
    navigation_state = NAV_STATE_IDLE;
    pid_reset(&heading_pid);
    pid_reset(&distance_pid);
    
    // Create navigation task
    BaseType_t ret = xTaskCreatePinnedToCore(
        navigation_task,
        "navigation_task",
        8192,
        NULL,
        4,  // Medium priority
        &nav_task_handle,
        0   // Pin to core 0
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create navigation task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Navigation started successfully");
    return ESP_OK;
}

esp_err_t navigation_stop(void)
{
    ESP_LOGI(TAG, "Stopping navigation");
    
    navigation_state = NAV_STATE_IDLE;
    stop_motors();
    
    if (nav_task_handle != NULL) {
        vTaskDelete(nav_task_handle);
        nav_task_handle = NULL;
    }
    
    return ESP_OK;
}

void navigation_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Navigation task started");
    
    uint32_t last_gnss_update = 0;
    uint32_t last_imu_update = 0;
    uint32_t current_time;
    
    while (navigation_state != NAV_STATE_MISSION_COMPLETE) {
        current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
        
        // Update sensor data
        if (xQueueReceive(gnss_queue, &current_position, pdMS_TO_TICKS(10)) == pdTRUE) {
            last_gnss_update = current_time;
            rover_status.current_lat = current_position.latitude;
            rover_status.current_lon = current_position.longitude;
        }
        
        if (xQueueReceive(imu_queue, &current_imu, pdMS_TO_TICKS(10)) == pdTRUE) {
            last_imu_update = current_time;
            rover_status.current_heading = current_imu.heading;
        }
        
        // Check if we have recent sensor data (within last 2 seconds)
        if ((current_time - last_gnss_update) > 2000 || (current_time - last_imu_update) > 2000) {
            ESP_LOGW(TAG, "Missing sensor data - GNSS: %lums ago, IMU: %lums ago", 
                     current_time - last_gnss_update, current_time - last_imu_update);
            stop_motors();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Check if we have a valid GPS fix
        if (current_position.fix_quality < 1 || current_position.latitude == 0.0) {
            ESP_LOGW(TAG, "No valid GPS fix, waiting...");
            stop_motors();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        // Navigation state machine
        switch (navigation_state) {
            case NAV_STATE_IDLE:
                if (current_waypoint_index < num_waypoints) {
                    ESP_LOGI(TAG, "Starting navigation to waypoint %d", current_waypoint_index + 1);
                    navigation_state = NAV_STATE_TURNING;
                } else {
                    ESP_LOGI(TAG, "Mission complete!");
                    navigation_state = NAV_STATE_MISSION_COMPLETE;
                }
                break;
                
            case NAV_STATE_TURNING:
            case NAV_STATE_MOVING:
                {
                    // Calculate distance and bearing to current waypoint
                    waypoint_t *target = &waypoints[current_waypoint_index];
                    float distance = calculate_distance(current_position.latitude, current_position.longitude,
                                                      target->latitude, target->longitude);
                    float bearing = calculate_bearing(current_position.latitude, current_position.longitude,
                                                    target->latitude, target->longitude);
                    
                    rover_status.distance_to_target = distance;
                    rover_status.bearing_to_target = bearing;
                    
                    // Check if waypoint is reached
                    if (distance <= target->tolerance) {
                        ESP_LOGI(TAG, "Waypoint %d reached! Distance: %.2fm", 
                                 current_waypoint_index + 1, distance);
                        current_waypoint_index++;
                        navigation_state = NAV_STATE_ARRIVED;
                        stop_motors();
                        vTaskDelay(pdMS_TO_TICKS(1000));  // Pause at waypoint
                        break;
                    }
                    
                    // Calculate heading error
                    float heading_error = angle_difference(bearing, current_imu.heading);
                    
                    // Determine navigation mode based on heading error
                    if (fabs(heading_error) > HEADING_TOLERANCE) {
                        // Need to turn
                        navigation_state = NAV_STATE_TURNING;
                        
                        float turn_output = pid_update(&heading_pid, heading_error, current_time);
                        turn_output = fmax(-TURN_SPEED, fmin(TURN_SPEED, turn_output));
                        
                        differential_drive(0.0, turn_output);
                        
                        ESP_LOGD(TAG, "Turning - Heading error: %.1f°, Turn output: %.2f", 
                                 heading_error, turn_output);
                    } else {
                        // Move forward
                        navigation_state = NAV_STATE_MOVING;
                        
                        float distance_output = pid_update(&distance_pid, distance, current_time);
                        distance_output = fmax(MIN_SPEED, fmin(MAX_SPEED, distance_output));
                        
                        // Small heading correction while moving
                        float heading_correction = heading_error * 0.1;  // Gentle correction
                        
                        differential_drive(distance_output, heading_correction);
                        
                        rover_status.current_speed = distance_output;
                        
                        ESP_LOGD(TAG, "Moving - Distance: %.2fm, Speed: %.2f, Heading correction: %.2f", 
                                 distance, distance_output, heading_correction);
                    }
                }
                break;
                
            case NAV_STATE_ARRIVED:
                if (current_waypoint_index < num_waypoints) {
                    navigation_state = NAV_STATE_IDLE;  // Go to next waypoint
                } else {
                    navigation_state = NAV_STATE_MISSION_COMPLETE;
                }
                break;
                
            case NAV_STATE_MISSION_COMPLETE:
                stop_motors();
                ESP_LOGI(TAG, "Mission completed successfully!");
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;
        }
        
        // Update rover status
        rover_status.state = navigation_state;
        rover_status.current_waypoint = current_waypoint_index;
        
        // Navigation loop rate: 10Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Navigation task finished");
    vTaskDelete(NULL);
}

float calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    // Haversine formula for calculating distance between two GPS coordinates
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return EARTH_RADIUS * c;  // Distance in meters
}

float calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
    // Calculate bearing from point 1 to point 2
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    
    double y = sin(dLon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
    
    double bearing_rad = atan2(y, x);
    double bearing_deg = bearing_rad * 180.0 / M_PI;
    
    return normalize_angle(bearing_deg);
}

float normalize_angle(float angle)
{
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
}

float angle_difference(float target, float current)
{
    float diff = target - current;
    
    // Normalize to [-180, 180] range
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    
    return diff;
}

float pid_update(pid_controller_t *pid, float error, uint32_t current_time)
{
    if (pid->last_time == 0) {
        pid->last_time = current_time;
        return 0.0;
    }
    
    float dt = (current_time - pid->last_time) / 1000.0;  // Convert to seconds
    if (dt <= 0.0) return 0.0;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->previous_error) / dt;
    float d_term = pid->kd * derivative;
    
    // Calculate output
    float output = p_term + i_term + d_term;
    
    // Update for next iteration
    pid->previous_error = error;
    pid->last_time = current_time;
    
    return output;
}

void pid_reset(pid_controller_t *pid)
{
    pid->previous_error = 0.0;
    pid->integral = 0.0;
    pid->last_time = 0;
}

rover_status_t get_rover_status(void)
{
    return rover_status;
}