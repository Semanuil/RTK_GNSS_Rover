#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_timer.h"

static const char *TAG = "IMU";

// Gyroscope sensitivity (degrees per second per LSB)
#define GYRO_SENSITIVITY 131.0  // For +/- 250 deg/s range

// Accelerometer sensitivity (g per LSB)
#define ACCEL_SENSITIVITY 16384.0  // For +/- 2g range

// Heading calculation variables
static float current_heading = 0.0;
static uint32_t last_update_time = 0;

esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initializing IMU (MPU6050)...");
    
    // Initialize I2C master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                                       I2C_MASTER_RX_BUF_DISABLE, 
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
    
    // Initialize MPU6050
    //ESP_ERROR_CHECK(mpu6050_init());
    
    // Create IMU reading task
    BaseType_t ret = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        NULL,
        6,  // High priority for navigation
        NULL,
        0   // Pin to core 0
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "IMU initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    
    // Wake up MPU6050
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Wake up
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    
    // Configure gyroscope range (+/- 250 deg/s)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_GYRO_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true);  // +/- 250 deg/s
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050 gyroscope");
        return ret;
    }
    
    // Configure accelerometer range (+/- 2g)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true);  // +/- 2g
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050 accelerometer");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

void imu_task(void *pvParameters)
{
    int16_t raw_gyro[3], raw_accel[3];
    imu_data_t imu_data;
    
    ESP_LOGI(TAG, "IMU task started");
    
    // Initialize timing
    last_update_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    while (1) {
        // Read raw sensor data
        if (imu_read_raw(raw_gyro, raw_accel) == ESP_OK) {
            // Convert raw data to engineering units
            imu_data.gyro_x = raw_gyro[0] / GYRO_SENSITIVITY;
            imu_data.gyro_y = raw_gyro[1] / GYRO_SENSITIVITY;
            imu_data.gyro_z = raw_gyro[2] / GYRO_SENSITIVITY;
            
            imu_data.accel_x = raw_accel[0] / ACCEL_SENSITIVITY;
            imu_data.accel_y = raw_accel[1] / ACCEL_SENSITIVITY;
            imu_data.accel_z = raw_accel[2] / ACCEL_SENSITIVITY;
            
            // Calculate heading using gyroscope integration
            calculate_heading(&imu_data);
            
            imu_data.timestamp = esp_timer_get_time() / 1000;  // Convert to milliseconds
            
            // Send data to queue
            if (xQueueSend(imu_queue, &imu_data, 0) != pdTRUE) {
                ESP_LOGD(TAG, "IMU queue full, dropping data");
            }
            
            ESP_LOGD(TAG, "Heading: %.1f°, Gyro Z: %.2f°/s", imu_data.heading, imu_data.gyro_z);
        }
        
        // IMU update rate: 50Hz (20ms)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    vTaskDelete(NULL);
}

esp_err_t imu_read_raw(int16_t *gyro, int16_t *accel)
{
    uint8_t data[14];
    i2c_cmd_handle_t cmd;
    
    // Read all sensor data (14 bytes starting from ACCEL_XOUT_H)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);  // ACCEL_XOUT_H register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IMU data");
        return ret;
    }
    
    // Convert bytes to 16-bit values
    accel[0] = (data[0] << 8) | data[1];   // ACCEL_X
    accel[1] = (data[2] << 8) | data[3];   // ACCEL_Y
    accel[2] = (data[4] << 8) | data[5];   // ACCEL_Z
    // Skip temperature (data[6], data[7])
    gyro[0] = (data[8] << 8) | data[9];    // GYRO_X
    gyro[1] = (data[10] << 8) | data[11];  // GYRO_Y
    gyro[2] = (data[12] << 8) | data[13];  // GYRO_Z
    
    return ESP_OK;
}

void calculate_heading(imu_data_t *imu_data)
{
    uint32_t current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
    float dt = (current_time - last_update_time) / 1000.0;  // Convert to seconds
    
    if (dt > 0.1) {  // Prevent integration with too large time steps
        dt = 0.02;  // Default to 50Hz update rate
    }
    
    // Integrate gyroscope Z-axis to get heading change
    // Note: This is a simple integration. In a real application, you'd want
    // to use a more sophisticated algorithm like complementary or Kalman filter
    current_heading += imu_data->gyro_z * dt;
    
    // Keep heading in 0-360 degree range
    while (current_heading >= 360.0) current_heading -= 360.0;
    while (current_heading < 0.0) current_heading += 360.0;
    
    imu_data->heading = current_heading;
    last_update_time = current_time;
}