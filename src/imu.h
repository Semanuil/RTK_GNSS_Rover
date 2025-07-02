#ifndef IMU_H
#define IMU_H

#include "driver/i2c.h"
#include "esp_log.h"

// I2C configuration
#define I2C_MASTER_SCL_IO      22    // GPIO22 for SCL
#define I2C_MASTER_SDA_IO      21    // GPIO21 for SDA
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// MPU6050 configuration
#define MPU6050_ADDR           0x68
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_GYRO_XOUT_H    0x43

// IMU data structure
typedef struct {
    float gyro_x;        // Angular velocity around X axis (deg/s)
    float gyro_y;        // Angular velocity around Y axis (deg/s)
    float gyro_z;        // Angular velocity around Z axis (deg/s)
    float accel_x;       // Acceleration along X axis (g)
    float accel_y;       // Acceleration along Y axis (g)
    float accel_z;       // Acceleration along Z axis (g)
    float heading;       // Calculated heading (degrees, 0-360)
    uint32_t timestamp;  // System timestamp
} imu_data_t;

// External queue handle
extern QueueHandle_t imu_queue;

// Function prototypes
esp_err_t imu_init(void);
esp_err_t mpu6050_init(void);
void imu_task(void *pvParameters);
esp_err_t imu_read_raw(int16_t *gyro, int16_t *accel);
void calculate_heading(imu_data_t *imu_data);

#endif // IMU_H