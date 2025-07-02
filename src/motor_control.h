#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

// PWM configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          50                // Frequency in Hertz (50Hz for RC servos)

// Motor PWM pins
#define MOTOR_LEFT_PIN          25    // Left motor PWM
#define MOTOR_RIGHT_PIN         26    // Right motor PWM

// PWM channels
#define MOTOR_LEFT_CHANNEL      LEDC_CHANNEL_0
#define MOTOR_RIGHT_CHANNEL     LEDC_CHANNEL_1

// PWM values for RC-style control (1000-2000 microseconds)
#define PWM_NEUTRAL             1500  // Neutral position (stop)
#define PWM_FULL_FORWARD        2000  // Full forward
#define PWM_FULL_REVERSE        1000  // Full reverse
#define PWM_MIN_THRESHOLD       1050  // Minimum threshold for movement
#define PWM_MAX_THRESHOLD       1950  // Maximum threshold for movement

// Convert microseconds to duty cycle
#define US_TO_DUTY(us)          ((us) * 8192 / 20000)  // 13-bit resolution, 20ms period

// Motor command structure
typedef struct {
    float left_speed;   // -1.0 to 1.0 (negative = reverse)
    float right_speed;  // -1.0 to 1.0 (negative = reverse)
    uint32_t duration;  // Duration in milliseconds (0 = continuous)
} motor_command_t;

// Function prototypes
esp_err_t motor_control_init(void);
esp_err_t set_motor_speeds(float left_speed, float right_speed);
esp_err_t stop_motors(void);
esp_err_t move_forward(float speed);
esp_err_t move_backward(float speed);
esp_err_t turn_left(float speed);
esp_err_t turn_right(float speed);
esp_err_t differential_drive(float linear_speed, float angular_speed);

#endif // MOTOR_CONTROL_H