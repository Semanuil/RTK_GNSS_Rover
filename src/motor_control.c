#include "motor_control.h"
#include <math.h>

static const char *TAG = "MOTOR_CONTROL";

esp_err_t motor_control_init(void)
{
    ESP_LOGI(TAG, "Initializing motor control...");
    
    // Prepare and set configuration of timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Prepare and set configuration of left motor channel
    ledc_channel_config_t ledc_left_channel = {
        .channel    = MOTOR_LEFT_CHANNEL,
        .duty       = US_TO_DUTY(PWM_NEUTRAL),
        .gpio_num   = MOTOR_LEFT_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_left_channel));
    
    // Prepare and set configuration of right motor channel
    ledc_channel_config_t ledc_right_channel = {
        .channel    = MOTOR_RIGHT_CHANNEL,
        .duty       = US_TO_DUTY(PWM_NEUTRAL),
        .gpio_num   = MOTOR_RIGHT_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_right_channel));
    
    // Set initial neutral position (motors stopped)
    ESP_ERROR_CHECK(stop_motors());
    
    ESP_LOGI(TAG, "Motor control initialized successfully");
    return ESP_OK;
}

esp_err_t set_motor_speeds(float left_speed, float right_speed)
{
    // Clamp speeds to valid range
    if (left_speed > 1.0) left_speed = 1.0;
    if (left_speed < -1.0) left_speed = -1.0;
    if (right_speed > 1.0) right_speed = 1.0;
    if (right_speed < -1.0) right_speed = -1.0;
    
    // Convert speed (-1.0 to 1.0) to PWM microseconds (1000 to 2000)
    uint16_t left_pwm = PWM_NEUTRAL + (int16_t)(left_speed * 500);
    uint16_t right_pwm = PWM_NEUTRAL + (int16_t)(right_speed * 500);
    
    // Apply deadband to prevent motor humming at low speeds
    if (fabs(left_speed) < 0.05) left_pwm = PWM_NEUTRAL;
    if (fabs(right_speed) < 0.05) right_pwm = PWM_NEUTRAL;
    
    // Set PWM duty cycles
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_LEFT_CHANNEL, US_TO_DUTY(left_pwm)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_LEFT_CHANNEL));
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_RIGHT_CHANNEL, US_TO_DUTY(right_pwm)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_RIGHT_CHANNEL));
    
    ESP_LOGD(TAG, "Motor speeds set - Left: %.2f (PWM: %d), Right: %.2f (PWM: %d)", 
             left_speed, left_pwm, right_speed, right_pwm);
    
    return ESP_OK;
}

esp_err_t stop_motors(void)
{
    return set_motor_speeds(0.0, 0.0);
}

esp_err_t move_forward(float speed)
{
    return set_motor_speeds(speed, speed);
}

esp_err_t move_backward(float speed)
{
    return set_motor_speeds(-speed, -speed);
}

esp_err_t turn_left(float speed)
{
    return set_motor_speeds(-speed, speed);
}

esp_err_t turn_right(float speed)
{
    return set_motor_speeds(speed, -speed);
}

esp_err_t differential_drive(float linear_speed, float angular_speed)
{
    // Differential drive kinematics
    // linear_speed: forward/backward speed (-1.0 to 1.0)
    // angular_speed: turning speed (-1.0 to 1.0, positive = right turn)
    
    float left_speed = linear_speed - angular_speed;
    float right_speed = linear_speed + angular_speed;
    
    // Normalize speeds if they exceed [-1.0, 1.0] range
    float max_speed = fmax(fabs(left_speed), fabs(right_speed));
    if (max_speed > 1.0) {
        left_speed /= max_speed;
        right_speed /= max_speed;
    }
    
    return set_motor_speeds(left_speed, right_speed);
}