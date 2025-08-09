#ifndef PWM_write_h
#define PWM_write_h

#include "freertos/FreeRTOS.h" // For FreeRTOS base functions and types
#include "freertos/timers.h" // For timer functions and types
#include "driver/gpio.h" // For GPIO configuration and control
#include "esp_log.h" // For logging macros like ESP_LOGI, ESP_LOGE
#include "driver/ledc.h" // For LEDC PWM hardware driver
#include "freertos/queue.h" // For FreeRTOS queue usage

#define TAG_PWM_WRITE "PWM_OUT" // Tag for PWM write logs
#define microseconds 1000000 // Microseconds per second constant

typedef enum {
    PWM_NOT_CONFIGURED = false, // PWM not configured state
    PWM_CONFIGURED = true // PWM configured state
} pwm_config_state_t;

typedef struct {
    gpio_num_t GPIO; // GPIO pin for PWM output
    ledc_channel_t CHANNEL; // LEDC channel number
    ledc_timer_t TIMER; // LEDC timer number
    uint32_t FREQ_HZ; // PWM frequency in Hertz
    QueueHandle_t QUEUE; // Queue handle for receiving PWM pulse widths
} motor_config_t;

// Task function to update PWM output duty cycle from queue values
void PWM_output_update(void* pvParameters);

// Function to initialize PWM output hardware and related resources
esp_err_t PWM_output_config(motor_config_t *config);

#endif