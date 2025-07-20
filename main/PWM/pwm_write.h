#ifndef PWM_write_h
#define PWM_write_h

#include "freertos/FreeRTOS.h" //For using FreeRTOS
#include "freertos/timers.h" // For creating and using timers
#include "driver/gpio.h" // For use of hardware GPIO's
#include "esp_log.h" // For printing tagged debug message (ESP_LOGI)
#include "driver/ledc.h" // For generating pwm output signals
#include "freertos/queue.h" // For using freertos queues

extern const char *TAG_PWM_LEDC;

typedef enum {
    PWM_NOT_CONFIGURED = false,
    PWM_CONFIGURED = true
} pwm_config_state_t;

typedef struct {
    gpio_num_t GPIO;
    ledc_channel_t CHANNEL;
    ledc_timer_t TIMER;
    uint32_t FREQ_HZ;
    QueueHandle_t QUEUE;
} motor_config_t;

// Timer to output the PWM signal
void PWM_output_update(void* pvParameters);

// Function to initialize PWM output
void PWM_output_config(motor_config_t *config);


#endif