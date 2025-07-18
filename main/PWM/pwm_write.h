#ifndef PWM_write_h
#define PWM_write_h

#include "freertos/FreeRTOS.h" //For using FreeRTOS
#include "freertos/timers.h" // For creating and using timers
#include "driver/gpio.h" // For use of hardware GPIO's
#include "esp_log.h" // For printing tagged debug message (ESP_LOGI)
#include "driver/ledc.h" // For generating pwm output signals
#include "freertos/queue.h" // For using freertos queues

#define LEDC_FREQ_HZ 50 // DS3240 expects a PWM period of 20ms aka 50hz

extern const char *TAG_PWM_LEDC;

// Function to config the queue to read from
void pwm_set_receive_queue(QueueHandle_t queue1);

// Timer to output the PWM signal
void PWM_output_update(TimerHandle_t xTimer);

// Function to initialize PWM output
void PWM_output_config(gpio_num_t PWM_OUTPUT_GPIO);

#endif