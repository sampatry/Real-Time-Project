#ifndef pid_H
#define pid_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG_pid "PID calc"
#define GPIO_OUT1 NULL
#define GPIO_OUT2 NULL
#define GPIO_OUT3 NULL
#define GPIO_OUT4 NULL
#define deadzone 300

typedef enum {
    PID_NOT_CONFIGURED = false,
    PID_CONFIGURED = true
} pid_config_state_t;

typedef struct {
    gpio_num_t OUT1;
    gpio_num_t OUT2;
} motor_driver_config_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_t;

void gpio_init(gpio_num_t pin_num);
esp_err_t pid_config(motor_driver_config_t *config1, motor_driver_config_t *config2, float setpoint, QueueHandle_t angle_queue, QueueHandle_t pwm_queue);
void pid_compute(void* pvParameters);

#endif
