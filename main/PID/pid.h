#ifndef pid_H
#define pid_H

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions
#include "freertos/queue.h" // FreeRTOS queue for inter-task communication
#include "driver/gpio.h" // GPIO driver for pin configuration and control
#include "esp_log.h" // ESP logging functions for debugging and info output
#include "esp_timer.h" // High-resolution timer for accurate timing (microseconds)

#define TAG_pid "PID calc" // Tag used for logging from PID module
#define GPIO_OUT1 NULL // Default placeholder for GPIO output 1
#define GPIO_OUT2 NULL // Default placeholder for GPIO output 2
#define GPIO_OUT3 NULL // Default placeholder for GPIO output 3 (unused here)
#define GPIO_OUT4 NULL // Default placeholder for GPIO output 4 (unused here)
#define deadzone 0 // PID output deadzone (offset to avoid motor jitter)

typedef enum {
    PID_NOT_CONFIGURED = false, // PID module not yet configured
    PID_CONFIGURED = true // PID module configured and ready
} pid_config_state_t;

typedef struct {
    gpio_num_t OUT1; // GPIO pin controlling motor driver output 1
    gpio_num_t OUT2; // GPIO pin controlling motor driver output 2
} motor_driver_config_t;

typedef struct {
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
} PID_t;

void gpio_init(gpio_num_t pin_num); // Initialize GPIO pin as output and reset it
esp_err_t pid_config(motor_driver_config_t *config1, motor_driver_config_t *config2, float setpoint, QueueHandle_t angle_queue, QueueHandle_t remote_queue, QueueHandle_t DC_queue, QueueHandle_t servo_queue); // Configure PID module
void pid_compute(void* pvParameters); // PID control task function

#endif
