#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "mpu6050.h" // Raw IMU data definitions
#include "freertos/FreeRTOS.h" // FreeRTOS types and functions
#include "freertos/queue.h" // FreeRTOS queue for inter-task communication
#include <math.h> // atan2f function for angle calculation
#include "esp_log.h" // ESP logging functions for debugging and info output

#define TAG_Kalman "Kalman Filter" // Tag for logging Kalman filter messages

typedef enum {
    KALMAN_NOT_CONFIGURED = false, // Filter not initialized
    KALMAN_CONFIGURED = true // Filter initialized
} kalman_config_state_t;

typedef struct {
    float angle; // Estimated tilt angle (degrees)
    float bias; // Estimated gyro bias (degrees/second)
    float rate; // Rate after bias correction (degrees/second)
    float P[2][2]; // Error covariance matrix for the Kalman filter
} kalman_filter_t;

esp_err_t kalman_config(QueueHandle_t raw_queue, QueueHandle_t angle_queue, float initial_angle); // Configure Kalman filter
void kalman_filter_step(float dt); // Perform Kalman filter update step given elapsed time dt
void KalmanTask(void* pvParameters); // FreeRTOS task for continuous Kalman filter operation

#endif
