#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h> // For use of atan2f
#include "esp_log.h"

typedef enum {
    NOT_CONFIGURED = false,
    CONFIGURED = true,
} Configuration;

extern const char *TAG_Kalman;

typedef struct {
    float angle;      // Estimated angle
    float bias;       // Gyro bias estimate
    float rate;       // Unbiased rate
    float P[2][2];    // Error covariance matrix
} kalman_filter_t;

void kalman_config(QueueHandle_t raw_queue, QueueHandle_t angle_queue, float initial_angle);
void kalman_filter_step(float dt);

#endif
