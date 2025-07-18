#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "mpu6050_v2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h> // For use of atan2f
#include "esp_log.h"

extern const char *TAG_Kalman;

typedef struct {
    float angle;      // Estimated angle
    float bias;       // Gyro bias estimate
    float rate;       // Unbiased rate
    float P[2][2];    // Error covariance matrix
} kalman_filter_t;

void kalman_filter_set_receive_queue(QueueHandle_t queue1);
void kalman_filter_set_send_queue(QueueHandle_t queue2);
void kalman_filter_config(float initial_angle);
void kalman_filter_step(float dt);

#endif
