#ifndef pid_H
#define pid_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h> // For use of Fabs function for mapping
#include "esp_log.h"

extern const char *TAG_pid;

void pid_set_receive_queue(QueueHandle_t queue1);
void pid_set_send_queue(QueueHandle_t queue2);
void pid_compute(float setpoint);

#endif
