#ifndef pid_H
#define pid_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h> // For use of Fabs function for mapping
#include "esp_log.h"
#include <inttypes.h>

typedef enum {
    PID_NOT_CONFIGURED = false,
    PID_CONFIGURED = true
} pid_config_state_t;



extern const char *TAG_pid;

void pid_config(QueueHandle_t send_queue, QueueHandle_t receive_queue);
void pid_compute(float setpoint);

#endif
