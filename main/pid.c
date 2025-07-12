#include "pid.h"

const char *TAG_pid = "PID calc";
static int configured = 0;

// PID parameters (control tuning parameters)
static float Kp = 30.0f;
static float Ki = 1.0f;
static float Kd = 2.0f;

static float integral = 0.0f;
static float last_error = 0.0f;

static QueueHandle_t tilt_angle_queue = NULL; // Queue for receiving tilt angle from kalman
static QueueHandle_t pwm_output_queue = NULL; // Queue for sending pwm signal to pwm output

void pid_set_receive_queue(QueueHandle_t queue1) {
    tilt_angle_queue = queue1;
    configured++;
}
void pid_set_send_queue(QueueHandle_t queue2) {
    pwm_output_queue = queue2;
    configured++;
}

// PID control from angle to PWM duty
void pid_compute(float setpoint) {
    if (!(configured == 2)){
        ESP_LOGE(TAG_pid, "Not properly set up: %d / 2 configured", configured); // Return immedietly if the pid is not fully configured
        return;
    }

    if (tilt_angle_queue == NULL) return; // Return immediatly if the queue is empty

    float angle;
    if (xQueueReceive(tilt_angle_queue, &angle, 0) == pdPASS) {

        float error = setpoint - angle;
        integral += error;
        float derivative = error - last_error;
        last_error = error;

        float output = Kp * error + Ki * integral + Kd * derivative;//PID output is the control signal

        // Map PID output to PWM duty range
        int32_t pwm_output = (int32_t)(fabs(output) * 10);  // Output with scale factor

        // Add direction control
        if (output <0) {
            pwm_output = -pwm_output; // Reverse direction if output is negative
            ESP_LOGI(TAG_pid, "Reversing direction, PWM duty: %d\n", pwm_output);
        }
        else {
            ESP_LOGI(TAG_pid, "Forward direction, PWM duty: %d\n", pwm_output);
        }
        xQueueSend(pwm_output_queue, &pwm_output, 0); //  Sends pid calulated pwm to queue
    }
}