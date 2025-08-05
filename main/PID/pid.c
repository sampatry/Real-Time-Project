#include "pid.h"

static bool PID_SETUP = PID_NOT_CONFIGURED;
static motor_driver_config_t *motor1 = NULL;
static motor_driver_config_t *motor2 = NULL;

static int Servo_left = 600;
static int Servo_up = 1500;
static int Servo_right = 2400;
static int remote_pulse = 1500;
static int max_angle_threshold = 12;
static int min_angle_threshold = 2;

static float integral = 0.0f;
static float last_error = 0.0f;
static float target_angle = 0.0f;

static QueueHandle_t tilt_angle_queue = NULL; // Queue for receiving tilt angle from kalman
static QueueHandle_t DC_output_queue = NULL; // Queue for sending pwm signal to dc motor output
static QueueHandle_t servo_output_queue = NULL; // Queue for sending pwm signal to servo output
static QueueHandle_t remote_input_queue = NULL; // Queue for sending pwm signal to servo output


void gpio_init(gpio_num_t pin_num) {
    ESP_ERROR_CHECK(gpio_reset_pin(pin_num)); //reset pin to defaults for safety
    ESP_ERROR_CHECK(gpio_set_direction(pin_num, GPIO_MODE_OUTPUT)); //Set gpio as output pin
    ESP_ERROR_CHECK(gpio_intr_disable(pin_num)); // Disable interrupts on GPIO.
    ESP_ERROR_CHECK(gpio_pullup_dis(pin_num)); // Disable pull-up on GPIO.
    ESP_ERROR_CHECK(gpio_pulldown_dis(pin_num)); // Disable pull-down on GPIO.
}

esp_err_t pid_config(motor_driver_config_t *config1, motor_driver_config_t *config2, float setpoint, QueueHandle_t angle_queue, QueueHandle_t remote_queue, QueueHandle_t DC_queue, QueueHandle_t servo_queue){
    motor1 = config1;
    motor2 = config2;
    if (motor1 != NULL){
        gpio_init(config1->OUT1);
        gpio_init(config1->OUT2);
        ESP_LOGI(TAG_pid, "Motor 1 configured");
    }else{
        ESP_LOGE(TAG_pid, "Motor 1 not configured");
    }
    if (motor2 != NULL){
        gpio_init(config2->OUT1);
        gpio_init(config2->OUT2);
        ESP_LOGI(TAG_pid, "Motor 2 configured");
    }else{
        ESP_LOGE(TAG_pid, "Motor 2 not configured");
    }

    target_angle = setpoint;
    tilt_angle_queue = angle_queue;
    remote_input_queue = remote_queue;
    DC_output_queue = DC_queue;
    servo_output_queue = servo_queue;
    PID_SETUP = PID_CONFIGURED;
    return ESP_OK;
}

static void set_motor_direction_left() {
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT1, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT2, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT1, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT2, 1));
}

static void set_motor_direction_right() {
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT1, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT2, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT1, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT2, 0));
}

// PID control from angle to PWM duty
void pid_compute(void* pvParameters) {
    PID_t *gain = (PID_t *) pvParameters; // Convert the config back from void *
    if (PID_SETUP == PID_NOT_CONFIGURED){
        ESP_LOGE(TAG_pid, "Not properly set up: %d / 2 configured", PID_SETUP); // Return immedietly if the pid is not fully configured
        vTaskDelete(NULL); // Deletes itself since it wasn't configured correctly
    }
    float dt = 0.0f;
    int64_t now = 0;
    int64_t previous_time = esp_timer_get_time();
    float angle;
    int output;
    while (1) {
        if (xQueueReceive(tilt_angle_queue, &angle, portMAX_DELAY) == pdPASS) {
            now = esp_timer_get_time();
            dt = (float)(now - previous_time) / 1000000.0f;
            previous_time = now;

            float error = target_angle - angle;
            integral += error * dt;
            float derivative = (error - last_error) / dt;
            last_error = error;

            float pid_calc = gain->Kp * error + gain->Ki * integral + gain->Kd * derivative;
            xQueueReceive(remote_input_queue, &remote_pulse, 0);  // optional read

            output = 0; // DC motor default off

            // === PRIORITY 1: REMOTE OVERRIDE ===
            if (remote_pulse != 1500) {
                xQueueSend(DC_output_queue, &output, 0); // Stop motors
                if (remote_pulse < 1500) {
                    xQueueSend(servo_output_queue, &Servo_left, 0);
                } else {
                    xQueueSend(servo_output_queue, &Servo_right, 0);
                }
                continue;
            }

            // === PRIORITY 2: ANGLE CONTROL ===
            // Small deadzone
            if (abs((int)error) < min_angle_threshold) {
                xQueueSend(DC_output_queue, &output, 0); // No motor
                xQueueSend(servo_output_queue, &Servo_up, 0); // Neutral
                continue;
            }

            // Out-of-range override
            if ((int)error > max_angle_threshold) {
                xQueueSend(DC_output_queue, &output, 0);
                xQueueSend(servo_output_queue, &Servo_left, 0);
                continue;
            } else if ((int)error < -max_angle_threshold) {
                xQueueSend(DC_output_queue, &output, 0);
                xQueueSend(servo_output_queue, &Servo_right, 0);
                continue;
            }

            // PID motor control (valid error and remote = 1500)
            if (pid_calc < 0) {
                output = (uint32_t)(-pid_calc + deadzone);
                set_motor_direction_left();
            } else {
                output = (uint32_t)(pid_calc + deadzone);
                set_motor_direction_right();
            }
            xQueueSend(DC_output_queue, &output, 0);
            xQueueSend(servo_output_queue, &Servo_up, 0); // Neutral

            ESP_LOGW(TAG_pid, "remote=%d | error=%.2f | pid=%.2f | out=%d", remote_pulse, error, pid_calc, output);
        }
    }
}