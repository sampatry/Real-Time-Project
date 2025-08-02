#include "pid.h"

static bool PID_SETUP = PID_NOT_CONFIGURED;
static motor_driver_config_t *motor1 = NULL;
static motor_driver_config_t *motor2 = NULL;

static float integral = 0.0f;
static float last_error = 0.0f;
static float target_angle = 0.0f;

static QueueHandle_t tilt_angle_queue = NULL; // Queue for receiving tilt angle from kalman
static QueueHandle_t pwm_output_queue = NULL; // Queue for sending pwm signal to pwm output

void gpio_init(gpio_num_t pin_num) {
    ESP_ERROR_CHECK(gpio_reset_pin(pin_num)); //reset pin to defaults for safety
    ESP_ERROR_CHECK(gpio_set_direction(pin_num, GPIO_MODE_OUTPUT)); //Set gpio as output pin
    ESP_ERROR_CHECK(gpio_intr_disable(pin_num)); // Disable interrupts on GPIO.
    ESP_ERROR_CHECK(gpio_pullup_dis(pin_num)); // Disable pull-up on GPIO.
    ESP_ERROR_CHECK(gpio_pulldown_dis(pin_num)); // Disable pull-down on GPIO.
}

esp_err_t pid_config(motor_driver_config_t *config1, motor_driver_config_t *config2, float setpoint, QueueHandle_t angle_queue, QueueHandle_t pwm_queue){
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
    pwm_output_queue = pwm_queue;
    PID_SETUP = PID_CONFIGURED;
    return ESP_OK;
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
    while(1){
        if (xQueueReceive(tilt_angle_queue, &angle, portMAX_DELAY) == pdPASS) {
            now = esp_timer_get_time(); // 64 bit intead of standard 32 to avoid overflow after ~35 min (probably overkill but better safe than sorry)
            dt = (float)(now - previous_time) / 1000000.0; // Get change in seconds
            previous_time = now; //Update previous time for the next loop
            float error = target_angle - angle;
            //ESP_LOGI(TAG_pid, "target_angle: %f, angle: %f", target_angle, angle);
            integral += error * dt;
            float derivative = (error - last_error) / dt;
            last_error = error;
            float pid_calc = gain->Kp * error + gain->Ki * integral + gain->Kd * derivative;//PID output is the control signal

            //ESP_LOGI(TAG_pid, "error: %f, derivative: %f, integral: %f, pid calc: %f, dt: %f", error, derivative, integral, pid_calc, dt);
            // float pid_calc;

            // if(abs(error) < 3){
            //     pid_calc = 0;//PID output is the deadzone signal
            // }
            // else{
            //     pid_calc = 10000;//PID output is the deadzone signal
            // }

            // else if (abs(error) < 3){
            //     pid_calc = gain->Kp * error + gain->Ki * integral + gain->Kd * derivative;//PID output is the control signal
            // }else{
            //     pid_calc = gain->Kp * 2 * error + gain->Ki * integral + gain->Kd * derivative;//PID output is the control signal
            // }
            //ESP_LOGI(TAG_pid, "error: %f, pid calc: %f", error, pid_calc);
            //pid_calc = 0.0;


            // Add direction control
            uint32_t output;
            if (pid_calc < 0) {
                output = (uint32_t)-(pid_calc - deadzone); // Switch pulse width to positive if calculated as negative and cast to int and add to avoid motor deadzone
                //printf("    pid: %f\n", pid_calc - deadzone);
                ESP_ERROR_CHECK(gpio_set_level(motor1->OUT1, 1));
                ESP_ERROR_CHECK(gpio_set_level(motor1->OUT2, 0));
                ESP_ERROR_CHECK(gpio_set_level(motor2->OUT1, 1));
                ESP_ERROR_CHECK(gpio_set_level(motor2->OUT2, 0));
                //ESP_LOGI(TAG_pid, "Backward direction, pwm output: %d", output);
                xQueueSend(pwm_output_queue, &output, 0); //  Sends pid calulated pwm to queue
            }
            else {
                output = (uint32_t)pid_calc + deadzone; // cast to int
                //printf("    pid: %f\n", pid_calc + deadzone);
                ESP_ERROR_CHECK(gpio_set_level(motor1->OUT1, 0));
                ESP_ERROR_CHECK(gpio_set_level(motor1->OUT2, 1));
                ESP_ERROR_CHECK(gpio_set_level(motor2->OUT1, 0));
                ESP_ERROR_CHECK(gpio_set_level(motor2->OUT2, 1));
                ;//ESP_LOGI(TAG_pid, "Forward direction, pwm output: %d", output);
                xQueueSend(pwm_output_queue, &output, 0); //  Sends pid calculated pwm to queue
            }
        }
    }
}