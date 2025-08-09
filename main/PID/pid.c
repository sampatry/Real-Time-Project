#include "pid.h"

static bool PID_SETUP = PID_NOT_CONFIGURED;
static motor_driver_config_t *motor1 = NULL;
static motor_driver_config_t *motor2 = NULL;

static int Servo_left = 600; // PWM value for servo full left
static int Servo_up = 1500; // PWM value for servo center (neutral)
static int Servo_right = 2400; // PWM value for servo full right
static int remote_pulse = 1500; // Default neutral remote pulse
static int max_angle_threshold = 10; // Maximum allowed angle error before override
static int min_angle_threshold = 1; // Minimum angle error deadzone threshold

static float integral = 0.0f; // Integral term for PID controller
static float last_error = 0.0f; // Last error for derivative calculation
static float target_angle = 0.0f; // Desired target angle for PID

static QueueHandle_t tilt_angle_queue = NULL; // Queue to receive filtered tilt angle
static QueueHandle_t DC_output_queue = NULL; // Queue to send PWM duty cycle to DC motors
static QueueHandle_t servo_output_queue = NULL;// Queue to send PWM commands to servo
static QueueHandle_t remote_input_queue = NULL;// Queue to receive remote control PWM input

// Initialize GPIO pins for motor driver outputs
void gpio_init(gpio_num_t pin_num) {
    ESP_ERROR_CHECK(gpio_reset_pin(pin_num)); // Reset pin to default state
    ESP_ERROR_CHECK(gpio_set_direction(pin_num, GPIO_MODE_OUTPUT)); // Set pin as output
    ESP_ERROR_CHECK(gpio_intr_disable(pin_num)); // Disable interrupts on pin
    ESP_ERROR_CHECK(gpio_pullup_dis(pin_num)); // Disable pull-up resistor
    ESP_ERROR_CHECK(gpio_pulldown_dis(pin_num)); // Disable pull-down resistor
}

// Configure PID controller and motor GPIOs, initialize queues and target setpoint
esp_err_t pid_config(motor_driver_config_t *config1, motor_driver_config_t *config2, float setpoint, QueueHandle_t angle_queue, QueueHandle_t remote_queue, QueueHandle_t DC_queue, QueueHandle_t servo_queue){
    motor1 = config1; // Assign motor1 configuration pointer
    motor2 = config2; // Assign motor2 configuration pointer

    if (motor1 != NULL) { // Check if motor1 config is provided
        gpio_init(config1->OUT1); // Initialize GPIO pin OUT1 for motor1
        gpio_init(config1->OUT2); // Initialize GPIO pin OUT2 for motor1
        ESP_LOGI(TAG_pid, "Motor 1 configured"); // Log successful motor1 configuration
    } else {
        ESP_LOGE(TAG_pid, "Motor 1 not configured"); // Log error if motor1 config missing
    }

    if (motor2 != NULL) { // Check if motor2 config is provided
        gpio_init(config2->OUT1); // Initialize GPIO pin OUT1 for motor2
        gpio_init(config2->OUT2); // Initialize GPIO pin OUT2 for motor2
        ESP_LOGI(TAG_pid, "Motor 2 configured"); // Log successful motor2 configuration
    } else {
        ESP_LOGE(TAG_pid, "Motor 2 not configured"); // Log error if motor2 config missing
    }

    target_angle = setpoint; // Set desired angle for PID to maintain
    tilt_angle_queue = angle_queue; // Assign queue for tilt angle input
    remote_input_queue = remote_queue; // Assign queue for remote input override
    DC_output_queue = DC_queue; // Assign queue for DC motor PWM output
    servo_output_queue = servo_queue; // Assign queue for servo PWM output
    PID_SETUP = PID_CONFIGURED; // Mark PID as configured
    return ESP_OK;
}

// Set motor driver to move robot left by setting motor driver GPIO's'
static void set_motor_direction_left() {
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT1, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT2, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT1, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT2, 1));
}

// Set motor driver to move robot right by setting motor driver GPIO's
static void set_motor_direction_right() {
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT1, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor1->OUT2, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT1, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor2->OUT2, 0));
}

// PID control loop task, receives tilt angle, computes PID output, controls motors and servo accordingly
void pid_compute(void* pvParameters) {
    PID_t *gain = (PID_t *) pvParameters; // Cast parameter to PID gain structure
    if (PID_SETUP == PID_NOT_CONFIGURED){
        ESP_LOGE(TAG_pid, "Not properly set up: %d / 2 configured", PID_SETUP); // Log error if not configured
        vTaskDelete(NULL); // Delete task since configuration missing
    }
    float dt = 0.0f; // Change in time between control loops in seconds
    int64_t now = 0; // Set time at startup to 0
    int64_t previous_time = esp_timer_get_time(); // Get initial time for dt calculation
    float angle; // Measured current tilt angle from Kalman filter
    int output; // PWM output value for DC motors

    while (1) {
        if (xQueueReceive(tilt_angle_queue, &angle, portMAX_DELAY) == pdPASS) {
            now = esp_timer_get_time(); // Get current time in microseconds
            dt = (float)(now - previous_time) / 1000000.0f; // Compute elapsed time in seconds
            previous_time = now;

            float error = target_angle - angle; // Calculate error between target and actual angle
            integral += error * dt; // Integrate error over time
            float derivative = (error - last_error) / dt; // Rate of error change
            last_error = error; // Save error for next derivative calculation
            float pid_calc = gain->Kp * error + gain->Ki * integral + gain->Kd * derivative; // Compute PID output

            xQueueReceive(remote_input_queue, &remote_pulse, 0); // Optional remote pulse read for override
            ESP_LOGW(TAG_pid, "remote=%d | error=%.2f | pid=%.2f", remote_pulse, error, pid_calc);

            output = 0; // Default: turn DC motors off

            // PRIORITY 1: REMOTE OVERRIDE
            if (remote_pulse != 1500) {             // If remote command deviates from neutral
                xQueueSend(DC_output_queue, &output, 0); // Stop DC motors immediately
                if (remote_pulse < 1500) {
                    xQueueSend(servo_output_queue, &Servo_left, 0);  // Servo steer left
                } else {
                    xQueueSend(servo_output_queue, &Servo_right, 0); // Servo steer right
                }
                continue; // Skip PID control, remote has priority
            }

            // PRIORITY 2: ANGLE CONTROL
            if (abs((int)error) < min_angle_threshold) {  // If error is within deadzone
                xQueueSend(DC_output_queue, &output, 0);  // Motors off
                xQueueSend(servo_output_queue, &Servo_up, 0); // Servo neutral
                continue; // Skip PID control
            }

            // Out-of-range angle override, stop motors and steer accordingly
            if ((int)error > max_angle_threshold) {
                xQueueSend(DC_output_queue, &output, 0);
                xQueueSend(servo_output_queue, &Servo_left, 0);
                continue;
            } else if ((int)error < -max_angle_threshold) {
                xQueueSend(DC_output_queue, &output, 0);
                xQueueSend(servo_output_queue, &Servo_right, 0);
                continue;
            }

            // PID motor control for valid error and no remote override
            if (pid_calc < 0) {
                output = (uint32_t)(-pid_calc + deadzone); // Add deadzone offset
                set_motor_direction_left();                 // Set motors to rotate left
            } else {
                output = (uint32_t)(pid_calc + deadzone);  // Add deadzone offset
                set_motor_direction_right();                // Set motors to rotate right
            }
            xQueueSend(DC_output_queue, &output, 0);        // Send PWM to motors
            xQueueSend(servo_output_queue, &Servo_up, 0);   // Servo neutral

            ESP_LOGW(TAG_pid, "out=%d", output); // Log PWM output value
        }
    }
}
