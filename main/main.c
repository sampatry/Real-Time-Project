#include <stdio.h>
#include "mpu6050.h" // Custom driver for MPU6050 IMU (accelerometer + gyroscope)
#include "pwm_read.h" // Module for reading PWM inputs (remote control)
#include "pwm_write.h" // Module for writing PWM outputs (DC motors, servos)
#include "kalman.h" // Kalman filter module for sensor fusion
#include "pid.h" // PID control module

// MPU6050 Sensor Configuration
#define accel_scale 1 // Accelerometer sensitivity setting (driver interprets scale index)
#define gyro_scale 1      // Gyroscope sensitivity setting (driver interprets scale index)

// GPIO Pin Assignments
#define REMOTE_INPUT_GPIO GPIO_NUM_18 // Pin connected to remote PWM signal input
#define DC_MOTOR_PWM_GPIO   GPIO_NUM_19 // PWM output pin for DC motor speed control
#define SERVO_MOTOR_PWM_GPIO GPIO_NUM_5 // PWM output pin for servo position control

// PWM Frequencies
#define DC_MOTOR_FREQ_HZ 2000 // PWM frequency for DC motor control (higher for smoother motor drive)
#define SERVO_FREQ_HZ 50 // Standard servo frequency (20ms period)

// FreeRTOS Queues & Timer Handles
static TimerHandle_t mpu6050Timer; // Software timer to periodically read IMU data
static QueueHandle_t rawImuQueue; // Passes raw IMU data from MPU6050 task to Kalman filter
static QueueHandle_t tiltAngleQueue; // Passes fused tilt angle from Kalman to PID
static QueueHandle_t dcMotorOutputQueue; // Passes computed motor control signal from PID to motor output
static QueueHandle_t servoOutputQueue; // Passes servo control signal from PID to servo output
static QueueHandle_t RemoteControlQueue; // Passes remote control input to PID

// Misc Global Variables
int32_t pulse_width_out_us = 0; // Holds last PWM output pulse width (in microseconds)
int IMU_timer_period_ms = 20; // IMU polling rate in milliseconds (50 Hz read rate)
float initial_tilt_angle = 9.3f; // Robot's initial tilt offset due to physical mounting

// PID Gains
PID_t pid_gains = {
    .Kp = 70.0f, // Proportional gain - main correction factor
    .Ki = 0.0f, // Integral gain - unused for now
    .Kd = 0.0f // Derivative gain - unused for now
};

// Motor Driver Pin Configurations
motor_driver_config_t left_motor = {
    .OUT1 = GPIO_NUM_32, // H-bridge control pin 1 for left motor
    .OUT2 = GPIO_NUM_33 // H-bridge control pin 2 for left motor
};
motor_driver_config_t right_motor = {
    .OUT1 = GPIO_NUM_25, // H-bridge control pin 1 for right motor
    .OUT2 = GPIO_NUM_26 // H-bridge control pin 2 for right motor
};

// DC Motor PWM Output Configuration
motor_config_t DC_motor = {
    .GPIO = DC_MOTOR_PWM_GPIO, // Output pin
    .CHANNEL = LEDC_CHANNEL_0, // LEDC hardware PWM channel
    .TIMER = LEDC_TIMER_0, // LEDC timer group
    .FREQ_HZ = DC_MOTOR_FREQ_HZ, // PWM frequency
    .QUEUE = NULL // Will be assigned later
};

// Servo Motor PWM Output Configuration
motor_config_t Servo_motor = {
    .GPIO = SERVO_MOTOR_PWM_GPIO, // Output pin
    .CHANNEL = LEDC_CHANNEL_1, // LEDC hardware PWM channel
    .TIMER = LEDC_TIMER_1, // LEDC timer group
    .FREQ_HZ = SERVO_FREQ_HZ, // PWM frequency
    .QUEUE = NULL // Will be assigned later
};

// Remote Input Configuration
RX_config_t Remote_Control = {
    .GPIO = REMOTE_INPUT_GPIO, // Input pin for PWM signal
    .digital = false, // This is analog pulse width input, not digital high/low
    .servo = true, // This remote input controls a servo
    .QUEUE = NULL, // Will be assigned to RemoteControlQueue
    .rise_time_us = 0, // Time of last rising edge (for PWM pulse measurement)
    .pulse_width_us = 0, // Current measured pulse width
    .last_pulse_width = 0 // Last measured pulse width
};

void app_main(void){
    rawImuQueue = xQueueCreate(1, sizeof(mpu6050_data_t)); // Holds raw IMU readings
    tiltAngleQueue = xQueueCreate(1, sizeof(float)); // Holds computed tilt angle
    dcMotorOutputQueue = xQueueCreate(1, sizeof(int32_t)); // Holds DC motor PWM values
    servoOutputQueue = xQueueCreate(1, sizeof(int32_t)); // Holds servo PWM values
    RemoteControlQueue = xQueueCreate(1, sizeof(int32_t)); // Holds remote input

    // Assign queues to components so their tasks can read/write to the correct queue
    Remote_Control.QUEUE = RemoteControlQueue;
    DC_motor.QUEUE = dcMotorOutputQueue;
    Servo_motor.QUEUE = servoOutputQueue;

    // Configures MPU6050 with accel/gyro scales and queue to write to
    ESP_ERROR_CHECK(mpu6050_config(accel_scale, gyro_scale, rawImuQueue));
    // Sets up Kalman filter with data sources and output queue, initializes filter state with initial tilt angle offset
    ESP_ERROR_CHECK(kalman_config(rawImuQueue, tiltAngleQueue, initial_tilt_angle));
    // Configures PID with motor control pins, initial offset, angle input, remote control input, and output queues
    ESP_ERROR_CHECK(pid_config(&left_motor, &right_motor, initial_tilt_angle, tiltAngleQueue, Remote_Control.QUEUE, DC_motor.QUEUE, Servo_motor.QUEUE));
        
    ESP_ERROR_CHECK(PWM_output_config(&DC_motor)); // Configures LEDC for DC motor output
    ESP_ERROR_CHECK(PWM_output_config(&Servo_motor)); // Configures LEDC for servo output
    ESP_ERROR_CHECK(PWM_input_config(&Remote_Control)); // Sets up GPIO interrupt for PWM input measurement

    // IMU Read Timer Setup
    mpu6050Timer = xTimerCreate("MPU6050", pdMS_TO_TICKS(IMU_timer_period_ms), pdTRUE, NULL, IMU_timer);
    if (mpu6050Timer == NULL || xTimerStart(mpu6050Timer, 0) != pdPASS) {
        ESP_LOGE(TAG_MPU6050, "Failed to create/start MPU6050 timer");
    }
    if(xTaskCreate(KalmanTask, "Kalman Update", 4096, NULL, 3, NULL) != pdPASS){ // Task Creation for Kalman filter
        ESP_LOGE(TAG_Kalman, "Failed to create/start Kalman Update");
    }
    if(xTaskCreate(pid_compute, "PID_Task", 2048, &pid_gains, 3, NULL) != pdPASS){ // Task Creation for PID calculations and servo control
        ESP_LOGE(TAG_pid, "Failed to create/start PID_Task");
    }
    if(xTaskCreate(PWM_output_update, "DC_motor_PWM_update", 2048, &DC_motor, 3, NULL) != pdPASS){ // Task Creation for dc motor PWM output
        ESP_LOGE(TAG_PWM_WRITE, "Failed to create/start DC_motor_PWM_update");
    }
    if(xTaskCreate(PWM_output_update, "Servo_PWM_update", 2048, &Servo_motor, 3, NULL) != pdPASS){ // Task Creation for servo motor PWM output
        ESP_LOGE(TAG_PWM_WRITE, "Failed to create/start Servo_PWM_update");
    }
}
