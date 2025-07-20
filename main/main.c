#include <stdio.h>
#include "mpu6050.h"
#include "pwm_read.h"
#include "pwm_write.h"
#include "kalman.h"
#include "pid.h"

#define accel_scale 1
#define gyro_scale 1

#define PWM_INPUT_GPIO GPIO_NUM_18
#define DC_MOTOR_OUTPUT_GPIO GPIO_NUM_19
#define SERVO_MOTOR_OUTPUT_GPIO GPIO_NUM_5
#define DC_MOTOR_FREQ_HZ 1000
#define SERVO_FREQ_HZ 50 // DS3240 servo expects a PWM period of 20ms aka 50hz

static TimerHandle_t mpu6050Timer; // Timer for polling MPU at desired rate
static QueueHandle_t rawImuQueue; // Queue for passing raw IMU data from mpu6050 to kalman
static QueueHandle_t tiltAngleQueue; // Queue for passing tilt angle from kalman to pid
static QueueHandle_t dcMotorOutputQueue; // Queue for passing pwm signal from pid to pwm output
static QueueHandle_t servoOutputQueue; // Queue for passing pwm signal from pid to pwm output

motor_config_t DC_motor = {
    .GPIO = DC_MOTOR_OUTPUT_GPIO,
    .CHANNEL = LEDC_CHANNEL_0,
    .TIMER = LEDC_TIMER_0,
    .FREQ_HZ = DC_MOTOR_FREQ_HZ,
    .QUEUE = NULL
};

motor_config_t Servo_motor = {
    .GPIO = SERVO_MOTOR_OUTPUT_GPIO,
    .CHANNEL = LEDC_CHANNEL_1,
    .TIMER = LEDC_TIMER_1,
    .FREQ_HZ = SERVO_FREQ_HZ,
    .QUEUE = NULL
};

int32_t pulse_width_out_us = 0;
int IMU_timer_period_ms = 100;
int PWM_output_period_ms = 10;
float initial_tilt_angle = 90.0f;

// Task: Waits for angle data and controls PWM (might have to change to timer to avoid overload of cpu)
void pidTask(void* pvParameters) {
    while(1){
        pid_compute(initial_tilt_angle);
    }
}

void app_main(void){
    rawImuQueue = xQueueCreate(5, sizeof(mpu6050_data_t)); // Create queue to store up to 5 sets of IMU data
    tiltAngleQueue = xQueueCreate(5, sizeof(float)); // Create queue to store up to 5 tilt angles
    dcMotorOutputQueue = xQueueCreate(5, sizeof(int32_t));// Create a queue for DC motor output values
    servoOutputQueue = xQueueCreate(5, sizeof(int32_t));// Create a queue for DC motor output values

    DC_motor.QUEUE = dcMotorOutputQueue;
    Servo_motor.QUEUE = servoOutputQueue;

    ESP_ERROR_CHECK(mpu6050_config(accel_scale, gyro_scale, rawImuQueue));
    kalman_config(rawImuQueue, tiltAngleQueue, initial_tilt_angle);
    pid_config(tiltAngleQueue, dcMotorOutputQueue);
    PWM_output_config(&DC_motor); // Setup DC motor output
    PWM_output_config(&Servo_motor); // Setup servo output

    // Start IMU read timer
    mpu6050Timer = xTimerCreate("MPU6050", pdMS_TO_TICKS(IMU_timer_period_ms), pdTRUE, NULL, IMU_timer);
    if (mpu6050Timer == NULL || xTimerStart(mpu6050Timer, 0) != pdPASS) {
        ESP_LOGE(TAG_MPU6050, "Failed to create/start MPU6050 timer");
    }

    xTaskCreate(KalmanTask, "Kalman Update", 2048, NULL, 3, NULL);
    xTaskCreate(pidTask, "PID_Task", 2048, NULL, 3, NULL);
    xTaskCreate(PWM_output_update, "DC_motor_PWM_update", 2048, &DC_motor, 3, NULL);
    xTaskCreate(PWM_output_update, "Servo_PWM_update", 2048, &Servo_motor, 3, NULL);
}