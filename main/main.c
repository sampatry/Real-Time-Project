#include <stdio.h>
#include "mpu6050.h"
#include "pwm_read.h"
#include "pwm_write.h"
#include "kalman.h"
#include "pid.h"

#define accel_scale 1
#define gyro_scale 1

#define PWM_INPUT_GPIO GPIO_NUM_18
#define PWM_OUTPUT_GPIO GPIO_NUM_19


ledc_channel_t Servo_channel = LEDC_CHANNEL_0; 
ledc_timer_t Servo_timer = LEDC_TIMER_0; 
uint32_t Servo_freq_hz = LEDC_FREQ_HZ;

QueueHandle_t rawImuQueue; // Queue for passing raw IMU data from mpu6050 to kalman
QueueHandle_t tiltAngleQueue; // Queue for passing tilt angle from kalman to pid
QueueHandle_t pwmOutputQueue; // Queue for passing pwm signal from pid to pwm output

int32_t pulse_width_out_us = 0;
int IMU_timer_period_ms = 100;
int PWM_output_period_ms = 10;
float initial_tilt_angle = 90.0f;

static TimerHandle_t mpu6050Timer;
static TimerHandle_t pwmOutTimer;



// Task: Waits for angle data and controls PWM (might have to change to timer to avoid overload of cpu)
void pidTask(void* pvParameters) {
    while(1){
        pid_compute(initial_tilt_angle);
        vTaskDelay(pdMS_TO_TICKS(IMU_timer_period_ms));
    }
}

void app_main(void){
    rawImuQueue = xQueueCreate(5, sizeof(mpu6050_data_t)); // Create queue to store up to 5 sets of IMU data
    tiltAngleQueue = xQueueCreate(5, sizeof(float)); // Create queue to store up to 5 tilt angles
    pwmOutputQueue = xQueueCreate(5, sizeof(int32_t));// Create a queue for pwm output values

    ESP_ERROR_CHECK(mpu6050_config(accel_scale, gyro_scale, rawImuQueue));
    kalman_config(rawImuQueue, tiltAngleQueue, initial_tilt_angle);
    pid_config(pwmOutputQueue, tiltAngleQueue);
    PWM_output_config(PWM_OUTPUT_GPIO, Servo_channel, Servo_timer, Servo_freq_hz,pwmOutputQueue); // Setup PWM output

    // Start IMU read timer
    mpu6050Timer = xTimerCreate("MPU6050", pdMS_TO_TICKS(IMU_timer_period_ms), pdTRUE, NULL, IMU_timer);
    if (mpu6050Timer == NULL || xTimerStart(mpu6050Timer, 0) != pdPASS) {
        ESP_LOGE(TAG_MPU6050, "Failed to create/start MPU6050 timer");
    }

    // Create periodic timer to output new PWM value to motor
    pwmOutTimer = xTimerCreate("PWM OUTPUT", pdMS_TO_TICKS(PWM_output_period_ms), pdTRUE, NULL, PWM_output_update);
    if (pwmOutTimer == NULL || xTimerStart(pwmOutTimer, 0) != pdPASS) {
        ESP_LOGE(TAG_PWM_LEDC, "Failed to create/start PWM OUTPUT timer");
    }

    xTaskCreate(KalmanTask, "Kalman Update", 2048, NULL, 3, NULL);
    xTaskCreate(pidTask, "PID_Task", 2048, NULL, 3, NULL);
}