#include <stdio.h>
#include "mpu6050_v2.h"
#include "PWM_read.h"
#include "PWM_write.h"
#include "Kalman_filter.h"
#include "pid.h"

void app_main(void)
{
    printf("Hello World!\n");
}

/*


#define accel_scale 1
#define gyro_scale 1

#define PWM_INPUT_GPIO GPIO_NUM_18
#define PWM_OUTPUT_GPIO GPIO_NUM_19

QueueHandle_t rawImuQueue; // Queue for passing raw IMU data from mpu6050 to kalman
QueueHandle_t tiltAngleQueue; // Queue for passing tilt angle from kalman to pid
QueueHandle_t pwmOutputQueue; // Queue for passing pwm signal from pid to pwm output

int32_t pulse_width_out_us = 0;
int IMU_timer_period_ms = 10;
int PWM_output_period_ms = 10;
float initial_tilt_angle = 90.0f;

static TimerHandle_t mpu6050Timer;
static TimerHandle_t pwmOutTimer;

void KalmanTask(void* pvParameters){
    while(1){
        kalman_filter_step((float) (IMU_timer_period_ms/1000.0f));
        vTaskDelay(pdMS_TO_TICKS(IMU_timer_period_ms));
    }
}

// Task: Waits for angle data and controls PWM (might have to change to timer to avoid overload of cpu)
void pidTask(void* pvParameters) {
    while(1){
        pid_compute(initial_tilt_angle);
        vTaskDelay(pdMS_TO_TICKS(IMU_timer_period_ms));
    }
}

void app_main(void)
{
    rawImuQueue = xQueueCreate(5, sizeof(mpu6050_data_t)); // Create queue to store up to 5 sets of IMU data
    tiltAngleQueue = xQueueCreate(5, sizeof(float)); // Create queue to store up to 5 tilt angles
    pwmOutputQueue = xQueueCreate(5, sizeof(int32_t));// Create a queue for pwm output values

    IMU_set_send_queue(rawImuQueue); // Set queue to send IMU data to in g's and deg/s
    kalman_filter_set_receive_queue(rawImuQueue); // Set queue to receive IMU data to in g's and deg/s
    kalman_filter_set_send_queue(tiltAngleQueue); // Set queue to send tilt angle in deg
    pid_set_receive_queue(tiltAngleQueue); // // Set queue to receive tilt angle in deg
    pid_set_send_queue(pwmOutputQueue); // Set queue to send pwm value
    pwm_set_receive_queue(pwmOutputQueue); // Set queue to receive pwm value

    mpu6050_config(accel_scale, gyro_scale); // Setup MPU6050
//    PWM_input_config(PWM_INPUT_GPIO); //Setup pwm input
    PWM_output_config(PWM_OUTPUT_GPIO); // Setup PWM output
    kalman_filter_config(initial_tilt_angle); // Setup Kalman filter

    // Start IMU read timer
    mpu6050Timer = xTimerCreate("MPU6050", pdMS_TO_TICKS(IMU_timer_period_ms), pdTRUE, NULL, IMU_get_data);
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
*/