#include "kalman.h"

#define RAD_TO_DEG 57.2957795f
#define Q_angle 0.003f
#define Q_bias 0.003f
#define R_measure 0.001f

static kalman_filter_t kf;
static bool KALMAN_FILTER = KALMAN_NOT_CONFIGURED;
static QueueHandle_t raw_imu_queue = NULL;
static QueueHandle_t tilt_angle_queue = NULL;

esp_err_t kalman_config(QueueHandle_t raw_queue, QueueHandle_t angle_queue, float initial_angle){
    raw_imu_queue = raw_queue;
    tilt_angle_queue = angle_queue;
    kf.angle = initial_angle;
    kf.bias = 0.0f;
    kf.P[0][0] = 0.0f;
    kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f;
    kf.P[1][1] = 0.0f;
    KALMAN_FILTER = KALMAN_CONFIGURED;
    return ESP_OK;
}

static void kalman_filter_update(float angle_measured, float rate_measured, float dt) {
    // Predict
    float rate = rate_measured - kf.bias;
    kf.angle += dt * rate;

    kf.P[0][0] += dt * (dt*kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle); // Alter constant addition value to change response
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += Q_bias * dt; // Alter constant addition value to change response

    float S = kf.P[0][0] + R_measure; // Alter constant addition value to change response
    float K0 = kf.P[0][0] / S;
    float K1 = kf.P[1][0] / S;

    float y = angle_measured - kf.angle;
    kf.angle += K0 * y;
    kf.bias  += K1 * y;

    float P00_temp = kf.P[0][0];
    float P01_temp = kf.P[0][1];

    kf.P[0][0] -= K0 * P00_temp;
    kf.P[0][1] -= K0 * P01_temp;
    kf.P[1][0] -= K1 * P00_temp;
    kf.P[1][1] -= K1 * P01_temp;
    ESP_LOGI("KF", "kf.angle= %.2f acc= %.2f", kf.angle, angle_measured);
}

void kalman_filter_step(float dt) {
    if (KALMAN_FILTER == KALMAN_NOT_CONFIGURED){
        ESP_LOGE(TAG_Kalman, "Not properly set up: %d / 1 configured", KALMAN_FILTER); // Return immedietly if the kalman filter is not fully configured
        vTaskDelete(NULL); // Deletes itself since it wasn't configured correctly
    }

    mpu6050_data_t imu_data_in;
    if (xQueueReceive(raw_imu_queue, &imu_data_in, portMAX_DELAY) == pdPASS) {
        float acc_angle = atan2f(imu_data_in.accel_z, imu_data_in.accel_y) * RAD_TO_DEG;
        kalman_filter_update(acc_angle, imu_data_in.gyro_x, dt); //Calculate the new tilt angle
        //ESP_LOGI(TAG_Kalman, "Calculated tilt angle: %.2f, dt: %.6f", kf.angle, dt);
        xQueueSend(tilt_angle_queue, &kf.angle, 0); //sends filtered tilt angle to the queue
    }
}

void KalmanTask(void* pvParameters) {
    TickType_t previous_ticks = xTaskGetTickCount();
    while(1){
        TickType_t current_ticks = xTaskGetTickCount(); // Get current tick since startup (1 tick is 10ms by default)
        float dt_seconds = (float)(current_ticks - previous_ticks) * portTICK_PERIOD_MS / 1000.0f; // Convert Ticks to change in time (should be the same as MPU timer but this makes sure)
        previous_ticks = current_ticks; // Update previous ticks value
        kalman_filter_step(dt_seconds); // Update tilt angle estimate
    }
}