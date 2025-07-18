#include "Kalman_filter.h"

#define RAD_TO_DEG 57.2957795f

const char *TAG_Kalman = "Kalman Filter";

static kalman_filter_t kf;
static int configured = 0;
static QueueHandle_t raw_imu_queue = NULL;
static QueueHandle_t tilt_angle_queue = NULL;

void kalman_filter_set_receive_queue(QueueHandle_t queue1) {
    raw_imu_queue = queue1;
    configured++;
}
void kalman_filter_set_send_queue(QueueHandle_t queue2) {
    tilt_angle_queue = queue2;
    configured++;
}

// Kalman filter needs initial angle guess, to be determined by physical build angle when on kickstand
void kalman_filter_config(float initial_angle) {
    kf.angle = initial_angle;
    kf.bias = 0.0f;
    kf.P[0][0] = 0.0f;
    kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f;
    kf.P[1][1] = 0.0f;
    configured++;
}

static void kalman_filter_update(float angle_measured, float rate_measured, float dt) {
    // Predict
    float rate = rate_measured - kf.bias;
    kf.angle += dt * rate;

    kf.P[0][0] += dt * (dt*kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + 0.001f);
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += 0.003f * dt;

    float S = kf.P[0][0] + 0.03f;
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
}

void kalman_filter_step(float dt) {
    if (!(configured == 3)){
        ESP_LOGE(TAG_Kalman, "Not properly set up: %d / 3 configured", configured); // Return immedietly if the kalman filter is not fully configured
        return;
    }
    if (raw_imu_queue == NULL) return; // Return immedietly if the queue is empty

    mpu6050_data_t imu_data_in;
    if (xQueueReceive(raw_imu_queue, &imu_data_in, 0) == pdPASS) {
        float acc_angle = atan2f(imu_data_in.accel_y, imu_data_in.accel_z) * RAD_TO_DEG;
        float rate = imu_data_in.gyro_y;  // already in degrees/sec
        kalman_filter_update(acc_angle, rate, dt); //Calculate the new tilt angle
        ESP_LOGI(TAG_Kalman, "Calculated tilt angle: %.2f", kf.angle);
        xQueueSend(tilt_angle_queue, &kf.angle, 0); //sends filtered tilt angle to the queue
    }
}