#ifndef MPU6050_V2_H
#define MPU6050_V2_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"

#define MPU6050_ADDR               0x68
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_SCALE        0x1C
#define MPU6050_GYRO_SCALE         0x1B

#define I2C_PORT       0
#define I2C_SDA_PIN    GPIO_NUM_21
#define I2C_SCL_PIN    GPIO_NUM_22
#define I2C_FREQ_HZ    100000

extern const char *TAG_MPU6050;

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;
} mpu6050_data_t;

void IMU_set_send_queue(QueueHandle_t queue);

void IMU_get_data(TimerHandle_t xTimer);

esp_err_t mpu6050_config(uint8_t accel_scale, uint8_t gyro_scale);

#endif
