#ifndef MPU6050_V2_H
#define MPU6050_V2_H

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions
#include "driver/gpio.h" // GPIO driver for pin configuration and control
#include "driver/i2c_master.h" // I2C master driver for sensor communication
#include "freertos/timers.h" // FreeRTOS software timer API
#include "freertos/queue.h" // FreeRTOS queue for inter-task communication
#include "esp_log.h" // ESP logging functions for debugging and info output

#define MPU6050_ADDR 0x68 // MPU6050 I2C address
#define MPU6050_REG_PWR_MGMT_1 0x6B // Power management register 1
#define MPU6050_REG_ACCEL_XOUT_H 0x3B // Accelerometer X output high byte register
#define MPU6050_ACCEL_SCALE 0x1C // Accelerometer configuration register
#define MPU6050_GYRO_SCALE 0x1B // Gyroscope configuration register
#define MPU6050_REG_CONFIG 0x1A // Configuration register for DLPF settings

#define I2C_PORT 0 // I2C port number used
#define I2C_SDA_PIN GPIO_NUM_21 // I2C SDA pin
#define I2C_SCL_PIN GPIO_NUM_22 // I2C SCL pin
#define I2C_FREQ_HZ 100000 // I2C clock frequency in Hz

#define TAG_MPU6050 "MPU6050" // Tag used for ESP logging

extern TaskHandle_t imu_handle; // Handle for IMU data reading task

// MPU6050 configuration state enumeration
typedef enum {
    MPU6050_NOT_CONFIGURED = false,
    MPU6050_CONFIGURED = true
} mpu6050_config_state_t;

// Structure for holding processed IMU data
typedef struct {
    float accel_x, accel_y, accel_z; // Accelerometer readings in g
    float gyro_x, gyro_y, gyro_z; // Gyroscope readings in degrees per second
} mpu6050_data_t;

// Configure MPU6050 sensor scales and queue for data transmission
esp_err_t mpu6050_config(uint8_t accel_scale, uint8_t gyro_scale, QueueHandle_t queue);

// Task for continuous reading and filtering of MPU6050 data
void IMU_get_data(void* pvParameters);

// Timer callback to notify IMU data reading task
void IMU_timer(TimerHandle_t xTimer);

#endif