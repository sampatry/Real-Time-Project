#include "mpu6050_v2.h"

const char *TAG_MPU6050 = "MPU6050";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_handle = NULL;

static const uint8_t scale_value[4]    = {0x00, 0x08, 0x10, 0x18}; // ±2g/4g/8g/16g or ±250/500/1000/2000 dps
static const float accel_conv[4]       = {16384.0, 8192.0, 4096.0, 2048.0};
static const float gyro_conv[4]        = {131.0, 65.5, 32.8, 16.4};

static uint8_t current_accel_scale = 1;
static uint8_t current_gyro_scale  = 1;
static int configured = 0;

static QueueHandle_t raw_imu_queue = NULL;

void IMU_set_send_queue(QueueHandle_t queue) {
    raw_imu_queue = queue;
    configured++;
}

void IMU_get_data(TimerHandle_t xTimer) {
    if (!(configured == 2)){
        ESP_LOGE(TAG_MPU6050, "Not properly set up: %d / 2 configured", configured); // Return immedietly if the mpu6050 is not fully configured
        return;
    }
    
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t raw_data[14];

    if (i2c_master_transmit_receive(mpu6050_handle, &reg, 1, raw_data, sizeof(raw_data), -1) == ESP_OK) {
        // Convert raw to physical values
        mpu6050_data_t imu_data_out;

        int16_t ax = (raw_data[0] << 8) | raw_data[1];
        int16_t ay = (raw_data[2] << 8) | raw_data[3];
        int16_t az = (raw_data[4] << 8) | raw_data[5];
        int16_t temp_raw = (raw_data[6] << 8) | raw_data[7];
        int16_t gx = (raw_data[8] << 8) | raw_data[9];
        int16_t gy = (raw_data[10] << 8) | raw_data[11];
        int16_t gz = (raw_data[12] << 8) | raw_data[13];
        // Convert to G's
        imu_data_out.accel_x = ax / accel_conv[current_accel_scale - 1] - 0.13f; //subtract to correct systematic error from testing
        imu_data_out.accel_y = ay / accel_conv[current_accel_scale - 1] - 0.02f;
        imu_data_out.accel_z = az / accel_conv[current_accel_scale - 1] - 0.1f;
        // Convert to deg/s
        imu_data_out.gyro_x = gx / gyro_conv[current_gyro_scale - 1];
        imu_data_out.gyro_y = gy / gyro_conv[current_gyro_scale - 1];
        imu_data_out.gyro_z = gz / gyro_conv[current_gyro_scale - 1];

        imu_data_out.temperature = (temp_raw / 340.0f) + 36.53f;

        xQueueSend(raw_imu_queue, &imu_data_out, 0); //sends raw IMU data to the queue

        // Debug output
        ESP_LOGI(TAG_MPU6050, "Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Temp: %.2f°C", imu_data_out.accel_x, imu_data_out.accel_y, imu_data_out.accel_z, imu_data_out.gyro_x, imu_data_out.gyro_y, imu_data_out.gyro_z, imu_data_out.temperature);

    } else {
        ESP_LOGE(TAG_MPU6050, "Failed to read from MPU6050");
    }
}

esp_err_t mpu6050_config(uint8_t accel_scale, uint8_t gyro_scale) {
    current_accel_scale = accel_scale;
    current_gyro_scale  = gyro_scale;

    if (accel_scale < 1 || accel_scale > 4 || gyro_scale < 1 || gyro_scale > 4) {
        ESP_LOGE(TAG_MPU6050, "Invalid scale values");
        return ESP_FAIL;
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_config, &i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG_MPU6050, "Failed to create I2C bus");
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &mpu6050_handle) != ESP_OK) {
        ESP_LOGE(TAG_MPU6050, "Failed to add MPU6050 device");
        return ESP_FAIL;
    }

    // Wake MPU6050
    uint8_t wake_cmd[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    if (i2c_master_transmit(mpu6050_handle, wake_cmd, sizeof(wake_cmd), -1) != ESP_OK) {
        ESP_LOGE(TAG_MPU6050, "Failed to wake MPU6050");
        return ESP_FAIL;
    }

    // Set accel and gyro scale
    uint8_t accel_cmd[] = {MPU6050_ACCEL_SCALE, scale_value[accel_scale - 1]};
    uint8_t gyro_cmd[]  = {MPU6050_GYRO_SCALE, scale_value[gyro_scale - 1]};
    if (i2c_master_transmit(mpu6050_handle, accel_cmd, sizeof(accel_cmd), -1) != ESP_OK ||
        i2c_master_transmit(mpu6050_handle, gyro_cmd, sizeof(gyro_cmd), -1) != ESP_OK) {
        ESP_LOGE(TAG_MPU6050, "Failed to set scale");
        return ESP_FAIL;
    }
    configured++;
    return ESP_OK;
}
