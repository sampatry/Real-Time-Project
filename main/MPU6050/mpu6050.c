#include "mpu6050.h" // Include custom header

static i2c_master_bus_handle_t i2c_bus_handle = NULL; // Handle for I2C bus instance
static i2c_master_dev_handle_t mpu6050_handle = NULL; // Handle for MPU6050 device on I2C bus

// Low pass filter smoothing factors for accelerometer and gyro data
static const float alpha_accel = 0.1f; 
static const float alpha_gyro = 0.1f; 

// Scale values to set MPU6050 sensitivity for accel and gyro registers
static const uint8_t scale_value[4] = {0x00, 0x08, 0x10, 0x18}; 

// Conversion factors to convert raw sensor counts to physical units
static const float accel_conv[4] = {16384.0, 8192.0, 4096.0, 2048.0}; 
static const float gyro_conv[4] = {131.0, 65.5, 32.8, 16.4}; 

static uint8_t current_accel_scale = 1; // Store current accel scale index
static uint8_t current_gyro_scale = 1;  // Store current gyro scale index

static bool IMU_SETUP = MPU6050_NOT_CONFIGURED; // Flag indicating if MPU6050 is configured

static QueueHandle_t raw_imu_queue = NULL; // Queue to send raw IMU data to Kalman filter

TaskHandle_t imu_handle; // Task handle for IMU data read task

// Configures MPU6050 sensor and creates task to read data
esp_err_t mpu6050_config(uint8_t accel_scale, uint8_t gyro_scale, QueueHandle_t queue) {
    current_accel_scale = accel_scale; // Save requested accel scale
    current_gyro_scale = gyro_scale; // Save requested gyro scale
    raw_imu_queue = queue; // Assign queue for passing IMU data

    if (accel_scale < 1 || accel_scale > 4 || gyro_scale < 1 || gyro_scale > 4) { // Validate scale range
        ESP_LOGE(TAG_MPU6050, "Invalid scale values"); // Error for invalid scale parameters
        return ESP_FAIL;
    }

    // Configure I2C master bus parameters
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, // Use default I2C clock source
        .i2c_port = I2C_PORT, // Use configured I2C port number
        .sda_io_num = I2C_SDA_PIN, // SDA pin for I2C
        .scl_io_num = I2C_SCL_PIN, // SCL pin for I2C
        .glitch_ignore_cnt = 7, // Ignore glitches shorter than this count
        .flags.enable_internal_pullup = true, // Enable internal pull-ups on SDA/SCL lines
    };

    if (i2c_new_master_bus(&bus_config, &i2c_bus_handle) != ESP_OK) { // Initialize I2C bus
        ESP_LOGE(TAG_MPU6050, "Failed to create I2C bus"); // Error if I2C bus creation fails
        return ESP_FAIL;
    }

    // Configure MPU6050 I2C device parameters
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit I2C address length
        .device_address = MPU6050_ADDR, // MPU6050 I2C address
        .scl_speed_hz = I2C_FREQ_HZ, // I2C clock speed in Hz
    };

    if (i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &mpu6050_handle) != ESP_OK) { // Add MPU6050 device to I2C bus
        ESP_LOGE(TAG_MPU6050, "Failed to add MPU6050 device"); // Error if device addition fails
        return ESP_FAIL;
    }

    uint8_t wake_cmd[] = {MPU6050_REG_PWR_MGMT_1, 0x00}; // Command to wake MPU6050 from sleep mode
    if (i2c_master_transmit(mpu6050_handle, wake_cmd, sizeof(wake_cmd), -1) != ESP_OK) { // Send wake command
        ESP_LOGE(TAG_MPU6050, "Failed to wake MPU6050"); // Error if wake command fails
        return ESP_FAIL;
    }

    // Set accelerometer scale register value
    uint8_t accel_cmd[] = {MPU6050_ACCEL_SCALE, scale_value[accel_scale - 1]};
    // Set gyroscope scale register value
    uint8_t gyro_cmd[] = {MPU6050_GYRO_SCALE, scale_value[gyro_scale - 1]};

    // Transmit accel and gyro scale settings to MPU6050
    if (i2c_master_transmit(mpu6050_handle, accel_cmd, sizeof(accel_cmd), -1) != ESP_OK ||
        i2c_master_transmit(mpu6050_handle, gyro_cmd, sizeof(gyro_cmd), -1) != ESP_OK) {
        ESP_LOGE(TAG_MPU6050, "Failed to set scale"); // Error if setting scales fails
        return ESP_FAIL;
    }

    uint8_t dlpf_cmd[] = {MPU6050_REG_CONFIG, 0x06}; // Configure digital low pass filter for max bandwidth (low latency)
    if (i2c_master_transmit(mpu6050_handle, dlpf_cmd, sizeof(dlpf_cmd), -1) != ESP_OK) { // Send DLPF config
        ESP_LOGE(TAG_MPU6050, "Failed to set DLPF config"); // Error if DLPF config fails
        return ESP_FAIL;
    }

    IMU_SETUP = MPU6050_CONFIGURED; // Mark IMU as configured

    // Create FreeRTOS task for continuous IMU data reading
    if (xTaskCreate(IMU_get_data, "IMU_get_data", 4096, NULL, 3, &imu_handle) != pdPASS) {
        ESP_LOGE(TAG_MPU6050, "Failed to create IMU task"); // Error if task creation fails
        return ESP_FAIL;
    }
    return ESP_OK; // Configuration successful
}

// Task function to continuously read and process MPU6050 sensor data
void IMU_get_data(void* pvParameters) {
    static float prev_ax = 0, prev_ay = 0, prev_az = 0; // Previous filtered accelerometer values
    static float prev_gx = 0, prev_gy = 0, prev_gz = 0; // Previous filtered gyro values

    while (1) {
        if (IMU_SETUP == MPU6050_NOT_CONFIGURED) { // Check if MPU6050 is configured
            ESP_LOGE(TAG_MPU6050, "Not properly set up: %d / 2 configured", IMU_SETUP); // Log error
            vTaskDelete(NULL); // Delete task if not configured properly
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification from timer callback

        uint8_t reg = MPU6050_REG_ACCEL_XOUT_H; // Register address for accel X MSB
        uint8_t raw_data[14]; // Buffer to hold raw accel, temp, gyro data

        // Read 14 bytes starting from accel X high byte register
        if (i2c_master_transmit_receive(mpu6050_handle, &reg, 1, raw_data, sizeof(raw_data), -1) == ESP_OK) {
            mpu6050_data_t imu_data_out; // Struct to hold converted IMU data

            // Combine high and low bytes for accelerometer and gyro values, bringing two 8 bits into a 16
            int16_t ax = (raw_data[0] << 8) | raw_data[1];
            int16_t ay = (raw_data[2] << 8) | raw_data[3];
            int16_t az = (raw_data[4] << 8) | raw_data[5];
            int16_t gx = (raw_data[8] << 8) | raw_data[9];
            int16_t gy = (raw_data[10] << 8) | raw_data[11];
            int16_t gz = (raw_data[12] << 8) | raw_data[13];

            // Convert raw accel counts to G units
            float ax_g = ax / accel_conv[current_accel_scale - 1];
            float ay_g = ay / accel_conv[current_accel_scale - 1];
            float az_g = az / accel_conv[current_accel_scale - 1];

            // Convert raw gyro counts to degrees per second
            float gx_dps = gx / gyro_conv[current_gyro_scale - 1];
            float gy_dps = gy / gyro_conv[current_gyro_scale - 1];
            float gz_dps = gz / gyro_conv[current_gyro_scale - 1];

            // Apply low-pass filter to smooth accelerometer data
            imu_data_out.accel_x = prev_ax = alpha_accel * ax_g + (1 - alpha_accel) * prev_ax;
            imu_data_out.accel_y = prev_ay = alpha_accel * ay_g + (1 - alpha_accel) * prev_ay;
            imu_data_out.accel_z = prev_az = alpha_accel * az_g + (1 - alpha_accel) * prev_az;

            // Apply low-pass filter to smooth gyroscope data
            imu_data_out.gyro_x = prev_gx = alpha_gyro * gx_dps + (1 - alpha_gyro) * prev_gx;
            imu_data_out.gyro_y = prev_gy = alpha_gyro * gy_dps + (1 - alpha_gyro) * prev_gy;
            imu_data_out.gyro_z = prev_gz = alpha_gyro * gz_dps + (1 - alpha_gyro) * prev_gz;

            xQueueSend(raw_imu_queue, &imu_data_out, 0); // Send filtered IMU data to queue for processing

            // Debug log accelerometer Y/Z and gyro X axis data for quick monitoring
            ESP_LOGE(TAG_MPU6050, "Accel: %.2f %.2f | Gyro: %.2f", imu_data_out.accel_y, imu_data_out.accel_z, imu_data_out.gyro_x);
        } else {
            ESP_LOGE(TAG_MPU6050, "Failed to read from MPU6050"); // Log error on failed I2C read
        }
    }
}

// Timer callback function to notify IMU task to read data
void IMU_timer(TimerHandle_t xTimer) {
    xTaskNotifyGive(imu_handle); // Notify the IMU task to proceed reading sensor data
}
