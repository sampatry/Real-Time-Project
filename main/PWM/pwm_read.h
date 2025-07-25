//CH1: Roll
//CH2: Pitch
//CH3: Throttle
//CH4: Yaw
//DS3240 mg 270 servo stall current 0.8A @ 6V
#ifndef PWM_read_h
#define PWM_read_h

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h" // For use of hardware GPIO's
#include "esp_timer.h" // For accurate timing of PWM input reading (xTaskGetTickCount() is limited to ms not us)
#include "esp_log.h" // For use of ESP_LOG functions (ESP_LOGI, ESP_LOGE)
#include <stdint.h> // For use of int64_t
#include "esp_attr.h" //Required for IRAM_ATTR

#define TAG_PWM_READ "PWM_IN"
#define ER5A_V2_PWM_MIN 988
#define ER5A_V2_PWM_MAX 2012
#define SERVO_PWM_MIN 500
#define SERVO_PWM_MAX 2500

typedef struct {
    gpio_num_t GPIO;
    bool digital;
    bool servo;
    QueueHandle_t QUEUE;
} RX_config_t;

// Function to initialize the PWM input capture ISR
esp_err_t PWM_input_config(RX_config_t *config);

#endif