//CH1: Roll
//CH2: Pitch
//CH3: Throttle
//CH4: Yaw
//DS3240 mg 270 servo stall current 0.8A @ 6V
#ifndef PWM_read_h
#define PWM_read_h

#include "driver/gpio.h" // For use of hardware GPIO's
#include "esp_timer.h" // For accurate timing of PWM input reading (xTaskGetTickCount() is limited to ms not us)
#include "esp_log.h" // For use of ESP_LOG functions (ESP_LOGI, ESP_LOGE)
#include <stdint.h> // For use of int64_t

// Function to initialize the PWM input capture ISR
void PWM_input_config(gpio_num_t PWM_INPUT_GPIO);

// Returns the latest pulse width in microseconds
int64_t PWM_get_input_us(void);

#endif