#include "pwm_read.h" // Include custom header

// ISR to capture rising/falling edges of PWM signal
static void IRAM_ATTR PWM_gpio_isr_handler(void* pvParameters) {
    RX_config_t *config = (RX_config_t *) pvParameters; // Convert the config back from void *
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // We have not woken a task at the start of the ISR.

    bool level = gpio_get_level(config->GPIO);
    int64_t now = esp_timer_get_time(); // 64-bit int for pulse width measurement to avoid overflow

    if(config->digital != true){
        if (level == PWM_LOGIC_HIGH) { // Rising edge
            config->rise_time_us = now;
        } else { // Falling edge
            config->pulse_width_us = now - config->rise_time_us;

            if(config->servo == true){
                // Map ER5A PWM range to servo PWM range
                config->pulse_width_us = ((config->pulse_width_us - ER5A_V2_PWM_MIN) * (SERVO_PWM_MAX - SERVO_PWM_MIN)) / (ER5A_V2_PWM_MAX - ER5A_V2_PWM_MIN) + SERVO_PWM_MIN;

                // Clamp servo pulse widths to safe limits
                if(config->pulse_width_us < SERVO_PWM_LOWER_THRESH){
                    config->pulse_width_us = SERVO_PWM_CLAMP_MIN;
                } else if(config->pulse_width_us > SERVO_PWM_UPPER_THRESH_LOW && config->pulse_width_us < SERVO_PWM_UPPER_THRESH_HIGH){
                    config->pulse_width_us = SERVO_PWM_CLAMP_MAX;
                } else {
                    config->pulse_width_us = (SERVO_PWM_MAX + SERVO_PWM_MIN) / 2;
                }
            }
        }
        if(config->pulse_width_us != config->last_pulse_width){
            config->last_pulse_width = config->pulse_width_us; // Update previous pulse width to current
            xQueueSendFromISR(config->QUEUE, &config->pulse_width_us, &xHigherPriorityTaskWoken); // Send calculated pulse width to queue
        }
    } else {
        xQueueSendFromISR(config->QUEUE, &level, &xHigherPriorityTaskWoken); // Send received digital value to queue
    }
}

// To be called in main to configure PWM input reading and interrupt handling
esp_err_t PWM_input_config(RX_config_t *config){
    static bool isr_service_installed = false; // Track if ISR service has already been installed to avoid reinstalling

    // Prepare GPIO configuration structure for input pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->GPIO), // Select the GPIO pin to configure using a bitmask
        .mode = GPIO_MODE_INPUT, // Set pin as input (to receive PWM signal)
        .pull_up_en = GPIO_PULLUP_DISABLE, // Disable internal pull-up resistor (to avoid interference with external PWM signal)
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable internal pull-down resistor for same reason
        .intr_type = GPIO_INTR_ANYEDGE, // Configure interrupt on both rising and falling edges to measure pulse widths accurately
    };

    // Apply GPIO configuration and check for success
    if(gpio_config(&io_conf) != ESP_OK){
        ESP_LOGE(TAG_PWM_READ, "GPIO config failed on GPIO %d", config->GPIO);
        return ESP_FAIL; // Return failure if pin configuration fails
    }

    // Install ISR service once if not already installed
    if(!isr_service_installed){
        if(gpio_install_isr_service(0) != ESP_OK){
            isr_service_installed = true; // Mark as installed to avoid repeated attempts
            ESP_LOGE(TAG_PWM_READ, "GPIO install isr failed on GPIO %d", config->GPIO);
            return ESP_FAIL; // Return failure if ISR service installation fails
        }
    }

    // Attach the PWM_gpio_isr_handler ISR to the GPIO pin interrupt
    if(gpio_isr_handler_add(config->GPIO, PWM_gpio_isr_handler, config) != ESP_OK){
        ESP_LOGE(TAG_PWM_READ, "GPIO add isr handler failed on GPIO %d", config->GPIO);
        return ESP_FAIL; // Return failure if attaching ISR handler fails
    }

    ESP_LOGI(TAG_PWM_READ, "PWM edge capture started on GPIO %d", config->GPIO); // Log successful configuration
    return ESP_OK; // Indicate successful setup
}
