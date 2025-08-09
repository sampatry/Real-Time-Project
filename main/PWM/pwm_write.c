#include "pwm_write.h"

static bool PWM_SETUP = PWM_NOT_CONFIGURED; // Flag to track PWM output configuration status

// Task to receive PWM pulse width from queue and update PWM output
void PWM_output_update(void* pvParameters) {
    motor_config_t *config = (motor_config_t *) pvParameters; // Cast parameter to motor config struct pointer
    if (PWM_SETUP == PWM_NOT_CONFIGURED){
        ESP_LOGE(TAG_PWM_WRITE, "Not properly set up: %d / 2 configured", PWM_SETUP); // Abort if PWM not configured
        vTaskDelete(NULL); // Delete this task if not configured
    }

    while (1){
        uint32_t pulse_width_out_us;
        if (xQueueReceive(config->QUEUE, &pulse_width_out_us, portMAX_DELAY) == pdPASS){ // Wait indefinitely for PWM value
            // Clamp pulse width if greater than period to prevent invalid duty cycle
            if (pulse_width_out_us > (microseconds/config->FREQ_HZ)){
                pulse_width_out_us = (microseconds/config->FREQ_HZ); // Clamp to period length in microseconds
            }
            // Convert pulse width (us) to duty cycle value for 10-bit resolution PWM
            uint32_t duty = (pulse_width_out_us * ((1 << LEDC_TIMER_10_BIT) - 1)) / (microseconds/config->FREQ_HZ);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, config->CHANNEL, duty); // Set LEDC duty cycle
            ledc_update_duty(LEDC_LOW_SPEED_MODE, config->CHANNEL); // Update LEDC PWM output
        }
    }
}

// Configure PWM timer and channel with parameters from motor_config_t struct
esp_err_t PWM_output_config(motor_config_t *config) {
    ledc_timer_config_t timer_conf = {
		.duty_resolution = LEDC_TIMER_10_BIT, // 10-bit duty resolution
		.freq_hz = config->FREQ_HZ, // PWM frequency in Hz
		.speed_mode = LEDC_LOW_SPEED_MODE, // Use low speed mode for LEDC
		.timer_num = config->TIMER, // Timer number for this PWM
		.clk_cfg = LEDC_AUTO_CLK // Automatic clock selection
	};
    if (ledc_timer_config(&timer_conf) != ESP_OK) { // Apply timer config and check for errors
        ESP_LOGE(TAG_PWM_WRITE, "Failed to configure PWM output timer for gpio: %d", config->GPIO);
        return ESP_FAIL;
    }

    ledc_channel_config_t ledc_conf = {
		.channel = config->CHANNEL, // LEDC channel number
		.duty = 0, // Initial duty cycle set to 0 (off)
		.gpio_num = config->GPIO, // GPIO pin used for PWM output
		.intr_type = LEDC_INTR_DISABLE, // Disable LEDC interrupts
		.speed_mode = LEDC_LOW_SPEED_MODE, // Low speed mode for LEDC
		.timer_sel = config->TIMER // Select timer configured above
	};
    if (ledc_channel_config(&ledc_conf) != ESP_OK) { // Apply channel config and check for errors
        ESP_LOGE(TAG_PWM_WRITE, "Failed to configure PWM output channel for gpio: %d", config->GPIO);
        return ESP_FAIL;
    }

    if (config->QUEUE == NULL) { // Ensure queue handle is valid before starting task
        ESP_LOGE(TAG_PWM_WRITE, "PWM task missing queue in motor_config_t");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_PWM_WRITE, "PWM output started on GPIO %d", config->GPIO); // Log successful start
    PWM_SETUP = PWM_CONFIGURED; // Mark PWM as configured
    return ESP_OK;
}