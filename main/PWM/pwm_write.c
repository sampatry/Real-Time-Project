#include "pwm_write.h"

const char *TAG_PWM_LEDC = "PWM_OUT"; // Tag for identifying LOGI messages

static bool PWM_SETUP = PWM_NOT_CONFIGURED;

// Timer function to get and output the PWM signal
void PWM_output_update(void* pvParameters) {
    motor_config_t *config = (motor_config_t *) pvParameters; // Convert the config back from void *
    if (PWM_SETUP == PWM_NOT_CONFIGURED){
        ESP_LOGE(TAG_PWM_LEDC, "Not properly set up: %d / 2 configured", PWM_SETUP); // Return immedietly if the pwm output is not fully configured
        vTaskDelete(NULL); // Deletes itself since it wasn't configured correctly
    }

    while (1){
        uint32_t pulse_width_out_us;
        if (xQueueReceive(config->QUEUE, &pulse_width_out_us, portMAX_DELAY) == pdPASS){
            ESP_LOGE(TAG_PWM_LEDC, "The pulse width is: %d and the period is: %d", pulse_width_out_us, (1000000/config->FREQ_HZ));
            if (pulse_width_out_us > (1000000/config->FREQ_HZ)){
                ESP_LOGE(TAG_PWM_LEDC, "The pulse width is greater than the period and has been clamped");
                pulse_width_out_us = (1000000/config->FREQ_HZ); // If pulse is greater than period set to period length
            }
            ESP_LOGI(TAG_PWM_LEDC, "pulse_width_out_us : %d", pulse_width_out_us);
            uint32_t duty = (pulse_width_out_us * ((1 << LEDC_TIMER_15_BIT) - 1)) / (1000000/config->FREQ_HZ);
            ESP_LOGI(TAG_PWM_LEDC, "Duty : %d", duty);
            ESP_LOGI(TAG_PWM_LEDC, "Duty cycle: %.2f%%", ((float)duty * 100.0f) / (float)((1 << LEDC_TIMER_15_BIT) - 1));
            ledc_set_duty(LEDC_LOW_SPEED_MODE, config->CHANNEL, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, config->CHANNEL);
        }
    }
}

void PWM_output_config(motor_config_t *config) {
    // Configure timer for LEDC PWM output
    ledc_timer_config_t timer_conf = {
		.duty_resolution = LEDC_TIMER_15_BIT,
		.freq_hz = config->FREQ_HZ,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = config->TIMER,
		.clk_cfg = LEDC_AUTO_CLK,
	};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf)); // Apply timer config

    // Configure channel for LEDC PWM output
    ledc_channel_config_t ledc_conf = {
		.channel = config->CHANNEL,
		.duty = 0,
		.gpio_num = config->GPIO,
		.intr_type = LEDC_INTR_DISABLE,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_sel = config->TIMER,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_conf)); //  Apply channel config

    if (config->QUEUE == NULL) {
        ESP_LOGE(TAG_PWM_LEDC, "PWM task missing queue in motor_config_t");
        return;
    }

    ESP_LOGI(TAG_PWM_LEDC, "PWM output started on GPIO %d", config->GPIO);
    PWM_SETUP = PWM_CONFIGURED;
    return;
}