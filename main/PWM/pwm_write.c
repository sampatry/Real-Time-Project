#include "pwm_write.h"

const char *TAG_PWM_LEDC = "PWM_OUT"; // Tag for identifying LOGI messages

static QueueHandle_t pwm_output_queue = NULL; // Queue for sending pwm signal to pwm output
static bool PWM_SETUP = PWM_NOT_CONFIGURED;

// Timer function to get and output the PWM signal
void PWM_output_update(TimerHandle_t xTimer) {
    if (PWM_SETUP == PWM_NOT_CONFIGURED){
        ESP_LOGE(TAG_PWM_LEDC, "Not properly set up: %d / 2 configured", PWM_SETUP); // Return immedietly if the pwm output is not fully configured
        return;
    }
    if (pwm_output_queue == NULL) return; // Return immediatly if the queue is empty

    static int32_t pulse_width_out_us;
    if (xQueueReceive(pwm_output_queue, &pulse_width_out_us, 0) == pdPASS){
        ESP_LOGI(TAG_PWM_LEDC, "PWM Pulse Width Output: %" PRId32 " us", pulse_width_out_us);
        uint32_t duty = (pulse_width_out_us * ((1 << LEDC_TIMER_15_BIT) - 1)) / 20000; //currently setup for servo!!
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void PWM_output_config(gpio_num_t PWM_OUTPUT_GPIO, QueueHandle_t receive_queue) {
    // Configure timer for LEDC PWM output
    ledc_timer_config_t timer_conf = {
		.duty_resolution = LEDC_TIMER_15_BIT,
		.freq_hz = LEDC_FREQ_HZ,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.clk_cfg = LEDC_AUTO_CLK,
	};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf)); // Apply timer config

    // Configure channel for LEDC PWM output
    ledc_channel_config_t ledc_conf = {
		.channel = LEDC_CHANNEL_0,
		.duty = 0,
		.gpio_num = PWM_OUTPUT_GPIO,
		.intr_type = LEDC_INTR_DISABLE,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_sel = LEDC_TIMER_0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_conf)); //  Apply channel config

    ESP_LOGI(TAG_PWM_LEDC, "PWM output started on GPIO %d", (int)PWM_OUTPUT_GPIO);

    pwm_output_queue = receive_queue;

    PWM_SETUP = PWM_CONFIGURED;
    return;
}