#include "PWM_read.h" // Include custom header

static const char *TAG = "PWM_IN"; // Tag for identifying LOGI messages
static gpio_num_t pwm_input_gpio = GPIO_NUM_19; // Default value, will be overwritten

// Volatile variables shared with ISR
static volatile int64_t rise_time_us = 0; // 64 bit intead of standard 32 to avoid overflow
static volatile int64_t pulse_width_us = 0; // 64 bit intead of standard 32 to avoid overflow

// ISR to capture rising/falling edges
static void IRAM_ATTR PWM_gpio_isr_handler(void* arg) {
    bool level = gpio_get_level(pwm_input_gpio);
    int64_t now = esp_timer_get_time(); // 64 bit intead of standard 32 to avoid overflow after ~35 min

    if (level == 1) { // Rising edge
        // Store pulse start time
        rise_time_us = now;
    } else { // Falling edge
        // Calculte time from rising edge until now
        pulse_width_us = now - rise_time_us;
    }
}

// To be called in main to configure PWM input reading
void PWM_input_config(gpio_num_t PWM_INPUT_GPIO){
    pwm_input_gpio = PWM_INPUT_GPIO;  // Save for ISR use
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PWM_INPUT_GPIO), //define what pin is being configured
        .mode = GPIO_MODE_INPUT, //set up pin as input
        .pull_up_en = GPIO_PULLUP_DISABLE, //dont interfere with ER5A pwm signal
        .pull_down_en = GPIO_PULLDOWN_DISABLE, //dont interfere with ER5A pwm signal
        .intr_type = GPIO_INTR_ANYEDGE, //trigger interupt on rising and falling edges
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Install ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PWM_INPUT_GPIO, PWM_gpio_isr_handler, NULL));

    ESP_LOGI(TAG, "PWM edge capture started on GPIO %d", PWM_INPUT_GPIO);
}

// To be called in main to get PWM input reading
int64_t PWM_get_input_us(void) {
    return pulse_width_us;
}