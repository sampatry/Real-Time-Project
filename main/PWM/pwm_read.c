#include "pwm_read.h" // Include custom header

static int64_t rise_time_us = 0; // 64 bit intead of standard 32 to avoid overflow
static int64_t pulse_width_us = 0; // 64 bit intead of standard 32 to avoid overflow

// ISR to capture rising/falling edges of PWM signal
static void IRAM_ATTR PWM_gpio_isr_handler(void* pvParameters) {
    RX_config_t *config = (RX_config_t *) pvParameters; // Convert the config back from void *
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // We have not woken a task at the start of the ISR. (Needed for QueueSendFromISR)
    bool level = gpio_get_level(config->GPIO);
    int64_t now = esp_timer_get_time(); // 64 bit intead of standard 32 to avoid overflow after ~35 min (probably overkill but better safe than sorry)

    if(config->digital != true){ // Check if gpio is set for pulse width or just digital value tracking
        if (level == 1) { // Rising edge
            // Store pulse start time
            rise_time_us = now;
        } else { // Falling edge
            // Calculte time from rising edge until now
            pulse_width_us = now - rise_time_us;
        }
        if(config->servo == true){
            pulse_width_us = ((pulse_width_us - ER5A_V2_PWM_MIN) * (SERVO_PWM_MAX - SERVO_PWM_MIN)) / (ER5A_V2_PWM_MAX - ER5A_V2_PWM_MIN) + SERVO_PWM_MIN; // Map ER5A-V2 pwm range to servo range
            if(pulse_width_us < ((SERVO_PWM_MAX - SERVO_PWM_MIN) / 2)){
                pulse_width_us = SERVO_PWM_MIN;
            }else if(pulse_width_us > (SERVO_PWM_MAX - SERVO_PWM_MIN)){
                pulse_width_us = SERVO_PWM_MAX;
            }else{
                pulse_width_us = (SERVO_PWM_MAX + SERVO_PWM_MIN) / 2;
            }
        }
        xQueueSendFromISR(config->QUEUE, &pulse_width_us, &xHigherPriorityTaskWoken); // Sends pulse width to respective queue
    }else{
        xQueueSendFromISR(config->QUEUE, &level, &xHigherPriorityTaskWoken); // Sends digital value to respective queue
    }
}

// To be called in main to configure PWM input reading
esp_err_t PWM_input_config(RX_config_t *config){
    static bool isr_service_installed = false; // Remember between calls if isr has already been installed

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->GPIO), //define what pin is being configured
        .mode = GPIO_MODE_INPUT, //set up pin as input
        .pull_up_en = GPIO_PULLUP_DISABLE, //dont interfere with ER5A pwm signal
        .pull_down_en = GPIO_PULLDOWN_DISABLE, //dont interfere with ER5A pwm signal
        .intr_type = GPIO_INTR_ANYEDGE, //trigger interupt on rising and falling edges
    };
    if(gpio_config(&io_conf) != ESP_OK){ // Configure GPIO and check it succeeds
        ESP_LOGE(TAG_PWM_READ, "GPIO config failed on GPIO %d", config->GPIO);
        return ESP_FAIL;
    }
    if(isr_service_installed != true){ // Ensure the ISR service hasnt already been installed on previous call
        if(gpio_install_isr_service(0) != ESP_OK){ // Install ISR service and check it succeeds
            isr_service_installed = true;
            ESP_LOGE(TAG_PWM_READ, "GPIO install isr failed on GPIO %d", config->GPIO);
            return ESP_FAIL;
        }
    }
    if(gpio_isr_handler_add(config->GPIO, PWM_gpio_isr_handler, config) != ESP_OK){ // Configure ISR routine and check it succeeds
        ESP_LOGE(TAG_PWM_READ, "GPIO add isr handler failed on GPIO %d", config->GPIO);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_PWM_READ, "PWM edge capture started on GPIO %d", config->GPIO);
    return ESP_OK;
}