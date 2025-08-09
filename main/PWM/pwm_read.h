// CH1: Roll
// CH2: Pitch
// CH3: Throttle
// CH4: Yaw
// DS3240 mg 270 servo stall current 0.8A @ 6V

#ifndef PWM_READ_H
#define PWM_READ_H

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions
#include "freertos/queue.h" // FreeRTOS queue for inter-task communication
#include "driver/gpio.h" // GPIO hardware interface
#include "esp_timer.h" // High-resolution timer for PWM pulse measurement in microseconds
#include "esp_log.h" // ESP logging functions for debugging and info output
#include <stdint.h> // Standard integer types (int64_t)
#include "esp_attr.h" // IRAM_ATTR for ISR functions

#define TAG_PWM_READ "PWM_IN" // Logging tag for PWM input module

// PWM logic levels for readability
#define PWM_LOGIC_HIGH 1
#define PWM_LOGIC_LOW 0

// Represents the min and max pulse widths expected from the ER5A-V2 remote receiver output
#define ER5A_V2_PWM_MIN 988
#define ER5A_V2_PWM_MAX 2012

// Defines the range of PWM pulse widths for controlling servos
#define SERVO_PWM_MIN 500
#define SERVO_PWM_MAX 2500

// Servo pulse width clamping thresholds and limits
#define SERVO_PWM_CLAMP_MIN 700
#define SERVO_PWM_LOWER_THRESH 1200
#define SERVO_PWM_UPPER_THRESH_LOW 1800
#define SERVO_PWM_UPPER_THRESH_HIGH 3000
#define SERVO_PWM_CLAMP_MAX 2300

// Holds GPIO pin info, mode flags, timing measurements, and a queue handle for communication
typedef struct {
    gpio_num_t GPIO; // GPIO number to monitor PWM input signal
    bool digital; // Flag to indicate digital (true) or pulse width measurement mode (false)
    bool servo; // Flag indicating whether the input controls a servo (affects pulse width mapping)
    QueueHandle_t QUEUE; // FreeRTOS queue handle for sending measured pulse widths or digital levels
    int64_t rise_time_us; // Timestamp of last rising edge (in microseconds)
    int64_t pulse_width_us; // Measured pulse width (in microseconds)
    int64_t last_pulse_width; // Previous pulse width, used to detect changes before queue send
} RX_config_t;

// Configures GPIO as input, sets up interrupts on both edges, and installs ISR for pulse width measurement.
esp_err_t PWM_input_config(RX_config_t *config);

#endif // PWM_READ_H
