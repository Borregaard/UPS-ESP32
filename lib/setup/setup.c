#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <pin_config.h>

void setup() {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWM_PIN0, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWM_PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BOARD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SELECT_PIN, GPIO_MODE_OUTPUT);
}