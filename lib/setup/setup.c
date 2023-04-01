#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <pin_config.h>

void setup() {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}