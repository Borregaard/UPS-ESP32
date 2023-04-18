#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <pin_config.h>
#include <setup.h>
#include <pwm.h>

TaskHandle_t myTaskHandle = NULL;
TaskHandle_t myTaskHandle2 = NULL;

void mainLoop(void *arg);
void PWMLoop(void *arg);

void app_main(void)
{
    setup();

    xTaskCreatePinnedToCore(mainLoop, "mainLoop", 4096, NULL, 10, &myTaskHandle, 0);
    xTaskCreatePinnedToCore(PWMLoop, "PWMLoop", 4096, NULL, 100, &myTaskHandle2, 1);
}

void mainLoop(void *arg) {
    while (1) {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(100);

        gpio_set_level(LED_PIN, 1);
        vTaskDelay(100);
    }
}

void PWMLoop(void *arg) {
    PWM_controller();
}