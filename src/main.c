#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <pin_config.h>
#include <setup.h>
#include <pwm.h>
#include <triangle_generator.h>

void app_main(void)
{
    setup();

    PWM_controller();

    dac_controller();

    while(1){

    }
}
