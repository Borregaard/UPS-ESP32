#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pin_config.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL0 LEDC_CHANNEL_0
#define PWM_CHANNEL1 LEDC_CHANNEL_1
#define PWM_DUTY_RES LEDC_TIMER_8_BIT
#define PWM_DUTY (255)
#define SCALAR (8)
#define SIN_WAVE_RES (20 * SCALAR)
#define PWM_FREQUENCY (SIN_WAVE_RES * 50)

static int SINE_LOOKUP_TABLE[SIN_WAVE_RES];
static int sine_table_index0 = 0;
static int sine_table_index1 = SIN_WAVE_RES / 2;

void sin_wave_table()
{
    int i;
    double wave = (2 * M_PI) / SIN_WAVE_RES;
    for (i = 0; i < SIN_WAVE_RES; i++)
    {
        SINE_LOOKUP_TABLE[i] = sin(wave * i) * PWM_DUTY;
    }
}

static void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_DUTY_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel0 = {

        .gpio_num = PWM_PIN0,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = PWM_DUTY,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel0));
    ledc_channel_config_t ledc_channel1 = {

        .gpio_num = PWM_PIN1,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = PWM_DUTY,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));
}

static void timer_callback(void *param)
{
    int DUTY0 = SINE_LOOKUP_TABLE[sine_table_index1];
    int DUTY1 = SINE_LOOKUP_TABLE[sine_table_index0];

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL0, (uint32_t)DUTY0));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL0));
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL1, (uint32_t)DUTY1));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL1));

    (DUTY0 > 0) ? (gpio_set_level(SELECT_PIN, true)) : (gpio_set_level(SELECT_PIN, false));

    if (++sine_table_index0 >= SIN_WAVE_RES)
    {
        sine_table_index0 = 0;
    }
    if (++sine_table_index1 >= SIN_WAVE_RES)
    {
        sine_table_index1 = 0;
    }
}

static void TIMER_INIT(void)
{
    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "My Timer"};
    esp_timer_handle_t timer_handler;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, PWM_FREQUENCY / (pow(SCALAR, 2))));
}

void PWM_controller(void)
{
    sin_wave_table();

    ledc_init();

    TIMER_INIT();
}
