#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pin_config.h>
#include "driver/ledc.h"
#include "driver/timer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "esp_intr_alloc.h"
#include "sdkconfig.h"
#include "hal/timer_types.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_DUTY (128)
#define SIN_WAVE_RES (20)
#define LEDC_FREQUENCY (SIN_WAVE_RES*50)
#define TIMER_DELAY (1000000/LEDC_FREQUENCY)

static int SINE_LOOKUP_TABLE[SIN_WAVE_RES];
// static int sine_table_index = 0;

static void sin_wave_table()
{
    int i;
    float wave = (2 * M_PI) / SIN_WAVE_RES;
    for (i = 0; i < SIN_WAVE_RES; i++)
    {
        SINE_LOOKUP_TABLE[i] = sin(wave * i) * LEDC_DUTY + LEDC_DUTY;
    }
}

static void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {

        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = LEDC_DUTY,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static intr_handle_t s_timer_handle;

static void IRAM_ATTR timer_tg0_isr(void* arg)
{
	static int io_state = 0;

	// //Reset irq and set for next time
    // TIMER_0.int_clr_timers.t0 = 1;
    // TIMERG0.hw_timer[0].config.alarm_en = 1;


    //----- HERE EVERY #uS -----

	//Toggle a pin so we can verify the timer is working using an oscilloscope
	io_state ^= 1;									//Toggle the pins state
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_PIN, io_state);



}

void timer_tg0_initialise (int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,				//Alarm Enable
            .counter_en = false,			//If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
            .intr_type = TIMER_INTR_LEVEL,	//Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
            .counter_dir = TIMER_COUNT_UP,	//Does counter increment or decrement (timer_count_dir_t)
            .auto_reload = true,			//If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
            .divider = 80   				//Divisor of the incoming 80 MHz (12.5nS). Default the clock source is APB_CLK (typically 80 MHz). (Range 2 to 65536). 80 = 1uS per timer tick
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);  //(group_num, timer_num, alarm_value)  alarm_value (uint64) is how many timer ticks before the irq will be fired. (divider x alarm_value)=Time period, so 1uS x 100 = 100uS
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_tg0_isr, NULL, 0, &s_timer_handle);

    timer_start(TIMER_GROUP_0, TIMER_0);
}





void PWM_controller(void)
{
    // Initilizes the PWM and the timer
    ledc_init();
    // creates the sine wave look up table
    sin_wave_table();
    
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 128));
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    timer_tg0_initialise(100);

    while(1) {

    }
}
