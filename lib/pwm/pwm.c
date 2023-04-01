#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "soc/ledc_reg.h"
#include <pin_config.h>

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_DUTY (128)
#define SIN_WAVE_RES (128)
#define LEDC_FREQUENCY (50)

static int SINE_LOOKUP_TABLE[SIN_WAVE_RES];

static void sinWaveTable()
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

static int sine_table_index = 0;

void IRAM_ATTR ledc_timer0_overflow_isr(void *arg)
{
    // clear the interrupt
    REG_SET_BIT(LEDC_INT_CLR_REG, LEDC_HSTIMER0_OVF_INT_CLR);

    // update duty, shift the duty 4 bits to the left due to ESP32 register format
    REG_WRITE(LEDC_HSCH0_DUTY_REG, SINE_LOOKUP_TABLE[sine_table_index] << 4);
    REG_SET_BIT(LEDC_HSCH0_CONF1_REG, LEDC_DUTY_START_HSCH0);

    // increment the sine table index
    if (++sine_table_index >= SIN_WAVE_RES)
        sine_table_index = 0;
}

void PWM_controller(void)
{
    // Initilizes the PWM and the timer
    ledc_init();
    // creates the sine wavr look up table
    sinWaveTable();

    // register overflow interrupt handler for timer0
    ledc_isr_register(ledc_timer0_overflow_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    // enable the overflow interrupt
    REG_SET_BIT(LEDC_INT_ENA_REG, LEDC_HSTIMER0_OVF_INT_ENA);
}

void timer_init(void *arg)
{
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = LEDC_FREQUENCY,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
}