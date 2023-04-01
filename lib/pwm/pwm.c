#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "soc/ledc_reg.h"
#include <pin_config.h>
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_DUTY (128)
#define SIN_WAVE_RES (128)
#define LEDC_FREQUENCY (50)

// static int SINE_LOOKUP_TABLE[SIN_WAVE_RES];

// static void sinWaveTable()
// {
//     int i;
//     float wave = (2 * M_PI) / SIN_WAVE_RES;
//     for (i = 0; i < SIN_WAVE_RES; i++)
//     {
//         SINE_LOOKUP_TABLE[i] = sin(wave * i) * LEDC_DUTY + LEDC_DUTY;
//     }
// }

// static void ledc_init(void)
// {
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode = LEDC_MODE,
//         .duty_resolution = LEDC_DUTY_RES,
//         .timer_num = LEDC_TIMER,
//         .freq_hz = LEDC_FREQUENCY,
//         .clk_cfg = LEDC_AUTO_CLK};
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
//     ledc_channel_config_t ledc_channel = {

//         .gpio_num = PWM_PIN,
//         .speed_mode = LEDC_MODE,
//         .channel = LEDC_CHANNEL,
//         .intr_type = LEDC_INTR_DISABLE,
//         .timer_sel = LEDC_TIMER,
//         .duty = LEDC_DUTY,
//         .hpoint = 0};
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
// }

// static int sine_table_index = 0;

// void IRAM_ATTR ledc_timer0_overflow_isr(void *arg)
// {
//     // clear the interrupt
//     REG_SET_BIT(LEDC_INT_CLR_REG, LEDC_HSTIMER0_OVF_INT_CLR);

//     // update duty, shift the duty 4 bits to the left due to ESP32 register format
//     REG_WRITE(LEDC_HSCH0_DUTY_REG, SINE_LOOKUP_TABLE[sine_table_index] << 4);
//     REG_SET_BIT(LEDC_HSCH0_CONF1_REG, LEDC_DUTY_START_HSCH0);

//     // increment the sine table index
//     if (++sine_table_index >= SIN_WAVE_RES)
//         sine_table_index = 0;
// }

static const char *TAG = "example";

typedef struct
{
    uint64_t event_count;
} example_queue_element_t;

static bool IRAM_ATTR example_timer_on_alarm_cb_v1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    // stop timer immediately
    gptimer_stop(timer);
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value};
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

void PWM_controller(void)
{
    // Initilizes the PWM and the timer
    // ledc_init();
    // // creates the sine wavr look up table
    // sinWaveTable();

    // // register overflow interrupt handler for timer0
    // ledc_isr_register(ledc_timer0_overflow_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    // // enable the overflow interrupt
    // REG_SET_BIT(LEDC_INT_ENA_REG, LEDC_HSTIMER0_OVF_INT_ENA);

    example_queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!queue)
    {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    ESP_LOGI(TAG, "Create timer handle");
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = example_timer_on_alarm_cb_v1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG, "Start timer, stop it at alarm event");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 1000000, // period = 1s
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    if (xQueueReceive(queue, &ele, pdMS_TO_TICKS(2000)))
    {
        ESP_LOGI(TAG, "Timer stopped, count=%llu", ele.event_count);
    }
    else
    {
        ESP_LOGW(TAG, "Missed one count event");
    }

    ESP_LOGI(TAG, "Set count value");
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 100));
    ESP_LOGI(TAG, "Get count value");
    uint64_t count;
    ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &count));
    ESP_LOGI(TAG, "Timer count value=%llu", count);

    // before updating the alarm callback, we should make sure the timer is not in the enable state
    ESP_LOGI(TAG, "Disable timer");
    ESP_ERROR_CHECK(gptimer_disable(gptimer));
    

    vQueueDelete(queue);
}
