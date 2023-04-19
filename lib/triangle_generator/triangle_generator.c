#include <pin_config.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_err.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>

#define TARGET_FREQ (5000)
#define STEP_RES (80)
#define FREQ (STEP_RES * TARGET_FREQ)
#define PERIODE (1 / FREQ * pow(10, 6))
#define AMPLITUDE (255)

#define SAMPLE_RATE (44100*9)
#define DMA_BUF_LEN (32)
#define DMA_NUM_BUF (2)
#define I2S_NUM (0)
#define WAVE_FREQ_HZ (FREQ*9)

static int tri_wave_arr[STEP_RES];
static int tri_wave_idx = 0;

static void triangle_wave_signal_generator()
{
    float inc = AMPLITUDE / STEP_RES;  
    for (int i = 0; i < STEP_RES+2; i++)
    {
        if (i < STEP_RES / 2)
        {
            tri_wave_arr[i] = (int)2*(inc * i);
        }
        else
        {
            tri_wave_arr[i] = (int)2*(AMPLITUDE - inc * (i  + 1 - (STEP_RES / 2)));
        }
    }
}

// Output buffer (2ch interleaved)
static uint16_t out_buf[DMA_BUF_LEN * 2];

// Fill the output buffer and write to I2S DMA
static void write_buffer()
{
    float samp = 0.0f;
    size_t bytes_written;

    for (int i = 0; i < DMA_BUF_LEN; i++)
    {

        samp = tri_wave_arr[tri_wave_idx];
        if (++tri_wave_idx >= STEP_RES)
        {
            tri_wave_idx = 0;
        }
        // Shift to MSB of 16-bit int for internal DAC
        out_buf[i * 2] = out_buf[i * 2 + 1] = (uint16_t)samp << 8;
    }

    i2s_write(I2S_NUM, out_buf, sizeof(out_buf), &bytes_written, portMAX_DELAY);
}

static void audio_task(void *userData)
{
    while (1)
    {
        write_buffer();
    }
}

void dac_controller()
{
    triangle_wave_signal_generator();

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = DMA_NUM_BUF,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2};

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

    i2s_set_pin(I2S_NUM, NULL); // Internal DAC

    // Highest possible priority for realtime audio task
    xTaskCreate(audio_task, "audio", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
}