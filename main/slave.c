#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "esp_log.h"




#define COIL_DRV_PIN GPIO_NUM_10
#define LED_PIN      GPIO_NUM_8
#define ADC_GPIO     GPIO_NUM_0


// Tuning knobs
#define SOUND_CM_PER_US       0.0343f
#define PULSE_ON_US           8
#define PULSE_OFF_US          14
#define SAMPLES_PER_MEAS      400
#define ECHO_RISE_TIMEOUT_US  50000
#define ECHO_FALL_TIMEOUT_US  50000
#define BASELINE_MS           2000
#define THRESHOLD             20



static adc_oneshot_unit_handle_t adc_handle;


static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg));
}

static int read_metric_once(void)
{
    int sum = 0;

    for (int i = 0; i < SAMPLES_PER_MEAS; i++) {
        gpio_set_level(COIL_DRV_PIN, 1);
        esp_rom_delay_us(PULSE_ON_US);

        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw));
        sum += adc_raw;

        gpio_set_level(COIL_DRV_PIN, 0);
        esp_rom_delay_us(PULSE_OFF_US);
    }

    return sum / SAMPLES_PER_MEAS;
}


void app_main(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1);

    gpio_reset_pin(1);
    gpio_set_direction(1, GPIO_MODE_OUTPUT);
    gpio_set_level(1, 0);

    gpio_reset_pin(COIL_DRV_PIN);
    gpio_set_direction(COIL_DRV_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_DRV_PIN, 0);


    adc_init();

    int baseline_sum = 0;
    int baseline_count = 0;
    int baseline_ticks = BASELINE_MS / 50;

    for (int k = 0; k < baseline_ticks; k++) {
        int m = read_metric_once();
        baseline_sum += m;
        baseline_count++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    int baseline = baseline_sum / (baseline_count ? baseline_count : 1);

    printf("Baseline metric = %d\n", baseline);
    printf("THRESHOLD=%d SAMPLES=%d\n", THRESHOLD, SAMPLES_PER_MEAS);



    while (1) {
        int gooner = read_metric_once();
        int diff = gooner - baseline;
        if (diff < 0) diff = -diff;

        bool detected = (diff >= THRESHOLD);
        gpio_set_level(LED_PIN, detected ? 0 : 1);
        gpio_set_level(1, detected ? 1 : 0);


        vTaskDelay(pdMS_TO_TICKS(2));
       

        gpio_set_level(COIL_DRV_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(60));


        printf("metric=%d diff=%d detected=%d\n", gooner, diff, detected);


        vTaskDelay(pdMS_TO_TICKS(100));
    }
}