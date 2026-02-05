#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"                 
#include "esp_adc/adc_oneshot.h"         
#include "hal/adc_types.h"

//pins
#define COIL_DRV_PIN   GPIO_NUM_10   // GPIO driving pulse for 430Ω->coil->GND
#define LED_PIN        GPIO_NUM_8    // LED
#define ADC_GPIO       GPIO_NUM_0    // GPIO0 is ADC reader for coil

// -------- Tuning knobs --------
#define PULSE_ON_US    1            // ms coil is at high state
#define PULSE_OFF_US   80            // ms coil is LOW state
#define SAMPLES_PER_MEAS  200        // more samples = less noise, slower response based off mapleaf config

#define BASELINE_MS    1500          // baseline averaging time on boot
#define THRESHOLD      10            // ADC counts; adjust after you see readings

static adc_oneshot_unit_handle_t adc_handle;

static void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,   // ADC1 (Analog-to-Digital Converter Unit 1)
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11, 
    };

    // for the ESP32-C3 the GPIO0 corresponds to the ADC_CHANNEL_0 on ADC_UNIT_1 to check for uncertainty
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg));
}

static int read_metric_once(void) {
    int sum = 0;
    for (int i = 0; i < SAMPLES_PER_MEAS; i++) {
        // square wave pulse driver
        gpio_set_level(COIL_DRV_PIN, 1);
        esp_rom_delay_us(PULSE_ON_US);

        // Sample the NODE using ADC 
        int adc_raw = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw);
        sum += adc_raw;

        // Turn off coil to allow for shift
        gpio_set_level(COIL_DRV_PIN, 0);
        esp_rom_delay_us(PULSE_OFF_US);
        //shift ma balz
    }
    return sum / SAMPLES_PER_MEAS;
}

void app_main(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    gpio_reset_pin(COIL_DRV_PIN);
    gpio_set_direction(COIL_DRV_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_DRV_PIN, 0);

    // ADC setup
    adc_init();

    //Baseline calibration to compare to init phase shift
    int baseline_sum = 0;
    int baseline_count = 0;

    int baseline_ticks = BASELINE_MS / 50; // sample baseline every 50ms
    for (int k = 0; k < baseline_ticks; k++) {
        int m = read_metric_once();
        baseline_sum += m;
        baseline_count++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    int baseline = baseline_sum / (baseline_count ? baseline_count : 1);

    printf("Baseline metric = %d\n", baseline);

    //phase shift detector main loop
    while (1) {
        int gooner = read_metric_once();
        int diff = gooner - baseline;
        if (diff < 0) diff = -diff;

        bool detected = (diff > THRESHOLD);
        gpio_set_level(LED_PIN, detected ? 1 : 0);

        printf("metric=%d diff=%d detected=%d\n", gooner, diff, detected);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
