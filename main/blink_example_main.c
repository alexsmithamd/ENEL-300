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

//pins
// driving pins START
// these are the direction pins
#define IN1 GPIO_NUM_7
#define IN2 GPIO_NUM_6
#define IN3 GPIO_NUM_5
#define IN4 GPIO_NUM_4

// these are the PWM pins
#define PWM_L GPIO_NUM_15
#define PWM_R GPIO_NUM_16
// driving pins END

#define COIL_DRV_PIN   GPIO_NUM_10   // GPIO driving pulse for 430Ω->coil->GND
#define LED_PIN        GPIO_NUM_8    // LED
#define ADC_GPIO       GPIO_NUM_0    // GPIO0 is ADC reader for coil
// Ultrasonic pins 
#define US_TRIG_PIN   GPIO_NUM_4
#define US_ECHO_PIN   GPIO_NUM_5

//  Tuning knobs 
#define SOUND_CM_PER_US  0.0343f
#define PULSE_ON_US    8//70//20//1            // ms coil is at high state
#define PULSE_OFF_US   14//130//80            // ms coil is LOW state
#define SAMPLES_PER_MEAS  400//200        // more samples = less noise, slower response based off mapleaf config
// timeouts
#define ECHO_RISE_TIMEOUT_US   50000   // 30 ms (covers > 5m, but safe)
#define ECHO_FALL_TIMEOUT_US   50000
#define BASELINE_MS    1500//1500          // baseline averaging time on boot
#define THRESHOLD      18//10            // ADC counts; adjust after you see readings

#define BUZZER_PIN GPIO_NUM_7

static adc_oneshot_unit_handle_t adc_handle;

static void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,   // ADC1 (Analog-to-Digital Converter Unit 1)
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, 
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

static void ultrasonic_init(void) {
    gpio_reset_pin(US_TRIG_PIN);
    gpio_set_direction(US_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(US_TRIG_PIN, 0);

    gpio_reset_pin(US_ECHO_PIN);
    gpio_set_direction(US_ECHO_PIN, GPIO_MODE_INPUT);
}

static int64_t pulse_in_us(gpio_num_t pin, int level, int64_t timeout_us) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) return -1;
    }
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(pin) == level) {
        if ((esp_timer_get_time() - t0) > timeout_us) return -1;
    }
    return esp_timer_get_time() - t0;
}

static float ultrasonic_read_cm(void) {

    // Ensure low
    gpio_set_level(US_TRIG_PIN, 0);
    esp_rom_delay_us(2);

    // 10us trigger pulse
    gpio_set_level(US_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG_PIN, 0);

    // Wait for echo HIGH then measure HIGH width
    // First wait for rising edge (echo goes high)
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start > ECHO_RISE_TIMEOUT_US) return -1.0f;
    }

    // Now measure high pulse width
    int64_t width = pulse_in_us(US_ECHO_PIN, 1, ECHO_FALL_TIMEOUT_US);
    if (width < 0) return -1.0f;

    // distance = (time * speed) / 2
    float cm = (width * SOUND_CM_PER_US) / 2.0f;
    return cm;
}

static void buzzer_beep(int duration_ms)
{
    int cycles = duration_ms * 0.8;  // approx 2kHz tone

    for (int i = 0; i < cycles; i++) {
        gpio_set_level(BUZZER_PIN, 1);
        esp_rom_delay_us(250);   // half period
        gpio_set_level(BUZZER_PIN, 0);
        esp_rom_delay_us(250);
    }
}

void app_main(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    gpio_reset_pin(COIL_DRV_PIN);
    gpio_set_direction(COIL_DRV_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_DRV_PIN, 0);

    // buzzer init
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);

    // ADC setup
    adc_init();

    // Ultrasonic setup  <-- IMPORTANT
    ultrasonic_init();

    // Baseline calibration
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
    printf("THRESHOLD=%d SAMPLES=%d\n", THRESHOLD, SAMPLES_PER_MEAS);

    while (1) {
        int gooner = read_metric_once();
        int diff = gooner - baseline;
        if (diff < 0) diff = -diff;

        bool detected = (diff >= THRESHOLD);
        gpio_set_level(LED_PIN, detected ? 1 : 0);

        if(detected){
            buzzer_beep(100);  
        }

        // Small gap so ultrasonic isn't triggered right on switching noise
        vTaskDelay(pdMS_TO_TICKS(2));
        printf("ECHO=%d\n", gpio_get_level(US_ECHO_PIN));

        gpio_set_level(COIL_DRV_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(3));   // quiet time

        float d = ultrasonic_read_cm();
        if (d < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            d = ultrasonic_read_cm();
        }

        if (d < 0) {
            printf("Ultrasonic: timeout\n");
        } else {
            printf("Ultrasonic: %.1f cm\n", d);
        }

        printf("metric=%d diff=%d detected=%d\n", gooner, diff, detected);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


