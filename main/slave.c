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
#include "driver/i2c_slave.h"

#define SDA GPIO_NUM_8
#define SCL GPIO_NUM_9
#define I2C_PORT I2C_NUM_0
#define SLAVE_ADDR 0x28

// pins
// driving pins START
#define IN1 GPIO_NUM_7
#define IN2 GPIO_NUM_6
#define IN3 GPIO_NUM_5
#define IN4 GPIO_NUM_4
#define PWM_L GPIO_NUM_15
#define PWM_R GPIO_NUM_16
// driving pins END

#define COIL_DRV_PIN GPIO_NUM_10
#define LED_PIN      GPIO_NUM_1
#define ADC_GPIO     GPIO_NUM_0

// Ultrasonic pins
#define US_TRIG_PIN  GPIO_NUM_4
#define US_ECHO_PIN  GPIO_NUM_5

// Tuning knobs
#define SOUND_CM_PER_US       0.0343f
#define PULSE_ON_US           8
#define PULSE_OFF_US          14
#define SAMPLES_PER_MEAS      400
#define ECHO_RISE_TIMEOUT_US  50000
#define ECHO_FALL_TIMEOUT_US  50000
#define BASELINE_MS           1500
#define THRESHOLD             18

#define BUZZER_PIN GPIO_NUM_7

static const char *TAG = "slave_v1";
static adc_oneshot_unit_handle_t adc_handle;

/* ---------------- I2C packet ---------------- */

typedef struct __attribute__((packed)) {
    uint8_t sof;
    uint8_t version;
    uint16_t seq;
    int16_t ultrasonic_cm_x10;
    int16_t coil_metric;
    uint8_t coil_detected;
    uint8_t checksum;
} sensor_packet_t;

static i2c_slave_dev_handle_t slave;
static sensor_packet_t pkt = {
    .sof = 0xA5,
    .version = 1,
    .seq = 0,
    .ultrasonic_cm_x10 = -1,
    .coil_metric = 0,
    .coil_detected = 0,
    .checksum = 0,
};

static uint8_t checksum(sensor_packet_t *p)
{
    uint8_t *d = (uint8_t *)p;
    uint8_t c = 0;
    for (int i = 0; i < (int)sizeof(sensor_packet_t) - 1; i++) {
        c ^= d[i];
    }
    return c;
}

/* ---------------- ADC / coil ---------------- */

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

/* ---------------- Ultrasonic ---------------- */

static void ultrasonic_init(void)
{
    gpio_reset_pin(US_TRIG_PIN);
    gpio_set_direction(US_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(US_TRIG_PIN, 0);

    gpio_reset_pin(US_ECHO_PIN);
    gpio_set_direction(US_ECHO_PIN, GPIO_MODE_INPUT);
}

static int64_t pulse_in_us(gpio_num_t pin, int level, int64_t timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return -1;
        }
    }

    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(pin) == level) {
        if ((esp_timer_get_time() - t0) > timeout_us) {
            return -1;
        }
    }

    return esp_timer_get_time() - t0;
}

static float ultrasonic_read_cm(void)
{
    gpio_set_level(US_TRIG_PIN, 0);
    esp_rom_delay_us(2);

    gpio_set_level(US_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG_PIN, 0);

    int64_t start = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start > ECHO_RISE_TIMEOUT_US) {
            return -1.0f;
        }
    }

    int64_t width = pulse_in_us(US_ECHO_PIN, 1, ECHO_FALL_TIMEOUT_US);
    if (width < 0) {
        return -1.0f;
    }

    return (width * SOUND_CM_PER_US) / 2.0f;
}

/* ---------------- Buzzer ---------------- */

static void buzzer_beep(int duration_ms)
{
    int cycles = (int)(duration_ms * 0.8f);

    for (int i = 0; i < cycles; i++) {
        gpio_set_level(BUZZER_PIN, 1);
        esp_rom_delay_us(250);
        gpio_set_level(BUZZER_PIN, 0);
        esp_rom_delay_us(250);
    }
}

/* ---------------- I2C v1 helpers ---------------- */

static void update_packet(int coil_metric, bool detected, float distance_cm)
{
    pkt.seq++;
    pkt.coil_metric = (int16_t)coil_metric;
    pkt.coil_detected = detected ? 1 : 0;
    pkt.ultrasonic_cm_x10 = (distance_cm < 0.0f) ? -1 : (int16_t)(distance_cm * 10.0f);
    pkt.checksum = checksum(&pkt);
}

/*
 * For slave v1, there is no on_request callback.
 * We keep the slave TX buffer loaded with the newest packet.
 */
static void push_packet_to_i2c(void)
{
    esp_err_t err = i2c_slave_transmit(
        slave,
        (const uint8_t *)&pkt,
        sizeof(pkt),
        0
    );

    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "i2c_slave_transmit failed: %s", esp_err_to_name(err));
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

    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);

    adc_init();
    ultrasonic_init();

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

    i2c_slave_config_t cfg = {
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .slave_addr = SLAVE_ADDR,
        .send_buf_depth = sizeof(sensor_packet_t) * 4,
        //.enable_internal_pullup = 1,
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&cfg, &slave));

    /* preload once so the master has something to read immediately */
    pkt.checksum = checksum(&pkt);
    push_packet_to_i2c();

    while (1) {
        int gooner = read_metric_once();
        int diff = gooner - baseline;
        if (diff < 0) diff = -diff;

        bool detected = (diff >= THRESHOLD);
        gpio_set_level(LED_PIN, detected ? 1 : 0);

        if (detected) {
            buzzer_beep(100);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
        printf("ECHO=%d\n", gpio_get_level(US_ECHO_PIN));

        gpio_set_level(COIL_DRV_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(60));

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

        update_packet(gooner, detected, d);
        push_packet_to_i2c();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}