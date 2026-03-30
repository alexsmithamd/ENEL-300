/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_lcd_panel_vendor.h"

#include "espnow_example.h"
#include "read_adc.h"
#include <driver/ledc.h>

#define BUZZER_PIN      GPIO_NUM_21
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT // 1024 duty resolution
#define FREQ            (2000)            // 2 kHz sound

// isr code
#define BUTTON_GPIO GPIO_NUM_38
#define DEBOUNCE_DELAY_US 200000ULL

static volatile uint64_t last_isr_time = 0;
static volatile uint32_t counter = 0;
static QueueHandle_t button_queue;

uint32_t button_counter;

//button isr
static void IRAM_ATTR button_isr(void *arg) {
    uint64_t now = esp_timer_get_time(); // Get current time in microseconds
    // Check if debounce period has passed, then process the button press
    if (now - last_isr_time > DEBOUNCE_DELAY_US) {
        counter++;
        uint32_t cnt = counter;
        BaseType_t higher_priority_task_woken = pdFALSE;
        xQueueSendFromISR(button_queue, &cnt, &higher_priority_task_woken); // Send counter to queue from ISR
        last_isr_time = now;
        if (higher_priority_task_woken) {
            portYIELD_FROM_ISR();
        }
    }
}


#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";
static const char *DISP_TAG = "oled_display";

static QueueHandle_t s_example_espnow_queue = NULL;

// mac addr of the SALVIA
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xDC, 0xB4, 0xD9, 0x0B, 0x32, 0xE0 };
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = { 0xDC, 0xB4, 0xD9, 0x0B, 0x32, 0xE0 };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

typedef struct {
    float raw[4];
} adc_snap_t;

// OLED / LVGL configuration from the SSD1306 example
#define I2C_HOST                    0
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (128 * 64)
#define EXAMPLE_PIN_NUM_SDA         18
#define EXAMPLE_PIN_NUM_SCL         17
#define EXAMPLE_PIN_NUM_RST         -1
#define EXAMPLE_I2C_HW_ADDR         0x3C
#define EXAMPLE_LCD_H_RES           128
#define EXAMPLE_LCD_V_RES           64
#define EXAMPLE_LCD_CMD_BITS        8
#define EXAMPLE_LCD_PARAM_BITS      8



static lv_disp_t *s_disp = NULL;
static lv_obj_t *s_distance_label = NULL;
static lv_obj_t *s_detected_label = NULL;

// for locking the adc_struct between threads
static portMUX_TYPE adc_struct_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE ultrasonic_mux = portMUX_INITIALIZER_UNLOCKED;

adc_snap_t adc_struct = {{-1.0, -1.0, -1.0, -1.0}}; // init as neg
static float g_ultrasonic_cm = -1.0f;

static void display_create_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    s_distance_label = lv_label_create(scr);
    lv_label_set_text(s_distance_label, "Dist: -- cm");
    lv_obj_set_width(s_distance_label, EXAMPLE_LCD_H_RES);
    lv_obj_align(s_distance_label, LV_ALIGN_TOP_LEFT, 0, 0);

    s_detected_label = lv_label_create(scr);
    lv_label_set_text(s_detected_label, "Detected: NO");
    lv_obj_align(s_detected_label, LV_ALIGN_TOP_LEFT, 0, 20);
}

static void display_update_labels(float distance_cm)
{
    if (s_distance_label == NULL || s_detected_label == NULL) {
        return;
    }

    char line1[32];
    char line2[32];

    if (distance_cm < 0.0f) {
        snprintf(line1, sizeof(line1), "Dist: waiting...");
        snprintf(line2, sizeof(line2), "Detected: NO");
    } else {
        snprintf(line1, sizeof(line1), "Dist: %.2f cm", distance_cm);
        snprintf(line2, sizeof(line2), "Detected: %s", (distance_cm > 0.0f) ? "YES" : "NO");
    }

    lv_label_set_text(s_distance_label, line1);
    lv_label_set_text(s_detected_label, line2);
}

static void display_init(void)
{
    ESP_LOGI(DISP_TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(DISP_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(DISP_TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,3,0))
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
#endif
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(DISP_TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    s_disp = lvgl_port_add_disp(&disp_cfg);
    //lv_disp_set_rotation(s_disp, 0);

    if (lvgl_port_lock(0)) {
        display_create_ui(s_disp);
        lvgl_port_unlock();
    }
}

static void display_task(void *arg)
{
    (void)arg;

    while (1) {
        float dist = -1.0f;

        taskENTER_CRITICAL(&ultrasonic_mux);
        dist = g_ultrasonic_cm;
        taskEXIT_CRITICAL(&ultrasonic_mux);

        if (lvgl_port_lock(pdMS_TO_TICKS(50))) {
            display_update_labels(dist);
            lvgl_port_unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}



// receive distance data
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal != crc) {
        return -1;
    }

    float rx_ultrasonic_cm = -1.0f;
    int metal_detected = 0;
    float recv_pay[2] = {-1.0, -1.0};

    size_t payload_len = data_len - sizeof(example_espnow_data_t);

    if (payload_len >= sizeof(float)) {
        memcpy(recv_pay, buf->payload, 2*sizeof(float));

        taskENTER_CRITICAL(&ultrasonic_mux);
        g_ultrasonic_cm = recv_pay[0];
        metal_detected = recv_pay[1];
        taskEXIT_CRITICAL(&ultrasonic_mux);
        if(metal_detected){
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 512);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        } else{
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        }
        ESP_LOGI("US", "received ultrasonic distance: %.2f cm", g_ultrasonic_cm);
        
    } else {
        ESP_LOGW(TAG, "Payload too small for ultrasonic float");
    }

    return buf->type;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;

    adc_snap_t snap_copy;

    taskENTER_CRITICAL(&adc_struct_mux);
    snap_copy = adc_struct;
    taskEXIT_CRITICAL(&adc_struct_mux);

    float control_data[5] = {snap_copy.raw[0], snap_copy.raw[1], snap_copy.raw[2], snap_copy.raw[3], counter % 2};
    uint8_t payload[sizeof(control_data)];

    memcpy(payload, control_data, sizeof(control_data));
    memcpy(buf->payload, payload, sizeof(payload));

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}


static void adc_task(void* arg)
{
    button_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Rising edge interrupt trigger
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL);


    adc_snap_t *snap = (adc_snap_t *)arg;
    assert(snap != NULL);

    esp_err_t ret_adc;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, 4, &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1) {
        if (xQueueReceive(button_queue, &button_counter, 1)) {
            printf("Button pressed %lu times.\n", button_counter);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        ret_adc = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, pdMS_TO_TICKS(200));

        if (ret_adc == ESP_OK) {
            uint32_t values[4] = {-1, -1, -1, -1};
            int j = 0;

            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES, j++) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t data = EXAMPLE_ADC_GET_DATA(p) & 0xFFF;
                if (j < 4) {
                    values[j] = data;
                }
            }

            static int64_t last_print_us = 0;
            int64_t now_us = esp_timer_get_time();

            if (now_us - last_print_us >= 10 * 1000) {
                last_print_us = now_us;

                adc_snap_t temp = *snap;
                float Vref = 3.3f;

                temp.raw[0] = ((float)values[0] / 4095.0f) * Vref;
                temp.raw[1] = ((float)values[1] / 4095.0f) * Vref;
                temp.raw[2] = ((float)values[2] / 4095.0f) * Vref;
                temp.raw[3] = ((float)values[3] / 4095.0f) * Vref;

                taskENTER_CRITICAL(&adc_struct_mux);
                adc_struct = temp;
                taskEXIT_CRITICAL(&adc_struct_mux);
            }
        } else if (ret_adc == ESP_ERR_TIMEOUT) {
            // ignore timeouts for now
        }
    }
}

static void controller_tx_task(void *arg)
{
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)arg;
    assert(send_param != NULL);

    ESP_LOGI(TAG, "Controller TX task started");

    while (1) {
        example_espnow_data_prepare(send_param);
        esp_err_t err = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "controller esp_now_send failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    int ret;

    (void)pvParameter;
    ESP_LOGI(TAG, "Controller RX task started");

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGD(TAG, "Send callback from " MACSTR ", status=%d",
                         MAC2STR(send_cb->mac_addr), send_cb->status);
                break;
            }

            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len,
                                                &recv_state, &recv_seq, &recv_magic);

                if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGD(TAG, "Received unicast from " MACSTR ", len=%d",
                             MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                } else if (ret < 0) {
                    ESP_LOGW(TAG, "Receive parse error from " MACSTR, MAC2STR(recv_cb->mac_addr));
                }

                free(recv_cb->data);
                break;
            }

            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;

    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_peer_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreatePinnedToCore(example_espnow_task, "controller_rx_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(controller_tx_task, "controller_tx_task", 4096, send_param, 1, NULL, 0);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}

float get_ultrasonic_cm(void)
{
    float value;
    taskENTER_CRITICAL(&ultrasonic_mux);
    value = g_ultrasonic_cm;
    taskEXIT_CRITICAL(&ultrasonic_mux);
    return value;
}

static void comms_task(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    ESP_ERROR_CHECK(example_espnow_init());
}


void app_main(void)
{
     ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // 2. Prepare Channel Configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = BUZZER_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0 initially
        .hpoint         = 0,
    };
    ledc_channel_config(&ledc_channel);
    
    display_init();
    xTaskCreatePinnedToCore(display_task, "display_task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, &adc_struct, 5, NULL, 1);

    comms_task();
}
