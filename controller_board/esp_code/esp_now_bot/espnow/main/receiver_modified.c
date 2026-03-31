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
#include <math.h>
#include <time.h>
#include <complex.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"


#include "esp_timer.h"
#include "driver/gpio.h"
#include <driver/ledc.h>

#define L_Y_OFFSET 1.56
#define R_X_OFFSET 1.59

// these are the direction pins
#define IN1 GPIO_NUM_7
#define IN2 GPIO_NUM_6
#define IN3 GPIO_NUM_5
#define IN4 GPIO_NUM_4


#define PWM_1_CHANNEL LEDC_CHANNEL_0
#define PWM_2_CHANNEL LEDC_CHANNEL_1
#define PWM_3_CHANNEL LEDC_CHANNEL_2
#define PWM_4_CHANNEL LEDC_CHANNEL_3


#define LED_PIN 4           
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY 3000   

#define US_TRIG_PIN   GPIO_NUM_17
#define US_ECHO_PIN   GPIO_NUM_18

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

#define HEADLIGHT_GPIO GPIO_NUM_10
#define METAL_DETECT GPIO_NUM_11



typedef struct {
    float raw[4];
} adc_snap_t;

adc_snap_t adc_struct = {{-1.0, -1.0, -1.0, -1.0}};

static float g_ultrasonic_cm = -1.0f;
static portMUX_TYPE g_ultrasonic_mux = portMUX_INITIALIZER_UNLOCKED;

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;

// mac address of the master <-_->
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = { 0xDC, 0xB4, 0xD9, 0x0B, 0x64, 0x5C };
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xDC, 0xB4, 0xD9, 0x0B, 0x64, 0x5C };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);


int map(double x, double in_min, double in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ultrasonic code

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
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
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

/* Parse received ESPNOW data. */

// lock
static portMUX_TYPE adc_struct_mux = portMUX_INITIALIZER_UNLOCKED; 

int light_state = 0;

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

    float temp_arr [5] = {-1.0, -1.0, -1.0, -1.0, -1.0};
    memcpy(temp_arr, buf->payload, sizeof(temp_arr));
    light_state = (int)temp_arr[4];

    gpio_set_level(HEADLIGHT_GPIO, light_state);

    float control_data[4] = {-1.0, -1.0, -1.0, -1.0};
    memcpy(control_data, temp_arr, sizeof(temp_arr) - 1); // copy the first four elements
   
    adc_snap_t temp = {{-1.0, -1.0, -1.0, -1.0}}; // kind of forgot what the purpose of this is
    memcpy(temp.raw, control_data, sizeof(control_data));

    taskENTER_CRITICAL(&adc_struct_mux);
        adc_struct = temp;
    taskEXIT_CRITICAL(&adc_struct_mux);

    //ESP_LOGI("RECIEVED ADC DATA", "%f %f %f %f", temp.raw[0], temp.raw[1], temp.raw[2], temp.raw[3]);



    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;

    // Clear payload first
    memset(buf->payload, 0, send_param->len - sizeof(example_espnow_data_t));

    // Copy ultrasonic reading into payload
    float ultrasonic_cm;
    taskENTER_CRITICAL(&g_ultrasonic_mux);
    ultrasonic_cm = g_ultrasonic_cm;
    taskEXIT_CRITICAL(&g_ultrasonic_mux);

    int metal_state = gpio_get_level(METAL_DETECT);

    float combined_pay[2] = {-1.0, -1.0};
    combined_pay[0] = ultrasonic_cm; 
    combined_pay[1] = (float)metal_state;

    memcpy(buf->payload, combined_pay, sizeof(combined_pay));

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}




static void ultrasonic_tx_task(void *arg)
{
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)arg;
    assert(send_param != NULL);

    ESP_LOGI(TAG, "Ultrasonic TX task started");

    while (1) {
        example_espnow_data_prepare(send_param);
        esp_err_t err = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ultrasonic esp_now_send failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
    ESP_LOGI(TAG, "Robot RX task started");

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

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
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

    /* Initialize sending parameters. */
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


    xTaskCreatePinnedToCore(example_espnow_task, "robot_rx_task", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(ultrasonic_tx_task, "ultrasonic_tx_task", 4096, send_param, 6, NULL, 0);

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

static void configure_PWM(void){
    gpio_reset_pin(IN1);
    gpio_reset_pin(IN2);
    gpio_reset_pin(IN3);
    gpio_reset_pin(IN4);

    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    ledc_timer_config_t motora_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY
    };

    ledc_timer_config_t motorb_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY
    };

    ledc_timer_config_t motorc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY
    };

    ledc_timer_config_t motord_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY
    };
    
    ledc_timer_config(&motora_timer);
    ledc_timer_config(&motorb_timer);
    ledc_timer_config(&motorc_timer);
    ledc_timer_config(&motord_timer);



    ledc_channel_config_t motora_channel = {
        .gpio_num = IN1,
        .speed_mode = LEDC_MODE,
        .channel = PWM_1_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };

    ledc_channel_config_t motorb_channel = {
        .gpio_num = IN2,
        .speed_mode = LEDC_MODE,
        .channel = PWM_2_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };

    ledc_channel_config_t motorc_channel = {
        .gpio_num = IN3,
        .speed_mode = LEDC_MODE,
        .channel = PWM_3_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };

    ledc_channel_config_t motord_channel = {
        .gpio_num = IN4,
        .speed_mode = LEDC_MODE,
        .channel = PWM_4_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };

    ledc_channel_config(&motora_channel);
    ledc_channel_config(&motorb_channel);
    ledc_channel_config(&motorc_channel);
    ledc_channel_config(&motord_channel);

    return;
}

static void pwm_task(void * arg){
    adc_snap_t *adc_struct = (adc_snap_t *) arg;
    assert(adc_struct != NULL);

    //s_task_handle = xTaskGetCurrentTaskHandle();

    // ** NOTE **
    // Direction is uncertain at the momentprintf("LED duty cycle: %d\n", duty);

    // for setting directionadc_struct

    // Motor_L Clockwise
    //gpio_set_level(IN1, 1);
    //gpio_set_level(IN2, 0);

    // Motor_L Counter-Clockwise
    //gpio_set_level(IN1, 0);
    //gpio_set_level(IN2, 1);

    // Motor_R Clockwise
    //gpio_set_level(IN3, 1);
    //gpio_set_level(IN4, 0);

    // Motor_R Counter-Clockwise
    //gpio_set_level(IN3, 0);
    //gpio_set_level(IN4, 1);

    // waiting for command, if negs then we don't start
    while (1){
        int count = 0;
        // check to make ALL values are not -1.0
        for(int i = 0; i < sizeof(adc_struct->raw)/sizeof(float); i++){
            if (adc_struct->raw[i] != -1.0)
                count++;
        }
        if(count == 4)
            break; 

        vTaskDelay(1);
    }

    double complex z = 0 + 0*I;

    while (1) {

        adc_snap_t temp = *adc_struct; // copy the struct
        // now we need to write the code that copies the values from esp now into this struct
        

        double z_real =  adc_struct->raw[2] - R_X_OFFSET;
        if(fabsf(z_real) < 0.02)
            z_real = 0.00;
        
        double z_imm = -1 * adc_struct->raw[1] + L_Y_OFFSET; 
        if(fabsf(z_imm) < 0.02)
            z_imm = 0.00;

        z = z_real + z_imm*I;

        int duty;

        ESP_LOGI("","COMPLEX Values: %f %f", creal(z), cimag(z));

        // for the immaginary axis
        if(cimag(z) >= 0.00){
            // move forward
            duty = map(cimag(z), 0, 1.56, 0, 1023);
            //gpio_set_level(IN1, 1);
            //gpio_set_level(IN2, 0);

            //gpio_set_level(IN3, 0);
            //gpio_set_level(IN4, 1);

            ledc_set_duty(LEDC_MODE, PWM_1_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, PWM_1_CHANNEL);

            ledc_set_duty(LEDC_MODE, PWM_2_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, PWM_2_CHANNEL);

            ledc_set_duty(LEDC_MODE, PWM_3_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, PWM_3_CHANNEL);

            ledc_set_duty(LEDC_MODE, PWM_4_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, PWM_4_CHANNEL);

    
        } else{
            // move backwards
            duty = map(cimag(z), 0, -1.74, 0, 1023);


            ledc_set_duty(LEDC_MODE, PWM_1_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, PWM_1_CHANNEL);

            ledc_set_duty(LEDC_MODE, PWM_2_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, PWM_2_CHANNEL);

            ledc_set_duty(LEDC_MODE, PWM_3_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, PWM_3_CHANNEL);

            ledc_set_duty(LEDC_MODE, PWM_4_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, PWM_4_CHANNEL);
        }

        int turn_duty_r = 0, turn_duty_l = 0;

        // for the REAL axis (not fake)
        if (creal(z) >= 0){ // turning right
           turn_duty_r = -1.0 * map(creal(z) * 0.75, 0, 1.56, 0, 1023); // 0.5 is the weight that the turning has on the PWM signal
           if (duty == 0){
            turn_duty_r = 1.0 * map(creal(z) * 0.75, 0, 1.56, 0, 1023);

            // need to flip rotation
            
                    gpio_set_level(IN1, 0);
            gpio_set_level(IN2, 1);

            gpio_set_level(IN3, 0);
            gpio_set_level(IN4, 1);

            turn_duty_l = 1.0 * map(creal(z) * 0.75, 0, 1.56, 0, 1023);
           }
        } else{ // turning left
            turn_duty_l = -1.0 * map(creal(z) * 0.75, 0, -1.74, 0, 1023); // 0.5 is the weight that the turning has on the PWM signal
            if(duty == 0){
                turn_duty_l = 1.0 * map(creal(z) * 0.75, 0, -1.74, 0, 1023);

                gpio_set_level(IN3, 1);
                gpio_set_level(IN4, 0);
                turn_duty_r = 1.0 * map(creal(z) * 0.75, 0, -1.74, 0, 1023);
            }
        }
        
        /*
        ledc_set_duty(LEDC_MODE, PWM_L_CHANNEL, duty + turn_duty_l);
        ledc_update_duty(LEDC_MODE, PWM_L_CHANNEL);

        ledc_set_duty(LEDC_MODE, PWM_R_CHANNEL, duty + turn_duty_r);
        ledc_update_duty(LEDC_MODE, PWM_R_CHANNEL);
        */



        /*
        for (int duty = 0; duty <= 1023; duty += 10) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            
        }
        
        for (int duty = 1023; duty >= 0; duty -= 10) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            
        }
            */
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

}



static void ultrasonic_task(void *params)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(3000);

    while (1) {
        float d = ultrasonic_read_cm();
        if (d < 0) {
            d = ultrasonic_read_cm();   // optional retry
        }

        taskENTER_CRITICAL(&g_ultrasonic_mux);
        g_ultrasonic_cm = d;
        taskEXIT_CRITICAL(&g_ultrasonic_mux);

        if (d < 0) {
            ESP_LOGI("US", "Ultrasonic: timeout");
        } else {
            ESP_LOGI("US", "Ultrasonic: %.1f cm", d);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 0);

    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 0);

    gpio_set_direction(METAL_DETECT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(METAL_DETECT, GPIO_PULLDOWN_ENABLE);


    gpio_set_direction(HEADLIGHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(HEADLIGHT_GPIO, 0);

     ultrasonic_init();
    configure_PWM();
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 2, NULL);
    xTaskCreate(pwm_task, "pwm_task", 4096, &adc_struct, 5, NULL);
    //xTaskCreatePinnedToCore(pwm_task, "pwm_task", 4096, &adc_struct, 5, NULL, 1);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    example_espnow_init();

    
}