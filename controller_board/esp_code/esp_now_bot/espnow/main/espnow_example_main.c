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
#include <stdio.h>
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

#include "freertos/task.h"

#include "driver/i2c_master.h"

#include "driver/gpio.h"
#include <driver/ledc.h>

#define SDA GPIO_NUM_18
#define SCL GPIO_NUM_17
#define I2C_PORT I2C_NUM_0
#define SLAVE_ADDR 0x28

typedef struct __attribute__((packed)) {
    uint8_t sof;
    uint8_t version;
    uint16_t seq;
    int16_t ultrasonic_cm_x10;
    int16_t coil_metric;
    uint8_t coil_detected;
    uint8_t checksum;
} sensor_packet_t;

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

uint8_t checksum(sensor_packet_t *p){
    uint8_t *d=(uint8_t*)p, c=0;
    for(int i=0;i<sizeof(sensor_packet_t)-1;i++) c^=d[i];
    return c;
}


#define L_Y_OFFSET 1.56
#define R_X_OFFSET 1.59

// these are the direction pins
#define IN1 GPIO_NUM_7
#define IN2 GPIO_NUM_6
#define IN3 GPIO_NUM_5
#define IN4 GPIO_NUM_4

// these are the PWM pins
#define PWM_L GPIO_NUM_7
#define PWM_R GPIO_NUM_5

#define PWM_L_CHANNEL LEDC_CHANNEL_0
#define PWM_R_CHANNEL LEDC_CHANNEL_1

#define LED_PIN 15           
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY 3000   

typedef struct {
    float raw[4];
} adc_snap_t;

adc_snap_t adc_struct = {{-1.0, -1.0, -1.0, -1.0}};


#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;

// mac address of the master dc:b4:d9:0b:32:e0
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xDC, 0xB4, 0xD9, 0x0B, 0x32, 0xE0 };
//static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);


int map(double x, double in_min, double in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

    float control_data[4] = {-1.0, -1.0, -1.0, -1.0};
    memcpy(control_data, buf->payload, sizeof(control_data));

    adc_snap_t temp = {{-1.0, -1.0, -1.0, -1.0}};
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
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}



static void example_espnow_task(void *pvParameter)
{
    i2c_master_bus_config_t bus_cfg={
        .clk_source=I2C_CLK_SRC_DEFAULT,
        .i2c_port=I2C_PORT,
        .sda_io_num=SDA,
        .scl_io_num=SCL,
        .flags.enable_internal_pullup=true,
    };
    i2c_new_master_bus(&bus_cfg,&bus);

    i2c_device_config_t dev_cfg={
        .device_address=SLAVE_ADDR,
        .dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz=100000,
    };
    i2c_master_bus_add_device(bus,&dev_cfg,&dev);


    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    while(1){
        sensor_packet_t pkt;
        if(i2c_master_receive(dev,(uint8_t*)&pkt,sizeof(pkt),100)==ESP_OK){
            if(pkt.checksum==checksum(&pkt)){
                ESP_LOGI("MASTER","Dist=%.1f cm, Coil=%d",
                    pkt.ultrasonic_cm_x10/10.0f,
                    pkt.coil_detected);
            }
        }
        vTaskDelay(60);
    }
    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
       

        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                //ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                //ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    //ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    //ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
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
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
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
    send_param->unicast = false;
    send_param->broadcast = true;
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
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);


    xTaskCreatePinnedToCore(example_espnow_task, "comms_task", 4096, send_param, 1, NULL, 0);
    //xTaskCreate(example_espnow_task, "example_espnow_task", 2560, send_param, 4, NULL);

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

    ledc_timer_config(&motora_timer);
    ledc_timer_config(&motorb_timer);



    ledc_channel_config_t motora_channel = {
        .gpio_num = PWM_L,
        .speed_mode = LEDC_MODE,
        .channel = PWM_L_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };

    ledc_channel_config_t motorb_channel = {
        .gpio_num = PWM_R,
        .speed_mode = LEDC_MODE,
        .channel = PWM_R_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };

    ledc_channel_config(&motora_channel);
    ledc_channel_config(&motorb_channel);

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
    gpio_set_level(IN1, 0);
    //gpio_set_level(IN2, 0);

    // Motor_L Counter-Clockwise
    //gpio_set_level(IN1, 0);
    //gpio_set_level(IN2, 1);

    // Motor_R Clockwise
    //gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);

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
            gpio_set_level(IN1, 1);
            //gpio_set_level(IN2, 0);

            gpio_set_level(IN3, 0);
            //gpio_set_level(IN4, 1);

    
        } else{
            // move backwards
            duty = map(cimag(z), 0, -1.74, 0, 1023);
            gpio_set_level(IN1, 0);
            gpio_set_level(IN2, 1);

            gpio_set_level(IN3, 1);
            gpio_set_level(IN4, 0);
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
        

        ledc_set_duty(LEDC_MODE, PWM_L_CHANNEL, duty + turn_duty_l);
        ledc_update_duty(LEDC_MODE, PWM_L_CHANNEL);

        ledc_set_duty(LEDC_MODE, PWM_R_CHANNEL, duty + turn_duty_r);
        ledc_update_duty(LEDC_MODE, PWM_R_CHANNEL);




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



void app_main(void)
{
    configure_PWM();
    xTaskCreatePinnedToCore(pwm_task, "pwm_task", 4096, &adc_struct, 5, NULL, 1);

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
