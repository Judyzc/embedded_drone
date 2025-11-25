/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
#include "espnow_basic_config.h"

#define ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit)  _EXAMPLE_ADC_UNIT_STR(unit)
#define ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN                   ADC_ATTEN_DB_6
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_FREQ                    20 * 1000
#define ADC_OUTPUT                  154.0 //V_out = 3.31 * (ADC_OUTPUT/255)
#define VOLT_MAX                    (3.31 * (ADC_OUTPUT/255.0))

#define DEADZONE                    0.1
#define DEADZONE_MAX                ((VOLT_MAX/2.0) + DEADZONE)
#define DEADZONE_MIN                ((VOLT_MAX/2.0) - DEADZONE)

#define READ_LEN                    16       //I beleive this is buffer size
#define BUFFER_SIZE                 128

#define GPIO_INPUT_PIN_SEL          ((1ULL<<GPIO_NUM_23) | (1ULL<<GPIO_NUM_22))
#define GPIO_OUTPUT_PIN_SEL          (1ULL<<GPIO_NUM_2)

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#endif

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[4] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4, ADC_CHANNEL_5};
#endif

static const char *TAG = "EXAMPLE";

static int led_level = 0;

//////////////////////// ESPNOW  //////////////////////////

#include <stdlib.h>
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_sleep.h"

static const char *TAG_ESPNOW = "Basic_Slave";

static EventGroupHandle_t s_evt_group;

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

static void packet_sent_cb(const esp_now_send_info_t *mac_addr, esp_now_send_status_t status)
{
    if (led_level == 0) {
        led_level = 1;
    } else {
        led_level = 0;
    }
    gpio_set_level(GPIO_NUM_2, led_level);

    if (mac_addr == NULL) {
        ESP_LOGE(TAG_ESPNOW, "Send cb arg error");
        return;
    }

    xEventGroupSetBits(s_evt_group, BIT(status));
}



static void init_espnow_slave(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start() );
#if MY_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(MY_ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(packet_sent_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK) );

    // Alter this if you want to specify the gateway mac, enable encyption, etc
    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = MY_RECEIVER_MAC,
        .channel = MY_ESPNOW_CHANNEL,
        .ifidx = MY_ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&broadcast_destination) );
}

void send_espnow_data(void* args)
{
    const uint8_t destination_mac[] = MY_RECEIVER_MAC;
     
    joystick_t *data= (joystick_t *) args;

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  // 20 ms

    
    for (;;) {
    // Send it
        ESP_LOGI(TAG, "Sending %u bytes to %02x:%02x:%02x:%02x:%02x:%02x",
         sizeof(joystick_t),
         destination_mac[0], destination_mac[1], destination_mac[2],
         destination_mac[3], destination_mac[4], destination_mac[5]);

        esp_err_t err = esp_now_send(destination_mac, (uint8_t*)data, sizeof(joystick_t));
        if(err != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error (%d)", err);
        }

        // Wait for callback function to set status bit
        EventBits_t bits = xEventGroupWaitBits(s_evt_group, BIT(ESP_NOW_SEND_SUCCESS) | BIT(ESP_NOW_SEND_FAIL), pdTRUE, pdFALSE, 2000 / portTICK_PERIOD_MS);
        if ( !(bits & BIT(ESP_NOW_SEND_SUCCESS)) )
        {
            if (bits & BIT(ESP_NOW_SEND_FAIL))
            {
                ESP_LOGE(TAG, "Send error");
            }
            ESP_LOGE(TAG, "Send timed out");
        }

        ESP_LOGI(TAG, "Sent!");

        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

////////////////////////               //////////////////////////

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUFFER_SIZE,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_FREQ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;
    } 
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void print_task(void * arg){
    joystick_t *my_joystick = (joystick_t *) arg;
    
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  // 20 ms

    for (;;){
        
        ESP_LOGI(TAG, "thrust: %.3f, yaw: %.3f, pitch: %.3f, roll: %.3f, L: %d, R: %d", my_joystick->joystick_thrust, my_joystick->joystick_yaw, my_joystick->joystick_pitch, my_joystick->joystick_roll, my_joystick->button_L, my_joystick->button_R);

        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void app_main(void)
{

    dac_oneshot_handle_t chan1_handle;
    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan1_cfg, &chan1_handle));

    ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan1_handle, ADC_OUTPUT));

    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

    init_espnow_slave();

    joystick_t my_joystick = {
        .joystick_thrust = 0.0,
        .joystick_yaw = 0.0,
        .joystick_pitch = 0.0,
        .joystick_roll = 0.0,
        .button_R = 0,
        .button_L = 0
    };

    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    gpio_config_t io_conf_2 = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf_2);

    // gpio_set_level(GPIO_NUM_2, 1);

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[READ_LEN] = {0};
    memset(result, 0xcc, READ_LEN);

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    ESP_ERROR_CHECK(adc_continuous_start(handle));

    // xTaskCreate(print_task, "print joystick values", 2048, (void *) &my_joystick, 1, NULL);
    xTaskCreate(send_espnow_data, "Sending joystick values", 2048, (void *) &my_joystick, 1, NULL);

    while (1) {

        char unit[] = EXAMPLE_ADC_UNIT_STR(ADC_UNIT);

        while (1) {
            ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);

            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = ADC_GET_CHANNEL(p);
                    uint32_t data = ADC_GET_DATA(p);

                    float joystick_volt = (((double) data) * ( VOLT_MAX/ 4095.0));
                    float joystick_percent = 0.0;

                    if (joystick_volt > DEADZONE_MIN && joystick_volt < DEADZONE_MAX){
                        joystick_percent = 0.0;
                    } else if (joystick_volt < DEADZONE_MIN) {
                        joystick_percent = ((joystick_volt - DEADZONE_MIN) / (DEADZONE_MIN)) * 100.0;
                    } else {
                        joystick_percent = ((joystick_volt - DEADZONE_MAX) / (VOLT_MAX - DEADZONE_MAX)) * 100.0;
                    }

                    switch (chan_num) {
                        case 4:
                            my_joystick.joystick_thrust = joystick_percent;
                            // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %f", unit, chan_num, my_joystick.joystick_thrust);
                            break;
                        case 5:
                            my_joystick.joystick_yaw = joystick_percent;
                            // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %f", unit, chan_num, my_joystick.joystick_yaw);
                            break;
                        case 6:
                            my_joystick.joystick_pitch = joystick_percent;
                            // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %f", unit, chan_num, my_joystick.joystick_pitch);
                            break;
                        case 7:
                            my_joystick.joystick_roll = joystick_percent;
                            // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %f", unit, chan_num, my_joystick.joystick_roll);
                            break;
                        default:
                            ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }

                    my_joystick.button_L = gpio_get_level(GPIO_NUM_23);
                    my_joystick.button_R = gpio_get_level(GPIO_NUM_22);
                    
                    // ESP_LOGI(TAG, "Unit: %s, Channel: Left, Value: %"PRIx32, unit, my_joystick.button_L);
                    // ESP_LOGI(TAG, "Unit: %s, Channel: Right, Value: %"PRIx32, unit, my_joystick.button_R);
                    
                }
            vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
