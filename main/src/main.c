#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"

#include "main.h"
#include "i2c_setup.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"
#include "controllers.h"
#include "motors.h"

#include "esp_now_example.h"

static const char *TAG = "main";

/* ------------------------------------------- Global Variables  ------------------------------------------- */
// Initialize global variables across files
QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_ToF_data, xQueue_state_data; 
TickType_t start_tick = 0, end_tick = 0; 

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
// void on_cmd(const char *cmd) { if (strcmp(cmd,"start")==0) start_my_process(); }
// espnow_register_cmd_cb(on_cmd);

#ifndef ROLE_TRANSMITTER
#define ROLE_TRANSMITTER 1
#endif

#define BUTTON_GPIO PIN_TOGGLE_A

static SemaphoreHandle_t s_button_sem = NULL;

static void on_cmd(const char *cmd)
{
    if (strcmp(cmd, "start") == 0) {
        ESP_LOGI(TAG, "%s", cmd);
        // start_my_process();
    }
}

static void IRAM_ATTR button_isr(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_button_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void button_task(void *pv)
{
    for (;;) {
        if (xSemaphoreTake(s_button_sem, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(50));
            int level = gpio_get_level(BUTTON_GPIO);
            if (level == 0) {
#if ROLE_TRANSMITTER
                ESP_LOGI(TAG, "espnow_send_start attempted");
                if (espnow_send_start() != ESP_OK) {
                    ESP_LOGW(TAG, "espnow_send_start failed");
                }
#endif
            }
        }
    }
}

void app_main(void) {
    // Initialize Queues between tasks
    xQueue_acc_data = xQueueCreate(1, sizeof(acc_data_t)); 
    xQueue_gyro_data = xQueueCreate(1, sizeof(gyro_data_t)); 
    xQueue_state_data = xQueueCreate(1, sizeof(state_data_t)); 
    xQueue_ToF_data = xQueueCreate(1, sizeof(uint16_t)); 
    
    // Call initialization functions
    ESP_LOGI(TAG, "Initializing components");

    init_osc_pin(PIN_TOGGLE_A);
    init_osc_pin(PIN_TOGGLE_B);


    i2c_master_init(); 
    sensors_init(); 
    estimator_init(); 
    controllers_init();
    motors_init(); 

    // Calibrate sensors and start stabilization loop
    // calibrate_sensors();
    // for (int i=3; i>0; i--) {
    //     ESP_LOGI(TAG, "Starting in: %d", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // start_control_loop();

    comms_init();
    espnow_register_cmd_cb(on_cmd);

#if ROLE_TRANSMITTER
    s_button_sem = xSemaphoreCreateBinary();
    if (!s_button_sem) {
        ESP_LOGE(TAG, "Failed to create button semaphore");
    } else {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << BUTTON_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
        };
        gpio_config(&io_conf);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL);
        xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    }
#endif

    while (1) {
        vTaskDelay(1); // Prevent watchdog from timing out
    }

    ESP_LOGE(TAG, "Exited main loop"); 
}
