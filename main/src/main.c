#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

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
QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_ToF_data, xQueue_state_data, xQueue_optf_data; 
TickType_t start_tick = 0, end_tick = 0; 

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void app_main(void) {
    // Initialize Queues between tasks
    xQueue_acc_data = xQueueCreate(1, sizeof(acc_data_t)); 
    xQueue_gyro_data = xQueueCreate(1, sizeof(gyro_data_t)); 
    xQueue_state_data = xQueueCreate(1, sizeof(state_data_t)); 
    xQueue_ToF_data = xQueueCreate(1, sizeof(uint16_t)); 
    xQueue_optf_data = xQueueCreate(1, sizeof(2*uint16_t));
    
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
    calibrate_sensors();
    for (int i=2; i>0; i--) {
        ESP_LOGI(TAG, "Starting in: %d", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    start_control_loop();

    // test_wifi();

    while (1) {
        vTaskDelay(1);      // Prevent watchdog from timing out
    }

    ESP_LOGE(TAG, "Exited main loop"); 
}