#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"
#include "controllers.h"
#include "motors.h"

static const char *TAG = "main";

/* ------------------------------------------- Global Variables  ------------------------------------------- */
// Initialize global variables across files
QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_ToF_data, xQueue_state_data; 
TickType_t start_tick = 0, end_tick = 0; 

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void app_main(void) {
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Time bomb
    xQueue_acc_data = xQueueCreate(1, sizeof(acc_data_t)); 
    xQueue_gyro_data = xQueueCreate(1, sizeof(gyro_data_t)); 
    xQueue_state_data = xQueueCreate(1, sizeof(state_data_t)); 
    
    sensors_init(); 
    estimator_init(); 
    controllers_init();
    motors_init(); 

    while (1) {
        vTaskDelay(2);      // Prevent watchdog from timing out
    }

    ESP_LOGE(TAG, "Exited main loop"); 
}