#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"

static const char *TAG = "main";

// Initialize global variables across files
QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_state_data, xQueue_ToF_data; 

void app_main(void) {
    xQueue_acc_data = xQueueCreate(1, sizeof(acc_data_t)); 
    xQueue_gyro_data = xQueueCreate(1, sizeof(gyro_data_t)); 
    
    sensors_init(); 
    estimator_init(); 

    while (1) {
        vTaskDelay(2); 
    }

    ESP_LOGE(TAG, "Exited main loop"); 
}