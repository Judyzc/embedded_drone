#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"

static const char *TAG = "main";

void app_main(void) {
    QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_state_data, xQueue_ToF_data; 

    xQueue_acc_data = xQueueCreate(1, sizeof(acc_data_t)); 
    xQueue_gyro_data = xQueueCreate(1, sizeof(gyro_data_t)); 
    
    sensors_init(xQueue_acc_data, xQueue_gyro_data); 
    estimator_init(xQueue_acc_data, xQueue_gyro_data); 

    while (1) {}

    ESP_LOGE(TAG, "Exited main loop"); 
}