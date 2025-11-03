#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"

/* ------------------------------------------- Global Variables  ------------------------------------------- */
static const char *TAG = "estimator"; 
SixAxis comp_filter;

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void vUpdateEstimatorTask(void *pvParameters) { 
    QueueHandle_t xQueue_acc_data = ((QueueHandle_t *) pvParameters)[0]; 
    QueueHandle_t xQueue_gyro_data = ((QueueHandle_t *) pvParameters)[1];
    free(pvParameters); 

    acc_data_t acc_data; 
    gyro_data_t gyro_data; 
    for (;;) {
        xQueueReceive(xQueue_acc_data, (void *) &acc_data, portMAX_DELAY); 
        xQueueReceive(xQueue_gyro_data, (void *) &gyro_data, portMAX_DELAY); 

        CompAccelUpdate(&comp_filter, acc_data.ax_m_s2, acc_data.ay_m_s2, acc_data.az_m_s2); 
        CompGyroUpdate(&comp_filter, gyro_data.Gx_rad_s, gyro_data.Gy_rad_s, gyro_data.Gz_rad_s); 
        CompUpdate(&comp_filter); 

        float pitch_rad, roll_rad; 
        CompAnglesGet(&comp_filter, &roll_rad, &pitch_rad); 
        float pitch_deg = CompRadiansToDegrees(pitch_rad); 
        if (pitch_deg > 180.0)
            pitch_deg -= 360.0; 
        float roll_deg = CompRadiansToDegrees(roll_rad); 
        if (roll_deg > 180.0)
            roll_deg-= 360.0; 
        roll_deg *= -1.0; 

        ESP_LOGI(TAG, "Attitude (deg): Pitch=%.1f Roll=%.1f", pitch_deg, roll_deg);
    } 
}

void estimator_init(QueueHandle_t xQueue_acc_data, QueueHandle_t xQueue_gyro_data) {
    // Initialize complementary filter
    const float deltaT = 1.0f/SENS_RATE_HZ;  
    CompInit(&comp_filter, deltaT, TAU);
    CompAccelUpdate(&comp_filter, 0.0, 0.0, 9.81); 
    CompStart(&comp_filter); 

    // Start filter update task
    QueueHandle_t *pxQueues = malloc(2*sizeof(QueueHandle_t)); 
    pxQueues[0] = xQueue_acc_data; 
    pxQueues[1] = xQueue_gyro_data; 
    xTaskCreate(vUpdateEstimatorTask, "Acc Data Processing", 4096, (void *) pxQueues, ESTIMATOR_PRIORITY, NULL);
}