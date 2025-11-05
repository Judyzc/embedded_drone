#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "math.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"

static const char *TAG = "estimator"; 

/* ------------------------------------------- Global Variables  ------------------------------------------- */
SixAxis comp_filter;

/* ------------------------------------------- Private Function Definitions  ------------------------------------------- */
void vUpdateEstimatorTask(void *pvParameters) { 
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
        if (pitch_rad > M_PI)
            pitch_rad -= 2.0*M_PI;  
        if (roll_rad > M_PI)
            roll_rad -= 2.0*M_PI; 
        roll_rad *= -1.0; 

        float pitch_deg = CompRadiansToDegrees(pitch_rad); 
        float roll_deg = CompRadiansToDegrees(roll_rad);
        ESP_LOGI(TAG, "Attitude (deg): Pitch=%.1f Roll=%.1f", pitch_deg, roll_deg);
    } 
}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void estimator_init() {
    // Initialize complementary filter 
    CompInit(&comp_filter, DELTA_T, TAU);
    CompAccelUpdate(&comp_filter, 0.0, 0.0, 9.81);          // Assume drone is level when starting
    CompStart(&comp_filter); 

    // Start filter update task
    xTaskCreate(vUpdateEstimatorTask, "Acc Data Processing", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}