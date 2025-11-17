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
    uint16_t raw_height_mm; 
    int16_t opt_flow_data_px[2]; 
    float height_mm = 0, last_height_mm = 0; 
    float altitude_rate_m_s = 0; 
    float vel_x_m_s, vel_y_m_s; 
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

        if (xQueueReceive(xQueue_ToF_data, (void *) &raw_height_mm, 0)) {       
            // ToF samples at every 50ms (rest of loop runs every 2ms)
            height_mm = ((float) raw_height_mm)*cos(pitch_rad)*cos(roll_rad); 
            altitude_rate_m_s = (height_mm - last_height_mm)/((float) TOF_SENS_PERIOD_MS);
            last_height_mm = height_mm; 
        }  else {
            // Update height and velo in between tof measurements 
            height_mm = height_mm + altitude_rate_m_s*DELTA_T*1000.0; 
            altitude_rate_m_s += (acc_data.az_m_s2 - ave_g_m_s2)*DELTA_T;
        }

        xQueueReceive(xQueue_optf_data, (void *) &opt_flow_data_px, portMAX_DELAY); 
        vel_x_m_s = raw_height_mm*.001*(42.0*0.0174533)*opt_flow_data_px[0]/(SENS_PERIOD_MS*.001*35) - raw_height_mm*.001*gyro_data.Gy_rad_s;
        vel_y_m_s = raw_height_mm*.001*(42.0*0.0174533)*opt_flow_data_px[1]/(SENS_PERIOD_MS*.001*35) - raw_height_mm*.001*gyro_data.Gy_rad_s;
        ESP_LOGI(TAG, "Velocity data (m/s): %.2f", vel_x_m_s); 

        state_data_t state_data = {
            .pitch_rad = pitch_rad, 
            .pitch_rate_rad_s = gyro_data.Gx_rad_s,
            .roll_rad = roll_rad, 
            .roll_rate_rad_s = gyro_data.Gy_rad_s, 
            .yaw_rate_rad_s = gyro_data.Gz_rad_s,
            .altitude_m = height_mm*.001, 
            .altitude_rate_m_s = altitude_rate_m_s,
            .vel_x_m_s = vel_x_m_s, 
            .vel_y_m_s = vel_y_m_s, 
        };
        if (!xQueueSendToBack(xQueue_state_data, (void *) &state_data, portMAX_DELAY))
            ESP_LOGE(TAG, "State data queue is full"); 

        // float pitch_deg = CompRadiansToDegrees(pitch_rad); 
        // float roll_deg = CompRadiansToDegrees(roll_rad);
        // ESP_LOGI(TAG, "Attitude (deg): Pitch=%.1f Roll=%.1f", pitch_deg, roll_deg);
        // ESP_LOGI(TAG, "Altitude Rate (m/z): %.2f", altitude_rate_m_s);
    } 
}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void estimator_init() {
    // Initialize complementary filter 
    CompInit(&comp_filter, DELTA_T, TAU);
    CompAccelUpdate(&comp_filter, 0.0, 0.0, 9.81);          // Assume drone is level when starting
    CompStart(&comp_filter); 
    ESP_LOGI(TAG, "Initialized complementary filter successfully"); 

    // Start filter update task
    xTaskCreate(vUpdateEstimatorTask, "Comp filter", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}