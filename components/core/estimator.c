#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "math.h"

#include "bmi088.h"
#include "pmw3901.h"
#include "vl53l1_platform.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"

static const char *TAG = "estimator"; 

/* ------------------------------------------- Private Global Variables  ------------------------------------------- */
static SixAxis comp_filter;
static QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_tof_data, xQueue_opt_flow_data, xQueue_state_data;

/* ------------------------------------------- Private Function Definitions  ------------------------------------------- */
float opt_flow_calc(int16_t dx_px, uint16_t height_mm, float attitude_rate_rad_s) { 
    float height_m = ((float) height_mm)*.001f; 
    float dt = ((float) SENS_PERIOD_MS)*.001f;
    float scalar = 100.0; 
    return height_m*OPT_FLOW_FOV_RAD*((float) dx_px)/(dt*((float) OPT_FLOW_PX_LENGTH)*scalar) - height_m*attitude_rate_rad_s; 
}

void vUpdateEstimatorTask(void *pvParameters) { 
    acc_data_t acc_data; 
    gyro_data_t gyro_data; 
    uint16_t raw_height_mm; 
    motionBurst_t motion;  
    float height_mm = 0, last_height_mm = 0; 
    float raw_altitude_rate_m_s = 0; 
    float filtered_altitude_rate_m_s = 0; 
    float alpha = DELTA_T/.04;
    float vel_x_m_s = 0, vel_y_m_s = 0; 
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

        if (xQueueReceive(xQueue_tof_data, (void *) &raw_height_mm, 0)) {       
            // ToF samples at every 50ms (rest of loop runs every 2ms)
            last_height_mm = height_mm; 
            // Convert body frame to inertial frame
            height_mm = ((float) raw_height_mm)*cos(pitch_rad)*cos(roll_rad); 
            raw_altitude_rate_m_s = (height_mm - last_height_mm)/((float) TOF_SENS_PERIOD_MS);
        }  else {
            // Update height and velo in between tof measurements
            // Convert accel meas from imu frame to body frame
            float accel_xb_m_s2 = acc_data.ay_m_s2*-1.0f; 
            float accel_yb_m_s2 = acc_data.ax_m_s2*-1.0f; 
            float accel_zb_m_s2 = acc_data.az_m_s2; 
            
            // Convert accel meas from body frame to inertial frame
            float accel_zi_m_s2 = -1.0*accel_xb_m_s2*sin(pitch_rad) \
                                  + accel_yb_m_s2*cos(pitch_rad)*sin(roll_rad) \
                                  + accel_zb_m_s2*cos(pitch_rad)*cos(roll_rad);

            // Subtract gravity vector
            accel_zi_m_s2 -= ave_g_m_s2; 

            // Numerically integrate
            last_height_mm = height_mm;
            height_mm = last_height_mm + filtered_altitude_rate_m_s*DELTA_T*1000.0; 
            raw_altitude_rate_m_s = filtered_altitude_rate_m_s - accel_zi_m_s2*DELTA_T;

            // "Spoof" raw height measurement for optical flow data processing
            raw_height_mm = (uint16_t) (height_mm/cos(pitch_rad)/cos(roll_rad));
        }
        // Low pass filter the altitude rate
        filtered_altitude_rate_m_s = filtered_altitude_rate_m_s - (alpha*(filtered_altitude_rate_m_s - raw_altitude_rate_m_s));

        if (xQueueReceive(xQueue_opt_flow_data, (void *) &motion, 0)) {
            vel_x_m_s = opt_flow_calc(motion.deltaX, raw_height_mm, -1.0f*gyro_data.Gy_rad_s); 
            vel_y_m_s = opt_flow_calc(-1*motion.deltaY, raw_height_mm, gyro_data.Gx_rad_s);
        }

        state_data_t state_data = {
            .pitch_rad = pitch_rad, 
            .pitch_rate_rad_s = gyro_data.Gx_rad_s,
            .roll_rad = roll_rad, 
            .roll_rate_rad_s = gyro_data.Gy_rad_s, 
            .yaw_rate_rad_s = gyro_data.Gz_rad_s,
            .altitude_m = height_mm*.001, 
            .altitude_rate_m_s = filtered_altitude_rate_m_s,
            .vel_x_m_s = vel_x_m_s, 
            .vel_y_m_s = vel_y_m_s, 
        };
        if (!xQueueSendToBack(xQueue_state_data, (void *) &state_data, portMAX_DELAY))
            ESP_LOGE(TAG, "State data queue is full"); 

        // float pitch_deg = CompRadiansToDegrees(pitch_rad); 
        // float roll_deg = CompRadiansToDegrees(roll_rad);
        // ESP_LOGI(TAG, "Attitude (deg): Pitch=%.1f Roll=%.1f", pitch_deg, roll_deg);
        // ESP_LOGI(TAG, "Altitude (m): %.3f", height_mm*.001);
        // ESP_LOGI(TAG, "Raw Altitude (m): %.3f", raw_height_mm*.001);
        // ESP_LOGI(TAG, "Altitude Rate (m/s): %.3f", filtered_altitude_rate_m_s);
        // ESP_LOGI(TAG, "Velocity data (m/s): x=%.2f, y=%.2f", vel_x_m_s, vel_y_m_s); 
    } 
}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void estimator_init(
    QueueHandle_t *pxQueue_acc_data, 
    QueueHandle_t *pxQueue_gyro_data,
    QueueHandle_t *pxQueue_tof_data,
    QueueHandle_t *pxQueue_opt_flow_data,
    QueueHandle_t *pxQueue_state_data) 
{
    // Save queues
    xQueue_acc_data = *pxQueue_acc_data; 
    xQueue_gyro_data = *pxQueue_gyro_data; 
    xQueue_tof_data = *pxQueue_tof_data;
    xQueue_opt_flow_data = *pxQueue_opt_flow_data; 
    xQueue_state_data = *pxQueue_state_data;

    // Initialize complementary filter 
    CompInit(&comp_filter, DELTA_T, TAU);
    CompAccelUpdate(&comp_filter, 0.0, 0.0, 9.81);          // Assume drone is level when starting
    CompStart(&comp_filter); 
    ESP_LOGI(TAG, "Initialized complementary filter successfully"); 

    // Start filter update task
    xTaskCreate(vUpdateEstimatorTask, "Comp filter", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}