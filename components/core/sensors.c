/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "i2c_setup.h"
#include "spi_setup.h"
#include "bmi088.h"
#include "pmw3901.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"

static const char *TAG = "sensors";

/* ------------------------------------------- Private Global Variables  ------------------------------------------- */
static QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_tof_data, xQueue_opt_flow_data; 
static SemaphoreHandle_t xSemaphore_i2c, xSemaphore_spi; 

/* ------------------------------------------- Private function definitions  ------------------------------------------- */
static void vPollI2CSensorsTask(void *pvParameters) {    
    uint8_t cyles_since_last_tof = 0; 
    for (;;) {        
        xSemaphoreTake(xSemaphore_i2c, portMAX_DELAY); 
        
        gpio_set_level(CONFIG_PIN_TOGGLE_A, 1);

        acc_data_t acc_data = get_acc_data();
        if (!xQueueSendToBack(xQueue_acc_data, (void *) &acc_data, portMAX_DELAY))
            ESP_LOGE(TAG, "Accel data queue is full");  
        // ESP_LOGI(TAG, "Accel (m/s2): x=%.2f y=%.2f z=%.2f", acc_data.ax_m_s2, acc_data.ay_m_s2, acc_data.az_m_s2); 

        gyro_data_t gyro_data = get_gyro_data();
        if (!xQueueSendToBack(xQueue_gyro_data, (void *) &gyro_data, portMAX_DELAY))
            ESP_LOGE(TAG, "Gyro data queue is full"); 
        // ESP_LOGI(TAG, "Attitude Rate (rad/s): x=%.2f y=%.2f z=%.2f", gyro_data.Gx_rad_s, gyro_data.Gy_rad_s, gyro_data.Gz_rad_s); 

        uint8_t range_status; 
        uint16_t height_mm; 
        cyles_since_last_tof++; 
        if (cyles_since_last_tof >= TOF_SENS_PERIOD_MS/SENS_PERIOD_MS) {
            VL53L1X_GetRangeStatus(0, &range_status);
            if (range_status == 0) {
                VL53L1X_GetDistance(0, &height_mm); 
            } else {
                ESP_LOGE(TAG, "Range Status: %d", range_status);
            }
            cyles_since_last_tof = 0; 
            // ESP_LOGI(TAG, "Drone height (mm): %d", height_mm); 
            if (!xQueueSendToBack(xQueue_tof_data, (void *) &height_mm, portMAX_DELAY))
                ESP_LOGE(TAG, "ToF data queue is full"); 
        }
        
        gpio_set_level(CONFIG_PIN_TOGGLE_A, 0);
    }
}

static void vPollSPISensorsTask(void *pvParameters) {
    for (;;) {        
        xSemaphoreTake(xSemaphore_spi, portMAX_DELAY);

        gpio_set_level(CONFIG_PIN_TOGGLE_B, 1);
        
        motionBurst_t motion;
        pmw3901ReadMotion(OPT_FLOW_CS_PIN, &motion);
        if (!xQueueSendToBack(xQueue_opt_flow_data, (void *) &motion, portMAX_DELAY)) 
            ESP_LOGE(TAG, "Optical flow data queue is full");
        // ESP_LOGI(TAG, "Opt flow (px): dx=%d, dy=%d", optf_data[0], optf_data[1]);
        
        gpio_set_level(CONFIG_PIN_TOGGLE_B, 0); 
    }
}

static void vPollSensorsTask(void *pvParameters) {
    ESP_LOGI(TAG, "Beginning sensor polling"); 

    xSemaphore_i2c = xSemaphoreCreateBinary(); 
    xSemaphore_spi = xSemaphoreCreateBinary(); 

    xTaskCreate(vPollI2CSensorsTask, "Poll I2C", 4096, NULL, 8, NULL);
    xTaskCreate(vPollSPISensorsTask, "Poll SPI", 4096, NULL, 8, NULL);
    
    BaseType_t xWasDelayed;
    const TickType_t xTimeIncrement = SENS_PERIOD_MS/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Can't poll sensors this fast");

        if(!xSemaphoreGive(xSemaphore_i2c))
            ESP_LOGE(TAG, "Can't give to i2c semaphore"); 
        if(!xSemaphoreGive(xSemaphore_spi))
            ESP_LOGE(TAG, "Can't give to spi semaphore"); 
    }
}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void sensors_init(
    i2c_master_bus_handle_t *bus_handle, 
    QueueHandle_t *pxQueue_acc_data, 
    QueueHandle_t *pxQueue_gyro_data, 
    QueueHandle_t *pxQueue_tof_data, 
    QueueHandle_t *pxQueue_opt_flow_data) 
{
    // Store queues
    xQueue_acc_data = *pxQueue_acc_data; 
    xQueue_gyro_data = *pxQueue_gyro_data;
    xQueue_tof_data = *pxQueue_tof_data; 
    xQueue_opt_flow_data = *pxQueue_opt_flow_data; 

    // Initialize sensors 
    ESP_ERROR_CHECK(IMU_acc_init(bus_handle)); 
    ESP_LOGI(TAG, "Initialized IMU successfully");
    
    ESP_ERROR_CHECK(IMU_gyro_init(bus_handle)); 
    ESP_LOGI(TAG, "Initialized gyroscope successfully"); 

    ESP_ERROR_CHECK(tof_init(bus_handle)); 
    ESP_LOGI(TAG, "Initialized ToF successfully"); 

    pmw3901Init(VSPI_HOST, OPT_FLOW_CS_PIN);
    ESP_LOGI(TAG, "Initialized Optical Flow successfully");

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(imu_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(gyro_handle));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void start_control_loop(void) {
    // Start Tasks
    xTaskCreate(vPollSensorsTask, "Sens loop", 4096, NULL, 10, NULL);
}