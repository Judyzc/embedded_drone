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

#include "i2c_setup.h"
#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "pmw3901.h"

static const char *TAG = "sensors";

/* ------------------------------------------- Initialize Public Global Variables  ------------------------------------------- */
i2c_master_dev_handle_t tof_handle;
float ave_g_m_s2; 

// static pmw3901_t g_pmw = {0};

/* ------------------------------------------- Private Global Variables  ------------------------------------------- */
QueueHandle_t xQueue_raw_acc_data, xQueue_raw_gyro_data; 
i2c_master_dev_handle_t acc_handle, gyro_handle;
static float acc_x_offset = 0, acc_y_offset = 0, gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0; 

/* ------------------------------------------- Private function definitions  ------------------------------------------- */
static esp_err_t ToF_init() 
{
    i2c_device_config_t tof_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOF_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &tof_config, &tof_handle));

    VL53L1X_ERROR Status;
    uint8_t state = 0;  
    while(!state){
        Status = VL53L1X_BootState(0, &state);
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Finished booting");
    Status = VL53L1X_SensorInit(0);
    Status = VL53L1X_SetInterMeasurementInMs(0, TOF_SENS_PERIOD_MS);
    Status = VL53L1X_SetTimingBudgetInMs(0, 33);
    Status = VL53L1X_StartRanging(0);

    return (Status == 0) ? ESP_OK : ESP_FAIL;
}

// static bool DECK_optf_init() 
// {
//     bool optf_ok = pmw3901_init(&g_pmw, VSPI_HOST, 
//                                 ESP_SCLK_IO, ESP_MOSI_IO, ESP_MISO_IO, ESP_CS_IO);
//     return optf_ok;
// }


static esp_err_t IMU_acc_init() 
{
    i2c_device_config_t acc_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ACC_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &acc_config, &acc_handle));

    // Turn on Accelerometer  
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_PWR_CTRL, 0x04)); 
    // Set data rate and filter to 1600 Hz
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_CONF, 0xAC)); 
    // Set range to 6G
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_RANGE, 0x01)); 

    return ESP_OK;
}

static esp_err_t IMU_gyro_init() 
{
    i2c_device_config_t gyro_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_GYRO_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &gyro_config, &gyro_handle));

    // Turn on gyro  
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_LPM1, 0x00)); 
    // Set data rate and filter to 1000 Hz and 116 Hz
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_BANDWIDTH, 0x02)); 
    // Set range to 1000 deg/s
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_RANGE, 0x01)); 

    return ESP_OK;
}

static acc_data_t process_acc_data(uint8_t *raw_data) {
    int x_val = (int16_t) (raw_data[0] | (raw_data[1]<<8)); 
    int y_val = (int16_t) (raw_data[2] | (raw_data[3]<<8)); 
    int z_val = (int16_t) (raw_data[4] | (raw_data[5]<<8)); 
    
    acc_data_t acc_data; 
    acc_data.ax_m_s2 = ((float) x_val)*6.0/32768.0*9.81+acc_x_offset; 
    acc_data.ay_m_s2 = ((float) y_val)*6.0/32768.0*9.81+acc_y_offset; 
    acc_data.az_m_s2 = ((float) z_val)*6.0/32768.0*9.81; 
    return acc_data; 
}

static gyro_data_t process_gyro_data(uint8_t *raw_data) {
    int x_val = (int16_t) (raw_data[0] | (raw_data[1]<<8)); 
    int y_val = (int16_t) (raw_data[2] | (raw_data[3]<<8)); 
    int z_val = (int16_t) (raw_data[4] | (raw_data[5]<<8)); 
    
    gyro_data_t gyro_data; 
    gyro_data.Gx_rad_s = CompDegreesToRadians(((float) x_val)*1000.0/32767.0)+gyro_x_offset;
    gyro_data.Gy_rad_s = CompDegreesToRadians(((float) y_val)*1000.0/32767.0)+gyro_y_offset; 
    gyro_data.Gz_rad_s = CompDegreesToRadians(((float) z_val)*1000.0/32767.0)+gyro_z_offset;  
    return gyro_data; 
}

/* ------------------------------------------- Free RTOS Tasks  ------------------------------------------- */
void vGetRawDataTask(void *pvParameters) {
    ESP_LOGI(TAG, "Beginning control loop"); 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = SENS_PERIOD_MS/portTICK_PERIOD_MS;
    BaseType_t xWasDelayed;
    uint8_t cyles_since_last_tof = 0; 
    for (;;) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Can't get I2C data this fast");
        
        gpio_set_level(PIN_TOGGLE_A, 1);
        uint8_t raw_data[3*2*sizeof(uint8_t)]; 
        ESP_ERROR_CHECK(register_read(acc_handle, ACC_DATA_START, raw_data, sizeof(raw_data))); 
        // ESP_LOGI(TAG, "Accel raw data: x=%x, y=%x, z=%x", (raw_data[0] | (raw_data[1]<<8)), (raw_data[2] | (raw_data[3]<<8)), (raw_data[4] | (raw_data[5]<<8)));
        if (!xQueueSendToBack(xQueue_raw_acc_data, (void *) raw_data, portMAX_DELAY))
            ESP_LOGE(TAG, "Raw accel data queue is full");  

        ESP_ERROR_CHECK(register_read(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data))); 
        // ESP_LOGI(TAG, "Attitude Rate (rad/s): x=%.2f y=%.2f z=%.2f", gyro_data.Gx_rad_s, gyro_data.Gy_rad_s, gyro_data.Gz_rad_s); 
        if (!xQueueSendToBack(xQueue_raw_gyro_data, (void *) raw_data, portMAX_DELAY))
            ESP_LOGE(TAG, "RAW gyro data queue is full"); 

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
            if (!xQueueSendToBack(xQueue_ToF_data, (void *) &height_mm, portMAX_DELAY))
                ESP_LOGE(TAG, "ToF data queue is full"); 
        }

        // uint16_t optf_data[2*sizeof(uint16_t)]; // optical flow
        // if (pmw3901_read_motion_count(&g_pmw, &optf_data[0], &optf_data[1])) {
        //     if (!xQueueSendToBack(xQueue_optf_data, (void *) optf_data, portMAX_DELAY)) {
        //         ESP_LOGE(TAG, "Failed to send optical flow to queue. Full?");
        //     }
        // }

        gpio_set_level(PIN_TOGGLE_A, 0);
    }
}

void vProcessAccDataTask(void *pvParameters) {
    uint8_t raw_data[6];
    for (;;) {
        xQueueReceive(xQueue_raw_acc_data, (void *) raw_data, portMAX_DELAY); 
        acc_data_t acc_data = process_acc_data(raw_data);
        // ESP_LOGI(TAG, "Acceleration (m/s2): x=%.2f y=%.2f z=%.2f", acc_data.ax_m_s2, acc_data.ay_m_s2, acc_data.az_m_s2);
        if (!xQueueSendToBack(xQueue_acc_data, (void *) &acc_data, portMAX_DELAY))
            ESP_LOGE(TAG, "Acc data queue is full"); 

    }
}

void vProcessGyroDataTask(void *pvParameters) {
    uint8_t raw_data[6];
    for (;;) {
        xQueueReceive(xQueue_raw_gyro_data, (void *) raw_data, portMAX_DELAY); 
        gyro_data_t gyro_data = process_gyro_data(raw_data);
        // ESP_LOGI(TAG, "Acceleration (m/s2): x=%.2f y=%.2f z=%.2f", gyro_data.Gx_rad_s, gyro_data.Gy_rad_s, gyro_data.Gz_rad_s);
        if (!xQueueSendToBack(xQueue_gyro_data, (void *) &gyro_data, portMAX_DELAY))
            ESP_LOGE(TAG, "Gyro data queue is full");  
    }
}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void init_osc_pin(int pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(pin, 0);
}

void sensors_init(void) {
    // Initialize sensors 
    ESP_ERROR_CHECK(IMU_acc_init()); 
    ESP_LOGI(TAG, "Initialized IMU successfully");
    ESP_ERROR_CHECK(IMU_gyro_init()); 
    ESP_LOGI(TAG, "Initialized gyroscope successfully"); 
    ESP_ERROR_CHECK(ToF_init()); 
    ESP_LOGI(TAG, "Initialized ToF successfully"); 
    // bool optf_ok = DECK_optf_init(); 
    // if (optf_ok) {
    //     ESP_LOGI(TAG, "Initialized Optical Flow successfully"); 
    // } else {
    //     ESP_LOGI(TAG, "Error with init optical flow"); 
    // }

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(imu_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(gyro_handle));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void calibrate_sensors(void) {
    ESP_LOGI(TAG, "Beginning Sensor calibration"); 
    float acc_x_running_total = 0.0; 
    float acc_y_running_total = 0.0; 
    float acc_z_running_total = 0.0; 
    float gyro_x_running_total = 0.0; 
    float gyro_y_running_total = 0.0; 
    float gyro_z_running_total = 0.0; 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = CALIBRATION_PERIOD_MS/portTICK_PERIOD_MS;
    BaseType_t xWasDelayed;
    for (int i=0; i<CALIBRATION_SAMPLES; i++) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Can't get I2C data this fast");

        uint8_t raw_data[3*2*sizeof(uint8_t)]; 
        ESP_ERROR_CHECK(register_read(acc_handle, ACC_DATA_START, raw_data, sizeof(raw_data))); 
        acc_data_t acc_data = process_acc_data(raw_data); 
        acc_x_running_total += acc_data.ax_m_s2; 
        acc_y_running_total += acc_data.ay_m_s2;
        acc_z_running_total += acc_data.az_m_s2;

        ESP_ERROR_CHECK(register_read(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data))); 
        gyro_data_t gyro_data = process_gyro_data(raw_data); 
        gyro_x_running_total += gyro_data.Gx_rad_s;
        gyro_y_running_total += gyro_data.Gy_rad_s;
        gyro_z_running_total += gyro_data.Gz_rad_s;
    }

    acc_x_offset = -1.0*acc_x_running_total/((float) CALIBRATION_SAMPLES); 
    acc_y_offset = -1.0*acc_y_running_total/((float) CALIBRATION_SAMPLES); 
    ave_g_m_s2 = acc_z_running_total/((float) CALIBRATION_SAMPLES); 
    gyro_x_offset = -1.0*gyro_x_running_total/((float) CALIBRATION_SAMPLES); 
    gyro_y_offset = -1.0*gyro_y_running_total/((float) CALIBRATION_SAMPLES); 
    gyro_z_offset = -1.0*gyro_z_running_total/((float) CALIBRATION_SAMPLES); 
}

void start_control_loop(void) {
    // Initialize internal queues
    xQueue_raw_acc_data = xQueueCreate(1, 6*sizeof(uint8_t)); 
    xQueue_raw_gyro_data = xQueueCreate(1, 6*sizeof(uint8_t));
    
    // Start Tasks
    xTaskCreate(vProcessAccDataTask, "Acc Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vProcessGyroDataTask, "Gyro Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vGetRawDataTask, "Get Raw Data", 4096, NULL, GET_RAW_DATA_PRIORITY, NULL);
}