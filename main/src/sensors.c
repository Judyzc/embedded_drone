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

// from components/
#include "VL53L1X_api.h"
#include "pmw3901.h"

// now in leftover/ 
// #include "six_axis_comp_filter.h"
// #include "vl53l1_platform.h"
// #include "VL53L1X_api.h"
// #include "VL53L1X_calibration.h"

static const char *TAG = "sensors";


/* ------------------------------------------- Initialize Public Global Variables  ------------------------------------------- */

/* ------------------------------------------- Private Global Variables  ------------------------------------------- */
// i2c
QueueHandle_t xQueue_raw_acc_data, xQueue_raw_gyro_data, xQueue_raw_tof_data, xQueue_raw_optf_data; 
i2c_master_dev_handle_t acc_handle, gyro_handle, tof_handle;
static float acc_x_offset = 0, acc_y_offset = 0, gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0; 

// spi
static pmw3901_t g_pmw = {0};

/* ------------------------------------------- Private function definitions  ------------------------------------------- */
static esp_err_t DECK_tof_init() 
{
    i2c_device_config_t tof_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DECK_TOF_SENSOR_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &tof_config, &tof_handle));
    if (VL53L1X_SensorInit(DECK_TOF_SENSOR_ADDRESS) == VL53L1_ERROR_NONE) {
        ESP_LOGI(TAG, "VL53L1X_SensorInit OK at 0x%02X", DECK_TOF_SENSOR_ADDRESS);
        if (VL53L1X_StartRanging(DECK_TOF_SENSOR_ADDRESS) == VL53L1_ERROR_NONE) {
            ESP_LOGI(TAG, "VL53L1X_StartRanging OK at 0x%02X", DECK_TOF_SENSOR_ADDRESS);
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "StartRanging failed at 0x%02X", DECK_TOF_SENSOR_ADDRESS);
        }
    } else {
        ESP_LOGW(TAG, "SensorInit failed at 0x%02X", DECK_TOF_SENSOR_ADDRESS);
    }
    ESP_LOGE(TAG, "VL53 init failed.");
    return ESP_FAIL;
}

static bool DECK_optf_init() 
{
    bool optf_ok = pmw3901_init(&g_pmw, VSPI_HOST, 
                                ESP_SCLK_IO, ESP_MOSI_IO, ESP_MISO_IO, ESP_CS_IO);
    return optf_ok;
}

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
    // const TickType_t xTimeIncrement = SENS_PERIOD_MS/portTICK_PERIOD_MS;
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(SENS_PERIOD_MS);
    BaseType_t xWasDelayed;
    for (;;) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Can't get I2C data this fast");
        start_tick = xLastWakeTime; 
        
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

        VL53L1X_Result_t result;  // for tof
        uint16_t distance_mm;  // for tof
        if (VL53L1X_GetResult((uint16_t) DECK_TOF_SENSOR_ADDRESS, &result) == VL53L1_ERROR_NONE) {
            distance_mm = (uint16_t) result.Distance;
            ESP_LOGI(TAG, "distance_mm=%d", distance_mm);
            if (!xQueueSendToBack(xQueue_raw_tof_data, &distance_mm, portMAX_DELAY)) {
                ESP_LOGE(TAG, "Failed to send TOF reading to queue");
            }
        }

        // int16_t dx = 0, dy = 0;  // optical flow
        int16_t optf_data[2*sizeof(int16_t)]; // optical flow
        if (pmw3901_read_motion_count(&g_pmw, &optf_data[0], &optf_data[1])) {
            if (!xQueueSendToBack(xQueue_raw_optf_data, (void *) optf_data, portMAX_DELAY)) {
                ESP_LOGE(TAG, "Failed to send optical flow to queue");
            }
        }

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

void vProcessTofDataTask(void *pvParameters) {
    // doesn't actually do any processing lol but keep form and priority 
    uint16_t distance;
    ESP_LOGI(TAG, "vProcessTofDataTask started");
    for (;;) {
        xQueueReceive(xQueue_raw_tof_data, &distance, portMAX_DELAY);
        ESP_LOGI(TAG, "inside process, distance=%d", distance);
        tof_data_t tof_data; 
        tof_data.distance = (float) distance;
        if (!xQueueSendToBack(xQueue_tof_data, (void *) &tof_data, portMAX_DELAY)) {
            ESP_LOGE(TAG, "Failed to send TOF reading to queue");
        }
    }
}

void vProccessOptfDataTask(void *pvParameters) {
    // doesn't actually do any processing either 
    int16_t raw_data[2];
    ESP_LOGI(TAG, "vProccessOptfDataTask started");
    for (;;) {
        xQueueReceive(xQueue_raw_tof_data, (void *) raw_data, portMAX_DELAY);
        ESP_LOGI(TAG, "motion dx=%d dy=%d", raw_data[0], raw_data[1]);
        optf_data_t optf_data;
        optf_data.motion_dx = (float) raw_data[0];
        optf_data.motion_dy = (float) raw_data[1];
        if (!xQueueSendToBack(xQueue_optf_data, (void *) &optf_data, portMAX_DELAY)) {
            ESP_LOGE(TAG, "Failed to send Optical Flow reading to queue");
        }
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

// called in main.c
void sensors_init(void) {
    // Initialize sensors 
    ESP_ERROR_CHECK(IMU_acc_init()); 
    ESP_LOGI(TAG, "Initialized IMU successfully");
    ESP_ERROR_CHECK(IMU_gyro_init()); 
    ESP_LOGI(TAG, "Initialized gyroscope successfully"); 
    ESP_ERROR_CHECK(DECK_tof_init()); 
    ESP_LOGI(TAG, "Initialized ToF successfully"); 
    bool optf_ok = DECK_optf_init(); 
    if (optf_ok) {
        ESP_LOGI(TAG, "Initialized Optical Flow successfully"); 
    } else {
        ESP_LOGI(TAG, "Error with init optical flow"); 
    }

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(imu_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(gyro_handle));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}

// called in main.c
void calibrate_sensors(void) {
    ESP_LOGI(TAG, "Beginning Sensor calibration"); 
    float acc_x_running_total = 0.0; 
    float acc_y_running_total = 0.0; 
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

        ESP_ERROR_CHECK(register_read(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data))); 
        gyro_data_t gyro_data = process_gyro_data(raw_data); 
        gyro_x_running_total += gyro_data.Gx_rad_s;
        gyro_y_running_total += gyro_data.Gy_rad_s;
        gyro_z_running_total += gyro_data.Gz_rad_s;
    }

    acc_x_offset = -1.0*acc_x_running_total/((float) CALIBRATION_SAMPLES); 
    acc_y_offset = -1.0*acc_y_running_total/((float) CALIBRATION_SAMPLES); 
    gyro_x_offset = -1.0*gyro_x_running_total/((float) CALIBRATION_SAMPLES); 
    gyro_y_offset = -1.0*gyro_y_running_total/((float) CALIBRATION_SAMPLES); 
    gyro_z_offset = -1.0*gyro_z_running_total/((float) CALIBRATION_SAMPLES); 
}

// called in main.c
void start_control_loop(void) {
    // Initialize internal queues
    xQueue_raw_acc_data = xQueueCreate(1, 6*sizeof(uint8_t)); 
    xQueue_raw_gyro_data = xQueueCreate(1, 6*sizeof(uint8_t));
    xQueue_raw_tof_data = xQueueCreate(1, sizeof(uint16_t));
    xQueue_raw_optf_data = xQueueCreate(1, 2*sizeof(int16_t));

    // Start Tasks
    xTaskCreate(vProcessAccDataTask, "Acc Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vProcessGyroDataTask, "Gyro Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vProcessTofDataTask, "TOF Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vProccessOptfDataTask, "Optical Flow Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vGetRawDataTask, "Get Raw Data", 4096, NULL, GET_RAW_DATA_PRIORITY, NULL);
}