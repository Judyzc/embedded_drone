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
#include "driver/i2c_master.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"

static const char *TAG = "example";

/* ------------------------------------------- Global Variables  ------------------------------------------- */
QueueHandle_t xQueue_raw_acc_data, xQueue_raw_gyro_data; 
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t acc_handle, gyro_handle;
static float acc_x_offset = 0, acc_y_offset = 0, gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0; 

/* ------------------------------------------- Private function definitions  ------------------------------------------- */
/**
 * @brief Read a sequence of bytes from sensor registers
 */
static esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a sensor register
 */
static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t imu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ACC_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &imu_config, &acc_handle));

    i2c_device_config_t gyro_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_GYRO_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &gyro_config, &gyro_handle));
}

static esp_err_t IMU_acc_init() 
{
    vTaskDelay(1 / portTICK_PERIOD_MS);         // IMU requires 1ms pause after power-on
    // Turn on Accelerometer  
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_PWR_CTRL, 0x04)); 
    // Set data rate and filter to 1600 Hz
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_CONF, 0xAC)); 
    // Set range to 6G
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_RANGE, 0x01)); 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
    return ESP_OK;
}

static esp_err_t IMU_gyro_init() 
{
    vTaskDelay(1 / portTICK_PERIOD_MS);         // IMU requires 1ms pause after power-on
    // Turn on gyro  
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_LPM1, 0x00)); 
    // Set data rate and filter to 1000 Hz and 116 Hz
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_BANDWIDTH, 0x02)); 
    // Set range to 1000 deg/s
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_RANGE, 0x01)); 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
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
    for (;;) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Can't get I2C data this fast");
        start_tick = xLastWakeTime; 
 
        uint8_t raw_data[3*2*sizeof(uint8_t)]; 
        ESP_ERROR_CHECK(register_read(acc_handle, ACC_DATA_START, raw_data, sizeof(raw_data))); 
        // ESP_LOGI(TAG, "Accel raw data: x=%x, y=%x, z=%x", (raw_data[0] | (raw_data[1]<<8)), (raw_data[2] | (raw_data[3]<<8)), (raw_data[4] | (raw_data[5]<<8)));
        if (!xQueueSendToBack(xQueue_raw_acc_data, (void *) raw_data, portMAX_DELAY))
            ESP_LOGE(TAG, "Raw accel data queue is full");  

        ESP_ERROR_CHECK(register_read(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data))); 
        // ESP_LOGI(TAG, "Attitude Rate (rad/s): x=%.2f y=%.2f z=%.2f", gyro_data.Gx_rad_s, gyro_data.Gy_rad_s, gyro_data.Gz_rad_s); 
        if (!xQueueSendToBack(xQueue_raw_gyro_data, (void *) raw_data, portMAX_DELAY))
            ESP_LOGE(TAG, "RAW gyro data queue is full"); 
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
void sensors_init(void) {
    // Start I2C
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize IMU
    ESP_ERROR_CHECK(IMU_acc_init()); 
    ESP_LOGI(TAG, "Initialized accelerometer successfully");
    ESP_ERROR_CHECK(IMU_gyro_init()); 
    ESP_LOGI(TAG, "Initialized gyroscope  successfully"); 

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(imu_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(gyro_handle));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}

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

void start_control_loop(void) {
    // Initialize internal queues
    xQueue_raw_acc_data = xQueueCreate(1, 6*sizeof(uint8_t)); 
    xQueue_raw_gyro_data = xQueueCreate(1, 6*sizeof(uint8_t));
    
    // Start Tasks
    xTaskCreate(vProcessAccDataTask, "Acc Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vProcessGyroDataTask, "Gyro Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    xTaskCreate(vGetRawDataTask, "Get Raw Data", 4096, NULL, GET_RAW_DATA_PRIORITY, NULL);
}