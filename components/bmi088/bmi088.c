#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "math.h"
#include "driver/i2c_master.h"

#include "bmi088.h"
#include "i2c_setup.h"

static const char *TAG = "bmi088";

/* ------------------------------------------- Public Global Variables ------------------------------------------- */
float ave_g_m_s2 = 0;

/* ------------------------------------------- Private Global Variables ------------------------------------------- */
static float acc_x_offset = 0, acc_y_offset = 0, gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
i2c_master_dev_handle_t acc_handle, gyro_handle; 

/* ------------------------------------------- Private Macros ------------------------------------------- */
#define DEG_TO_RAD(deg) deg*M_PI/180.0f

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
esp_err_t IMU_acc_init(i2c_master_bus_handle_t *bus_handle) 
{
    i2c_device_config_t acc_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ACC_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &acc_config, &acc_handle));

    // Turn on Accelerometer  
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_PWR_CTRL, 0x04)); 
    // Set data rate and filter to 1600 Hz
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_CONF, 0xAC)); 
    // Set range to 6G
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_RANGE, 0x01)); 

    return ESP_OK;
}

esp_err_t IMU_gyro_init(i2c_master_bus_handle_t *bus_handle) 
{
    i2c_device_config_t gyro_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_GYRO_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &gyro_config, &gyro_handle));

    // Turn on gyro  
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_LPM1, 0x00)); 
    // Set data rate and filter to 1000 Hz and 116 Hz
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_BANDWIDTH, 0x02)); 
    // Set range to 1000 deg/s
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_RANGE, 0x01)); 

    return ESP_OK;
}

acc_data_t get_acc_data(void) {
    uint8_t raw_data[6]; 
    ESP_ERROR_CHECK(register_read(acc_handle, ACC_DATA_START, raw_data, sizeof(raw_data))); 
    int x_val = (int16_t) ((raw_data[1]<<8) | raw_data[0]); 
    int y_val = (int16_t) ((raw_data[3]<<8) | raw_data[2]); 
    int z_val = (int16_t) ((raw_data[5]<<8) | raw_data[4]); 
    // ESP_LOGI(TAG, "Accel raw data: x=%x, y=%x, z=%x", x_val, y_val, z_val);
    
    acc_data_t acc_data; 
    acc_data.ax_m_s2 = ((float) x_val)*6.0/32768.0*9.81+acc_x_offset; 
    acc_data.ay_m_s2 = ((float) y_val)*6.0/32768.0*9.81+acc_y_offset; 
    acc_data.az_m_s2 = ((float) z_val)*6.0/32768.0*9.81; 
    return acc_data; 
}

gyro_data_t get_gyro_data(void) {
    uint8_t raw_data[6]; 
    ESP_ERROR_CHECK(register_read(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data))); 
    int x_val = (int16_t) ((raw_data[1]<<8) | raw_data[0]); 
    int y_val = (int16_t) ((raw_data[3]<<8) | raw_data[2]); 
    int z_val = (int16_t) ((raw_data[5]<<8) | raw_data[4]); 
    // ESP_LOGI(TAG, "Gyro raw data: x=%x, y=%x, z=%x", x_val, y_val, z_val);
    
    gyro_data_t gyro_data; 
    gyro_data.Gx_rad_s = DEG_TO_RAD(((float) x_val)*1000.0/32767.0)+gyro_x_offset;
    gyro_data.Gy_rad_s = DEG_TO_RAD(((float) y_val)*1000.0/32767.0)+gyro_y_offset; 
    gyro_data.Gz_rad_s = DEG_TO_RAD(((float) z_val)*1000.0/32767.0)+gyro_z_offset;  
    return gyro_data; 
}

void calibrate_IMU(void) {
    ESP_LOGI(TAG, "Beginning Sensor calibration"); 
    float acc_x_running_total = 0.0; 
    float acc_y_running_total = 0.0; 
    float acc_z_running_total = 0.0; 
    float gyro_x_running_total = 0.0; 
    float gyro_y_running_total = 0.0; 
    float gyro_z_running_total = 0.0; 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = IMU_CALIBRATION_PERIOD_MS/portTICK_PERIOD_MS;
    BaseType_t xWasDelayed;
    for (int i=0; i<CALIBRATION_SAMPLES; i++) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Can't get I2C data this fast");

        acc_data_t acc_data = get_acc_data(); 
        acc_x_running_total += acc_data.ax_m_s2; 
        acc_y_running_total += acc_data.ay_m_s2;
        acc_z_running_total += acc_data.az_m_s2;

        gyro_data_t gyro_data = get_gyro_data(); 
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