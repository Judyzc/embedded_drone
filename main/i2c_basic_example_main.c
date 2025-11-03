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
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "six_axis_comp_filter.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define IMU_ACC_SENSOR_ADDR         0x18                        /* Accelerometer I2C Address */
#define ACC_PWR_CTRL                0x7D                        /* Accelerometer registers */
#define ACC_CONF                    0x40
#define ACC_RANGE                   0x41
#define ACC_DATA_START              0x12

#define IMU_GYRO_SENSOR_ADDR        0x69                        /* Gyroscope I2C Address */
#define GYRO_LPM1                   0x11                        /* Gyroscope registers */
#define GYRO_RANGE                  0x0F
#define GYRO_BANDWIDTH              0x10
#define GYRO_DATA_START             0x02

#define TAU                         1.0f                        /* Constants for complementary filter */
#define SENS_RATE_HZ                100                         /* Sensor polling rate */

typedef struct acc_data {
    float ax_m_s2; 
    float ay_m_s2; 
    float az_m_s2; 
} acc_data_t;

typedef struct gyro_data {
    float Gx_rad_s; 
    float Gy_rad_s; 
    float Gz_rad_s; 
} gyro_data_t;

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *imu_handle, i2c_master_dev_handle_t *gyro_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t imu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ACC_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &imu_config, imu_handle));

    i2c_device_config_t gyro_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_GYRO_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &gyro_config, gyro_handle));
}

static esp_err_t IMU_acc_init(i2c_master_dev_handle_t dev_handle) 
{
    // Turn on Accelerometer  
    ESP_ERROR_CHECK(register_write_byte(dev_handle, ACC_PWR_CTRL, 0x04)); 
    // Set data rate and filter to 1600 Hz
    ESP_ERROR_CHECK(register_write_byte(dev_handle, ACC_CONF, 0xAC)); 
    // Set range to 6G
    ESP_ERROR_CHECK(register_write_byte(dev_handle, ACC_RANGE, 0x01)); 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
    return ESP_OK;
}

static esp_err_t IMU_gyro_init(i2c_master_dev_handle_t dev_handle) 
{
    // Turn on gyro  
    ESP_ERROR_CHECK(register_write_byte(dev_handle, GYRO_LPM1, 0x00)); 
    // Set data rate and filter to 1000 Hz and 116 Hz
    ESP_ERROR_CHECK(register_write_byte(dev_handle, GYRO_BANDWIDTH, 0x02)); 
    // Set range to 1000 deg/s
    ESP_ERROR_CHECK(register_write_byte(dev_handle, GYRO_RANGE, 0x01)); 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
    return ESP_OK;
}

static void IMU_acc_get_data(i2c_master_dev_handle_t imu_handle, acc_data_t *acc_data) {
    uint8_t raw_data[3*2*sizeof(uint8_t)]; 
    ESP_ERROR_CHECK(register_read(imu_handle, ACC_DATA_START, raw_data, sizeof(raw_data)));
    // ESP_LOGI(TAG, "Accel raw data: x=%x, y=%x, z=%x", (raw_data[0] | (raw_data[1]<<8)), (raw_data[2] | (raw_data[3]<<8)), (raw_data[4] | (raw_data[5]<<8)));
    int x_val = (int16_t) (raw_data[0] | (raw_data[1]<<8)); 
    int y_val = (int16_t) (raw_data[2] | (raw_data[3]<<8)); 
    int z_val = (int16_t) (raw_data[4] | (raw_data[5]<<8)); 
    
    acc_data->ax_m_s2 = ((float) x_val)*6.0/32768.0*9.81; 
    acc_data->ay_m_s2 = ((float) y_val)*6.0/32768.0*9.81; 
    acc_data->az_m_s2 = ((float) z_val)*6.0/32768.0*9.81; 
}

static void IMU_gyro_get_data(i2c_master_dev_handle_t gyro_handle, gyro_data_t *gyro_data) {
    uint8_t raw_data[3*2*sizeof(uint8_t)]; 
    ESP_ERROR_CHECK(register_read(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data))); 
    // ESP_LOGI(TAG, "Gyro raw data: x=%x, y=%x, z=%x", (raw_data[0] | (raw_data[1]<<8)), (raw_data[2] | (raw_data[3]<<8)), (raw_data[4] | (raw_data[5]<<8)));
    int x_val = (int16_t) (raw_data[0] | (raw_data[1]<<8)); 
    int y_val = (int16_t) (raw_data[2] | (raw_data[3]<<8)); 
    int z_val = (int16_t) (raw_data[4] | (raw_data[5]<<8)); 
    
    gyro_data->Gx_rad_s = CompDegreesToRadians(((float) x_val)*1000.0/32767.0);
    gyro_data->Gy_rad_s = CompDegreesToRadians(((float) y_val)*1000.0/32767.0); 
    gyro_data->Gz_rad_s = CompDegreesToRadians(((float) z_val)*1000.0/32767.0);  
}

void app_main(void)
{
    // Start I2C
    vTaskDelay(1 / portTICK_PERIOD_MS);         // IMU requires a 1ms after power-on
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t imu_handle, gyro_handle;
    i2c_master_init(&bus_handle, &imu_handle, &gyro_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize IMU
    ESP_ERROR_CHECK(IMU_acc_init(imu_handle)); 
    ESP_ERROR_CHECK(IMU_gyro_init(gyro_handle)); 
    ESP_LOGI(TAG, "IMU initialized successfully"); 

    // Initialize complementary filter
    SixAxis comp_filter;
    const float deltaT = 1.0f/SENS_RATE_HZ;  
    CompInit(&comp_filter, deltaT, TAU); 
    acc_data_t acc_data; 
    gyro_data_t gyro_data; 
    IMU_acc_get_data(imu_handle, &acc_data); 
    IMU_gyro_get_data(gyro_handle, &gyro_data);
    CompAccelUpdate(&comp_filter, acc_data.ax_m_s2, acc_data.ay_m_s2, acc_data.az_m_s2); 
    CompStart(&comp_filter); 

    TickType_t xLastWakeTime = xTaskGetTickCount ();
    const TickType_t xTimeIncrement = (1000/SENS_RATE_HZ)/portTICK_PERIOD_MS;
    BaseType_t xWasDelayed;
    for (;;) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
        assert(xWasDelayed);

        IMU_acc_get_data(imu_handle, &acc_data); 
        // acc_data.ax_m_s2 = 0; 
        // acc_data.ay_m_s2 = 0; 
        // acc_data.az_m_s2 = 9.81; 
        IMU_gyro_get_data(gyro_handle, &gyro_data);
        // gyro_data.Gx_rad_s = 0;
        // gyro_data.Gy_rad_s = 0;
        // gyro_data.Gz_rad_s = 0;

        // ESP_LOGI(TAG, "Acceleration (m/s2): x=%.2f y=%.2f z=%.2f", acc_data.ax_m_s2, acc_data.ay_m_s2, acc_data.az_m_s2); 
        // ESP_LOGI(TAG, "Attitude Rate (rad/s): x=%.2f y=%.2f z=%.2f", gyro_data.Gx_rad_s, gyro_data.Gy_rad_s, gyro_data.Gz_rad_s);
              
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

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(imu_handle));
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(gyro_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
