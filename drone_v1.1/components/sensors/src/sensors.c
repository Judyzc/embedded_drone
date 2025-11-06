#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sensors.h"
#include "helper.h"
#include "bmp3_defs.h"
#include "bmp3.h"
#include "bstdr_comm_support.h"
#include "i2c.h"
#include "six_axis_comp_filter.h"
#include <math.h>

static const char *TAG = "SENSORS";

#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_RATE_HZ            1000
#define BMP3_I2C_ADDR_SEC	UINT8_C(0x77)
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ)
#define BMP388_I2C_TIMEOUT_MS SENSORS_DELAY_BARO

static bool isBarometerPresent = false;
static bool isInit = false;
static struct bmp3_dev bmp388Dev;

i2c_master_bus_handle_t i2c_bus;
i2c_master_dev_handle_t baro_handle;
i2c_master_dev_handle_t acc_handle;
i2c_master_dev_handle_t gyro_handle;

int8_t bmp3_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    if (!reg_data || len == 0) return -1;

    // prefer persistent device handle
    if (baro_handle) {
        esp_err_t err = i2c_master_transmit_receive(baro_handle,
                                                    &reg_addr, 1,
                                                    reg_data, len,
                                                    BMP388_I2C_TIMEOUT_MS);
        return (err == ESP_OK) ? 0 : -1;
    }

    // If you don't have a persistent device handle, return error (or implement temp device logic)
    return -1;
}

int8_t bmp3_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    if (!reg_data || len == 0) return -1;

    size_t total = 1 + len;
    uint8_t *buf = malloc(total);
    if (!buf) return -1;
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);

    if (baro_handle) {
        esp_err_t err = i2c_master_transmit(baro_handle, buf, total, BMP388_I2C_TIMEOUT_MS);
        free(buf);
        return (err == ESP_OK) ? 0 : -1;
    }

    free(buf);
    return -1;
}

void bmp3_ms_delay(uint32_t ms)
{
    if (ms == 0) return;
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void acc_init() {
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_PWR_CTRL, 0x04)); 
    // Set data rate and filter to 1600 Hz
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_CONF, 0xAC)); 
    // Set range to 6G
    ESP_ERROR_CHECK(register_write_byte(acc_handle, ACC_RANGE, 0x01)); 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
    return;
}

void gyro_init() {
    // Turn on gyro  
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_LPM1, 0x00)); 
    // Set data rate and filter to 1000 Hz and 116 Hz
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_BANDWIDTH, 0x02)); 
    // Set range to 1000 deg/s
    ESP_ERROR_CHECK(register_write_byte(gyro_handle, GYRO_RANGE, 0x01)); 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
    return;
}

void acc_read(acc_sample_t* acc_sample) {
    uint8_t raw_data[3*2*sizeof(uint8_t)]; 
    ESP_ERROR_CHECK(i2c_master_read_reg(acc_handle, ACC_DATA_START, raw_data, sizeof(raw_data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)); 
    
    acc_sample->timestamp = get_time();
    acc_sample->raw_ax = (int16_t) (raw_data[0] | (raw_data[1]<<8));
    acc_sample->raw_ay = (int16_t) (raw_data[2] | (raw_data[3]<<8));
    acc_sample->raw_az = (int16_t) (raw_data[4] | (raw_data[5]<<8));

    acc_sample->ax = ((float) acc_sample->raw_ax)*6.0/32768.0*9.81; 
    acc_sample->ay = ((float) acc_sample->raw_ay)*6.0/32768.0*9.81; 
    acc_sample->az = ((float) acc_sample->raw_az)*6.0/32768.0*9.81; 

    return;
}

void gyro_read(gyro_sample_t* gyro_sample) {
    uint8_t raw_data[3*2*sizeof(uint8_t)];
    ESP_ERROR_CHECK(i2c_master_read_reg(gyro_handle, GYRO_DATA_START, raw_data, sizeof(raw_data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)); 
    
    gyro_sample->timestamp = get_time();
    gyro_sample->raw_gx = (int16_t) (raw_data[0] | (raw_data[1]<<8));
    gyro_sample->raw_gy = (int16_t) (raw_data[2] | (raw_data[3]<<8));
    gyro_sample->raw_gz = (int16_t) (raw_data[4] | (raw_data[5]<<8));

    gyro_sample->gx = CompDegreesToRadians(((float)  gyro_sample->raw_gx)*1000.0/32767.0);
    gyro_sample->gy = CompDegreesToRadians(((float)  gyro_sample->raw_gy)*1000.0/32767.0); 
    gyro_sample->gz = CompDegreesToRadians(((float)  gyro_sample->raw_gz)*1000.0/32767.0); 

    return;
}

void baro_init() {
    if (isInit) return;

    bstdr_ret_t rslt;
    isBarometerPresent = false;

    // Wait for sensors to startup
    vTaskDelay(SENSORS_STARTUP_TIME_MS);

    /* BMP388 */
    bmp388Dev.dev_id = BMP3_I2C_ADDR_SEC;
    bmp388Dev.intf = BMP3_I2C_INTF;
    bmp388Dev.read  = bmp3_i2c_read;
    bmp388Dev.write = bmp3_i2c_write;
    bmp388Dev.delay_ms = bmp3_ms_delay; 

    int i = 3;
    do {
        bmp388Dev.delay_ms(1);
        // For some reason it often doesn't work first time
        rslt = bmp3_init(&bmp388Dev);
    } while (rslt != BMP3_OK && i-- > 0);

    if (rslt == BMP3_OK) {
        isBarometerPresent = true;
        if (bmp3_chip_id == BMP388_CHIP_ID){
            ESP_LOGI(TAG, "BMP388 I2C connection [OK]\n");
        }
        else{
            ESP_LOGI(TAG, "BMP388 I2C connection [FAIL]\n");
            isInit = false;
            return;
        }

        /* Used to select the settings user needs to change */
        uint16_t settings_sel;
        /* Select the pressure and temperature sensor to be enabled */
        bmp388Dev.settings.press_en = BMP3_ENABLE;
        bmp388Dev.settings.temp_en = BMP3_ENABLE;
        /* Select the output data rate and oversampling settings for pressure and temperature */
        bmp388Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
        bmp388Dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
        bmp388Dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
        bmp388Dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
        /* Assign the settings which needs to be set in the sensor */
        settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
        rslt = bmp3_set_sensor_settings(settings_sel, &bmp388Dev);

        /* Set the power mode to normal mode */
        bmp388Dev.settings.op_mode = BMP3_NORMAL_MODE;
        rslt = bmp3_set_op_mode(&bmp388Dev);

        bmp388Dev.delay_ms(20); // wait before first read out
        // read out data
        /* Variable used to select the sensor component */
        uint8_t sensor_comp;
        /* Variable used to store the compensated data */
        struct bmp3_data data;

        /* Sensor component selection */
        sensor_comp = BMP3_PRESS | BMP3_TEMP;
        /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

        /* Print the temperature and pressure data */
        if (bmp3_chip_id == BMP388_CHIP_ID){
            ESP_LOGI("BMP388", "BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0f);
        }
        // baroMeasDelayMin = SENSORS_DELAY_BARO;
  }
  else {
    #ifndef CONFIG_SENSORS_IGNORE_BAROMETER_FAIL
        ESP_LOGI(TAG, "BMP3XX I2C connection [FAIL]\n");
        isInit = false;
        return;
    #endif
  }

  isInit = true;
}

void baro_read(baro_sample_t *baro_sample) {
    baro_sample->timestamp = get_time();

    uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
    struct bmp3_data bmp3_raw_data;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    bmp3_get_sensor_data(sensor_comp, &bmp3_raw_data, &bmp388Dev);
    
    baro_sample->pressure = bmp3_raw_data.pressure*0.01f;
    baro_sample->temperature = bmp3_raw_data.temperature;
    baro_sample->asl = ((powf((1015.7f / baro_sample->pressure), 0.1902630958f)
        - 1.0f) * (25.0f + 273.15f)) / 0.0065f; 
}

void mock_tof_read(tof_sample_t *tof_sample) {
    tof_sample->timestamp = get_time();
    static float d = 500.0f; // mm
    d += 0.2f;
    tof_sample->distance_mm = d;
}

void sensors_init() {
    // initialize I2C bus
    ESP_ERROR_CHECK(i2c_master_init_bus(-1, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 400000, &i2c_bus));

    // add BMP388 to I2C bus
    ESP_ERROR_CHECK(i2c_master_add_device(i2c_bus, 0x77, &baro_handle));
    
    // add IMU ACC to I2C bus
    ESP_ERROR_CHECK(i2c_master_add_device(i2c_bus, 0x18, &acc_handle));

    // add IMU GYRO to I2C bus
    ESP_ERROR_CHECK(i2c_master_add_device(i2c_bus, 0x69, &gyro_handle));

    // initialize BMP388
    baro_init();

    // initialize ACC
    acc_init();

    // initialize GRYO
    gyro_init();
}

