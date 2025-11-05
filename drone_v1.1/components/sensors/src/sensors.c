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

static i2c_master_dev_handle_t bmp_handle;

int8_t bmp3_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    if (!reg_data || len == 0) return -1;

    // prefer persistent device handle
    if (bmp_handle) {
        esp_err_t err = i2c_master_transmit_receive(bmp_handle,
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

    if (bmp_handle) {
        esp_err_t err = i2c_master_transmit(bmp_handle, buf, total, BMP388_I2C_TIMEOUT_MS);
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

void mock_imu_read(imu_sample_t *imu_sample) {
    imu_sample->timestamp = get_time();
    // simple synthetic motion: slow oscillation so filter shows something
    static float t = 0.0f;
    t += 0.05f;

    imu_sample->accel[0] = 0.0f; // X
    imu_sample->accel[1] = 0.0f; // Y
    imu_sample->accel[2] = 9.81f; // Z (gravity)

    // pretend a slow roll rate about X and pitch about Y (deg/s)
    imu_sample->gyro[0] = 5.0f * sinf(t); // roll rate
    imu_sample->gyro[1] = 3.0f * cosf(t); // pitch rate
    imu_sample->gyro[2] = 0.0f; // yaw rate
}

void baro_init(i2c_master_dev_handle_t* bmp_dev_handle) {
    if (isInit) return;

    bmp_handle = *bmp_dev_handle;

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

void mock_baro_read(baro_sample_t *baro_sample) {
    baro_sample->timestamp = get_time();
    static float alt = 1000.0f;
    // small noise
    alt += 0.1f;
    baro_sample->pressure = 101325.0f - alt; // arbitrary mapping
    baro_sample->temperature = 25.0f;
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

