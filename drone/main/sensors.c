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
#include "driver/spi_master.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"

#include "VL53L1X_api.h"
#include "pmw3901.h"

static const char *TAG = "example";
static uint16_t tof_i2c_addr = DECK_TOF_SENSOR_ADDRESS;

/* ------------------------------------------- Global Variables  ------------------------------------------- */
// i2c
QueueHandle_t xQueue_raw_acc_data, xQueue_raw_gyro_data, xQueue_raw_tof_data; 
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t acc_handle, gyro_handle, tof_handle;

// spi
static TaskHandle_t xPMWTaskHandle = NULL;
static pmw3901_t g_pmw = {0};
static const char *TAG_PMW = "pmw3901";


/* ---------------------------------------------- Functions  ----------------------------------------------- */
static void i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // change back to false later
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t tof_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DECK_TOF_SENSOR_ADDRESS,
        .scl_speed_hz = 100000, // temp
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &tof_cfg, &tof_handle));
}

static esp_err_t DECK_tof_init(void)
{
    // Ensure bus & device handle exist for default address
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "bus_handle NULL");
        return ESP_FAIL;
    }
    // We'll try two candidate addresses: 7-bit 0x29 and 8-bit 0x52 (0x29<<1)
    uint16_t candidates[] = { 0x29, (uint16_t)(0x29 << 1) };
    const int nc = sizeof(candidates) / sizeof(candidates[0]);
    for (int i = 0; i < nc; ++i) {
        uint16_t cand = candidates[i];
        ESP_LOGI(TAG, "Attempting VL53 init with addr 0x%02X", cand);
        // If you use the library API that expects 7-bit addresses, pass cand as-is.
        // If API expects 8-bit address, the second candidate will work.
        if (VL53L1X_SensorInit(cand) == VL53L1_ERROR_NONE) {
            ESP_LOGI(TAG, "VL53L1X_SensorInit OK at 0x%02X", cand);
            if (VL53L1X_StartRanging(cand) == VL53L1_ERROR_NONE) {
                ESP_LOGI(TAG, "VL53L1X_StartRanging OK at 0x%02X", cand);
                tof_i2c_addr = cand;
                return ESP_OK;
            } else {
                ESP_LOGW(TAG, "StartRanging failed at 0x%02X", cand);
                // continue trying other candidate
            }
        } else {
            ESP_LOGW(TAG, "SensorInit failed at 0x%02X", cand);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // small pause between tries
    }
    ESP_LOGE(TAG, "VL53 init failed on all candidate addresses");
    return ESP_FAIL;
}

void vProcessTofDataTask(void *pvParameters) {
    (void)pvParameters;
    VL53L1X_Result_t result;
    uint16_t distance_mm;
    ESP_LOGI(TAG, "vProcessTofDataTask started, queue handle=%p, tof_i2c_addr=0x%02X", (void*)xQueue_raw_tof_data, (unsigned)tof_i2c_addr);
    for (;;) {
        if (xQueue_raw_tof_data == NULL) {
            ESP_LOGW(TAG, "TOF queue not created yet; delaying");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        uint16_t dev = (uint16_t) tof_i2c_addr;
        if (dev == 0) {
            // Not initialized yet
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (VL53L1X_GetResult(dev, &result) == VL53L1_ERROR_NONE) {
            distance_mm = (uint16_t) result.Distance;
            // Defensive: check queue handle again before sending
            if (xQueue_raw_tof_data != NULL) {
                BaseType_t ok = xQueueSendToBack(xQueue_raw_tof_data, &distance_mm, pdMS_TO_TICKS(10));
                if (ok != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send TOF reading to queue (err=%ld)", (long)ok);
                }
            } else {
                ESP_LOGE(TAG, "Queue became NULL unexpectedly while sending");
            }
        } else {
            // no data or error; avoid busy-loop
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void vConsumeTofTask(void *pv) {
    uint16_t rx;
    for (;;) {
        if (xQueueReceive(xQueue_raw_tof_data, &rx, portMAX_DELAY) == pdTRUE) {
            // process rx
            ESP_LOGI(TAG, "Consumed TOF: %u mm", rx);
        }
    }
}

static void vPMWTask(void *pvParameters)
{
    (void)pvParameters;
    int16_t dx = 0, dy = 0;
    ESP_LOGI(TAG_PMW, "PMW task started");
    for (;;) {
        pmw3901_read_motion_count(&g_pmw, &dx, &dy);
        ESP_LOGI(TAG_PMW, "motion dx=%d dy=%d", dx, dy);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void sensors_init(){
    // Start I2C
    vTaskDelay(1 / portTICK_PERIOD_MS); // IMU requires 1ms pause after power-on
    // i2c_master_init();

    // ESP_LOGI(TAG, "I2C initialized successfully");

    // // Initialize TOF (deck)
    // ESP_ERROR_CHECK(DECK_tof_init()); 
    // ESP_LOGI(TAG, "Time of Flight initialized successfully"); 

    // // Initialize internal queues
    // xQueue_raw_tof_data = xQueueCreate(1, sizeof(uint16_t));
    
    // // Start Tasks
    // // // xTaskCreate(vGetRawDataTask, "Get Raw Data", 4096, NULL, GET_RAW_DATA_PRIORITY, NULL);
    // xTaskCreate(vProcessTofDataTask, "TOF Data Processing", 4096, NULL, DATA_PROC_PRIORITY, NULL);
    // xTaskCreate(vConsumeTofTask, "TOF Consumer", 4096, NULL, 5, NULL);

    bool pmw_ok = pmw3901_init(&g_pmw, VSPI_HOST,
                            PMW3901_SCLK_IO,
                            PMW3901_MOSI_IO,
                            PMW3901_MISO_IO,
                            PMW3901_CS_IO);
    if (!pmw_ok) {
        ESP_LOGW(TAG_PMW, "PMW3901 init failed");
    } else {
        ESP_LOGI(TAG_PMW, "PMW3901 init OK; starting PMW task");
        xTaskCreate(vPMWTask, "PMW Task", 2048, NULL, 5, &xPMWTaskHandle);
    }

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(imu_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(gyro_handle));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}
