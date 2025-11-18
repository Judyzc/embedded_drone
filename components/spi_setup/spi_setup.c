#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/spi_master.h"

#include "spi_setup.h"

static const char *TAG = "spi setup"; 

/* ------------------------------------------- Public function definitions  ------------------------------------------- */
void spi_master_init(spi_host_device_t host)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = ESP_MISO_IO,
        .mosi_io_num = ESP_MOSI_IO,
        .sclk_io_num = ESP_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");
}