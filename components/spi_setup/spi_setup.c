#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "driver/spi_master.h"

#include "spi_setup.h"

static const char *TAG = "spi setup"; 

/* ------------------------------------------- Private global variables  ------------------------------------------- */
static SemaphoreHandle_t spiMutex;
static bool isInit = false;

/* ------------------------------------------- Public function definitions  ------------------------------------------- */
void spi_master_init(spi_host_device_t host)
{
    spiMutex = xSemaphoreCreateMutex();

    spi_bus_config_t buscfg = {
        .miso_io_num = ESP_MISO_IO,
        .mosi_io_num = ESP_MOSI_IO,
        .sclk_io_num = ESP_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "I2C initialized successfully");
    isInit = true; 
}

static void spiConfigureWithSpeed(uint32_t baudRatePrescaler)
{
    //TODO:
}


bool spiTest(void)
{
    return isInit;
}

bool spiExchange(spi_device_handle_t dev, size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx)
{
    if (isInit != true) {
        return false;
    }

    if (length == 0) {
        return true;    //no need to send anything
    }

    esp_err_t ret;

    if (is_tx == true) {

        static spi_transaction_t t;
        memset(&t, 0, sizeof(t));					//Zero out the transaction
        t.length = length * 8;						//Len is in bytes, transaction length is in bits.
        t.tx_buffer = data_tx;						//Data
        // ret = spi_device_polling_transmit(dev, &t); //Transmit!
        ret = spi_device_transmit(dev, &t);
        assert(ret == ESP_OK);						//Should have had no issues.
        //DEBUG_PRINTD("spi send = %d",t.length);
        return true;
    }

    static spi_transaction_t r;
    memset(&r, 0, sizeof(r));
    r.length = length * 8;
    r.flags = SPI_TRANS_USE_RXDATA;
    // ret = spi_device_polling_transmit(dev, &r);
    ret = spi_device_transmit(dev, &r);
    assert(ret == ESP_OK);

    if (r.rxlength > 0) {
        //DEBUG_PRINTD("rxlength = %d",r.rxlength);
        memcpy(data_rx, r.rx_data, length);
    }

    return true;
}

void spiBeginTransaction(uint32_t baudRatePrescaler)
{
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction(void)
{
    xSemaphoreGive(spiMutex);
}