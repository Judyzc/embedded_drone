#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdio.h>

#include "pmw3901.h"
#include "spi_setup.h"

// static const char *TAG = "PMW3901";

/* ------------------------------------------- Private global variables  ------------------------------------------- */
spi_device_handle_t opt_flow_handle;
static bool isInit = false; 

/* ------------------------------------------- Private Function Definitions  ------------------------------------------- */
void sleepus(uint32_t us)
{
  int64_t start = esp_timer_get_time();

  while ((start+us) > esp_timer_get_time());
}

static void registerWrite(uint32_t csPin, uint8_t reg, uint8_t value)
{
    // Set MSB to 1 for write
    reg |= 0x80u;

    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    gpio_set_level(csPin, 0);

    sleepus(50);

    spiExchange(opt_flow_handle, 1, 1, &reg, &reg);
    sleepus(50);
    spiExchange(opt_flow_handle, 1, 1, &value, &value);

    sleepus(50);

    gpio_set_level(csPin, 1);
    spiEndTransaction();
    sleepus(200);
}

static uint8_t registerRead(uint32_t csPin, uint8_t reg)
{
    uint8_t data = 0;
    uint8_t dummy = 0;

    // Set MSB to 0 for read
    reg &= ~0x80u;

    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    gpio_set_level(csPin, 0);

    sleepus(50);

    spiExchange(opt_flow_handle, 1, 1, &reg, &reg);
    sleepus(500);
    spiExchange(opt_flow_handle, 1, 0, &dummy, &data);

    sleepus(50);

    gpio_set_level(csPin, 1);
    spiEndTransaction();
    sleepus(200);

    return data;
}

static void InitRegisters(uint32_t csPin)
{
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x61, 0xAD);
    registerWrite(csPin, 0x7F, 0x03);
    registerWrite(csPin, 0x40, 0x00);
    registerWrite(csPin, 0x7F, 0x05);
    registerWrite(csPin, 0x41, 0xB3);
    registerWrite(csPin, 0x43, 0xF1);
    registerWrite(csPin, 0x45, 0x14);
    registerWrite(csPin, 0x5B, 0x32);
    registerWrite(csPin, 0x5F, 0x34);
    registerWrite(csPin, 0x7B, 0x08);
    registerWrite(csPin, 0x7F, 0x06);
    registerWrite(csPin, 0x44, 0x1B);
    registerWrite(csPin, 0x40, 0xBF);
    registerWrite(csPin, 0x4E, 0x3F);
    registerWrite(csPin, 0x7F, 0x08);
    registerWrite(csPin, 0x65, 0x20);
    registerWrite(csPin, 0x6A, 0x18);
    registerWrite(csPin, 0x7F, 0x09);
    registerWrite(csPin, 0x4F, 0xAF);
    registerWrite(csPin, 0x5F, 0x40);
    registerWrite(csPin, 0x48, 0x80);
    registerWrite(csPin, 0x49, 0x80);
    registerWrite(csPin, 0x57, 0x77);
    registerWrite(csPin, 0x60, 0x78);
    registerWrite(csPin, 0x61, 0x78);
    registerWrite(csPin, 0x62, 0x08);
    registerWrite(csPin, 0x63, 0x50);
    registerWrite(csPin, 0x7F, 0x0A);
    registerWrite(csPin, 0x45, 0x60);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x4D, 0x11);
    registerWrite(csPin, 0x55, 0x80);
    registerWrite(csPin, 0x74, 0x1F);
    registerWrite(csPin, 0x75, 0x1F);
    registerWrite(csPin, 0x4A, 0x78);
    registerWrite(csPin, 0x4B, 0x78);
    registerWrite(csPin, 0x44, 0x08);
    registerWrite(csPin, 0x45, 0x50);
    registerWrite(csPin, 0x64, 0xFF);
    registerWrite(csPin, 0x65, 0x1F);
    registerWrite(csPin, 0x7F, 0x14);
    registerWrite(csPin, 0x65, 0x67);
    registerWrite(csPin, 0x66, 0x08);
    registerWrite(csPin, 0x63, 0x70);
    registerWrite(csPin, 0x7F, 0x15);
    registerWrite(csPin, 0x48, 0x48);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x41, 0x0D);
    registerWrite(csPin, 0x43, 0x14);
    registerWrite(csPin, 0x4B, 0x0E);
    registerWrite(csPin, 0x45, 0x0F);
    registerWrite(csPin, 0x44, 0x42);
    registerWrite(csPin, 0x4C, 0x80);
    registerWrite(csPin, 0x7F, 0x10);
    registerWrite(csPin, 0x5B, 0x02);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x40, 0x41);
    registerWrite(csPin, 0x70, 0x00);

    vTaskDelay(10/portTICK_PERIOD_MS); // delay 10ms

    registerWrite(csPin, 0x32, 0x44);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x40, 0x40);
    registerWrite(csPin, 0x7F, 0x06);
    registerWrite(csPin, 0x62, 0xF0);
    registerWrite(csPin, 0x63, 0x00);
    registerWrite(csPin, 0x7F, 0x0D);
    registerWrite(csPin, 0x48, 0xC0);
    registerWrite(csPin, 0x6F, 0xD5);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x5B, 0xA0);
    registerWrite(csPin, 0x4E, 0xA8);
    registerWrite(csPin, 0x5A, 0x50);
    registerWrite(csPin, 0x40, 0x80);

    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x5A, 0x10);
    registerWrite(csPin, 0x54, 0x00);
}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
bool pmw3901Init(spi_host_device_t host, uint32_t csPin)
{
    if (isInit) {
        return true;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_BAUDRATE_2MHZ, //Clock out at 10 MHz
        .mode = 3,							 //SPI mode 0
        .spics_io_num = -1,					 //CS pin
        .queue_size = 8,					 //We want to be able to queue 7 transactions at a time
        /*.pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line*/
    };
    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &opt_flow_handle)); 

    // Initialize CS Pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << csPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(csPin, 1);

    gpio_set_level(csPin, 1);
    vTaskDelay(2/portTICK_PERIOD_MS);
    gpio_set_level(csPin, 0);
    vTaskDelay(2/portTICK_PERIOD_MS);
    gpio_set_level(csPin, 1);
    vTaskDelay(2/portTICK_PERIOD_MS);

    uint8_t chipId    = registerRead(csPin, 0x00);
    uint8_t invChipId = registerRead(csPin, 0x5f);

    // ESP_LOGI(TAG, "Motion chip id: 0x%x:0x%x\n", chipId, invChipId);

    if (chipId == 0x49 || invChipId == 0xB6) {
        // Power on reset
        registerWrite(csPin, 0x3a, 0x5a);
        vTaskDelay(5/portTICK_PERIOD_MS);

        // Reading the motion registers one time
        registerRead(csPin, 0x02);
        registerRead(csPin, 0x03);
        registerRead(csPin, 0x04);
        registerRead(csPin, 0x05);
        registerRead(csPin, 0x06);
        vTaskDelay(1/portTICK_PERIOD_MS);

        InitRegisters(csPin);

        isInit = true;
    }

    return isInit;
}

void pmw3901ReadMotion(uint32_t csPin, motionBurst_t *motion)
{
    uint8_t address = 0x16;

    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    gpio_set_level(csPin, 0);
    sleepus(10);
    spiExchange(opt_flow_handle, 1, 1, &address, &address);
    sleepus(10);
    spiExchange(opt_flow_handle, sizeof(motionBurst_t), 0, (uint8_t *)motion, (uint8_t *)motion);
    sleepus(10);
    gpio_set_level(csPin, 1);
    spiEndTransaction();

    uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
    realShutter |= (motion->shutter & 0x0ff) << 8;
    motion->shutter = realShutter;
}