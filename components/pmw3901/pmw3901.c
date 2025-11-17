#include "pmw3901.h"
#include <string.h>
#include <stdio.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "PMW3901";

static void write_reg(pmw3901_t *dev, uint8_t reg, uint8_t value);
static uint8_t read_reg(pmw3901_t *dev, uint8_t reg);
static void init_registers(pmw3901_t *dev);

/* Helpers */
static inline void pmw_cs_low(pmw3901_t *dev)
{
    gpio_set_level(dev->cs_io, 0);
    esp_rom_delay_us(10);
}
static inline void pmw_cs_high(pmw3901_t *dev)
{
    esp_rom_delay_us(10);
    gpio_set_level(dev->cs_io, 1);
    esp_rom_delay_us(20);
}

/* Write register: CS low, send (reg|0x80) then value, small delays, CS high. */
static void write_reg(pmw3901_t *dev, uint8_t reg, uint8_t value)
{
    if (!dev || !dev->spi) return;
    pmw_cs_low(dev);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 2; // 2 B
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (uint8_t)(reg | 0x80u);
    t.tx_data[1] = value;
    esp_err_t ret = spi_device_polling_transmit(dev->spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write error: %d", ret);
    }
    // small post-write delay
    esp_rom_delay_us(200);
    pmw_cs_high(dev);
}


/* Read registers */
static uint8_t read_reg(pmw3901_t *dev, uint8_t reg)
{
    if (!dev || !dev->spi) return 0;
    uint8_t rx = 0;
    reg &= ~0x80u;
    pmw_cs_low(dev);
    // send address Byte
    {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8;
        t.flags = SPI_TRANS_USE_TXDATA;
        t.tx_data[0] = reg;
        esp_err_t r = spi_device_polling_transmit(dev->spi, &t);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "SPI addr tx error: %d", r);
            pmw_cs_high(dev);
            return 0;
        }
    }
    // data delay
    esp_rom_delay_us(160);
    // read data Byte
    {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8;
        t.flags = SPI_TRANS_USE_RXDATA;
        t.tx_data[0] = 0x00; // dummy
        esp_err_t r = spi_device_polling_transmit(dev->spi, &t);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "SPI read error: %d", r);
            pmw_cs_high(dev);
            return 0;
        }
        rx = t.rx_data[0];
    }
    esp_rom_delay_us(200);
    pmw_cs_high(dev);
    return rx;
}


/* Register initialization sequence */
static void init_registers(pmw3901_t *dev)
{
    if (!dev) return;
    write_reg(dev, 0x7F, 0x00);
    write_reg(dev, 0x61, 0xAD);
    write_reg(dev, 0x7F, 0x03);
    write_reg(dev, 0x40, 0x00);
    write_reg(dev, 0x7F, 0x05);
    write_reg(dev, 0x41, 0xB3);
    write_reg(dev, 0x43, 0xF1);
    write_reg(dev, 0x45, 0x14);
    write_reg(dev, 0x5B, 0x32);
    write_reg(dev, 0x5F, 0x34);
    write_reg(dev, 0x7B, 0x08);
    write_reg(dev, 0x7F, 0x06);
    write_reg(dev, 0x44, 0x1B);
    write_reg(dev, 0x40, 0xBF);
    write_reg(dev, 0x4E, 0x3F);
    write_reg(dev, 0x7F, 0x08);
    write_reg(dev, 0x65, 0x20);
    write_reg(dev, 0x6A, 0x18);
    write_reg(dev, 0x7F, 0x09);
    write_reg(dev, 0x4F, 0xAF);
    write_reg(dev, 0x5F, 0x40);
    write_reg(dev, 0x48, 0x80);
    write_reg(dev, 0x49, 0x80);
    write_reg(dev, 0x57, 0x77);
    write_reg(dev, 0x60, 0x78);
    write_reg(dev, 0x61, 0x78);
    write_reg(dev, 0x62, 0x08);
    write_reg(dev, 0x63, 0x50);
    write_reg(dev, 0x7F, 0x0A);
    write_reg(dev, 0x45, 0x60);
    write_reg(dev, 0x7F, 0x00);
    write_reg(dev, 0x4D, 0x11);
    write_reg(dev, 0x55, 0x80);
    write_reg(dev, 0x74, 0x1F);
    write_reg(dev, 0x75, 0x1F);
    write_reg(dev, 0x4A, 0x78);
    write_reg(dev, 0x4B, 0x78);
    write_reg(dev, 0x44, 0x08);
    write_reg(dev, 0x45, 0x50);
    write_reg(dev, 0x64, 0xFF);
    write_reg(dev, 0x65, 0x1F);
    write_reg(dev, 0x7F, 0x14);
    write_reg(dev, 0x65, 0x67);
    write_reg(dev, 0x66, 0x08);
    write_reg(dev, 0x63, 0x70);
    write_reg(dev, 0x7F, 0x15);
    write_reg(dev, 0x48, 0x48);
    write_reg(dev, 0x7F, 0x07);
    write_reg(dev, 0x41, 0x0D);
    write_reg(dev, 0x43, 0x14);
    write_reg(dev, 0x4B, 0x0E);
    write_reg(dev, 0x45, 0x0F);
    write_reg(dev, 0x44, 0x42);
    write_reg(dev, 0x4C, 0x80);
    write_reg(dev, 0x7F, 0x10);
    write_reg(dev, 0x5B, 0x02);
    write_reg(dev, 0x7F, 0x07);
    write_reg(dev, 0x40, 0x41);
    write_reg(dev, 0x70, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    write_reg(dev, 0x32, 0x44);
    write_reg(dev, 0x7F, 0x07);
    write_reg(dev, 0x40, 0x40);
    write_reg(dev, 0x7F, 0x06);
    write_reg(dev, 0x62, 0xF0);
    write_reg(dev, 0x63, 0x00);
    write_reg(dev, 0x7F, 0x0D);
    write_reg(dev, 0x48, 0xC0);
    write_reg(dev, 0x6F, 0xD5);
    write_reg(dev, 0x7F, 0x00);
    write_reg(dev, 0x5B, 0xA0);
    write_reg(dev, 0x4E, 0xA8);
    write_reg(dev, 0x5A, 0x50);
    write_reg(dev, 0x40, 0x80);
    write_reg(dev, 0x7F, 0x00);
    write_reg(dev, 0x5A, 0x10);
    write_reg(dev, 0x54, 0x00);
}


/* Public APIs */
bool pmw3901_init(pmw3901_t *dev, spi_host_device_t host,
                  int sclk_io, int mosi_io, int miso_io, int cs_io)
{
    // ESP_LOGI(TAG, "pmw3901_init start");
    if (!dev) return false;
    memset(dev, 0, sizeof(*dev));
    dev->host = host;
    dev->sclk_io = sclk_io;
    dev->mosi_io = mosi_io;
    dev->miso_io = miso_io;
    dev->cs_io   = cs_io;
    // ESP_LOGI(TAG, "Using SPI host=%d SCLK=%d MOSI=%d MISO=%d CS=%d",
    //          (int)dev->host, dev->sclk_io, dev->mosi_io, dev->miso_io, dev->cs_io);
    spi_bus_config_t buscfg = {
        .miso_io_num = dev->miso_io,
        .mosi_io_num = dev->mosi_io,
        .sclk_io_num = dev->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    esp_err_t ret = spi_bus_initialize(dev->host, &buscfg, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
        return false;
    }
    // Configure CS as manual GPIO (we toggle it ourselves)
    gpio_set_direction(dev->cs_io, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->cs_io, 1); // idle high
    // Device config: mode 3 is required by PMW3901 (CPOL=1, CPHA=1)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // start conservative 1 MHz
        .mode = 3,
        .spics_io_num = -1,        // manual CS
        .queue_size = 1,
    };
    ret = spi_bus_add_device(dev->host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %d", ret);
        return false;
    }
    // ESP_LOGI(TAG, "SPI bus/device ready");
    // Reset (power-on reset register)
    write_reg(dev, 0x3A, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t chipId = read_reg(dev, 0x00);
    uint8_t chipIdInv = read_reg(dev, 0x5F);
    // ESP_LOGI(TAG, "chip id raw: %02x / %02x", chipId, chipIdInv);
    if (chipId != PMW_CHIP_ID || chipIdInv != PMW_CHIP_ID_INVERSE) {
        ESP_LOGE(TAG, "bad chip id: %02x / %02x", chipId, chipIdInv);
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
        return false;
    }
    // motion registers
    read_reg(dev, 0x02);
    read_reg(dev, 0x03);
    read_reg(dev, 0x04);
    read_reg(dev, 0x05);
    read_reg(dev, 0x06);
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Initialize sensor registers (Bitcraze sequence) */
    init_registers(dev);

    // ESP_LOGI(TAG, "PMW3901 initialized (chip id OK)");
    return true;
}


bool pmw3901_read_motion_count(pmw3901_t *dev, int16_t *delta_x, int16_t *delta_y)
{
    // ESP_LOGI(TAG, "trying to read motion");
    if (!dev) return false;
    // motion reg
    (void) read_reg(dev, 0x02);
    uint8_t dx_h = read_reg(dev, 0x04);
    uint8_t dx_l = read_reg(dev, 0x03);
    uint8_t dy_h = read_reg(dev, 0x06);
    uint8_t dy_l = read_reg(dev, 0x05);
    *delta_x = (int16_t)(dx_h << 8 | dx_l);
    *delta_y = (int16_t)(dy_h << 8 | dy_l);
    return true;
}