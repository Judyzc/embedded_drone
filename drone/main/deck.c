// main.c
// ESP-IDF example for PMW3901 (Crazyflie Flowdeck 2.1 / breakout).
// ESP-IDF v5.x style.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "rom/ets_sys.h" // ets_delay_us

static const char *TAG = "pmw3901_espidf";

/* ======= CONFIGURE PINS HERE ======= */
#define PIN_SPI_MOSI 23 // right side 5
#define PIN_SPI_MISO 19 // right side 4
#define PIN_SPI_SCLK 18 // right side 3
#define PIN_CS       5  // left side 8
/* =================================== */

#define SPI_HOST_USED HSPI_HOST
#define SPI_CLOCK_HZ   2000000UL // 2 MHz

#define PMW_REG_PRODUCT_ID      0x00
#define PMW_REG_INVERSE_PRODUCT_ID 0x5F
#define PMW_REG_POWER_UP_RESET  0x3A
#define PMW_POWERUP_RESET_VALUE 0x5A
#define PMW_MOTION_BURST        0x16
#define MOTION_BURST_LEN 12

static spi_device_handle_t spi_dev = NULL;

/* Manual CS helpers */
static inline void cs_low(void)  { gpio_set_level(PIN_CS, 0); }
static inline void cs_high(void) { gpio_set_level(PIN_CS, 1); }

/* Improved spi_xfer: set tx/rx lengths only when needed */
static esp_err_t spi_xfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t length)
{
    if (!spi_dev) return ESP_ERR_INVALID_STATE;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    if (txbuf && length) {
        t.length = length * 8;       // bits to send
        t.tx_buffer = txbuf;
    } else {
        t.length = 0;
        t.tx_buffer = NULL;
    }

    if (rxbuf && length) {
        t.rxlength = length * 8;     // bits to receive
        t.rx_buffer = rxbuf;
    } else {
        t.rxlength = 0;
        t.rx_buffer = NULL;
    }

    // Blocking transmit
    return spi_device_transmit(spi_dev, &t);
}

/* Raw register write using manual CS and CF-like timing */
static esp_err_t pmw_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80u), value };
    cs_low();
    ets_delay_us(50);                         // hold after CS low
    esp_err_t r = spi_xfer(tx, NULL, sizeof(tx));
    cs_high();
    ets_delay_us(200);                        // CF uses some 200us after writes in places
    return r;
}

/* Raw register read using two-step: send address, small delay, read byte */
static esp_err_t pmw_read_reg(uint8_t reg, uint8_t *out)
{
    uint8_t tx = reg & 0x7Fu;
    uint8_t rx = 0;

    cs_low();
    ets_delay_us(50);
    // send address (TX-only)
    esp_err_t r = spi_xfer(&tx, NULL, 1);
    if (r != ESP_OK) { cs_high(); return r; }

    // required sensor internal delay before reading
    ets_delay_us(500);

    // read returned byte (RX-only) by sending a dummy byte
    uint8_t dummy = 0x00;
    r = spi_xfer(&dummy, &rx, 1);
    cs_high();
    ets_delay_us(50);

    if (r == ESP_OK && out) *out = rx;
    return r;
}

/* Motion burst: send burst address then read 'len' bytes while CS held */
static esp_err_t pmw_read_motion_burst(uint8_t *buf, size_t len)
{
    if (len == 0 || len > 64) return ESP_ERR_INVALID_ARG;

    uint8_t reg = PMW_MOTION_BURST & 0x7Fu;
    cs_low();
    ets_delay_us(50);

    // send burst address
    esp_err_t r = spi_xfer(&reg, NULL, 1);
    if (r != ESP_OK) { cs_high(); return r; }

    ets_delay_us(50);

    // read 'len' bytes: send dummy bytes and receive into buf
    uint8_t tx_dummy[64];
    memset(tx_dummy, 0x00, len);
    r = spi_xfer(tx_dummy, buf, len);

    cs_high();
    ets_delay_us(50);

    return r;
}

/* Probe: manual SCLK bit-bang while sampling MISO as GPIO input.
   Temporarily reconfigures SCLK and MOSI pins to GPIO output and toggles clock
   while sampling MISO. Useful to detect wiring/tri-state. */
static void manual_clock_probe(int pulses)
{
    ESP_LOGI(TAG, "Starting manual clock probe (%d pulses)...", pulses);

    // Reconfigure pins as GPIO outputs/inputs
    gpio_set_direction(PIN_SPI_SCLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SPI_MOSI, 0);
    gpio_set_level(PIN_SPI_SCLK, 1); // idle high for mode 3
    gpio_set_direction(PIN_SPI_MISO, GPIO_MODE_INPUT);

    int idle = gpio_get_level(PIN_SPI_MISO);
    ESP_LOGI(TAG, "MISO idle level: %d", idle);

    for (int i = 0; i < pulses; ++i) {
        gpio_set_level(PIN_SPI_SCLK, 0);
        ets_delay_us(10);
        int s1 = gpio_get_level(PIN_SPI_MISO);
        gpio_set_level(PIN_SPI_SCLK, 1);
        ets_delay_us(10);
        int s2 = gpio_get_level(PIN_SPI_MISO);
        ESP_LOGI(TAG, "pulse %02d: MISO(low)=%d MISO(high)=%d", i, s1, s2);
    }

    ESP_LOGI(TAG, "Manual clock probe done. Re-init SPI device after this.");
}

/* Minimal wrapper for InitRegisters:
   -> COPY the full InitRegisters() sequence from the Crazyflie STM driver HERE.
   -> The PMW3901 expects that EXACT register sequence (and delays).
   -> The function below is a placeholder. Paste the registerWrite() calls from the CF code.
*/
static void InitRegisters_from_Crazyflie(void)
{
    // (you MUST paste the full sequence from the Crazyflie code):
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

  vTaskDelay(M2T(10)); // delay 10ms

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

    ESP_LOGW(TAG, "InitRegisters() placeholder called. Paste the Crazyflie InitRegisters() sequence here.");
}

/* Read several registers used for debugging */
static void read_and_log_registers(void)
{
    uint8_t v;
    if (pmw_read_reg(PMW_REG_PRODUCT_ID, &v) == ESP_OK) {
        ESP_LOGI(TAG, "Read PRODUCT_ID = 0x%02X", v);
    } else {
        ESP_LOGE(TAG, "Failed reading PRODUCT_ID");
    }
    if (pmw_read_reg(PMW_REG_INVERSE_PRODUCT_ID, &v) == ESP_OK) {
        ESP_LOGI(TAG, "Read INVERSE_PRODUCT_ID = 0x%02X", v);
    } else {
        ESP_LOGE(TAG, "Failed reading INVERSE_PRODUCT_ID");
    }

    const uint8_t regs_to_probe[] = { 0x02, 0x03, 0x04, 0x06, 0x07 };
    for (int i = 0; i < (int)sizeof(regs_to_probe); ++i) {
        uint8_t rr;
        if (pmw_read_reg(regs_to_probe[i], &rr) == ESP_OK) {
            ESP_LOGI(TAG, "Reg 0x%02X = 0x%02X", regs_to_probe[i], rr);
        } else {
            ESP_LOGW(TAG, "Reg 0x%02X read failed", regs_to_probe[i]);
        }
    }
}

/* motion task prints dx/dy and warns on all-zeros */
static void motion_task(void *arg)
{
    (void)arg;
    uint8_t buf[MOTION_BURST_LEN];

    while (1) {
        esp_err_t r = pmw_read_motion_burst(buf, sizeof(buf));
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "raw: %02X %02X %02X %02X %02X %02X ...", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
            int16_t dx = (int8_t)buf[1];
            int16_t dy = (int8_t)buf[2];
            printf("DX: %d  DY: %d\n", dx, dy);

            if (buf[0]==0 && buf[1]==0 && buf[2]==0 && buf[3]==0) {
                ESP_LOGW(TAG, "Motion burst all zeros - suggests no response from sensor");
            }
        } else {
            ESP_LOGW(TAG, "motion burst read failed: %d", r);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    esp_err_t ret;
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_LOGI(TAG, "Starting PMW3901 diagnostic main");

    // Configure CS pin as output, default HIGH
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    cs_high();

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    ret = spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
        return;
    }

    // Create SPI device with mode 3 (CPOL=1 CPHA=1) and no HW CS (we bit-bang CS)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = 0,
    };
    ret = spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %d", ret);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // Toggle CS a few times to wake device
    for (int i = 0; i < 3; ++i) {
        ESP_LOGI(TAG, "CS toggle %d", i);
        cs_low();
        ets_delay_us(50);
        cs_high();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Basic init: try product id reads, power-up reset
    ESP_LOGI(TAG, "Starting PMW3901 init sequence...");
    uint8_t id=0, inv=0;
    if (pmw_read_reg(PMW_REG_PRODUCT_ID, &id) != ESP_OK) {
        ESP_LOGW(TAG, "First attempt read PRODUCT_ID failed");
    }
    if (pmw_read_reg(PMW_REG_INVERSE_PRODUCT_ID, &inv) != ESP_OK) {
        ESP_LOGW(TAG, "First attempt read INVERSE_PRODUCT_ID failed");
    }
    ESP_LOGI(TAG, "Product ID: 0x%02X  InvID: 0x%02X", id, inv);

    // If product IDs look good, do power-up reset and the register init sequence
    if (id == 0x49 && inv == 0xB6) {
        ESP_LOGI(TAG, "PMW3901 detected. Performing power-up reset and register init.");
        if (pmw_write_reg(PMW_REG_POWER_UP_RESET, PMW_POWERUP_RESET_VALUE) == ESP_OK) {
            ESP_LOGI(TAG, "Wrote power-up reset");
        } else {
            ESP_LOGW(TAG, "Power-up reset write failed");
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        // Reading motion registers once (as in Crazyflie)
        pmw_read_reg(0x02, &id);
        pmw_read_reg(0x03, &id);
        pmw_read_reg(0x04, &id);
        pmw_read_reg(0x05, &id);
        pmw_read_reg(0x06, &id);
        vTaskDelay(pdMS_TO_TICKS(1));

        // IMPORTANT: paste the Crazyflie full InitRegisters() sequence into this function
        InitRegisters_from_Crazyflie();
    } else {
        ESP_LOGW(TAG, "Unexpected product ID values. You may still try InitRegisters(), but check wiring.");
    }

    // Diagnostic register snapshot
    read_and_log_registers();

    // Manual clock probe (helps detect MISO wiring)
    ESP_LOGI(TAG, "About to run manual clock probe. This will temporarily re-configure SCLK as GPIO");
    manual_clock_probe(8);

    // Re-init SPI device to restore hardware SPI after manual probe (safe)
    if (spi_dev) {
        spi_bus_remove_device(spi_dev);
        spi_dev = NULL;
    }
    spi_device_interface_config_t devcfg2 = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    esp_err_t r = spi_bus_add_device(SPI_HOST_USED, &devcfg2, &spi_dev);
    ESP_LOGI(TAG, "Re-added SPI device result=%d", r);

    // Final register snapshot
    read_and_log_registers();

    // Launch motion task
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
}
