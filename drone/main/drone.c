// main.c
// ESP-IDF v5.5 example to talk to the PMW3901 (Bitcraze breakout).
// Reads motion burst and prints DX/DY + simple direction to serial.
//
// Note: This implements manual CS toggling to respect PMW3901 timing.
// from CHAT and this: https://circuitdigest.com/microcontroller-projects/interfacing-pmw3901-optical-flow-sensor-with-esp32 
// bitcrazie 

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

/* Generic spi_xfer */
static esp_err_t spi_xfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t length) {
    if (!spi_dev) return ESP_ERR_INVALID_STATE;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.rxlength = length * 8;
    t.tx_buffer = txbuf;
    t.rx_buffer = rxbuf;
    return spi_device_transmit(spi_dev, &t);
}

/* register read/write using manual CS and timing like Crazyflie driver */
static esp_err_t pmw_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), value };
    cs_low(); ets_delay_us(50);
    esp_err_t r = spi_xfer(tx, NULL, 2);
    cs_high(); ets_delay_us(50);
    return r;
}

static esp_err_t pmw_read_reg(uint8_t reg, uint8_t *out) {
    uint8_t tx = reg & 0x7F;
    uint8_t rx;
    cs_low(); ets_delay_us(50);
    esp_err_t r = spi_xfer(&tx, NULL, 1);
    if (r != ESP_OK) { cs_high(); return r; }
    ets_delay_us(500);
    uint8_t dummy = 0x00;
    r = spi_xfer(&dummy, &rx, 1);
    cs_high(); ets_delay_us(20);
    if (r == ESP_OK) *out = rx;
    return r;
}

static esp_err_t pmw_read_motion_burst(uint8_t *buf, size_t len) {
    uint8_t reg = PMW_MOTION_BURST & 0x7F;
    cs_low(); ets_delay_us(50);
    esp_err_t r = spi_xfer(&reg, NULL, 1);
    if (r != ESP_OK) { cs_high(); return r; }
    ets_delay_us(500);
    uint8_t tx_dummy[MOTION_BURST_LEN];
    memset(tx_dummy, 0x00, sizeof(tx_dummy));
    r = spi_xfer(tx_dummy, buf, len);
    cs_high(); ets_delay_us(20);
    return r;
}

/* Probe: manual SCLK bit-bang while sampling MISO as GPIO input.
   Temporarily reconfigures SCLK and MOSI pins to GPIO output and toggles clock
   while sampling MISO. This will not harm SPI hardware and helps check if MISO
   toggles at all when clock pulses are delivered (useful to detect wiring/tri-state). */
static void manual_clock_probe(int pulses) {
    ESP_LOGI(TAG, "Starting manual clock probe (%d pulses)...", pulses);

    // Save current SCLK/MOSI pin configs by reconfiguring them as GPIO outputs.
    gpio_set_direction(PIN_SPI_SCLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_SPI_MOSI, GPIO_MODE_OUTPUT);
    // Ensure MOSI low (not driving sensor), set SCLK high then toggle
    gpio_set_level(PIN_SPI_MOSI, 0);
    gpio_set_level(PIN_SPI_SCLK, 1);
    // make sure MISO is input
    gpio_set_direction(PIN_SPI_MISO, GPIO_MODE_INPUT);

    // Sample MISO at idle (SCLK high), and then while toggling SCLK low/high.
    int last = gpio_get_level(PIN_SPI_MISO);
    ESP_LOGI(TAG, "MISO idle level: %d", last);

    for (int i = 0; i < pulses; ++i) {
        // toggle SCLK low
        gpio_set_level(PIN_SPI_SCLK, 0);
        ets_delay_us(10);
        int s1 = gpio_get_level(PIN_SPI_MISO);
        // toggle SCLK high
        gpio_set_level(PIN_SPI_SCLK, 1);
        ets_delay_us(10);
        int s2 = gpio_get_level(PIN_SPI_MISO);
        ESP_LOGI(TAG, "pulse %02d: MISO(low)=%d MISO(high)=%d", i, s1, s2);
    }

    // restore pins to SPI: easiest approach is to re-init SPI bus after this probe.
    ESP_LOGI(TAG, "Manual clock probe done.");
}

/* Helper: read common registers and print values */
static void read_and_log_registers(void) {
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

    // read a few more registers that often indicate status/firmware
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

/* Try reading product id repeatedly for a mode to detect intermittent comms */
static void try_mode_and_read(spi_device_handle_t *pdev, int mode) {
    ESP_LOGI(TAG, "Testing SPI mode %d", mode);

    // Recreate device with requested mode
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = mode,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = 0,
    };

    if (*pdev) {
        spi_bus_remove_device(*pdev);
        *pdev = NULL;
    }
    esp_err_t r = spi_bus_add_device(SPI_HOST_USED, &devcfg, pdev);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device mode %d failed: %d", mode, r);
        return;
    }

    // Toggle CS a few times to wake device
    for (int i = 0; i < 3; ++i) {
        cs_low(); ets_delay_us(50); cs_high(); vTaskDelay(pdMS_TO_TICKS(2));
    }

    // read product id repeatedly
    for (int i = 0; i < 5; ++i) {
        uint8_t id = 0, inv = 0;
        esp_err_t r1 = pmw_read_reg(PMW_REG_PRODUCT_ID, &id);
        esp_err_t r2 = pmw_read_reg(PMW_REG_INVERSE_PRODUCT_ID, &inv);
        if (r1 == ESP_OK && r2 == ESP_OK) {
            ESP_LOGI(TAG, "mode %d read %d: PROD=0x%02X INV=0x%02X", mode, i, id, inv);
        } else {
            ESP_LOGW(TAG, "mode %d read %d failed (r1=%d r2=%d)", mode, i, r1, r2);
        }
        ets_delay_us(20000);
    }
}

/* Motion polling task that also warns when everything is zero */
static void motion_task(void *arg) {
    (void)arg;
    uint8_t buf[MOTION_BURST_LEN];

    while (1) {
        esp_err_t r = pmw_read_motion_burst(buf, sizeof(buf));
        if (r == ESP_OK) {
            // Print raw header bytes
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
    ESP_LOGI(TAG, "Starting diagnostic main");

    // configure CS pin as output
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

    // Create device with initial mode=3
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

    // small startup pause
    vTaskDelay(pdMS_TO_TICKS(20));

    // Toggle CS a few times to wake device
    for (int i = 0; i < 3; ++i) {
        ESP_LOGI(TAG, "CS toggle %d", i);
        cs_low(); ets_delay_us(50); cs_high(); vTaskDelay(pdMS_TO_TICKS(2));
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

    // Issue power-up reset (best-effort)
    if (pmw_write_reg(PMW_REG_POWER_UP_RESET, PMW_POWERUP_RESET_VALUE) == ESP_OK) {
        ESP_LOGI(TAG, "Wrote power-up reset");
    } else {
        ESP_LOGW(TAG, "Power-up reset write failed");
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read a set of registers and log
    read_and_log_registers();

    // Try both modes to compare results
    try_mode_and_read(&spi_dev, 3);
    try_mode_and_read(&spi_dev, 0);

    // Manual clock probe (helps detect MISO wiring)
    ESP_LOGI(TAG, "About to run manual clock probe. This will temporarily re-configure SCLK as GPIO");
    manual_clock_probe(8);

    // Re-initialize SPI device after manual probe: set it back to mode 3
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

