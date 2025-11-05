#include "i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "I2C_HELPER";

esp_err_t i2c_master_init_bus(int i2c_port,
                             gpio_num_t sda_gpio,
                             gpio_num_t scl_gpio,
                             uint32_t clk_speed_hz,
                             i2c_master_bus_handle_t* out_bus)
{
    if (out_bus == NULL) return ESP_ERR_INVALID_ARG;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = (i2c_port_num_t)i2c_port,    // -1 allowed for auto-select
        .sda_io_num = sda_gpio, // D21
        .scl_io_num = scl_gpio, // D22
        .clk_source = I2C_CLK_SRC_DEFAULT,     // typical default
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,                     // 0 means let driver choose default
        .flags.enable_internal_pullup = false   // set true for quick prototyping; prefer external pullups
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, out_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Created I2C master bus (port=%d, SDA=%d, SCL=%d, clk=%u)",
             i2c_port, sda_gpio, scl_gpio, clk_speed_hz);
    return ESP_OK;
}

esp_err_t i2c_master_add_device(i2c_master_bus_handle_t bus,
                                uint8_t dev_addr,
                                i2c_master_dev_handle_t* out_dev)
{
    if (!bus || out_dev == NULL) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t dev_cfg = {
        .device_address = dev_addr,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = 400000, // default device speed; override if needed
        .flags = { 0 },
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device (0x%02X) failed: %s", dev_addr, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device 0x%02X added to bus", dev_addr);
    }
    return err;
}

esp_err_t i2c_master_remove_device(i2c_master_dev_handle_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_bus_rm_device(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_rm_device failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t i2c_master_write_reg(i2c_master_dev_handle_t dev,
                              uint8_t reg_addr,
                              const uint8_t *data,
                              size_t len,
                              uint32_t timeout_ms)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    size_t total = 1 + len;
    uint8_t *buf = (uint8_t *) malloc(total);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = reg_addr;
    if (len && data) memcpy(&buf[1], data, len);

    esp_err_t err = i2c_master_transmit(dev, buf, total, timeout_ms);
    
    free(buf);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write_reg (0x%02X) failed: %s", reg_addr, esp_err_to_name(err));
    }
    return err;
}

esp_err_t i2c_master_read_reg(i2c_master_dev_handle_t dev,
                             uint8_t reg_addr,
                             uint8_t *out_data,
                             size_t len,
                             uint32_t timeout_ms)
{
    if (!dev || !out_data || len == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t err = i2c_master_transmit_receive(dev, &reg_addr, 1, out_data, len, timeout_ms);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read_reg (0x%02X) failed: %s", reg_addr, esp_err_to_name(err));
    }
    return err;
}
