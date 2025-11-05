// i2c_master_helper.h

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "hal/gpio_types.h"
#include <stdint.h>

#ifndef I2C_HELPER_H
#define I2C_HELPER_H

// Initialize an I2C master bus.
// - i2c_port: use -1 to let driver auto-pick a port (i2c_port_num_t is used in API).
// - sda_gpio / scl_gpio: GPIO numbers for SDA / SCL
// - clk_speed_hz: desired bus speed (e.g. 100000 or 400000)
// - out_bus: pointer to receive the created bus handle
// Returns ESP_OK on success.
esp_err_t i2c_master_init_bus(int i2c_port,
                             gpio_num_t sda_gpio,
                             gpio_num_t scl_gpio,
                             uint32_t clk_speed_hz,
                             i2c_master_bus_handle_t *out_bus);

// Add a device (slave) to an existing bus.
// - bus: bus handle returned by i2c_master_init_bus
// - dev_addr: 7-bit device address
// - out_dev: pointer to receive the device handle
esp_err_t i2c_master_add_device(i2c_master_bus_handle_t bus,
                                uint8_t dev_addr,
                                i2c_master_dev_handle_t *out_dev);

// Remove device from bus and free its resources
esp_err_t i2c_master_remove_device(i2c_master_dev_handle_t dev);

// Write register: write register byte followed by data (convenience).
esp_err_t i2c_master_write_reg(i2c_master_dev_handle_t dev,
                              uint8_t reg_addr,
                              const uint8_t *data,
                              size_t len,
                              uint32_t timeout_ms);

// Read register: write reg_addr then read len bytes into out_data (repeated-start).
esp_err_t i2c_master_read_reg(i2c_master_dev_handle_t dev,
                             uint8_t reg_addr,
                             uint8_t *out_data,
                             size_t len,
                             uint32_t timeout_ms);


#endif