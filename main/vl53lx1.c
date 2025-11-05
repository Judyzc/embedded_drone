// vl53l1x_i2c.c
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <string.h>
#include "sensors.h"

extern i2c_master_dev_handle_t tof_handle;
// Read and write N bytes to a 16-bit register index (index: MSB then LSB)
// for vl53l1x, but general for dealing with 16-bit register 

static esp_err_t write_reg16(uint16_t index, const uint8_t *data, size_t len)
{
    uint8_t tx[2 + len];
    tx[0] = (uint8_t)(index >> 8);      // index MSB
    tx[1] = (uint8_t)(index & 0xFF);    // index LSB
    memcpy(&tx[2], data, len);
    return i2c_master_transmit(tof_handle, tx, sizeof(tx), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t read_reg16(uint16_t index, uint8_t *out, size_t len)
{
    uint8_t idx[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
    return i2c_master_transmit_receive(tof_handle, idx, 2, out, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Convenience: write a single byte to a 16-bit index
static esp_err_t vl53l1x_write_byte(uint16_t index, uint8_t val)
{
    return write_reg16(index, &val, 1);
}

// Convenience: read a single byte
static esp_err_t vl53l1x_read_byte(uint16_t index, uint8_t *val)
{
    return read_reg16(index, val, 1);
}
