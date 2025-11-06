// components/VL53L1_ULD_ESP/platform/VL53L1_platform.c
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

/* FreeRTOS headers for vTaskDelay, pdMS_TO_TICKS, xTaskGetTickCount, etc. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "VL53L1X_error_codes.h"
#include "vl53l1_platform.h"


/*Pass in bus and tof handle from sensors.c */
extern i2c_master_bus_handle_t bus_handle;
extern i2c_master_dev_handle_t tof_handle;

#ifndef I2C_MASTER_TIMEOUT_MS
#define I2C_MASTER_TIMEOUT_MS 1000
#endif

static inline VL53L1_Error i2c_write_prefixed(i2c_master_dev_handle_t dev, uint16_t index, const uint8_t *buf, uint32_t len)
{
    uint32_t tx_len = 2 + len;
    uint8_t *tx = malloc(tx_len);
    if (!tx) return VL53L1_ERROR_CONTROL_INTERFACE;
    tx[0] = (uint8_t)((index >> 8) & 0xFF);
    tx[1] = (uint8_t)(index & 0xFF);
    if (len && buf) memcpy(&tx[2], buf, len);
    TickType_t timeout_ticks = pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS);
    esp_err_t err = i2c_master_transmit(dev, tx, tx_len, timeout_ticks);
    free(tx);
    return (err == ESP_OK) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

static inline VL53L1_Error i2c_read_prefixed(i2c_master_dev_handle_t dev, uint16_t index, uint8_t *buf, uint32_t len)
{
    uint8_t idx[2];
    idx[0] = (uint8_t)((index >> 8) & 0xFF);
    idx[1] = (uint8_t)(index & 0xFF);
    TickType_t timeout_ticks = pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS);
    esp_err_t err = i2c_master_transmit_receive(dev, idx, 2, buf, len, timeout_ticks);
    return (err == ESP_OK) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

/* Platform functions called by the core (use the global tof_handle) */
VL53L1_Error VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    (void)dev;
    return i2c_write_prefixed(tof_handle, index, pdata, count);
}

VL53L1_Error VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    (void)dev;
    return i2c_read_prefixed(tof_handle, index, pdata, count);
}

VL53L1_Error VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    (void)dev;
    return i2c_write_prefixed(tof_handle, index, &data, 1);
}

VL53L1_Error VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
    (void)dev;
    return i2c_read_prefixed(tof_handle, index, pdata, 1);
}

VL53L1_Error VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t tmp[2] = { (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF) };
    (void)dev;
    return i2c_write_prefixed(tof_handle, index, tmp, 2);
}

VL53L1_Error VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t tmp[2];
    (void)dev;
    VL53L1_Error err = i2c_read_prefixed(tof_handle, index, tmp, 2);
    if (err == VL53L1_ERROR_NONE) {
        *pdata = (uint16_t)((tmp[0] << 8) | tmp[1]);
    }
    return err;
}

VL53L1_Error VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t tmp[4] = {
        (uint8_t)((data >> 24) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)(data & 0xFF)
    };
    (void)dev;
    return i2c_write_prefixed(tof_handle, index, tmp, 4);
}

VL53L1_Error VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t tmp[4];
    (void)dev;
    VL53L1_Error err = i2c_read_prefixed(tof_handle, index, tmp, 4);
    if (err == VL53L1_ERROR_NONE) {
        *pdata = ((uint32_t)tmp[0] << 24) | ((uint32_t)tmp[1] << 16) | ((uint32_t)tmp[2] << 8) | tmp[3];
    }
    return err;
}

/* Timing helpers */
VL53L1_Error VL53L1_WaitMs(void *pdev, int32_t wait_ms)
{
    (void)pdev;
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(void *pdev, int32_t wait_us)
{
    (void)pdev;
    int32_t ms = (wait_us + 999) / 1000;
    vTaskDelay(pdMS_TO_TICKS(ms));
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptime_ms)
{
    if (!ptime_ms) return VL53L1_ERROR_INVALID_PARAMS;
    *ptime_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return VL53L1_ERROR_NONE;
}
