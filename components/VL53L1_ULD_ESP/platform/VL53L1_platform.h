#ifndef VL53L1_PLATFORM_H
#define VL53L1_PLATFORM_H

#include <stdint.h>
#include "VL53L1X_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Core expects these signatures (dev = 7-bit i2c address) */
VL53L1_Error VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
VL53L1_Error VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
VL53L1_Error VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
VL53L1_Error VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);
VL53L1_Error VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
VL53L1_Error VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);

VL53L1_Error VL53L1_WaitMs(void *pdev, int32_t wait_ms);
VL53L1_Error VL53L1_WaitUs(void *pdev, int32_t wait_us);
VL53L1_Error VL53L1_GetTickCount(uint32_t *ptime_ms);

#ifdef __cplusplus
}
#endif

#endif /* VL53L1_PLATFORM_H */
