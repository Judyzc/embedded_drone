#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------- Constants ------------------------------------------- */
#define PMW_CHIP_ID         0x49u
#define PMW_CHIP_ID_INVERSE 0xB6u
#define OPT_FLOW_CS_PIN     5               // 8, left side of deck

/* ------------------------------------------- Structs ------------------------------------------- */
typedef struct motionBurst_s {
    union {
        uint8_t motion;
        struct {
            uint8_t frameFrom0    : 1;
            uint8_t runMode       : 2;
            uint8_t reserved1     : 1;
            uint8_t rawFrom0      : 1;
            uint8_t reserved2     : 2;
            uint8_t motionOccured : 1;
        };
    };

    uint8_t observation;
    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
} __attribute__((packed)) motionBurst_t;

/* ------------------------------------------- Public Function Declarations ------------------------------------------- */
bool pmw3901Init(spi_host_device_t host, uint32_t csPin);
void pmw3901ReadMotion(uint32_t csPin, motionBurst_t *motion);

#ifdef __cplusplus
}
#endif

#endif // PMW3901_H