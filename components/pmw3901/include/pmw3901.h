#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PMW_CHIP_ID         0x49u
#define PMW_CHIP_ID_INVERSE 0xB6u

typedef struct {
    spi_device_handle_t spi;       // SPI device handle (internal)
    spi_host_device_t   host;      // SPI host used (HSPI_HOST/VSPI_HOST)
    int sclk_io;
    int mosi_io;
    int miso_io;
    int cs_io;
} pmw3901_t;

bool pmw3901_init(pmw3901_t *dev, spi_host_device_t host,
                  int sclk_io, int mosi_io, int miso_io, int cs_io);
bool pmw3901_read_motion_count(pmw3901_t *dev, uint16_t *delta_x, uint16_t *delta_y);

#ifdef __cplusplus
}
#endif

#endif // PMW3901_H