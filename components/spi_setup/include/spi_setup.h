#ifndef SPI_SETUP_H
#define SPI_SETUP_H

#include "driver/spi_master.h"

/* ------------------------------------------- Constants ------------------------------------------- */
#define ESP_SCLK_IO 18  // 3, right side of deck
#define ESP_MOSI_IO 23  // 5, right side of deck
#define ESP_MISO_IO 19  // 4, right side of deck
#define ESP_CS_IO   5   // 8, left side of deck

/* ------------------------------------------- Public function declaration  ------------------------------------------- */
void spi_master_init(spi_host_device_t host);

#endif /* SPI_SETUP_H */