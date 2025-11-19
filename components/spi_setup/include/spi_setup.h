#ifndef SPI_SETUP_H
#define SPI_SETUP_H

#include "driver/spi_master.h"

/* ------------------------------------------- Constants ------------------------------------------- */
#define ESP_SCLK_IO             18              // 3, right side of deck
#define ESP_MOSI_IO             23              // 5, right side of deck
#define ESP_MISO_IO             19              // 4, right side of deck
#define SPI_BAUDRATE_2MHZ       2*1000*1000

/* ------------------------------------------- Public function declaration  ------------------------------------------- */
void spi_master_init(spi_host_device_t host);
bool spiExchange(spi_device_handle_t dev, size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx);
void spiBeginTransaction(uint32_t baudRatePrescaler);
void spiEndTransaction(void); 

#endif /* SPI_SETUP_H */