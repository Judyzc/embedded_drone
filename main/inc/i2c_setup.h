#ifndef I2C_SETUP_H
#define I2C_SETUP_H

#include "driver/i2c_master.h"

/* ------------------------------------------- Constants ------------------------------------------- */
// I2C Config info
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

/* ------------------------------------------- Global Variables ------------------------------------------- */
extern i2c_master_bus_handle_t bus_handle;

/* ------------------------------------------- Public Function Declarations ------------------------------------------- */
esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len); 
esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data); 
void i2c_master_init(); 

#endif /* I2C_SETUP_H */