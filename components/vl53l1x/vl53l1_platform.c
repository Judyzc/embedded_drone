/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include <string.h>
#include <time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "i2c_setup.h"

/* ------------------------------------------- Private Global Variables  ------------------------------------------- */
i2c_master_dev_handle_t tof_handle;

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
esp_err_t tof_init(i2c_master_bus_handle_t *bus_handle) {
	i2c_device_config_t tof_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOF_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &tof_config, &tof_handle));

    VL53L1X_ERROR Status;
    uint8_t state = 0;  
    while(!state){
        Status = VL53L1X_BootState(0, &state);
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    Status = VL53L1X_SensorInit(0);
    Status = VL53L1X_SetInterMeasurementInMs(0, TOF_SENS_PERIOD_MS);
    Status = VL53L1X_SetTimingBudgetInMs(0, 33);
    Status = VL53L1X_StartRanging(0);

    return (Status == 0) ? ESP_OK : ESP_FAIL;
}

/* ------------------------------------------- Platform API  ------------------------------------------- */
int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t *write_buf = malloc((2+count)*sizeof(uint8_t));
	write_buf[0] = index>>8; 
	write_buf[1] = index&0xFF; 
	memcpy(&write_buf[2], pdata, count);
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 2+count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	free(write_buf); 
	
	return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	uint8_t write_buf[] = {index>>8, index&0xFF};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, pdata, count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	
	return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t write_buf[] = {index>>8, index&0xFF, data};
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t write_buf[] = {index>>8, index&0xFF, data>>8, data&0xFF};
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t write_buf[] = {index>>8, index&0xFF, data>>24, (data>>16) & 0xFF, (data>>8) & 0xFF, data & 0xFF};
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	uint8_t write_buf[] = {index>>8, index&0xFF};

    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	
	return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

	uint8_t write_buf[] = {index>>8, index&0xFF};
	
	uint8_t read_buf[2];
	
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, read_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	*data = read_buf[0]<<8 | read_buf[1];
	
	return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

	uint8_t write_buf[] = {index>>8, index&0xFF};

	uint8_t read_buf[4]; 
	
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, read_buf, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	*data = read_buf[0]<<24 | read_buf[1]<<16 | read_buf[2]<<8 | read_buf[3];
	
	return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

	vTaskDelay(wait_ms / portTICK_PERIOD_MS); 
	
	return 0;
}
