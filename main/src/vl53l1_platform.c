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
#include <string.h>
#include <time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "i2c_setup.h"
#include "sensors.h"

static const char* TAG = "platform"; 

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t *write_buf = malloc((2+count)*sizeof(uint8_t));
	memcpy((void *) write_buf, (void *) &index, 2); 
	memcpy((void *) &write_buf[2], (void *) pdata, count); 
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 2+count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	free(write_buf); 
	
	return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	uint8_t write_buf[2];
	memcpy((void *) write_buf, (void *) &index, 2); 
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, pdata, count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	
	return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t write_buf[3];
	memcpy((void *) write_buf, (void *) &index, 2); 
	memcpy((void *) &write_buf[2], (void *) &data, 1); 
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t write_buf[4];
	memcpy((void *) write_buf, (void *) &index, 2); 
	memcpy((void *) &write_buf[2], (void *) &data, 2); 
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t write_buf[6];
	memcpy((void *) write_buf, (void *) &index, 2); 
	memcpy((void *) &write_buf[2], (void *) &data, 4); 
    ESP_ERROR_CHECK(i2c_master_transmit(tof_handle, write_buf, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	uint8_t write_buf[2];
	memcpy((void *) write_buf, (void *) &index, 2);

    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	
	return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

	uint8_t write_buf[2];
	memcpy((void *) write_buf, (void *) &index, 2);
	
	uint8_t read_buf[2];
	
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, read_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	memcpy((void *) data, (void *) read_buf, 2);
	
	return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	// uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

	uint8_t write_buf[2];
	memcpy((void *) write_buf, (void *) &index, 2);

	uint8_t read_buf[4]; 
	
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tof_handle, write_buf, 2, read_buf, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	memcpy((void *) data, (void *) read_buf, 4);
	
	return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	// uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

	vTaskDelay(wait_ms / portTICK_PERIOD_MS); 
	
	return 0;
}
