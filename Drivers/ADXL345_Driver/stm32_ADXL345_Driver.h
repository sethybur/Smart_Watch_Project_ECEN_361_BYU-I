/*
 * ADXL345_Driver.h
 *
 *
 *      Author: Everett
 */

#ifndef ADXL345_DRIVER_STM32_ADXL345_DRIVER_H_
#define ADXL345_DRIVER_STM32_ADXL345_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define ADXL345_I2C_ADDRESS (0x53 << 1)

typedef struct ADXL_Data_t {
	float x;
	float y;
	float z;
}ADXL_Data_t;


void adxl_write (I2C_HandleTypeDef *hi2cl, uint8_t reg, uint8_t value);
void adxl_read (I2C_HandleTypeDef *hi2cl, uint8_t reg, uint8_t value, uint8_t * data_rec);
void adxl_init (I2C_HandleTypeDef * hi2cl);
void adxl_retrive_data (I2C_HandleTypeDef *hi2cl, ADXL_Data_t * data);

#endif /* ADXL345_DRIVER_STM32_ADXL345_DRIVER_H_ */
