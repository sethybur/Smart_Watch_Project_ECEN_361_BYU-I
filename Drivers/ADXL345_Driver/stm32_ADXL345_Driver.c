/*
 * ADXL345.c
 *
 *
 *      Author: Everett
 */
#include "stm32_ADXL345_Driver.h"

void adxl_write (I2C_HandleTypeDef *hi2cl, uint8_t reg, uint8_t value){
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (hi2cl, ADXL345_I2C_ADDRESS, data, 2,10);
}

void adxl_read (I2C_HandleTypeDef *hi2cl, uint8_t reg, uint8_t value, uint8_t * data_rec){
	HAL_I2C_Mem_Read(hi2cl, ADXL345_I2C_ADDRESS, reg, 1,data_rec, 6, 100);
}

void adxl_init (I2C_HandleTypeDef * hi2cl)
{
	//reset everything
	adxl_write (hi2cl, 0x2d, 0);


	adxl_write(hi2cl, 0x2d, 0x08); // measure bit 1, wakeup 0, 0 at 8hz

	adxl_write (hi2cl, 0x31, 0x01); // +- 4g range

}

void adxl_retrive_data (I2C_HandleTypeDef *hi2cl, ADXL_Data_t *data)
{
	uint8_t rec[6];
	adxl_read(hi2cl, 0x32, 6, rec);

	data->x = (rec[1]<<8 | rec[0]) * .0078;
	data->y = (rec[3]<<8 | rec[2]) * .0078;
	data->z = (rec[5]<<8 | rec[4]) * .0078;


}

