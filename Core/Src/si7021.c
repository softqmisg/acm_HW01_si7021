//#include "main.h"
#include "i2c.h"
#include "si7021.h"
////////////////////////////////////////////////
void si7021_Init(void)
{
		HAL_I2C_MspInit(&hi2c1);
		MX_I2C1_Init();
}
////////////////////////////////////////////////
HAL_StatusTypeDef si7021_read_temperature(double *temperature)
{
	uint8_t buffer[5];
	uint16_t raw_temp;
	
	buffer[0]=CMD_TEMP_HOLD;
	
	if(HAL_I2C_Master_Transmit(&hi2c1,SI7021_BASEADDRESS<<1,buffer,1,1000)!=SI7021_OK)
		return SI7021_ERROR;
	//HAL_Delay(20);
	if(HAL_I2C_Master_Receive(&hi2c1,SI7021_BASEADDRESS<<1,buffer,2,2000)!=SI7021_OK)
		return SI7021_ERROR;
	
	/*
	buffer[0]=CMD_TEMP_HOLD;
	if(HAL_I2C_Mem_Read(&hi2c1,SI7021_BASEADDRESS<<1,buffer[0],I2C_MEMADD_SIZE_8BIT,buffer,2,100)!=SI7021_OK)
		return SI7021_ERROR;
	*/
	raw_temp=buffer[0]<<8|buffer[1];
	*temperature=(((double)raw_temp*175.72/65536.0) -46.85);
	return SI7021_OK;
}
///////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef si7021_read_humidity(double *humidity)
{
	uint8_t buffer[5];
	uint16_t raw_hum;
	
	buffer[0]=CMD_HUM_HOLD;
	if(HAL_I2C_Master_Transmit(&hi2c1,SI7021_BASEADDRESS<<1,buffer,1,1000)!=SI7021_OK)
		return SI7021_ERROR;
	//HAL_Delay(20);
	if(HAL_I2C_Master_Receive(&hi2c1,SI7021_BASEADDRESS<<1,buffer,2,2000)!=SI7021_OK)
		return SI7021_ERROR;
	
	/*
	buffer[0]=CMD_HUM_HOLD;
	if(HAL_I2C_Mem_Read(&hi2c1,SI7021_BASEADDRESS<<1,buffer[0],I2C_MEMADD_SIZE_8BIT,buffer,2,1000)!=SI7021_OK)
		return SI7021_ERROR;
	*/
	raw_hum=buffer[0]<<8|buffer[1];
	*humidity=(((double)raw_hum*125.0/65536.0)-6.0);
	return SI7021_OK;
}
