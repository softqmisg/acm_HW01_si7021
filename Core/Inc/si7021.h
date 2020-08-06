#ifndef __SI7021_H__
	#define __SI7021_H__
	//#include "main.h"
	#include "stm32f4xx_hal.h"
	#define SI7021_BASEADDRESS	0x40
	//////////////////////////////////////////////////////////
	#define CMD_TEMP_NOHOLD		0xF3
	#define CMD_TEMP_HOLD		0xE3
	#define CMD_HUM_NOHOLD		0xF5
	#define	CMD_HUM_HOLD		0xE5
	//////////////////////////////////////////////////////////
	#define SI7021_OK		HAL_OK
	#define SI7021_ERROR	HAL_ERROR
	
void si7021_Init(void);	
HAL_StatusTypeDef si7021_read_temperature(double *temperature);
HAL_StatusTypeDef si7021_read_humidity(double *humidity);

	
#endif
