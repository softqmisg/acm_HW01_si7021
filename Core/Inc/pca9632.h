#ifndef __PCA9632_H__
	#define __PCA9632_H__
	//#include "main.h"
	#define PCA9632_BASEADDRESS	0x60 //7bit address
	#define PCA9632_SWRST				0x03 //Software Reset Call	
	#define PCA9632_CMD_MODE1				0x00	//Mode register 1
	#define PCA9632_CMD_MODE2				0x01	//Mode register 2
	#define PCA9632_CMD_PWM0				0x02	//brightness control LED0
	#define	PCA9632_CMD_PWM1				0x03	//brightness control LED1
	#define	PCA9632_CMD_PWM2				0x04	//brightness control LED2
	#define	PCA9632_CMD_PWM3				0x05	//brightness control LED3
	#define	PCA9632_CMD_GRPPWM			0x06	//group duty cycle control
	#define	PCA9632_CMD_GRPFREQ			0x07	//group frequency
	#define	PCA9632_CMD_LEDOUT			0x08	//LED output state
	#define	PCA9632_CMD_SUBADR1			0x09	//I2C-bus subaddress 1
	#define	PCA9632_CMD_SUBADR2			0x0A	//I2C-bus subaddress 2
	#define	PCA9632_CMD_SUBADR3			0x0B	//I2C-bus subaddress 3
	#define	PCA9632_CMD_ALLCALLADR	0x0C	//LED All Call I2C-bus address
	#define LEDW	0x01
	#define LEDIR	0x02
	
	#define PCA9632_OK	HAL_OK
	#define PCA9632_ERROR	HAL_ERROR
	HAL_StatusTypeDef pca9632_init();
	HAL_StatusTypeDef pca9632_setonepwm(uint8_t leds,uint8_t duty);
	HAL_StatusTypeDef pca9632_setblinking(float second); //0->100 ,0=blink off,1=0.1second,100=10second

	

	
#endif