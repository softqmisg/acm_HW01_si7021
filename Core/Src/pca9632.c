#include "i2c.h"
#include "pca9632.h"

//////////////////////////////////////////////////////////////////
HAL_StatusTypeDef pca9632_init()
{
		HAL_I2C_MspInit(&hi2c1);
		MX_I2C1_Init();
		uint8_t buffer[5];
		buffer[0]=PCA9632_CMD_MODE1;
		buffer[1]=0x01;//00000001//normal & accept all call
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
		return PCA9632_ERROR;
	HAL_Delay(20);
	
		buffer[0]=PCA9632_CMD_MODE2;
		buffer[1]=0x35;//00110101//blinking/invert/stop/totempole
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
		return PCA9632_ERROR;	
	HAL_Delay(20);
	//LEDS in pwm mode
	buffer[0]=PCA9632_CMD_LEDOUT;
//	buffer[1]=0x0B;//00001011: LED0 in LDR0=11 , LED1 in LDR1=10
	buffer[1]=0x0F;//00001011: LED0 in LDR0=11 , LED1 in LDR1=11
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
		return PCA9632_ERROR;	
	HAL_Delay(20);
	pca9632_setonepwm(LEDW,0);
	pca9632_setonepwm(LEDIR,0);
	return PCA9632_OK;
}
//////////////////////////////////////////////////////////////////
HAL_StatusTypeDef pca9632_ledoff(uint8_t leds)
{
/*	uint8_t buffer[2];
		buffer[0]=PCA9632_CMD_LEDOUT;
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,1,500)!=PCA9632_OK)
		return PCA9632_ERROR;
	HAL_Delay(20);
	
	if(HAL_I2C_Master_Receive(&hi2c1,PCA9632_BASEADDRESS<<1,&buffer[1],1,500)!=PCA9632_OK)
		return PCA9632_ERROR;
	if(leds&0x01) buffer[1] &=0xFC;//LED0
	if(leds&0x02) buffer[1] &=0xF3;//LED1
	if(leds&0x04) buffer[1] &=0xCF;//LED2
	if(leds&0x08) buffer[1] &=0x3F;//LED3
	
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
		return PCA9632_ERROR;	
	*/
	return PCA9632_OK;
}
//////////////////////////////////////////////////////////////////
HAL_StatusTypeDef pca9632_ledon(uint8_t leds)
{
	/*
	uint8_t buffer[2];
		buffer[0]=PCA9632_CMD_LEDOUT;
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,1,500)!=PCA9632_OK)
		return PCA9632_ERROR;
	HAL_Delay(20);
	
	if(HAL_I2C_Master_Receive(&hi2c1,PCA9632_BASEADDRESS<<1,&buffer[1],1,500)!=PCA9632_OK)
		return PCA9632_ERROR;
	if(leds&0x01) {buffer[1] &=0xFD;buffer[1] |=0x01;}//LED0
	if(leds&0x02) {buffer[1] &=0xF7;buffer[1] |=0x04;}//LED1
	if(leds&0x04) {buffer[1] &=0xDF;buffer[1] |=0x10;}//LED2
	if(leds&0x08) {buffer[1] &=0x7F;buffer[1] |=0x40;}//LED3
	
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
		return PCA9632_ERROR;	
	*/
	return PCA9632_OK;
}
//////////////////////////////////////////////////////////////////
HAL_StatusTypeDef pca9632_setonepwm(uint8_t leds,uint8_t duty)
{
	uint8_t buffer[2];
	uint16_t tmp;
	//suppose:bightess mode :MOD2:DMBLK=1
	//LED0(White) in 11(individual brightness and group dimming/blinking)
	// LED1(IR) in 10 (individual brightness can be controlled through its PWM1)
	//STEP2:PWM registers 0 to 3, PWMx � Individual brightness control registers
	switch(leds)
	{
		case 0x01:
				buffer[0]=PCA9632_CMD_PWM0;
			break;
		case  0x02:
				buffer[0]=PCA9632_CMD_PWM1;			
			break;
		case 0x04:
				buffer[0]=PCA9632_CMD_PWM2;			
			break;
		case 0x08:
				buffer[0]=PCA9632_CMD_PWM3;			
			break;
	}
	tmp=((uint16_t)duty*256/100);
	if(tmp>255) tmp=255;
	buffer[1]=(uint8_t) tmp;
	if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
		return PCA9632_ERROR;	
	HAL_Delay(20);	
return PCA9632_OK;	
}
////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef pca9632_setblinking(float second) //0->100 ,0=blink off,1=0.1second,100=10second
{	
	uint8_t buffer[2];
	uint8_t tenthsecond=(uint8_t)((float)second*10.0);
	//suppose LED0(White) in 11(individual brightness and group dimming/blinking)
	// LED1(IR) in 10 (individual brightness can be controlled through its PWM1)
	//STEP2:PWM registers 0 to 3, PWMx � Individual brightness control registers
	if(tenthsecond==0)
	{
		//goto LDR0=11,LDR2=10;
			buffer[0]=PCA9632_CMD_LEDOUT;
			buffer[1]=0x0A;//00001010: LED0 in LDR0=10 , LED1 in LDR1=10
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);
	}
	else
	{
		//goto LDR0=11,LDR2=10;
			buffer[0]=PCA9632_CMD_LEDOUT;
			//buffer[1]=0x0B;//00001011: LED0 in LDR0=11 , LED1 in LDR1=10
			buffer[1]=0x0F;//00001011: LED0 in LDR0=11 , LED1 in LDR1=11
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);
			
			buffer[0]=PCA9632_CMD_GRPFREQ;
			buffer[1]=(uint8_t)((uint16_t)24*tenthsecond/5);
			if(buffer[1]>=1) 
				buffer[1]--;
			else
				buffer[1]=0;
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);	
			
			buffer[0]=PCA9632_CMD_GRPPWM;
			buffer[1]=128;
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);	
			

	}
return PCA9632_OK;		
}
/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
/*
HAL_StatusTypeDef pca9632_setdimming(uint8_t tenthsecond) //0->100 ,0=blink off,1=0.1second,100=10second
{	
	uint8_t buffer[2];
	//suppose LED0(White) in 11(individual brightness and group dimming/blinking)
	// LED1(IR) in 10 (individual brightness can be controlled through its PWM1)
	//STEP2:PWM registers 0 to 3, PWMx � Individual brightness control registers
	if(tenthsecond==0)
	{
		//goto LDR0=11,LDR2=10;
			buffer[0]=PCA9632_CMD_LEDOUT;
			buffer[1]=0x0A;//00001010: LED0 in LDR0=10 , LED1 in LDR1=10
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);
	}
	else
	{
		//goto LDR0=11,LDR2=10;
			buffer[0]=PCA9632_CMD_LEDOUT;
			buffer[1]=0x0B;//00001011: LED0 in LDR0=11 , LED1 in LDR1=10
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);
			
			buffer[0]=PCA9632_CMD_GRPPWM;
			buffer[1]=128;
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);	

			
			buffer[0]=PCA9632_CMD_GRPFREQ;
			buffer[1]=(uint8_t)((uint16_t)24*tenthsecond/10);
			if(buffer[1]>=1) 
				buffer[1]--;
			else
				buffer[1]=0;
			if(HAL_I2C_Master_Transmit(&hi2c1,PCA9632_BASEADDRESS<<1,buffer,2,500)!=PCA9632_OK)
				return PCA9632_ERROR;	
			HAL_Delay(20);	
	}
return PCA9632_OK;		
}
*/
