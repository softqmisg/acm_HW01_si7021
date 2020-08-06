/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "sevenseg.h"
#include "si7021.h"
#include "pca9632.h"
#include "vcnl4200.h"
#include "eeprom.h"
#include "astro.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DoorOpen_Pin GPIO_PIN_2
#define DoorOpen_GPIO_Port GPIOE
#define GPIO_2_Pin GPIO_PIN_3
#define GPIO_2_GPIO_Port GPIOE
#define GPIO_3_Pin GPIO_PIN_4
#define GPIO_3_GPIO_Port GPIOE
#define GPIO_5_Pin GPIO_PIN_6
#define GPIO_5_GPIO_Port GPIOE
#define ADC123_IN3_Pin GPIO_PIN_3
#define ADC123_IN3_GPIO_Port GPIOA
#define ON_OFF_Pin GPIO_PIN_5
#define ON_OFF_GPIO_Port GPIOA
#define CUR_DIR_Pin GPIO_PIN_6
#define CUR_DIR_GPIO_Port GPIOA
#define BTN_4_Pin GPIO_PIN_0
#define BTN_4_GPIO_Port GPIOB
#define BTN_3_Pin GPIO_PIN_7
#define BTN_3_GPIO_Port GPIOE
#define BTN_2_Pin GPIO_PIN_8
#define BTN_2_GPIO_Port GPIOE
#define BTN_1_Pin GPIO_PIN_9
#define BTN_1_GPIO_Port GPIOE
#define SEG_DIG4_Pin GPIO_PIN_10
#define SEG_DIG4_GPIO_Port GPIOE
#define SEG_B_Pin GPIO_PIN_11
#define SEG_B_GPIO_Port GPIOE
#define SEG_DIG3_Pin GPIO_PIN_12
#define SEG_DIG3_GPIO_Port GPIOE
#define SEG_DIG2_Pin GPIO_PIN_13
#define SEG_DIG2_GPIO_Port GPIOE
#define SEG_F_Pin GPIO_PIN_14
#define SEG_F_GPIO_Port GPIOE
#define SEG_A_Pin GPIO_PIN_15
#define SEG_A_GPIO_Port GPIOE
#define SEG_DIG1_Pin GPIO_PIN_14
#define SEG_DIG1_GPIO_Port GPIOB
#define SEG_G_Pin GPIO_PIN_8
#define SEG_G_GPIO_Port GPIOD
#define SEG_C_Pin GPIO_PIN_9
#define SEG_C_GPIO_Port GPIOD
#define SEG_DP_Pin GPIO_PIN_10
#define SEG_DP_GPIO_Port GPIOD
#define SEG_D_Pin GPIO_PIN_11
#define SEG_D_GPIO_Port GPIOD
#define SEG_E_Pin GPIO_PIN_12
#define SEG_E_GPIO_Port GPIOD
#define uSD_DETECT_Pin GPIO_PIN_3
#define uSD_DETECT_GPIO_Port GPIOD
#define VCNL4200_INT_Pin GPIO_PIN_7
#define VCNL4200_INT_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define FIRSTLOAD_EEPROM	0
#define TL_EEPROM			1
#define TH_EEPROM			2
#define DELTATEMP_EEPROM	3
#define HL_EEPROM			4
#define HH_EEPROM			5
#define BRIGHTW_EEPROM		6
#define BLINKW_EEPROM		7
#define BRIGHTIR_EEPROM		8
#define LEDONOFFREQ_EEPROM	9
#define PASSWORD_EEPROM		10
#define DUMMY_EEPROM		11
#define LAT_EERPROM			12
#define LONG_EEPROM			13


#define TL_MIN		0
#define TH_MAX		600
#define DELTAT_MAX	90
#define HL_MIN		0
#define HH_MAX		800
#define BLINKW_MAX	50//the value save in eeprom multiplied with 10, uint is second,90~9.0sec
#define BLINKW_MIN	0
#define NTCTL_MIN	0
#define NTCTL_MAX		1200
#define NTCTH_MIN	0
#define NTCTH_MAX		1200
#define LAT_MIN		200 //20.0
#define LAT_MAX		450//45.0
#define	LONG_MIN	400//40.0
#define LONG_MAX	650//65.0

#define TL_DEFAULT	100
#define TH_DEFAULT	400//700
#define DELTATEMP_DEFAULT 20
#define HL_DEFAULT	0
#define HH_DEFAULT	800
#define BRIGHTW_DEFAULT	5
#define BLINKW_DEFAULT	10//the value save in eeprom multiplied with 10	
#define BRIGHTIR_DEFAULT 0
#define PASSWORD_DEFAULT	0
#define NTCTL_DEFAULT	450
#define NTCTH_DEFAULT	500
#define LAT_DEFAULT		357//35.70//TEHRAN
#define LONG_DEFAULT	514//51.40//TEHRAN

#define FIRSTLOAD_ADDRESS	0x0000
#define TL_ADDRESS	0x0002
#define TH_ADDRESS	0x0004
#define DELTATEMP_ADDRESS	0x0006
#define HL_ADDRESS	0x0008
#define HH_ADDRESS	0x000a
#define BRIGHTW_ADDRESS	0x000c
#define BLINKW_ADDRESS	0x000e
#define BRIGHTIR_ADDRESS	0x0010
#define PASSWORD_ADDRESS	0x0012
#define DUMMY_ADDRESS			0x0014
#define NTCTL_ADDRESS		0x0016
#define NTCTH_ADDRESS		0x0018
#define LAT_ADDRESS			0x001a
#define LONG_ADDRESS		0x001c

#define LIMIT_RECORDS_COUNTER	60480	//7(day)*24(hr)*(3600)(s)/10(each 10 second)
#define LIMIT_WEEKS	999

void setpwm_value(uint16_t value);
extern float LedONOFFfreq_Value;
extern uint16_t FreqPWM_value;
extern uint16_t Duty_Value;

#define FAN_OFF()	HAL_GPIO_WritePin(ON_OFF_GPIO_Port,ON_OFF_Pin,GPIO_PIN_RESET)
#define FAN_ON()	HAL_GPIO_WritePin(ON_OFF_GPIO_Port,ON_OFF_Pin,GPIO_PIN_SET)
#define TEC_HOT()	HAL_GPIO_WritePin(CUR_DIR_GPIO_Port,CUR_DIR_Pin,GPIO_PIN_SET)
#define TEC_COLD()	HAL_GPIO_WritePin(CUR_DIR_GPIO_Port,CUR_DIR_Pin,GPIO_PIN_RESET)
#define FAN2_OFF()	HAL_GPIO_WritePin(GPIO_5_GPIO_Port,GPIO_5_Pin,GPIO_PIN_RESET)
#define FAN2_ON()	HAL_GPIO_WritePin(GPIO_5_GPIO_Port,GPIO_5_Pin,GPIO_PIN_SET)

#define REF_RES	10000.0
#define NTC_25	10000.0
#define B_coefficient	3445.00
#define KELVIN_TEMP	273.15
#define TEMP_25	(25.0+KELVIN_TEMP)

#define AVG_SLOPE	2.5
#define V_25			760.0


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
