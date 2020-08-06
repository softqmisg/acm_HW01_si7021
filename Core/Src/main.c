/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "sdio.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_1	0
#define BTN_2	1
#define BTN_3	2
#define BTN_4	3


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float TL_Value;
float TH_Value;
float DELTAT_Value;
float HL_Value;
float HH_Value;
uint16_t BrightW_Value;
float BlinkW_Value;
uint16_t BrightIR_Value;
uint16_t Password_Value;
float NTCTL_Value;
float NTCTH_Value;
double LAT_Value;
double LONG_Value;
////////////////////////////////////////////////////////////
/*void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
}
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
}
*/
/////////////////////////////////////////////////////////////////
/*
DWORD get_fattime(void)
{
	RTC_TimeTypeDef c_time;
	RTC_DateTypeDef c_date;
	HAL_RTC_GetTime(&hrtc,&c_time,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&c_date,RTC_FORMAT_BIN);
return	((DWORD)c_date.Year << 25 | (DWORD)c_date.Month << 21 | (DWORD)c_date.Date << 16 | (DWORD) c_time.Hours <<11 | (DWORD) c_time.Minutes <<5 | (DWORD) (c_time.Seconds>>1)<<0);
}
*/
///////////////////////////////////////////////////////////
#define MAX_PULSE	576

void setpwm_value(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
		uint32_t current_period=htim9.Init.Period;
		
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint16_t)((uint32_t)value*current_period)/100;//(uint16_t)((uint32_t)value*MAX_PULSE)/100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  
}
//////////////////////////////////////////////////////
void setfreqpwm_value(uint16_t freqpwm)
{
	uint32_t TIMER_CLOCK=HAL_RCC_GetPCLK1Freq();
	if((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
		{
			TIMER_CLOCK=TIMER_CLOCK*2;
		}	
	TIMER_CLOCK=TIMER_CLOCK/1000;
	uint32_t period=TIMER_CLOCK/freqpwm;
  
	HAL_TIM_PWM_DeInit(&htim9);
	htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1000-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = period-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }	
//	setpwm_value(Duty_Value);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	
}
//////////////////////////////////////////////////////
void setledonofffreq_value(float freq)
{

	uint32_t TIMER_CLOCK=HAL_RCC_GetPCLK1Freq();
	if((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
		{
			TIMER_CLOCK=TIMER_CLOCK*2;
		}	
	TIMER_CLOCK=TIMER_CLOCK/3000;
	if(freq)
	{
		uint32_t period=TIMER_CLOCK/(uint32_t)(freq*10.0);
			
		HAL_TIM_Base_DeInit(&htim11);	
		htim11.Init.Prescaler = 30000-1;
		htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim11.Init.Period = period-1;
		htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
		{
			Error_Handler();
		}	
//		setpwm_value(Duty_Value);		
		HAL_TIM_Base_Start_IT(&htim11);
		
	}
	else// leds allway on
	{
		HAL_TIM_Base_Stop_IT(&htim11);

	}
}	
/////////////////////////////////////////////////////////////////////////////////////
GPIO_TypeDef *key_ports[]={BTN_1_GPIO_Port,BTN_2_GPIO_Port,BTN_3_GPIO_Port,BTN_4_GPIO_Port};
uint16_t key_pins[]={BTN_1_Pin,BTN_2_Pin,BTN_3_Pin,BTN_4_Pin};
uint8_t ReadShort_KEY(uint8_t keynum)
{
	if(!HAL_GPIO_ReadPin(key_ports[keynum],key_pins[keynum]))
	{
		HAL_Delay(100);
		if(HAL_GPIO_ReadPin(key_ports[keynum],key_pins[keynum]))
		{
			return 1;
		}
	}
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////
uint8_t ReadLong_KEY(uint8_t keynum)
{
	if(!HAL_GPIO_ReadPin(key_ports[keynum],key_pins[keynum]))
	{
		HAL_Delay(1000);
		if(!HAL_GPIO_ReadPin(key_ports[keynum],key_pins[keynum]))
		{
			return 1;
		}
	}
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////
void update_values(void)
{
		uint16_t read_eeprom;
	
		if((EE_ReadVariable(TL_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E1-R");
    }		
		TL_Value=(float)read_eeprom/10.0;
		HAL_Delay(50);
    if((EE_ReadVariable(TH_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E2-R");
    }		
		TH_Value=(float)read_eeprom/10.0;		
		HAL_Delay(50);
    if((EE_ReadVariable(HL_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E3-R");
    }		
		HL_Value=(float)read_eeprom/10.0;		
		HAL_Delay(50);
    if((EE_ReadVariable(HH_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E4-R");
    }		
		HH_Value=(float)read_eeprom/10.0;		
		HAL_Delay(50);
    if((EE_ReadVariable(DELTATEMP_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E7-R");
    }		
		DELTAT_Value=(float)read_eeprom/10.0;			
		HAL_Delay(50);	
    if((EE_ReadVariable(BRIGHTW_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E5-R");
    }		
		BrightW_Value=read_eeprom;	
		HAL_Delay(50);
    if((EE_ReadVariable(BLINKW_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E8-R");
    }		
		BlinkW_Value=(float)read_eeprom/10.0;			
		HAL_Delay(50);	
		
    if((EE_ReadVariable(BRIGHTIR_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E5-R");
    }		
		BrightIR_Value=read_eeprom;	
		HAL_Delay(50);

    if((EE_ReadVariable(PASSWORD_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E6-R");
    }		
		Password_Value=read_eeprom;			
		HAL_Delay(50);
    if((EE_ReadVariable(NTCTL_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E7-R");
    }		
		NTCTL_Value=(float)read_eeprom/10.0;
		HAL_Delay(50);
    if((EE_ReadVariable(NTCTH_ADDRESS,  &read_eeprom)) != HAL_OK)
    {
			print_segment("E8-R");
    }		
		NTCTH_Value=(float)read_eeprom/10.0;
		HAL_Delay(50);

	if((EE_ReadVariable(LAT_ADDRESS,  &read_eeprom)) != HAL_OK)
	{
			print_segment("E9-R");
	}
		LAT_Value=(double)read_eeprom/10.0;
		HAL_Delay(50);
	if((EE_ReadVariable(LONG_ADDRESS,  &read_eeprom)) != HAL_OK)
	{
			print_segment("E10-R");
	}
		LONG_Value=(double)read_eeprom/10.0;
		HAL_Delay(50);
}
/////////////////////////////////////////////////////////////////////////////////////
void reset_keys()
{
				if(KEYSW1_longpressed)
				{
					KEYSW1_longpressed=0;
				}
				if(KEYSW4_shortpressed)
				{
					KEYSW4_shortpressed=0;
				}
				if(KEYSW4_longpressed)
				{
					KEYSW4_longpressed=0;
				}			
}
///////////////////////////////////////////////////////////////////////////////////
void print_pass(char *str,uint8_t index,uint8_t dot)
{
	char str_disp[10];
	uint8_t i,j;
	for( i=0;i<=index;i++)
	{
		str_disp[i]=str[i];
	}
	if(dot)
	{
		str_disp[i]='.';
		for( j=i+1;j<=4;j++)
		{
			str_disp[j]=str[i];i++;
		}
	}
	else
	{
		for( j=i;j<=3;j++)
		{
			str_disp[j]=str[j];
		}
	}
	str_disp[j]=0;
	print_segment(str_disp);
}
///////////////////////////////////////////////////////////////////////////////////
uint8_t flag_rtc_1s=0;
uint8_t flag_rtc_showtemp=1;
uint8_t flag_rtc_blink=0;
uint8_t counter_rtc_showtemp=0;
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	flag_rtc_1s=1;
	flag_rtc_blink=1-flag_rtc_blink;
	counter_rtc_showtemp++;
	if(counter_rtc_showtemp>=5)
	{
		flag_rtc_showtemp=1;
		counter_rtc_showtemp=0;
	}
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FRESULT File_log(char* path,char *wstr) 
{
	uint32_t  byteswritten;
	FIL MyFile;
	FRESULT fr;
	if((fr=f_mount(&SDFatFS, (TCHAR const*)SDPath, 1)) != FR_OK)
	{
		print_segment("E101");
		HAL_Delay(1000);
		return fr;
	}	
	HAL_Delay(10);
	fr=f_open(&MyFile, (const TCHAR*)path,  FA_OPEN_APPEND|FA_WRITE);
	if(fr!=FR_OK)
	{
		print_segment("E102");
		HAL_Delay(1000);
		return fr;		
	}
	HAL_Delay(10);
	fr = f_write(&MyFile, wstr, strlen(wstr), (void *)&byteswritten);	
	if(fr!=FR_OK)
	{
		print_segment("E103");
		HAL_Delay(1000);
		return fr;		
	}	
	HAL_Delay(10);
	f_close(&MyFile);	
	HAL_Delay(10);
	f_mount(0, "", 1);
	return fr;
}
/////////////////////////////////////////////////////////////////////////////////////////////
void generate_filename(RTC_DateTypeDef date,RTC_TimeTypeDef time,uint32_t *counter,char *filename)
{
	char file_str[500];
	FRESULT fr;
	
	*counter=(uint32_t)*counter+1;
	if(*counter>LIMIT_RECORDS_COUNTER)
	{
		*counter=1;
	}

	if(*counter==1)
	{
			sprintf(filename,"LOG %04d-%02d-%02d %02d-%02d-%02d.CSV",date.Year+1980,date.Month,date.Date,time.Hours,time.Minutes,time.Seconds);
			sprintf(file_str,"counter,DATE,TIME,Env_temperature,Env_humidity,Bright LED,Blink time,Bright IR,LDR,DOOR,TEC,FAN,NTC\n");
			if((fr=File_log(filename,file_str))!=FR_OK)
			{
				print_segment("E104");
				HAL_Delay(1000);
			}	
			HAL_Delay(10);
	}
}
//////////////////////////////////////////////////
#define NUM_adcDMA 3
#define NUM2_adcDMA	8
uint16_t rawadcValues[NUM_adcDMA];
uint16_t valid_adc[3];
uint8_t convCompleted=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/*	valid_adc[0]=0;
		valid_adc[1]=0;
		valid_adc[2]=0;
		for(uint8_t i=0;i<NUM_adcDMA;i+=3)
		{
			valid_adc[0]=(uint32_t)valid_adc[0]+(uint32_t)rawadcValues[i];
			valid_adc[1]=(uint32_t)valid_adc[1]+(uint32_t)rawadcValues[i+1];
			valid_adc[2]=(uint32_t)valid_adc[2]+(uint32_t)rawadcValues[i+1];
		}
		valid_adc[0]=(uint32_t)valid_adc[0]/NUM2_adcDMA;
		valid_adc[1]=(uint32_t)valid_adc[1]/NUM2_adcDMA;				
		valid_adc[2]=(uint32_t)valid_adc[2]/NUM2_adcDMA;
*/		
		valid_adc[0]=rawadcValues[0];
		valid_adc[1]=rawadcValues[1];
		valid_adc[2]=rawadcValues[2];		
	
	convCompleted=1;
}
///////////////////////////////////////////////////
uint16_t data_vcnl;
double LDR_resistance;
double NTC_Resistance,NTC_Env_temperature,NTC_Centigrade;
double Env_temperature,Env_humidity;
double Micro_temperature;
uint8_t TEC_overtemp=0;
Time_t sunrise_t,sunset_t,noon_t;
uint8_t daylightsaving=0;
//uint16_t raw_LDR,raw_Env_temperature,NTC_Centigrade_uint16;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	FRESULT fr;
	float prev_float;
	double prev_double;
	uint16_t prev_uint16t;
	uint16_t tmp_eeprom;
	uint16_t tmp_uint16t;
	uint8_t blink=0,index_disp=0,passsegment_value=0;
	uint16_t counter=0;
	char str_disp[10];
	char disp_pass[4]={'-','-','-','-'};
	uint16_t pwm_percent=0;
	float avg_Env_temperature=0.0,avg_Env_humidity=0.0;
	uint16_t avg_counter=0;
	float prev_Env_temperature=0.0,prev_Env_humidity,Delta_T;
	enum {MAIN_MENU=0,PASS_MENU,TEMPL_MENU,TEMPH_MENU,DELTAT_MENU,NTCTL_MENU,NTCTH_MENU,HUML_MENU,HUMH_MENU,BLINKW_MENU,BRIGHTW_MENU,BRIGHTIR_MENU,SETYEAR_MENU,SETMONTH_MENU,SETDAY_MENU,SETHOUR_MENU,SETMIN_MENU,SETSEC_MENU,SETLAT_MENU,SETLONG_MENU,SETPASS_MENU} Menu_choice=MAIN_MENU;
	enum {DISP_TEMP=0,DISP_HUM,DISP_BRIGHTW,DISP_BLINKW,DISP_BRIGHTIR,DISP_ERROR,DISP_NTC,DISP_SUNRISE,DISP_SUNSET} cur_Disp=DISP_TEMP,last_disp;
	///////////////////////////////////////////////////////////////////
	FIL File_HUTEMP,File_Door; 
	FIL	FIL_log;
	//FATFS SDFatFs;
	//static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */
	//FRESULT res;
  //uint32_t byteswritten, bytesread;                     /* File write/read counts */
  //uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  //uint8_t rtext[100];                                   /* File read buffer */
	char usb_str[80];
	char file_str[500];
	char filelog_name[255];
	uint8_t is_sddetected=0;
	uint32_t counterlog_file=0;
	uint8_t door_state=0,pre_door_state=0;//0=close ,1=open
	uint32_t weeklog_counter=1;
	uint8_t tmp_err;
	RTC_TimeTypeDef cur_time,tmp_time;
	RTC_DateTypeDef	cur_date={0},tmp_date,prev_date={0};
	uint16_t raw_LDR,raw_Env_temperature,NTC_Centigrade_uint16;
	enum {LDR_NONE=0,LDR_BROKE, LDR_NIGHT,LDR_DAY} LDR_State=LDR_NONE;

	uint8_t NTC_valid=0;
	uint8_t set_pwm_state=1;
	uint16_t counter_LDR=0;
	char  TEC_chstate[10];
	char FAN_chstate[10];
	char DOOR_chstate[10];
	Date_t date;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM10_Init();
  MX_TIM9_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();
  MX_TIM11_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	init_sevenseg();
//	/////////////////////////////////////////////////
/*
while(1)
{
	HAL_IWDG_Refresh(&hiwdg);
	FAN2_ON();
	HAL_Delay(3000);
	FAN2_OFF();
	HAL_Delay(3000);
}
*/
/*FAN_ON();
TEC_COLD();
while(1)
{
	HAL_IWDG_Refresh(&hiwdg);
}
*/
/*
while(1)
{
	HAL_IWDG_Refresh(&hiwdg);
	//FAN_OFF();
	//HAL_Delay(5000);
	FAN_ON();
	HAL_Delay(5000);
}
*/
//////////////////////////////////////////////////
	if(vcnl4200_init()!=VCNL4200_OK)
	{
		print_segment("E701");
		HAL_Delay(3000);
		vcnl4200_init();
	}
	HAL_Delay(200);
	////////////////////////////////////////////////////////////////
	if((tmp_err=pca9632_init())!=PCA9632_OK)
	{
		print_segment("E501");
		HAL_Delay(1000);
		pca9632_init();
	}	
	/////////////////////////////////////////////////////////////////
//	if(si7021_Init()!=SI7021_OK)
//	{
//		cur_Disp=DISP_ERROR;
//	}
	si7021_Init();
	////////////////////////////////////////////////////////////
	sprintf(SDPath,"");
	print_segment("----");
	HAL_Delay(1000);
	print_segment("EEEE");
	
	HAL_FLASH_Unlock();
	HAL_Delay(50);
  if( EE_Init() != EE_OK)
  {
    print_segment("ER00");
		while(1);
  }	
	if(EE_WriteVariable(DUMMY_ADDRESS,9567)!=HAL_OK)
  {
		print_segment("ER01");
		while(1);
  }	
  if((EE_ReadVariable(DUMMY_ADDRESS,  &tmp_eeprom)) != HAL_OK)
  {
		print_segment("ER02");
		while(1);
  }		
	sprintf(str_disp,"%04d",tmp_eeprom);
	print_segment(str_disp);
	
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(100);


	//HAL_RTCEx_DeactivateTamper(&hrtc, RTC_TAMPER_1);
	//__HAL_RTC_TAMPER_CLEAR_FLAG(&hrtc, RTC_FLAG_TAMP1F);

	if((HAL_RTCEx_BKUPRead (&hrtc,RTC_BKP_DR1) !=0x0001)||(!KEYSW1_GET()&&!KEYSW2_GET()))
	{
		HAL_IWDG_Refresh(&hiwdg);
		 HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0001);

		
		if(EE_WriteVariable(TL_ADDRESS,TL_DEFAULT)!=HAL_OK)
		{
			print_segment("ER04");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(TH_ADDRESS,TH_DEFAULT)!=HAL_OK)
		{
			print_segment("ER05");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(DELTATEMP_ADDRESS,DELTATEMP_DEFAULT)!=HAL_OK)
		{
			print_segment("ER09");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(HL_ADDRESS,HL_DEFAULT)!=HAL_OK)
		{
			print_segment("ER06");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(HH_ADDRESS,HH_DEFAULT)!=HAL_OK)
		{
			print_segment("ER07");
			while(1);
		}
		HAL_Delay(50);

		if(EE_WriteVariable(BRIGHTW_ADDRESS,BRIGHTW_DEFAULT)!=HAL_OK)
		{
			print_segment("ER08");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(BLINKW_ADDRESS,BLINKW_DEFAULT)!=HAL_OK)
		{
			print_segment("ER11");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(BRIGHTIR_ADDRESS,BRIGHTIR_DEFAULT)!=HAL_OK)
		{
			print_segment("ER11");
			while(1);
		}
		HAL_Delay(50);		
		
		if(EE_WriteVariable(PASSWORD_ADDRESS,PASSWORD_DEFAULT)!=HAL_OK)
		{
			print_segment("ER10");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(NTCTL_ADDRESS,NTCTL_DEFAULT)!=HAL_OK)
		{
			print_segment("ER12");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(NTCTH_ADDRESS,NTCTH_DEFAULT)!=HAL_OK)
		{
			print_segment("ER13");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(LAT_ADDRESS,LAT_DEFAULT)!=HAL_OK)
		{
			print_segment("ER14");
			while(1);
		}
		HAL_Delay(50);
		if(EE_WriteVariable(LONG_ADDRESS,LONG_DEFAULT)!=HAL_OK)
		{
			print_segment("ER15");
			while(1);
		}
		HAL_Delay(50);
	}
	//HAL_PWR_DisableBkUpAccess();
	update_values();
	print_segment("RRRR");
	HAL_Delay(1000);
	
	//////////////////////////////////////////////////////////////
	if(BSP_SD_IsDetected()==SD_PRESENT && (is_sddetected==0))
	{
		HAL_Delay(500);
		if(BSP_SD_IsDetected()==SD_PRESENT && (is_sddetected==0))
		{
				HAL_Delay(500);
				is_sddetected=1;
		}
	}
	//////////////////////////////////////////////////////////////////
	FAN_OFF();
	FAN2_OFF();
	sprintf(FAN_chstate,"OFF");	
	sprintf(TEC_chstate,"OFF");
	sprintf(DOOR_chstate,"---");
	

	LDR_resistance=0.0;
	NTC_Resistance=0.0;
	//////////////////////////////////////////////////////////////
	if(si7021_read_humidity(&Env_humidity)!=SI7021_OK)
	{
		si7021_Init();
		cur_Disp=DISP_ERROR;
	}
	prev_Env_humidity=Env_humidity;
	if(si7021_read_temperature(&Env_temperature)!=SI7021_OK)
	{
		si7021_Init();
		cur_Disp=DISP_ERROR;
	}
	prev_Env_temperature=Env_temperature;


	/////////////////////////////////////////////////////////////////////////
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawadcValues, NUM_adcDMA);
	flag_1s=flag_rtc_1s=1;
	last_disp=cur_Disp=DISP_TEMP;
	uint8_t winter=0; //1=summer 0=winter
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			HAL_IWDG_Refresh(&hiwdg);
			//////////////////////////Read RTC////////////////////////////////////////
			if(flag_rtc_1s)
			{
				flag_rtc_1s=0;
				////////////////////////Get Set time date & day light saving//////////////////
				HAL_RTC_GetDate(&hrtc,&cur_date,RTC_FORMAT_BIN);
				date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
				if(Astro_daylighsaving(date) && date.month==RTC_MONTH_MARCH)
					winter=0;

				if(!winter && Astro_daylighsaving(date) && !(hrtc.Instance->CR & RTC_STOREOPERATION_SET)) //summer time && BKP=0 =>start of summer time
				{
					tmp_time.Hours = 1;
					tmp_time.Minutes = 0;
					tmp_time.Seconds = 0;
					tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
					HAL_RTC_SetTime(&hrtc, &tmp_time, RTC_FORMAT_BIN);
				}
				else if(!Astro_daylighsaving(date) && (hrtc.Instance->CR & RTC_STOREOPERATION_SET)) //winter & BKP=1 =>start of winter
				{
					winter=1;
					tmp_time.Hours = 23;
					tmp_time.Minutes = 0;
					tmp_time.Seconds = 0;
					tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
					HAL_RTC_SetTime(&hrtc, &tmp_time, RTC_FORMAT_BIN);

					tmp_date.Month = RTC_MONTH_SEPTEMBER;
					tmp_date.Date = 20;
					tmp_date.Year = cur_date.Year;
					HAL_RTC_SetDate(&hrtc, &tmp_date, RTC_FORMAT_BIN);
				}
				HAL_RTC_GetTime(&hrtc,&cur_time,RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc,&cur_date,RTC_FORMAT_BIN);
				///////////////////////////////////////////////////////////////
				while(!convCompleted);
				convCompleted=0;
				/////////////////////find MICRO temperature////////////////////
				Micro_temperature=((double)valid_adc[2]*3300.0/4096.0-V_25)/AVG_SLOPE+25;
				/////////////////////Check NTC////////////////////////////////
				raw_Env_temperature=(uint16_t)valid_adc[0];//HAL_ADC_GetValue(&hadc1); //ADC123_IN0
	
				if(raw_Env_temperature>4000)
				{
					NTC_Centigrade=NTCTL_Value-1.0;
					NTC_Centigrade_uint16=0;
					NTC_valid=0;
				}
				else
				{
				//raw_Env_temperature=110;//ADC123_IN0
					NTC_valid=1;
					NTC_Resistance =  (double)raw_Env_temperature/(4095.00-(double)raw_Env_temperature);		
					NTC_Env_temperature = (1.0/TEMP_25)+(1.0/(double)B_coefficient)*log((double)NTC_Resistance);
					NTC_Centigrade = (1.0/NTC_Env_temperature) - 273.15;	// convert kelvin to �C		
					NTC_Centigrade_uint16=(uint16_t)((double)NTC_Centigrade*10.0);				
				}

				/////////////////////Check LDR////////////////////////////////

				raw_LDR=(uint16_t)valid_adc[1];//HAL_ADC_GetValue(&hadc1);//ADC123_IN3
				//raw_LDR=200;//ADC123_IN3
				//HAL_ADC_Stop(&hadc1);
				
				LDR_resistance=(4.7*(float)raw_LDR)/(4095.0-(float)raw_LDR);
				////////////////////check Astro clock///////////////////////////
				HAL_IWDG_Refresh(&hiwdg);
				if(cur_date.Date!=prev_date.Date || cur_date.Month != prev_date.Month || cur_date.Year!=prev_date.Year)
				{
					date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					Astro_sunRiseSet(LAT_Value, LONG_Value, UTCOEFF_TEHRAN, date,&sunrise_t,&noon_t,&sunset_t,1);
				}
				prev_date=cur_date;
				//////////////////////////apply LED parameters////////////////////////////////////////
				if(Astro_CheckDayNight(cur_time,sunrise_t,sunset_t)==ASTRO_DAY) //DAY
				{
					if(LDR_State!=LDR_DAY)
					{
						LDR_State=LDR_DAY;
						if(pca9632_setonepwm(LEDIR,1)!=PCA9632_OK)
						{
							if(Menu_choice==MAIN_MENU)
							{							
								print_segment("E501");
								HAL_Delay(1000);
								pca9632_init();
							}
						}						
						if(pca9632_setonepwm(LEDW,1)!=PCA9632_OK)
						{
							if(Menu_choice==MAIN_MENU)
							{							
								print_segment("E501");
								HAL_Delay(1000);
								pca9632_init();
							}
						}
						if(pca9632_setblinking(BlinkW_Value)!=PCA9632_OK)
						{
							if(Menu_choice==MAIN_MENU)
							{							
								print_segment("E501");
								HAL_Delay(1000);
								pca9632_init();
							}
						}						

					}
				}
				else //NIGHT
				{
					if(LDR_State!=LDR_NIGHT)
					{
						LDR_State=LDR_NIGHT;
						if(pca9632_setonepwm(LEDIR,BrightW_Value)!=PCA9632_OK)
						{
							if(Menu_choice==MAIN_MENU)
							{
								print_segment("E501");
								HAL_Delay(1000);
								pca9632_init();
							}
						}
						if(pca9632_setonepwm(LEDW,BrightW_Value)!=PCA9632_OK)
						{
							if(Menu_choice==MAIN_MENU)
							{
								print_segment("E501");
								HAL_Delay(1000);
								pca9632_init();
							}
						}
						if(pca9632_setblinking(0)!=PCA9632_OK)
						{
							if(Menu_choice==MAIN_MENU)
							{
								print_segment("E501");
								HAL_Delay(1000);
								pca9632_init();
							}
						}

					}

				}

				counter_LDR=(uint16_t)counter_LDR+1;
				if(counter_LDR>=120)
				{
					counter_LDR=0;
					LDR_State=LDR_NONE;
					HAL_I2C_MspInit(&hi2c1);
					MX_I2C1_Init();
				}	
			}
			/////////////////////////////////uSD trap/////////////////////////////////
			if((BSP_SD_IsDetected()==SD_PRESENT) && (is_sddetected==0))
			{
				HAL_Delay(100);
				if((BSP_SD_IsDetected()==SD_PRESENT) && (is_sddetected==0))
				{
						HAL_Delay(500);
						BSP_SD_Init();
						is_sddetected=1;
				}
			}

			if(BSP_SD_IsDetected()==SD_NOT_PRESENT)
			{
				HAL_Delay(100);
				if(BSP_SD_IsDetected()==SD_NOT_PRESENT)
				{
					is_sddetected=0;
					HAL_SD_DeInit(&hsd);
					if(f_mount(0, "", 0) != FR_OK )
					{
						SDFatFS.fs_type=0;
					}
					counterlog_file=0;
				}
			}
			///////////////////////////////////Door open trap////////////////////////////////////
			if((HAL_GPIO_ReadPin(VCNL4200_INT_GPIO_Port,VCNL4200_INT_Pin)==GPIO_PIN_SET) && (door_state==0))// door is openning
			{
				HAL_Delay(50);
				if((HAL_GPIO_ReadPin(VCNL4200_INT_GPIO_Port,VCNL4200_INT_Pin)==GPIO_PIN_SET) && (door_state==0))//door is opened
				{
					door_state=1;
					sprintf(DOOR_chstate,"OPEN");
					if(is_sddetected)
					{
						generate_filename(cur_date,cur_time,&counterlog_file,filelog_name);
						if(NTC_valid)
							sprintf(file_str,"%5ld,%04d-%02d-%02d,%02d:%02d:%02d,%5.1f,%4.1f,%03d,%4.1f,%03d,%4.1f,%s,%s,%s,%4.1f\n",
														counterlog_file,
														cur_date.Year+1980,cur_date.Month,cur_date.Date,
														cur_time.Hours,cur_time.Minutes,cur_time.Seconds,
														Env_temperature,Env_humidity,BrightW_Value,BlinkW_Value,BrightIR_Value,
														LDR_resistance,DOOR_chstate,TEC_chstate,FAN_chstate,NTC_Centigrade);
						else
							sprintf(file_str,"%5ld,%04d-%02d-%02d,%02d:%02d:%02d,%5.1f,%4.1f,%03d,%4.1f,%03d,%4.1f,%s,%s,%s,---\n",
														counterlog_file,
														cur_date.Year+1980,cur_date.Month,cur_date.Date,
														cur_time.Hours,cur_time.Minutes,cur_time.Seconds,
														Env_temperature,Env_humidity,BrightW_Value,BlinkW_Value,BrightIR_Value,
														LDR_resistance,DOOR_chstate,TEC_chstate,FAN_chstate);
						if((fr=File_log(filelog_name,file_str))!=FR_OK)
						{
							print_segment("E104");
							HAL_Delay(1000);
						}
					}
					else
					{
						if(Menu_choice==MAIN_MENU)
						{
							print_segment("EE.5D");
							HAL_Delay(1000);
						}
					}
				}
			}
			if((HAL_GPIO_ReadPin(VCNL4200_INT_GPIO_Port,VCNL4200_INT_Pin)==GPIO_PIN_RESET) && (door_state==1))// door is closing
			{
				HAL_Delay(50);
				if((HAL_GPIO_ReadPin(VCNL4200_INT_GPIO_Port,VCNL4200_INT_Pin)==GPIO_PIN_RESET) && (door_state==1))//door is closed
				{
					door_state=0;
					sprintf(DOOR_chstate,"CLOSE");

					if(is_sddetected)
					{
						generate_filename(cur_date,cur_time,&counterlog_file,filelog_name);
						if(NTC_valid)
							sprintf(file_str,"%5ld,%04d-%02d-%02d,%02d:%02d:%02d,%5.1f,%4.1f,%03d,%4.1f,%03d,%4.1f,%s,%s,%s,%4.1f\n",
														counterlog_file,
														cur_date.Year+1980,cur_date.Month,cur_date.Date,
														cur_time.Hours,cur_time.Minutes,cur_time.Seconds,
														Env_temperature,Env_humidity,BrightW_Value,BlinkW_Value,BrightIR_Value,
														LDR_resistance,DOOR_chstate,TEC_chstate,FAN_chstate,NTC_Centigrade);
						else
							sprintf(file_str,"%5ld,%04d-%02d-%02d,%02d:%02d:%02d,%5.1f,%4.1f,%03d,%4.1f,%03d,%4.1f,%s,%s,%s,---\n",
														counterlog_file,
														cur_date.Year+1980,cur_date.Month,cur_date.Date,
														cur_time.Hours,cur_time.Minutes,cur_time.Seconds,
														Env_temperature,Env_humidity,BrightW_Value,BlinkW_Value,BrightIR_Value,
														LDR_resistance,DOOR_chstate,TEC_chstate,FAN_chstate);
						if((fr=File_log(filelog_name,file_str))!=FR_OK)
						{
							print_segment("E104");
							HAL_Delay(1000);
						}
					}
					else
					{
						if(Menu_choice==MAIN_MENU)
						{
							print_segment("EE.5D");
							HAL_Delay(1000);
						}
					}
				}
			}
			
		////////////////////////////////////////////////////////////////////	
		switch(Menu_choice)
		{
			/////////////////////////////////////////////////////////CASE MAIN_MENU/////////////////////////////////////////////////////////////////
			case MAIN_MENU:
				if(flag_1s)
				{
					flag_1s=0;
					if(si7021_read_humidity(&Env_humidity)!=SI7021_OK)
					{
						si7021_Init();
						cur_Disp=DISP_ERROR;
					}
					else
						cur_Disp=last_disp;
					if(si7021_read_temperature(&Env_temperature)!=SI7021_OK)
					{
						si7021_Init();
						cur_Disp=DISP_ERROR;
					}
					else
						cur_Disp=last_disp;



					avg_Env_temperature+=(float)Env_temperature;
					avg_Env_humidity+=(float)Env_humidity;
					avg_counter++;
					
					switch(cur_Disp)
					{
						case DISP_NTC:
							if(NTC_valid)
								sprintf(str_disp,"N%03.0f",NTC_Centigrade);
							else
								sprintf(str_disp,"N---");
						break;
						case DISP_TEMP:
							if(flag_rtc_showtemp)
							{
								flag_rtc_showtemp=0;
								sprintf(str_disp,"%5.1f",Env_temperature);
							}
							else
							{
								if(flag_rtc_blink)
								{
									sprintf(str_disp,"%02d.%02d",cur_time.Hours,cur_time.Minutes);
								}
								else
								{
									sprintf(str_disp,"%02d%02d",cur_time.Hours,cur_time.Minutes);
								}
							}
						break;
						case DISP_HUM:
							sprintf(str_disp,"H%4.1f",Env_humidity);
						break;
						case DISP_BLINKW:
							sprintf(str_disp,"L%4.1f",BlinkW_Value);							
							break;
						case DISP_BRIGHTW:
							sprintf(str_disp,"T%3d",BrightW_Value);
							break;						
						case DISP_BRIGHTIR:
							sprintf(str_disp,"R%3d",BrightIR_Value);
							break;
						case DISP_ERROR:
							sprintf(str_disp,"E207");
							break;
					}
					print_segment(str_disp);
					////////////////////////////////////Control Algorithm////////////////////////////////
					if(0)//(NTC_Centigrade>NTCTH_Value))
					{
						FAN2_ON();
						sprintf(FAN_chstate,"ON");						
						FAN_OFF();
						sprintf(TEC_chstate,"OFF");	
						TEC_overtemp	=1;
					}
					else if(0)//NTC_Centigrade<NTCTH_Value  && NTC_Centigrade>NTCTL_Value && TEC_overtemp)
					{
						FAN2_ON();
						sprintf(FAN_chstate,"ON");						
						FAN_OFF();
						sprintf(TEC_chstate,"OFF");							
					}
					else //if((NTC_Centigrade<NTCTH_Value &&  TEC_overtemp==0)|| (NTC_Centigrade<NTCTL_Value))
					{
						TEC_overtemp=0;
						if(cur_Disp==DISP_ERROR)
						{
							FAN2_OFF();
							sprintf(FAN_chstate,"OFF");						
							FAN_OFF();
							sprintf(TEC_chstate,"OFF");

						}
						else
						{
							Delta_T=Env_temperature-prev_Env_temperature;
							if(Env_temperature>TH_Value) //s1
							{
								FAN2_ON();
								sprintf(FAN_chstate,"ON");
								FAN_ON();
								TEC_COLD();
								sprintf(TEC_chstate,"COLD");
							}
							else if(Env_temperature<=(TH_Value-DELTAT_Value) && Env_temperature>(TL_Value+DELTAT_Value))//s4
							{
								FAN2_OFF();
								sprintf(FAN_chstate,"OFF");						
								FAN_OFF();
								sprintf(TEC_chstate,"OFF");
							}
							else if((Delta_T>0.2) && (Env_temperature > (TH_Value-DELTAT_Value)))//s3
							{
								FAN2_OFF();
								sprintf(FAN_chstate,"OFF");						
								FAN_OFF();
								sprintf(TEC_chstate,"OFF");
							}
							else if((Delta_T<-0.2) && (Env_temperature > (TH_Value-DELTAT_Value)))//s2
							{
								FAN2_ON();
								sprintf(FAN_chstate,"ON");							
								FAN_ON();
								TEC_COLD();
								sprintf(TEC_chstate,"COLD");
								
							}
							else if(Env_temperature<TL_Value)//s7
							{
								FAN2_ON();
								sprintf(FAN_chstate,"ON");							
								FAN_ON();
								TEC_HOT();
								sprintf(TEC_chstate,"HOT");

							}
							else if((Env_temperature>=(TL_Value+DELTAT_Value))&& (Env_temperature<=(TH_Value-DELTAT_Value)))
							{
								FAN2_OFF();
								sprintf(FAN_chstate,"OFF");						
								FAN_OFF();
								sprintf(TEC_chstate,"OFF");
								
							}
							else if((Delta_T>0.2) && (Env_temperature >TL_Value))
							{
								FAN2_ON();
								sprintf(FAN_chstate,"ON");							
								FAN_ON();
								TEC_HOT();
								sprintf(TEC_chstate,"HOT");
								
							}
							else if((Delta_T<-0.2) && (Env_temperature > TL_Value))
							{
								FAN_OFF();
								FAN2_OFF();
								sprintf(FAN_chstate,"OFF");						
								
							}
							prev_Env_temperature=Env_temperature;
						}
					}
					////////////////////////////////////////////////////////////////////////////////////
				}
				
				/////////////////////log Env_humidity/Env_temperature/////////////////////////////
				if(flag_10s)
				{
					flag_10s=0;
					if(avg_counter==0)					
					{
						avg_Env_humidity=Env_humidity;
						avg_Env_temperature=Env_temperature;
					}
					else
					{
						avg_Env_humidity=avg_Env_humidity/(float)avg_counter;
						avg_Env_temperature=avg_Env_humidity/(float)avg_counter;
					}
					avg_counter=0;
					if(is_sddetected)
					{
						generate_filename(cur_date,cur_time,&counterlog_file,filelog_name);
						if(NTC_valid)
							sprintf(file_str,"%5ld,%04d-%02d-%02d,%02d:%02d:%02d,%5.1f,%4.1f,%03d,%4.1f,%03d,%4.1f,%s,%s,%s,%4.1f\n",
														counterlog_file,
														cur_date.Year+1980,cur_date.Month,cur_date.Date,
														cur_time.Hours,cur_time.Minutes,cur_time.Seconds,						
														Env_temperature,Env_humidity,BrightW_Value,BlinkW_Value,BrightIR_Value,
														LDR_resistance,DOOR_chstate,TEC_chstate,FAN_chstate,NTC_Centigrade);
						else
							sprintf(file_str,"%5ld,%04d-%02d-%02d,%02d:%02d:%02d,%5.1f,%4.1f,%03d,%4.1f,%03d,%4.1f,%s,%s,%s,---\n",
														counterlog_file,
														cur_date.Year+1980,cur_date.Month,cur_date.Date,
														cur_time.Hours,cur_time.Minutes,cur_time.Seconds,						
														Env_temperature,Env_humidity,BrightW_Value,BlinkW_Value,BrightIR_Value,
														LDR_resistance,DOOR_chstate,TEC_chstate,FAN_chstate);						
						
						if((fr=File_log(filelog_name,file_str))!=FR_OK)
						{
							print_segment("E104");
							HAL_Delay(1000);
						}		
					}
					else
					{
						if(Menu_choice==MAIN_MENU)
						{
							print_segment("EE.5D");
							HAL_Delay(1000);						
						}
					}
					avg_Env_temperature=0.0;
					avg_Env_humidity=0.0;
				}
				/////////////////////////////////////////////////////
				if(KEYSW4_shortpressed)
				{
					KEYSW4_shortpressed=0;
					counterlog_file=0;
					print_segment("5D. .E");
					HAL_Delay(2000);
				}
				//////////////////////////////////////////////////
				if(KEYSW1_shortpressed)
				{
					KEYSW1_shortpressed=0;
					index_disp=0;
					disp_pass[0]='-';disp_pass[1]='-';disp_pass[2]='-';disp_pass[3]='-';
					print_pass(disp_pass,index_disp,1);
					FAN2_OFF();
					sprintf(FAN_chstate,"OFF");						
					FAN_OFF();
					sprintf(TEC_chstate,"OFF");
					Menu_choice=PASS_MENU;
					
					GOMAINMENU_counter=GOMAINMENU_DELAY;
				}
				if(KEYSW1_longpressed)
				{
					KEYSW1_longpressed=0;
				}
				if(KEYSW2_shortpressed)
				{
					KEYSW2_shortpressed=0;
					switch(cur_Disp)
					{
						case DISP_NTC:
							cur_Disp=last_disp=DISP_SUNRISE;
							sprintf(str_disp,"%02d.%02d",sunrise_t.hr,sunrise_t.min);
						break;
						case DISP_SUNRISE:
							cur_Disp=last_disp=DISP_SUNSET;
							sprintf(str_disp,"%02d.%02d",sunset_t.hr,sunset_t.min);
						break;
						case DISP_SUNSET:
							cur_Disp=last_disp=DISP_TEMP;
							sprintf(str_disp,"%5.1f",Env_temperature);
						break;
						case DISP_TEMP:
							cur_Disp=last_disp=DISP_HUM;
							sprintf(str_disp,"H%4.1f",Env_humidity);
						break;
						case DISP_HUM:
							cur_Disp=last_disp=DISP_BLINKW;
							sprintf(str_disp,"L%4.1f",BlinkW_Value);							
						break;
						case DISP_BLINKW:
							cur_Disp=last_disp=DISP_BRIGHTW;
							sprintf(str_disp,"T%3d",BrightW_Value);
							break;
						case DISP_BRIGHTW:
							cur_Disp=last_disp=DISP_BRIGHTIR;
							sprintf(str_disp,"R%3d",BrightIR_Value);
							break;
						case DISP_BRIGHTIR:
							cur_Disp=last_disp=DISP_NTC;
							if(NTC_valid)
								sprintf(str_disp,"N%03.0f",NTC_Centigrade);
							else
								sprintf(str_disp,"N---");
						break;
						case DISP_ERROR:
							sprintf(str_disp,"E207");	
							break;
					}
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					print_segment(str_disp);					
				}
				if(KEYSW2_longpressed)
				{
					KEYSW2_longpressed=0;
				}
				if(KEYSW3_shortpressed)
				{
					KEYSW3_shortpressed=0;
					switch(cur_Disp)
					{

						case DISP_TEMP:
							cur_Disp=last_disp=DISP_SUNSET;
							sprintf(str_disp,"%02d.%02d",sunset_t.hr,sunset_t.min);
						break;
						case DISP_HUM:
							cur_Disp=last_disp=DISP_TEMP;
							sprintf(str_disp,"%5.1f",Env_temperature);
						break;
						case DISP_BLINKW:
							cur_Disp=last_disp=DISP_HUM;
							sprintf(str_disp,"H%4.1f",Env_humidity);
							break;
						case DISP_BRIGHTW:
							cur_Disp=last_disp=DISP_BLINKW;
							sprintf(str_disp,"L%4.1f",BlinkW_Value);							
							break;						
						case DISP_BRIGHTIR:
							cur_Disp=last_disp=DISP_BRIGHTW;
							sprintf(str_disp,"T%3d",BrightW_Value);
						break;
						case DISP_NTC:
							cur_Disp=last_disp=DISP_BRIGHTIR;
							sprintf(str_disp,"R%3d",BrightIR_Value);
						break;
						case DISP_SUNRISE:
							cur_Disp=last_disp=DISP_NTC;
							if(NTC_valid)
								sprintf(str_disp,"N%03.0f",NTC_Centigrade);
							else
								sprintf(str_disp,"N---");
						break;
						case DISP_SUNSET:
							cur_Disp=last_disp=DISP_SUNRISE;
							sprintf(str_disp,"%02d.%02d",sunrise_t.hr,sunrise_t.min);
						break;
						case DISP_ERROR:
							sprintf(str_disp,"E207");			
						break;
					}
					print_segment(str_disp);
					GOMAINMENU_counter=GOMAINMENU_DELAY;					
				}
				if(KEYSW3_longpressed)
				{
					KEYSW3_longpressed=0;
				}
				if(KEYSW4_longpressed)
				{
					KEYSW4_longpressed=0;
				}
				if(GOMAINMENU_counter<=0)
				{
					GOMAINMENU_counter=0;
					cur_Disp=DISP_TEMP;
				}
				break;
			/////////////////////////////////////////////////////////CASE PASS_MENU/////////////////////////////////////////////////////////////////				
			case PASS_MENU:
				reset_keys();
				if(KEYSW2_longpressed)	KEYSW2_longpressed=0;
				if(KEYSW3_longpressed)	KEYSW3_longpressed=0;
				if(KEYSW2_shortpressed)//up
				{
					KEYSW2_shortpressed=0;
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					if(disp_pass[index_disp]=='-')
					{
						passsegment_value=0;
					}
					else
					{
						passsegment_value++;
						if(passsegment_value>9) passsegment_value=0;
					}
					disp_pass[index_disp]=passsegment_value+'0';
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}					
				}
				if(KEYSW3_shortpressed)//dn
				{
					KEYSW3_shortpressed=0;
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					if(disp_pass[index_disp]=='-')
					{
						passsegment_value=9;
					}
					else
					{
						if(passsegment_value==0) passsegment_value=10;
						passsegment_value--;
						
					}
					disp_pass[index_disp]=passsegment_value+'0';
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}							
				}
				if(KEYSW1_shortpressed)//set
				{
					KEYSW1_shortpressed=0;
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					if(disp_pass[index_disp]=='-') disp_pass[index_disp]='0';
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}						
					
					index_disp++;
					if(index_disp==4)
					{
						tmp_uint16t=(disp_pass[0]-'0')*1000+(disp_pass[1]-'0')*100+(disp_pass[2]-'0')*10+(disp_pass[3]-'0');
						if(tmp_uint16t==Password_Value)
						{
							sprintf(str_disp,"%5.1f",TL_Value);prev_float=TL_Value;
							print_segment(str_disp);
							Blink_segments(1);
							Menu_choice=TEMPL_MENU;
							GOMAINMENU_counter=GOMAINMENU_DELAY;
						}
						else
							GOMAINMENU_counter=0;
					}
				}
				if(flag_1s && Menu_choice!=TEMPL_MENU)
				{
					flag_1s=0;
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}
					blink=1-blink;
				}
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
				}
			break;				
			/////////////////////////////////////////////////////////CASE TEMPL_MENU/////////////////////////////////////////////////////////////////				
			case TEMPL_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_float!=TL_Value)
				{
					if(EE_WriteVariable(TL_ADDRESS,(uint16_t)(TL_Value*10.0))!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				sprintf(str_disp,"%5.1f",TH_Value);prev_float=TH_Value;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=TEMPH_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				TL_Value+=0.1;
				if(TL_Value>=(TH_Value-DELTAT_Value)) TL_Value=TH_Value-DELTAT_Value;
				sprintf(str_disp,"%5.1f",TL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				TL_Value-=0.1;
				if(TL_Value<=((float)TL_MIN/10.0)) TL_Value=(float)TL_MIN/10.0;
				sprintf(str_disp,"%5.1f",TL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				TL_Value++;
				if(TL_Value>=(TH_Value-DELTAT_Value)) TL_Value=TH_Value-DELTAT_Value;
				sprintf(str_disp,"%5.1f",TL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				TL_Value--;
				if(TL_Value<=((float)TL_MIN/10.0)) TL_Value=(float)TL_MIN/10.0;
				sprintf(str_disp,"%5.1f",TL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(TL_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					TL_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);
				}
				break;
			/////////////////////////////////////////////////////////CASE TEMPH_MENU/////////////////////////////////////////////////////////////////				
			case TEMPH_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_float!=TH_Value)
				{
					if(EE_WriteVariable(TH_ADDRESS,(uint16_t)(TH_Value*10.0))!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				sprintf(str_disp,"DT%2.1f",DELTAT_Value);prev_float=DELTAT_Value;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=DELTAT_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				TH_Value+=0.1;
				if(TH_Value>=(float)TH_MAX/10.0) TH_Value=(float)TH_MAX/10.0;
				sprintf(str_disp,"%5.1f",TH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				TH_Value-=0.1;
				if(TH_Value<=(TL_Value+DELTAT_Value)) TH_Value=TL_Value+DELTAT_Value;
				sprintf(str_disp,"%5.1f",TH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				TH_Value++;
				if(TH_Value>=(float)TH_MAX/10.0) TH_Value=(float)TH_MAX/10.0;
				sprintf(str_disp,"%5.1f",TH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				TH_Value--;
				if(TH_Value<=(TL_Value+DELTAT_Value)) TH_Value=TL_Value+DELTAT_Value;
				sprintf(str_disp,"%5.1f",TH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(TH_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					TH_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);
				}

			break;
			/////////////////////////////////////////////////////////CASE DELTAT_MENU/////////////////////////////////////////////////////////////////				
			case DELTAT_MENU:
				reset_keys();
				if(KEYSW2_longpressed) KEYSW2_longpressed=0;
				if(KEYSW3_longpressed) KEYSW3_longpressed=0;			
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_float!=DELTAT_Value)
				{
					if(EE_WriteVariable(DELTATEMP_ADDRESS,(uint16_t)(DELTAT_Value*10.0))!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}

				sprintf(str_disp,"N%03.0f",NTCTL_Value);prev_float=NTCTL_Value;
				Menu_choice=NTCTL_MENU;
				print_segment(str_disp);
				Blink_segments(1);					

				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				DELTAT_Value++;
				if((DELTAT_Value>((TH_Value-TL_Value)/2.0))||(DELTAT_Value>(float)DELTAT_MAX/10.0)) DELTAT_Value=0.0;
				sprintf(str_disp,"DT%2.1f",DELTAT_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(DELTAT_Value==0.0) 
				{
					if(((TH_Value-TL_Value)/2.0)<(float)DELTAT_MAX/10.0)
						DELTAT_Value=(float)((uint8_t)((TH_Value-TL_Value))/2.0)+1.0;
					else
						DELTAT_Value=(float)DELTAT_MAX/10.0+1.0;
				}
				DELTAT_Value--;
				sprintf(str_disp,"DT%2.1f",DELTAT_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(DELTATEMP_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					DELTAT_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);
				}

			break;	
			/////////////////////////////////////////////////////////CASE NTC LOW MENU/////////////////////////////
			case NTCTL_MENU:
				reset_keys();
				if(KEYSW2_longpressed) KEYSW2_longpressed=0;
				if(KEYSW3_longpressed) KEYSW3_longpressed=0;
				if(KEYSW1_shortpressed) //set
				{
					KEYSW1_shortpressed=0;
					if(prev_float!=NTCTL_Value)
					{
						if(EE_WriteVariable(NTCTL_ADDRESS,(uint16_t)(NTCTL_Value*10.0))!=HAL_OK)
						{
							print_segment("ER02");
							while(1);
						}
						HAL_Delay(50);					
					}
					sprintf(str_disp,"N%03.0f",NTCTH_Value);prev_float=NTCTH_Value;
					Menu_choice=NTCTH_MENU;
				
					print_segment(str_disp);
					Blink_segments(1);					
					GOMAINMENU_counter=GOMAINMENU_DELAY;

				}
				if(KEYSW2_shortpressed)//up
				{
					KEYSW2_shortpressed=0;
					NTCTL_Value+=1.0;
					if(NTCTL_Value>(double)NTCTL_MAX/10.0) NTCTL_Value=0.0;
					sprintf(str_disp,"N%03.0f",NTCTL_Value);
					print_segment(str_disp);				
					GOMAINMENU_counter=GOMAINMENU_DELAY;					
				}	
				if(KEYSW3_shortpressed)//dn
				{
					KEYSW3_shortpressed=0;
					NTCTL_Value-=1.0;
					if(NTCTL_Value<(double)NTCTL_MIN/10.0) NTCTL_Value=(double)NTCTL_MAX/10.0;
					sprintf(str_disp,"N%03.0f",NTCTL_Value);
					print_segment(str_disp);				
					GOMAINMENU_counter=GOMAINMENU_DELAY;					

				}
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(NTCTL_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					NTCTL_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);					
				}					
			break;
			/////////////////////////////////////////////////////////CASE NTC HIGH MENU/////////////////////////////			
			case NTCTH_MENU:
				reset_keys();
				if(KEYSW2_longpressed) KEYSW2_longpressed=0;
				if(KEYSW3_longpressed) KEYSW3_longpressed=0;
			
				if(KEYSW1_shortpressed) //set
				{
					KEYSW1_shortpressed=0;
					if(prev_float!=NTCTH_Value)
					{
						if(EE_WriteVariable(NTCTH_ADDRESS,(uint16_t)(NTCTH_Value*10.0))!=HAL_OK)
						{
							print_segment("ER02");
							while(1);
						}
						HAL_Delay(50);					
					}
					sprintf(str_disp,"H%4.1f",HL_Value);prev_float=HL_Value;
					Menu_choice=HUML_MENU;
				
					print_segment(str_disp);
					Blink_segments(1);					
					GOMAINMENU_counter=GOMAINMENU_DELAY;

				}
				if(KEYSW2_shortpressed)//up
				{
					KEYSW2_shortpressed=0;
					NTCTH_Value+=1.0;
					if(NTCTH_Value>(double)NTCTH_MAX/10.0) NTCTH_Value=0.0;
					sprintf(str_disp,"N%03.0f",NTCTH_Value);
					print_segment(str_disp);				
					GOMAINMENU_counter=GOMAINMENU_DELAY;					
				}	
				if(KEYSW3_shortpressed)//dn
				{
					KEYSW3_shortpressed=0;
					NTCTH_Value-=1.0;
					if(NTCTH_Value<(double)NTCTH_MIN/10.0) NTCTH_Value=(double)NTCTH_MAX/10.0;
					sprintf(str_disp,"N%03.0f",NTCTH_Value);
					print_segment(str_disp);				
					GOMAINMENU_counter=GOMAINMENU_DELAY;					

				}
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(NTCTH_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					NTCTH_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);					
				}		
			break;
			/////////////////////////////////////////////////////////CASE HUML_MENU/////////////////////////////////////////////////////////////////				
			case HUML_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_float!=HL_Value)
				{
					if(EE_WriteVariable(HL_ADDRESS,(uint16_t)(HL_Value*10.0))!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				sprintf(str_disp,"H%4.1f",HH_Value);prev_float=HH_Value;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=HUMH_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				HL_Value+=0.1;
				if(HL_Value>=HH_Value) HL_Value=HH_Value;
				sprintf(str_disp,"H%4.1f",HL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				HL_Value-=0.1;
				if(HL_Value<=((float)HL_MIN/10.0)) HL_Value=(float)HL_MIN/10.0;
				sprintf(str_disp,"H%4.1f",HL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				HL_Value++;
				if(HL_Value>=HH_Value) HL_Value=HH_Value;
				sprintf(str_disp,"H%4.1f",HL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				HL_Value--;
				if(HL_Value<=((float)HL_MIN/10.0)) HL_Value=(float)HL_MIN/10.0;
				sprintf(str_disp,"H%4.1f",HL_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(HL_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					HL_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);
				}
				
				break;
			/////////////////////////////////////////////////////////CASE HUMH_MENU/////////////////////////////////////////////////////////////////				
			case HUMH_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_float!=HH_Value)
				{
					if(EE_WriteVariable(HH_ADDRESS,(uint16_t)(HH_Value*10.0))!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				sprintf(str_disp,"L%4.1f",BlinkW_Value);prev_float=BlinkW_Value;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=BLINKW_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				HH_Value+=0.1;
				if(HH_Value>=(float)HH_MAX/10.0) HH_Value=(float)HH_MAX/10.0;
				sprintf(str_disp,"H%4.1f",HH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				HH_Value-=0.1;
				if(HH_Value<=HL_Value) HH_Value=HL_Value;
				sprintf(str_disp,"H%4.1f",HH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				HH_Value++;
				if(HH_Value>=(float)HH_MAX/10.0) HH_Value=(float)HH_MAX/10.0;
				sprintf(str_disp,"H%4.1f",HH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				HH_Value--;
				if(HH_Value<=HL_Value) HH_Value=HL_Value;
				sprintf(str_disp,"H%4.1f",HH_Value);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(HH_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					HH_Value=(float)read_eeprom/10.0;
					HAL_Delay(50);
				}
				break;
			/////////////////////////////////////////////////////////CASE BLINKW_MENU/////////////////////////////////////////////////////////////////				
			case BLINKW_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
			
				if(prev_float!=BlinkW_Value)
				{
					if(EE_WriteVariable(BLINKW_ADDRESS,(uint16_t)(BlinkW_Value*10.0))!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				sprintf(str_disp,"T%3d",BrightW_Value);prev_uint16t=BrightW_Value;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=BRIGHTW_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				BlinkW_Value+=0.1;
				if(BlinkW_Value>=(float)BLINKW_MAX/10.0) BlinkW_Value=(float)BLINKW_MAX/10.0;
				sprintf(str_disp,"L%4.1f",BlinkW_Value);
				print_segment(str_disp);				
				pca9632_setblinking(BlinkW_Value);

				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				BlinkW_Value-=0.1;
				if(BlinkW_Value<=(float)BLINKW_MIN/10.0) BlinkW_Value=(float)BLINKW_MIN/10.0;
				sprintf(str_disp,"L%4.1f",BlinkW_Value);
				print_segment(str_disp);				
					pca9632_setblinking(BlinkW_Value);

				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				BlinkW_Value++;
				if(BlinkW_Value>=(float)BLINKW_MAX/10.0) BlinkW_Value=(float)BLINKW_MAX/10.0;
				sprintf(str_disp,"L%4.1f",BlinkW_Value);
				print_segment(str_disp);				
					pca9632_setblinking(BlinkW_Value);

				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				BlinkW_Value--;
				if(BlinkW_Value<=(float)BLINKW_MIN/10.0) BlinkW_Value=(float)BLINKW_MIN/10.0;
				sprintf(str_disp,"L%4.1f",BlinkW_Value);
				print_segment(str_disp);
				pca9632_setblinking(BlinkW_Value);
				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(BLINKW_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					BlinkW_Value=(float)read_eeprom/10.0;
					pca9632_setblinking(BlinkW_Value);
					
					HAL_Delay(50);
				}				
				break;
			/////////////////////////////////////////////////////////CASE BRIGHTW_MENU/////////////////////////////////////////////////////////////////				
			case BRIGHTW_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				
				if(prev_uint16t!=BrightW_Value)
				{
					if(EE_WriteVariable(BRIGHTW_ADDRESS,(uint16_t)BrightW_Value)!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				//jump to Date because of No IR
				//sprintf(str_disp,"R%3d",BrightIR_Value);prev_uint16t=BrightIR_Value;
				//print_segment(str_disp);					
				//Menu_choice=BRIGHTIR_MENU;
				//GOMAINMENU_counter=GOMAINMENU_DELAY;
				
				index_disp=0;
				HAL_RTC_GetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
				sprintf(str_disp,"%04d",tmp_date.Year+1980);
				prev_uint16t=tmp_date.Year;

				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETYEAR_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
				
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				BrightW_Value++;
				if(BrightW_Value>100) BrightW_Value=100;
				sprintf(str_disp,"T%3d",BrightW_Value);
				print_segment(str_disp);				
				pca9632_setonepwm(LEDW,BrightW_Value);
				pca9632_setonepwm(LEDIR,BrightW_Value);

				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(BrightW_Value<=0) BrightW_Value=1;
				BrightW_Value--;
				sprintf(str_disp,"T%3d",BrightW_Value);
				print_segment(str_disp);				
				pca9632_setonepwm(LEDW,BrightW_Value);
				pca9632_setonepwm(LEDIR,BrightW_Value);
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				BrightW_Value+=10;
				if(BrightW_Value>100) BrightW_Value=BrightW_Value-10;				
				sprintf(str_disp,"T%3d",BrightW_Value);
				print_segment(str_disp);				
				pca9632_setonepwm(LEDW,BrightW_Value);
				pca9632_setonepwm(LEDIR,BrightW_Value);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				if(BrightW_Value<10) BrightW_Value=BrightW_Value+10;
				BrightW_Value-=10;
				sprintf(str_disp,"T%3d",BrightW_Value);
				print_segment(str_disp);			
				pca9632_setonepwm(LEDW,BrightW_Value);
				pca9632_setonepwm(LEDIR,BrightW_Value);			
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(BRIGHTW_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					BrightW_Value=read_eeprom;
					pca9632_setonepwm(LEDW,BrightW_Value);
					pca9632_setonepwm(LEDIR,BrightW_Value);
					HAL_Delay(50);
				}				
				break;
				
			/////////////////////////////////////////////////////////CASE BRIGHTIR_MENU/////////////////////////////////////////////////////////////////				
			case BRIGHTIR_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				
				if(prev_uint16t!=BrightIR_Value)
				{
					if(EE_WriteVariable(BRIGHTIR_ADDRESS,(uint16_t)BrightIR_Value)!=HAL_OK)
					{
						print_segment("ER02");
						while(1);
					}
					HAL_Delay(50);					
				}
				index_disp=0;
				HAL_RTC_GetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
				sprintf(str_disp,"%04d",tmp_date.Year+1980);
				prev_uint16t=tmp_date.Year;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETYEAR_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				BrightIR_Value++;
				if(BrightIR_Value>100) BrightIR_Value=100;
				sprintf(str_disp,"R%3d",BrightIR_Value);
				print_segment(str_disp);				
				pca9632_setonepwm(LEDIR,BrightIR_Value);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(BrightIR_Value<=0) BrightIR_Value=0;
				BrightIR_Value--;
				sprintf(str_disp,"R%3d",BrightIR_Value);
				print_segment(str_disp);				
				pca9632_setonepwm(LEDIR,BrightIR_Value);
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				BrightIR_Value+=10;
				if(BrightIR_Value>100) BrightIR_Value=BrightIR_Value-10;				
				sprintf(str_disp,"R%3d",BrightIR_Value);
				print_segment(str_disp);				
				pca9632_setonepwm(LEDIR,BrightIR_Value);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				if(BrightIR_Value<10) BrightIR_Value=BrightIR_Value+10;
				BrightIR_Value-=10;
				sprintf(str_disp,"R%3d",BrightIR_Value);
				print_segment(str_disp);	
				pca9632_setonepwm(LEDIR,BrightIR_Value);
				GOMAINMENU_counter=GOMAINMENU_DELAY;				

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(BRIGHTIR_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					BrightIR_Value=read_eeprom;
					pca9632_setonepwm(LEDIR,BrightIR_Value);
					HAL_Delay(50);
				}				
				break;
			/////////////////////////////////////////////////////////CASE SETYEAR_MENU/////////////////////////////////////////////////////////////////				
			case SETYEAR_MENU:
				reset_keys();
			if(KEYSW2_longpressed) KEYSW2_longpressed=0;
			if(KEYSW3_longpressed) KEYSW3_longpressed=0;
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(tmp_date.Year!=prev_uint16t)
				{
					HAL_RTC_SetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &cur_date, RTC_FORMAT_BIN);
					HAL_RTC_GetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);

					//date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					//Astro_sunRiseSet(LAT_Value, LONG_Value, UTCOEFF_TEHRAN, date,&sunrise_t,&noon_t,&sunset_t,1);
					date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					if(Astro_daylighsaving(date)) //summer
					{
						tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
						winter=0;
					}
					else
					{
						tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
						winter=1;
					}
					tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					HAL_RTC_SetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);
				}
				index_disp=0;
				HAL_RTC_GetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
				sprintf(str_disp,"N %02d",tmp_date.Month);prev_uint16t=tmp_date.Month;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETMONTH_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				tmp_date.Year++;
				if(tmp_date.Year>99) tmp_date.Year=0;
				sprintf(str_disp,"%04d",tmp_date.Year+1980);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(tmp_date.Year<=0)tmp_date.Year=100;
				tmp_date.Year--;
				sprintf(str_disp,"%04d",tmp_date.Year+1980);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			/////////////////////////
			if(GOMAINMENU_counter==0)
			{
				Menu_choice=MAIN_MENU;
				flag_1s=1;
				Blink_segments(0);
				HAL_Delay(50);
			}				
			break;
			
			/////////////////////////////////////////////////////////CASE SETMONTH_MENU/////////////////////////////////////////////////////////////////				
			case SETMONTH_MENU:
				reset_keys();
			if(KEYSW2_longpressed) KEYSW2_longpressed=0;
			if(KEYSW3_longpressed) KEYSW3_longpressed=0;
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(tmp_date.Month!=prev_uint16t)
				{
					HAL_RTC_SetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &cur_date, RTC_FORMAT_BIN);
					HAL_RTC_GetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);

					//date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					//Astro_sunRiseSet(LAT_Value, LONG_Value, UTCOEFF_TEHRAN, date,&sunrise_t,&noon_t,&sunset_t,1);
					date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					if(Astro_daylighsaving(date)) //summer
					{
						tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
						winter=0;
					}
					else
					{
						tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
						winter=1;
					}
					tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					HAL_RTC_SetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);
				}
				index_disp=0;
				HAL_RTC_GetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
				sprintf(str_disp,"D %02d",tmp_date.Date);prev_uint16t=tmp_date.Date;
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETDAY_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				tmp_date.Month++;
				if(tmp_date.Month>12) tmp_date.Month=1;
				sprintf(str_disp,"N %02d",tmp_date.Month);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(tmp_date.Month<1)tmp_date.Month=13;
				tmp_date.Month--;
				sprintf(str_disp,"N %02d",tmp_date.Month);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			/////////////////////////
			if(GOMAINMENU_counter==0)
			{
				Menu_choice=MAIN_MENU;
				flag_1s=1;
				Blink_segments(0);
				HAL_Delay(50);
			}				
			break;
			
			/////////////////////////////////////////////////////////CASE SETDAY_MENU/////////////////////////////////////////////////////////////////				
			case SETDAY_MENU:
				reset_keys();
			if(KEYSW2_longpressed) KEYSW2_longpressed=0;
			if(KEYSW3_longpressed) KEYSW3_longpressed=0;
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(tmp_date.Date!=prev_uint16t)
				{
					HAL_RTC_SetDate(&hrtc,&tmp_date,RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &cur_date, RTC_FORMAT_BIN);
					HAL_RTC_GetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);

					//date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					//Astro_sunRiseSet(LAT_Value, LONG_Value, UTCOEFF_TEHRAN, date,&sunrise_t,&noon_t,&sunset_t,1);
					date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					if(Astro_daylighsaving(date)) //summer
					{
						tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
						winter=0;
					}
					else
					{
						tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
						winter=1;
					}
					tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					HAL_RTC_SetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);

				}
				index_disp=0;
				HAL_RTC_GetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);
				sprintf(str_disp,"HR%02d",tmp_time.Hours);
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETHOUR_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				tmp_date.Date++;
				if(tmp_date.Month==2)
				{
					if(tmp_date.Date>gDaysInMonth[tmp_date.Month-1]+leap(tmp_date.Year+1980)) tmp_date.Date=1;
				}
				else
				{
					if(tmp_date.Date>gDaysInMonth[tmp_date.Month-1]) tmp_date.Date=1;
				}
				sprintf(str_disp,"D %02d",tmp_date.Date);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				
				if(tmp_date.Month==2)
				{
					if(tmp_date.Date<=1) tmp_date.Date=gDaysInMonth[tmp_date.Month-1]+leap(tmp_date.Year+1980)+1;
				}
				else
				{
					if(tmp_date.Date<=1) tmp_date.Date=gDaysInMonth[tmp_date.Month-1]+1;
				}
				tmp_date.Date--;
				sprintf(str_disp,"D %02d",tmp_date.Date);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			/////////////////////////
			if(GOMAINMENU_counter==0)
			{
				Menu_choice=MAIN_MENU;
				flag_1s=1;
				Blink_segments(0);
				HAL_Delay(50);
			}				
			break;
			/////////////////////////////////////////////////////////CASE SETHOUR_MENU/////////////////////////////////////////////////////////////////				
			case SETHOUR_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
				if(Astro_daylighsaving(date)) //summer
				{
					tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
					winter=0;
				}
				else
				{
					tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
					winter=1;
				}

				tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				HAL_RTC_SetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);

				index_disp=0;
				HAL_RTC_GetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);
				sprintf(str_disp," %02d'",tmp_time.Minutes);
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETMIN_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				tmp_time.Hours++;
				if(tmp_time.Hours>23) tmp_time.Hours=0;
				sprintf(str_disp,"HR%02d",tmp_time.Hours);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				tmp_time.Hours+=5;
				if(tmp_time.Hours>23) tmp_time.Hours=0;
				sprintf(str_disp,"HR%02d",tmp_time.Hours);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}			
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(tmp_time.Hours<=0) tmp_time.Hours=24;
				tmp_time.Hours--;
				sprintf(str_disp,"HR%02d",tmp_time.Hours);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				if(tmp_time.Hours<=5) tmp_time.Hours=28;
				tmp_time.Hours-=5;
				sprintf(str_disp,"HR%02d",tmp_time.Hours);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			
			/////////////////////////
			if(GOMAINMENU_counter==0)
			{
				Menu_choice=MAIN_MENU;
				flag_1s=1;
				Blink_segments(0);
				HAL_Delay(50);
			}				
			break;

				/////////////////////////////////////////////////////////CASE SETMIN_MENU/////////////////////////////////////////////////////////////////				
			case SETMIN_MENU:
				reset_keys();
	
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;

				if(Astro_daylighsaving(date)) //summer
				{
					tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
					winter=0;
				}
				else
				{
					tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
					winter=1;
				}

				tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				HAL_RTC_SetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);

				index_disp=0;
				HAL_RTC_GetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);
				sprintf(str_disp," %02d\"",tmp_time.Seconds);
				print_segment(str_disp);
				Blink_segments(1);					
				Menu_choice=SETSEC_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				tmp_time.Minutes++;
				if(tmp_time.Minutes>59) tmp_time.Minutes=0;
				sprintf(str_disp," %02d'",tmp_time.Minutes);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(tmp_time.Minutes<=0) tmp_time.Minutes=60;
				tmp_time.Minutes--;
				sprintf(str_disp," %02d'",tmp_time.Minutes);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				tmp_time.Minutes+=10;
				if(tmp_time.Minutes>59) tmp_time.Minutes=tmp_time.Minutes-59;				
				sprintf(str_disp," %02d'",tmp_time.Minutes);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				if(tmp_time.Minutes<10) tmp_time.Minutes=tmp_time.Minutes+60;
				tmp_time.Minutes-=10;
				sprintf(str_disp," %02d'",tmp_time.Minutes);
				print_segment(str_disp);	
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			/////////////////////////
			if(GOMAINMENU_counter==0)
			{
				Menu_choice=MAIN_MENU;
				flag_1s=1;
				Blink_segments(0);
				HAL_Delay(50);
			}				
			break;
			/////////////////////////////////////////////////////////CASE SETSEC_MENU/////////////////////////////////////////////////////////////////				
			case SETSEC_MENU:
				reset_keys();
			/////////////////////////////				
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
				if(Astro_daylighsaving(date)) //summer
				{
					tmp_time.StoreOperation = RTC_STOREOPERATION_SET;
					winter=0;
				}
				else
				{
					tmp_time.StoreOperation = RTC_STOREOPERATION_RESET;
					winter=1;
				}

				tmp_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				HAL_RTC_SetTime(&hrtc,&tmp_time,RTC_FORMAT_BIN);
				
				sprintf(str_disp,"A.%4.1f",LAT_Value);prev_double=LAT_Value;
				print_segment(str_disp);
				Blink_segments(1);
				Menu_choice=SETLAT_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				tmp_time.Seconds++;
				if(tmp_time.Seconds>59) tmp_time.Seconds=0;
				sprintf(str_disp," %02d\"",tmp_time.Seconds);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				if(tmp_time.Seconds<=0) tmp_time.Seconds=60;
				tmp_time.Seconds--;
				sprintf(str_disp," %02d\"",tmp_time.Seconds);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				tmp_time.Seconds+=10;
				if(tmp_time.Seconds>59) tmp_time.Seconds=tmp_time.Seconds-59;				
				sprintf(str_disp," %02d\"",tmp_time.Seconds);
				print_segment(str_disp);				
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				if(tmp_time.Seconds<10) tmp_time.Seconds=tmp_time.Seconds+60;
				tmp_time.Seconds-=10;
				sprintf(str_disp," %02d\"",tmp_time.Seconds);
				print_segment(str_disp);	
				GOMAINMENU_counter=GOMAINMENU_DELAY;				
			}
			/////////////////////////
			if(GOMAINMENU_counter==0)
			{
				Menu_choice=MAIN_MENU;
				flag_1s=1;
				Blink_segments(0);
				HAL_Delay(50);
			}				
			break;
				/////////////////////////////////////////////////////////CASE SETLAT_MENU//////////////////////////////////////////////////////////////////
			case SETLAT_MENU:
				reset_keys();
			/////////////////////////////
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_double!=LAT_Value)
				{

					if(EE_WriteVariable(LAT_ADDRESS,(uint16_t)(LAT_Value*10.0))!=HAL_OK)
					{
						print_segment("ER12");
						while(1);
					}
					HAL_RTC_GetDate(&hrtc, &cur_date, RTC_FORMAT_BIN);
					date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					Astro_sunRiseSet(LAT_Value, LONG_Value, UTCOEFF_TEHRAN, date,&sunrise_t,&noon_t,&sunset_t,1);

					HAL_Delay(50);
				}

				index_disp=0;
				sprintf(str_disp,"O.%4.1f",LONG_Value);prev_double=LONG_Value;
				print_segment(str_disp);
				Blink_segments(1);
				Menu_choice=SETLONG_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				LAT_Value+=0.1;
				if(LAT_Value>(double)LAT_MAX/10.0) LAT_Value=(double)LAT_MIN/10.0;
				sprintf(str_disp,"A.%4.1f",LAT_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				LAT_Value-=0.1;
				if(LAT_Value<(double)LAT_MIN/10.0) LAT_Value=(double)LAT_MAX/10.0;
				sprintf(str_disp,"A.%4.1f",LAT_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				LAT_Value++;
				if(LAT_Value>(double)LAT_MAX/10.0) LAT_Value=(double)LAT_MIN/10.0;
				sprintf(str_disp,"A.%4.1f",LAT_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				LAT_Value--;
				if(LAT_Value<(double)LAT_MIN/10.0) LAT_Value=(double)LAT_MAX/10.0;
				sprintf(str_disp,"A.%4.1f",LAT_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(LAT_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}
					LAT_Value=(double)read_eeprom/10.0;
					HAL_Delay(50);
				}
				break;
			/////////////////////////////////////////////////////////CASE SETLONG_MENU//////////////////////////////////////////////////////////////////
			case SETLONG_MENU:
				reset_keys();
			/////////////////////////////
			if(KEYSW1_shortpressed) //set
			{
				KEYSW1_shortpressed=0;
				if(prev_double!=LONG_Value)
				{
					if(EE_WriteVariable(LONG_ADDRESS,(uint16_t)(LONG_Value*10.0))!=HAL_OK)
					{
						print_segment("ER12");
						while(1);
					}
					HAL_Delay(50);
					HAL_RTC_GetDate(&hrtc, &cur_date, RTC_FORMAT_BIN);
					date.day=cur_date.Date;date.month=cur_date.Month;date.year=cur_date.Year+1980;
					Astro_sunRiseSet(LAT_Value, LONG_Value, UTCOEFF_TEHRAN, date,&sunrise_t,&noon_t,&sunset_t,1);

				}

				index_disp=0;
				disp_pass[0]='-';disp_pass[1]='-';disp_pass[2]='-';disp_pass[3]='-';
				print_pass(disp_pass,index_disp,1);prev_uint16t=Password_Value;
				Blink_segments(1);
				counterlog_file=0;

				Menu_choice=SETPASS_MENU;
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_shortpressed)//up
			{
				KEYSW2_shortpressed=0;
				LONG_Value+=0.1;
				if(LONG_Value>(double)LONG_MAX/10.0) LONG_Value=(double)LONG_MIN/10.0;
				sprintf(str_disp,"O.%4.1f",LONG_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_shortpressed)//dn
			{
				KEYSW3_shortpressed=0;
				LONG_Value-=0.1;
				if(LONG_Value<(double)LONG_MIN/10.0) LONG_Value=(double)LONG_MAX/10.0;
				sprintf(str_disp,"O.%4.1f",LONG_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW2_longpressed)//up
			{
				KEYSW2_longpressed=0;
				LONG_Value++;
				if(LONG_Value>(double)LONG_MAX/10.0) LONG_Value=(double)LONG_MIN/10.0;
				sprintf(str_disp,"O.%4.1f",LONG_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;
			}
			if(KEYSW3_longpressed)//dn
			{
				KEYSW3_longpressed=0;
				LONG_Value--;
				if(LONG_Value<(double)LONG_MIN/10.0) LONG_Value=(double)LONG_MAX/10.0;
				sprintf(str_disp,"O.%4.1f",LONG_Value);
				print_segment(str_disp);
				GOMAINMENU_counter=GOMAINMENU_DELAY;

			}
			/////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(LONG_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}
					LONG_Value=(double)read_eeprom/10.0;
					HAL_Delay(50);
				}
				break;
			/////////////////////////////////////////////////////////CASE SETPASS_MENU/////////////////////////////////////////////////////////////////
			case SETPASS_MENU:
				reset_keys();
			///////////////////////////////////////////////////////////////
				if(KEYSW2_longpressed)	KEYSW2_longpressed=0;
				if(KEYSW3_longpressed)	KEYSW3_longpressed=0;
				if(KEYSW2_shortpressed)//up
				{
					KEYSW2_shortpressed=0;
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					if(disp_pass[index_disp]=='-')
					{
						passsegment_value=0;
					}
					else
					{
						passsegment_value++;
						if(passsegment_value>9) passsegment_value=0;
					}
					disp_pass[index_disp]=passsegment_value+'0';
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}					
				}
				if(KEYSW3_shortpressed)//dn
				{
					KEYSW3_shortpressed=0;
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					if(disp_pass[index_disp]=='-')
					{
						passsegment_value=9;
					}
					else
					{
						if(passsegment_value==0) passsegment_value=10;
						passsegment_value--;
						
					}
					disp_pass[index_disp]=passsegment_value+'0';
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}							
				}
				if(KEYSW1_shortpressed)//set
				{
					KEYSW1_shortpressed=0;
					GOMAINMENU_counter=GOMAINMENU_DELAY;
					if(disp_pass[index_disp]=='-') disp_pass[index_disp]='0';
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}						
					
					index_disp++;
					if(index_disp==4)
					{
						tmp_uint16t=(disp_pass[0]-'0')*1000+(disp_pass[1]-'0')*100+(disp_pass[2]-'0')*10+(disp_pass[3]-'0');
						if(prev_uint16t!=tmp_uint16t)
						{
							if(EE_WriteVariable(PASSWORD_ADDRESS,tmp_uint16t)!=HAL_OK)
							{
								print_segment("ER02");
								while(1);
							}
							HAL_Delay(50);						
					}
						GOMAINMENU_counter=0;
					}
				}
				if(flag_1s)
				{
					flag_1s=0;
					if(blink)
					{
						print_pass(disp_pass,index_disp,1);
					}
					else
					{
						print_pass(disp_pass,index_disp,0);
					}
					blink=1-blink;
				}
			
				////////////////////////////////////////////////////
				if(GOMAINMENU_counter==0)
				{
					Menu_choice=MAIN_MENU;
					flag_1s=1;
					Blink_segments(0);
					uint16_t read_eeprom;
					if((EE_ReadVariable(PASSWORD_ADDRESS,  &read_eeprom)) != HAL_OK)
					{
						print_segment("E1-R");
					}		
					Password_Value=read_eeprom;
					HAL_Delay(50);
				}						
				break;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
