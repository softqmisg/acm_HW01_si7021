#ifndef __sevenseg_h__
	#define __sevenseg_h__
	//#include "main.h"
	#include "stm32f4xx_hal.h"
#define KEYSW4_GET() HAL_GPIO_ReadPin(BTN_4_GPIO_Port,BTN_4_Pin)
#define KEYSW3_GET() HAL_GPIO_ReadPin(BTN_3_GPIO_Port,BTN_3_Pin)
#define KEYSW2_GET() HAL_GPIO_ReadPin(BTN_2_GPIO_Port,BTN_2_Pin)
#define KEYSW1_GET() HAL_GPIO_ReadPin(BTN_1_GPIO_Port,BTN_1_Pin)

#define FREQ_TIMER		500 //Hz
#define MS_TIME2			2//1000/FREQ_TIMER
#define GOMAINMENU_DELAY	10*FREQ_TIMER

//BLINK_DELAY*2ms= blink delays
#define BLINK_DELAY 300/MS_TIME2 //200ms
////long press T1 LIMIT_T1*2ms
#define LIMIT_SHORT_L	10/MS_TIME2//5/2 //10ms
#define LIMIT_SHORT_H	400/MS_TIME2//400/2 //800ms
/////about 3sec
#define LIMIT_T1_L 1500/MS_TIME2///800/2 //1.6s
#define LIMIT_T1_H 3500/MS_TIME2//1500/2	//3s
//////about >7s
#define LIMIT_T2_L 5000/MS_TIME2//2000/2	//4000ms
#define LIMIT_T2_H 10000/MS_TIME2//5000/2	//10000ms

	void init_sevenseg();
	void print_segment(char *str);
	void Blink_segments(uint8_t on_off);

	extern uint8_t flag_1s;
	extern uint8_t flag_10s;

	extern uint8_t KEYSW4_shortpressed;
	extern uint8_t KEYSW4_longpressed;

	extern uint8_t KEYSW3_shortpressed;
	extern uint8_t KEYSW3_longpressed;

	extern uint8_t KEYSW2_shortpressed;
	extern uint8_t KEYSW2_longpressed;

	extern uint8_t KEYSW1_shortpressed;
	extern uint8_t KEYSW1_longpressed;
	
	extern uint16_t GOMAINMENU_counter;	
#endif
