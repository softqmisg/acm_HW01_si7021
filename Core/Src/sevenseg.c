//#include "main.h"
#include "tim.h"
#include "sevenseg.h"
#include <ctype.h>
GPIO_TypeDef *digit_ports[]={SEG_DIG1_GPIO_Port,SEG_DIG2_GPIO_Port,SEG_DIG3_GPIO_Port,SEG_DIG4_GPIO_Port};
uint16_t digit_pins[]={SEG_DIG1_Pin,SEG_DIG2_Pin,SEG_DIG3_Pin,SEG_DIG4_Pin};
char str_segments[]={' ',' ',' ',' '};
uint8_t dp_segments[]={0,0,0,0};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void reset_digits()
{
			HAL_GPIO_WritePin(SEG_DIG1_GPIO_Port,SEG_DIG1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_DIG2_GPIO_Port,SEG_DIG2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_DIG3_GPIO_Port,SEG_DIG3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_DIG4_GPIO_Port,SEG_DIG4_Pin,GPIO_PIN_SET);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_sevenseg()
{
	reset_digits();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_segments(char ch)
{
	switch(toupper(ch))
	{
		case '0':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);	
		break;
		case '1':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);	
		break;	
		case '2':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case '3':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case '4':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);

		break;
		case '5':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;		
		case '6':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;			
		case '7':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);	
		break;	
		case '8':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;
		case '9':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case '-':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case 'A':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;
		case 'B':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case 'T':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case 'L':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);
		break;	
		case 'F':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case 'Q':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;	
		case 'R':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;
		case 'E':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;		
		case 'H':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;
	case 'P':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break;

		case 'U':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);	
		break;
		case 'D':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break	;
	case 'N':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break	;	
		case 'Y':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_RESET);
		break	;	
		case 'O':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);
		break;
		case '\'':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);
		break	;

		case '\"':
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);
		break	;		
		default	:
			HAL_GPIO_WritePin(SEG_A_GPIO_Port,SEG_A_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_B_GPIO_Port,SEG_B_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_C_GPIO_Port,SEG_C_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_D_GPIO_Port,SEG_D_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_E_GPIO_Port,SEG_E_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_F_GPIO_Port,SEG_F_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SEG_G_GPIO_Port,SEG_G_Pin,GPIO_PIN_SET);			
	}
	
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t counter_1s=0;
uint16_t counter_10s=0;
uint8_t flag_1s=0;
uint8_t flag_10s=0;
uint8_t cur_seg=0;

uint8_t current_state_sw4;
uint8_t previous_state_sw4=1;
uint8_t KEYSW4_shortpressed=0;
uint8_t KEYSW4_longpressed=0;
uint32_t KEYSW4_cntshort=0;

uint8_t current_state_sw3;
uint8_t previous_state_sw3=1;
uint8_t KEYSW3_shortpressed=0;
uint8_t KEYSW3_longpressed=0;
uint32_t KEYSW3_cntshort=0;

uint8_t current_state_sw2;
uint8_t previous_state_sw2=1;
uint8_t KEYSW2_shortpressed=0;
uint8_t KEYSW2_longpressed=0;
uint32_t KEYSW2_cntshort=0;

uint8_t current_state_sw1;
uint8_t previous_state_sw1=1;
uint8_t KEYSW1_shortpressed=0;
uint8_t KEYSW1_longpressed=1;
uint8_t KEYSW1_longlongpressed=0;
uint32_t KEYSW1_cntshort=0;

uint16_t GOMAINMENU_counter=0;
uint16_t counter_Blink=0;
uint8_t disp_Blink=0;
uint8_t flag_Blink=0;
uint8_t flag_blinkLED=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM10)
	{
		//////////////////////////////////////////////////////////////////////
	if(GOMAINMENU_counter)
		GOMAINMENU_counter--;
		/////////////////////////////////////////////////////////////////////////////////
		counter_Blink++;
		if(counter_Blink==BLINK_DELAY)
		{
			counter_Blink=0;
			flag_Blink=1-flag_Blink;
		}
		/////////////////////////////////////////////////////////////////////////////////
		counter_1s++;
		if(counter_1s==500)
		{
			counter_1s=0;
			flag_1s=1;
			counter_10s++;
			if(counter_10s==10)
			{
				counter_10s=0;
				flag_10s=1;
			}
		}
	///////////////////////////KEY STATE///////////////////////
	/////////////////////KEY SW4/////////////////////////////////
	current_state_sw4=(uint8_t)KEYSW4_GET();

	if(current_state_sw4 && !previous_state_sw4)
	{
		if((LIMIT_SHORT_L<KEYSW4_cntshort)&&(KEYSW4_cntshort<LIMIT_SHORT_H) && !KEYSW4_longpressed)
		{
			KEYSW4_shortpressed=1;
		}                        
		KEYSW4_cntshort=0;
	}
	else if(!current_state_sw4 && previous_state_sw4)
	{
		KEYSW4_cntshort=0;
	}
	else if(!current_state_sw4 && !previous_state_sw4)
	{
		KEYSW4_cntshort++;
		if((LIMIT_T1_L<KEYSW4_cntshort)&&(KEYSW4_cntshort<LIMIT_T1_H))
		{
			KEYSW4_longpressed=1;
			KEYSW4_cntshort=0;
		}                 
	}
	previous_state_sw4=current_state_sw4;		
		
	/////////////////////KEY SW3/////////////////////////////////
	current_state_sw3=(uint8_t)KEYSW3_GET();

	if(current_state_sw3 && !previous_state_sw3)
	{
		if((LIMIT_SHORT_L<KEYSW3_cntshort)&&(KEYSW3_cntshort<LIMIT_SHORT_H)&& !KEYSW3_longpressed)
		{
			KEYSW3_shortpressed=1;
		}                        
		KEYSW3_cntshort=0;
	}
	else if(!current_state_sw3 && previous_state_sw3)
	{
		KEYSW3_cntshort=0;
	}
	else if(!current_state_sw3 && !previous_state_sw3)
	{
		KEYSW3_cntshort++;
		if((LIMIT_T1_L<KEYSW3_cntshort)&&(KEYSW3_cntshort<LIMIT_T1_H))
		{
			KEYSW3_longpressed=1;
			KEYSW3_cntshort=0;
		}                 
	}
	previous_state_sw3=current_state_sw3;		
		/////////////////////KEY SW2/////////////////////////////////
	current_state_sw2=(uint8_t)KEYSW2_GET();

	if(current_state_sw2 && !previous_state_sw2)
	{
		if((LIMIT_SHORT_L<KEYSW2_cntshort)&&(KEYSW2_cntshort<LIMIT_SHORT_H)&& !KEYSW2_longpressed)
		{
			KEYSW2_shortpressed=1;
		}                        
		KEYSW2_cntshort=0;
	}
	else if(!current_state_sw2 && previous_state_sw2)
	{
		KEYSW2_cntshort=0;
	}
	else if(!current_state_sw2 && !previous_state_sw2)
	{
		KEYSW2_cntshort++;
		if((LIMIT_T1_L<KEYSW2_cntshort)&&(KEYSW2_cntshort<LIMIT_T1_H))
		{
			KEYSW2_longpressed=1;
			KEYSW2_cntshort=0;
		}                 
	}
	previous_state_sw2=current_state_sw2;		
	/////////////////////////////KEY SW1////////////////////////////////////
	current_state_sw1=(uint8_t)KEYSW1_GET();

	if(current_state_sw1 && !previous_state_sw1)
	{
		if((LIMIT_SHORT_L<KEYSW1_cntshort)&&(KEYSW1_cntshort<LIMIT_SHORT_H)&& !KEYSW1_longpressed)
		{
			KEYSW1_shortpressed=1;
		}
		KEYSW1_cntshort=0;			
	}
	else if(!current_state_sw1 && previous_state_sw1)
	{
		KEYSW1_cntshort=0;
	}
	else if(!current_state_sw1 && !previous_state_sw1)
	{	
		KEYSW1_cntshort++;
		if((LIMIT_T1_L<KEYSW1_cntshort)&&(KEYSW1_cntshort<LIMIT_T1_H))
		{
			KEYSW1_longpressed=1;
                        //KEYSET_cntshort=0;
		}		
	}
	previous_state_sw1=current_state_sw1;
	
	////////////////////////////////////////////////////////
		reset_digits(); //turn off all digits
		show_segments(str_segments[cur_seg]);
		if(dp_segments[cur_seg]) //turn on dp
			HAL_GPIO_WritePin(SEG_DP_GPIO_Port,SEG_DP_Pin,GPIO_PIN_RESET);			
		else
			HAL_GPIO_WritePin(SEG_DP_GPIO_Port,SEG_DP_Pin,GPIO_PIN_SET);			
		if(disp_Blink && flag_Blink)
		{
				HAL_GPIO_WritePin(digit_ports[cur_seg],digit_pins[cur_seg],GPIO_PIN_SET);//turn on current digits			
		}
		else
			HAL_GPIO_WritePin(digit_ports[cur_seg],digit_pins[cur_seg],GPIO_PIN_RESET);//turn on current digits
		cur_seg++;
		if(cur_seg==4) cur_seg=0;
	}
	if(htim->Instance==TIM11)
	{
		if(flag_blinkLED)
		{
			//setpwm_value(Duty_Value);
			//HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
		}
		else
		{
			setpwm_value(0);
			//HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_1);			
		}
		flag_blinkLED=1-flag_blinkLED;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_segment( char *str)
{
	uint8_t i=0,j=0;
	while(*(str+i))
	{
		if(*(str+i)=='.')
		{
			dp_segments[j-1]=1;
		}
		else
		{
			dp_segments[j]=0;
			str_segments[j]=*(str+i);
			j++;
		}
		i++;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void Blink_segments(uint8_t on_off)
{
	disp_Blink=on_off;
}
