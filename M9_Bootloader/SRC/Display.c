/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Display Fuctions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Display.h"
#include "SysInitialize.h"
#include "Speaker.h"
#include "Movement.h"
#include "SPI.h"

volatile uint8_t LED1_On_Counter=0,LED2_On_Counter=0,LED3_On_Counter=0,LED4_On_Counter=0,LED5_On_Counter=0,LED6_On_Counter=0;
volatile uint8_t LED1_On_Switch=0,LED2_On_Switch=0,LED3_On_Switch=0,LED4_On_Switch=0,LED5_On_Switch=0,LED6_On_Switch=0;
volatile uint8_t LED1_On_Blink=0,LED2_On_Blink=0,LED3_On_Blink=0,LED4_On_Blink=0,LED5_On_Blink=0,LED6_On_Blink=0;

const uint8_t One_Display_Light[100]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,68,70,72,74,76,78,80,82,
	                                    84,86,88,90,92,94,96,98,100,96,92,89,86,83,80,77,74,71,68,65,62,59,56,53,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2};

void Set_LED1_On_Switch(uint8_t led)
{
	LED1_On_Switch = led;
}
void Set_LED2_On_Switch(uint8_t led)
{
	LED2_On_Switch = led;
}
void Set_LED3_On_Switch(uint8_t led)
{
	LED3_On_Switch = led;
}
void Set_LED4_On_Switch(uint8_t led)
{
	LED4_On_Switch = led;
}
void Set_LED5_On_Switch(uint8_t led)
{
	LED5_On_Switch = led;
}
void Set_LED6_On_Switch(uint8_t led)
{
	LED6_On_Switch = led;
}

void Set_LED_On_Switch(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6)
{
	Set_LED1_On_Switch(led1);
	Set_LED2_On_Switch(led2);
	Set_LED3_On_Switch(led3);
	Set_LED4_On_Switch(led4);
	Set_LED5_On_Switch(led5);
	Set_LED6_On_Switch(led6);
}

void Set_LED1_On_Blink(uint8_t led)
{
	LED1_On_Blink = led;
}
void Set_LED2_On_Blink(uint8_t led)
{
	LED2_On_Blink = led;
}
void Set_LED3_On_Blink(uint8_t led)
{
	LED3_On_Blink = led;
}
void Set_LED4_On_Blink(uint8_t led)
{
	LED4_On_Blink = led;
}
void Set_LED5_On_Blink(uint8_t led)
{
	LED5_On_Blink = led;
}
void Set_LED6_On_Blink(uint8_t led)
{
	LED6_On_Blink = led;
}

void Set_LED_On_Blink(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6)
{
	Set_LED1_On_Blink(led1); 
	Set_LED2_On_Blink(led2); 
	Set_LED3_On_Blink(led3); 
	Set_LED4_On_Blink(led4); 
	Set_LED5_On_Blink(led5); 
	Set_LED6_On_Blink(led6); 	
}
void LED_Display(void)
{
	static uint8_t LED_Cycle_Counter=1,Home_Flag=0,cop_cnt=0;;
	static uint16_t blink_cnt=0,blink_total=5000,wifi_blink_cnt=0,wifi_blink_total=0;
	if(LED1_On_Switch)LED1_On_Blink=0;
	if(LED2_On_Switch)LED2_On_Blink=0;	
	if(LED3_On_Switch)LED3_On_Blink=0;	
	if(LED4_On_Switch)LED4_On_Blink=0;
	if(LED5_On_Switch)LED5_On_Blink=0;	
	LED6_On_Blink=1;		
	
	if(LED1_On_Blink)LED1_On_Switch=0;	
	if(LED2_On_Blink)LED2_On_Switch=0;	
	if(LED3_On_Blink)LED3_On_Switch=0;	
	if(LED4_On_Blink)LED4_On_Switch=0;	
	if(LED5_On_Blink)LED5_On_Switch=0;	
	if(LED6_On_Blink)LED6_On_Switch=0;	
	
	if(LED_Cycle_Counter<99)LED_Cycle_Counter++;
	else LED_Cycle_Counter = 1;	
	if(blink_cnt<10000)blink_cnt++;
	else blink_cnt=0;

	if(g_copy_led)
	{
		if(blink_cnt==blink_total)
		{
			cop_cnt++;
			if(cop_cnt>4)cop_cnt=1;
		}
		if(cop_cnt==1)
		{
			LED1_On;			
		}
		if(cop_cnt==2)
		{
			LED1_On;	
			LED2_On;				
		}
		if(cop_cnt==3)
		{
			LED1_On;	
			LED2_On;	
			LED3_On;				
		}
		if(cop_cnt==4)
		{
			LED1_Off;	
			LED2_Off;	
			LED3_Off;		
		}			
	}
			
	
	if(SysConfig.Ty_WifiState == 1)
	{
		wifi_blink_total = 2500;
		if(wifi_blink_cnt<5000)wifi_blink_cnt++;
		else wifi_blink_cnt=0;	
	}
	else if(SysConfig.Ty_WifiState == 2)
	{
		wifi_blink_total = 15000;
		if(wifi_blink_cnt<30000)wifi_blink_cnt++;
		else wifi_blink_cnt=0;	
	}	
	else if(SysConfig.Ty_WifiState == 3 ||SysConfig.Ty_WifiState == 6)
	{
		wifi_blink_total = 0;
		wifi_blink_cnt = 1;	
	}		
	else if(SysConfig.Ty_WifiState == 4 ||SysConfig.Ty_WifiState == 5)
	{
		wifi_blink_total = 1;
		wifi_blink_cnt = 0;	
	}	
	else
	{
		wifi_blink_total = 0;
		wifi_blink_cnt = 1;	
	}	
	
	if(LED1_On_Switch)
	{
		if(Mode_GetMode()==MODE_HOME)
		{
			LED1_On_Counter=blink_cnt/100;
			if(LED1_On_Counter>99)Home_Flag=1-Home_Flag;
			if(Home_Flag)
			{
				LED1_On_Counter=LED1_On_Counter;
			}
			else
			{
				LED1_On_Counter=100-LED1_On_Counter;
			}
			if(LED1_On_Counter<1)LED1_On_Counter=0;
		}			
		if(LED1_On_Counter>=LED_Cycle_Counter)
		{
			LED1_On;
		}
		else
		{
			LED1_Off;
		}
	}
	if(LED2_On_Switch)
	{		
		if(LED2_On_Counter>=LED_Cycle_Counter)
		{
			LED2_On;
		}
		else
		{
			LED2_Off;
		}
	}
	if(LED3_On_Switch)
	{
		if(LED3_On_Counter>=LED_Cycle_Counter)
		{
			LED3_On;
		}
		else
		{
			LED3_Off;
		}
	}
	if(LED4_On_Switch)
	{
		if(LED4_On_Counter>=LED_Cycle_Counter)
		{
			LED4_On;
		}
		else
		{
			LED4_Off;
		}
	}
	if(LED5_On_Switch)
	{
		if(LED5_On_Counter>=LED_Cycle_Counter)
		{
			LED5_On;
		}
		else
		{
			LED5_Off;
		}
	}
	if(LED6_On_Switch)
	{
		if(LED6_On_Counter>=LED_Cycle_Counter)
		{
			LED6_On;
		}
		else
		{
			LED6_Off;
		}
	}
	
	if(LED1_On_Blink)
	{
		if(blink_cnt<blink_total)
		{
			LED1_On;
		}
		else
		{
			LED1_Off;
		}
	}	
	if(LED2_On_Blink)
	{
		if(blink_cnt<blink_total)
		{
			LED2_On;
		}
		else
		{
			LED2_Off;
		}
	}	
	if(LED3_On_Blink)
	{
		if(blink_cnt<blink_total)
		{
			LED3_On;
		}
		else
		{
			LED3_Off;
		}
	}
	if(LED4_On_Blink)
	{
		if(blink_cnt<blink_total)
		{
			LED4_On;
		}
		else
		{
			LED4_Off;
		}
	}
	if(LED5_On_Blink)
	{
		if(blink_cnt<blink_total)
		{
			LED5_On;
		}
		else
		{
			LED5_Off;
		}
	}
	if(LED6_On_Blink)
	{
		if(wifi_blink_cnt<wifi_blink_total)
		{
			LED6_On;
		}
		else
		{
			LED6_Off;
		}
	}
	
	if(!g_copy_led)
	{
		if(LED1_On_Switch==0 && LED1_On_Blink==0)LED1_Off;
		if(LED2_On_Switch==0 && LED2_On_Blink==0)LED2_Off;	
		if(LED3_On_Switch==0 && LED3_On_Blink==0)LED3_Off;
		if(LED4_On_Switch==0 && LED4_On_Blink==0)LED4_Off;
		if(LED5_On_Switch==0 && LED5_On_Blink==0)LED5_Off;	
		if(LED6_On_Switch==0 && LED6_On_Blink==0)LED6_Off;	
	}
	
}
/*------------------------------------------------Display Battery-------------------------*/
void Display_Battery_Status(uint8_t temp)
{
	if(temp==Display_Full)
	{
		Set_LED4_On_Switch(1);
		Set_LED5_On_Switch(0);
		Set_LED5_On_Blink(0);
		Set_LED(100,100,100,100,100,100);
	}
	else
	{
		Set_LED4_On_Switch(1);
		Set_LED5_On_Switch(1);	
	}
	/*if(temp==Display_Full)
	{
		Set_LED_On_Switch(1,1,1,1,0,0);		
	}
	else
	{		
		switch(temp)
		{
			case 1:Set_LED_On_Switch(1,0,0,0,0,0);
						break;
			case 2:Set_LED_On_Switch(1,1,0,0,0,0);
						break;
			case 3:Set_LED_On_Switch(1,1,1,0,0,0);
						break;
			case 4:Set_LED_On_Switch(1,1,1,0,0,0);
						break;
			default:Set_LED_On_Switch(0,0,0,0,0,0);
						break;
		}
	}
	Set_LED(100,100,100,100,100,100);*/
	
}


void Display_Clean_Status(uint8_t temp)
{
	switch(temp)
	{
		case Display_Clean : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;  			                            
		case Display_Wall  : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,1,0,1,0,0); break;              
		case Display_Zizag : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;               
		case Display_Remote: Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;
		case Display_Test  : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(1,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;	
		case Display_Home  : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(1,0,0,1,0,0); break;	
		case Display_Spot  : Set_LED_On_Switch(0,0,0,1,0,0);Set_LED_On_Blink(0,1,0,0,0,0); break;	
		case Display_Userinterface  
											 : Set_LED_On_Switch(1,1,1,1,0,0);Set_LED_On_Blink(0,0,0,0,0,0); break;			
		default:break;		
	}		
}

/*----------------------------------------------------------Set led ----------------*/
void Set_LED(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6)
{ 
	LED1_On_Counter=led1;
	LED2_On_Counter=led2;	
	LED3_On_Counter=led3;	
	LED4_On_Counter=led4;
	LED5_On_Counter=led5;
	LED6_On_Counter=led6;	
}

void Set_LED_Table(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6)
{
	LED1_On_Counter=One_Display_Light[led1];
	LED2_On_Counter=One_Display_Light[led2];	
	LED3_On_Counter=One_Display_Light[led3];	
	LED4_On_Counter=One_Display_Light[led4];	
	LED5_On_Counter=One_Display_Light[led5];	
	LED6_On_Counter=One_Display_Light[led6];	
}

void Display_SetBattery(uint16_t bat)
{
	if(bat == LED_BAT_R)
	{
		Set_LED4_On_Switch(0);
		Set_LED5_On_Switch(1);		
	}	
	else if(bat == LED_BAT_G)
	{
		Set_LED4_On_Switch(1);
		Set_LED5_On_Switch(0);	
	}	
	else if(bat == LED_BAT_Y)
	{
		Set_LED4_On_Switch(1);
		Set_LED5_On_Switch(1);	
	}
}
