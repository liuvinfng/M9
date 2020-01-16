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
#include "include.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"

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


void LED_Display(void)//0.1ms
{
	static uint8_t LED_Cycle_Counter=1,Home_Flag=0;
	static uint16_t blink_cnt=0,blink_total=5000,wifi_blink_cnt=0,wifi_blink_total=0;
	if(LED1_On_Switch)LED1_On_Blink=0;
	if(LED2_On_Switch)LED2_On_Blink=0;	
	if(LED3_On_Switch)LED3_On_Blink=0;	
	if(LED4_On_Switch)LED4_On_Blink=0;
	if(LED5_On_Switch)LED5_On_Blink=0;	
	if(LED6_On_Switch)LED6_On_Blink=0;		
	
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

	
	if(LED1_On_Switch==0 && LED1_On_Blink==0)LED1_Off;
	if(LED2_On_Switch==0 && LED2_On_Blink==0)LED2_Off;	
	if(LED3_On_Switch==0 && LED3_On_Blink==0)LED3_Off;
	if(LED4_On_Switch==0 && LED4_On_Blink==0)LED4_Off;
	if(LED5_On_Switch==0 && LED5_On_Blink==0)LED5_Off;	
	if(LED6_On_Switch==0 && LED6_On_Blink==0)LED6_Off;	
}
/*------------------------------------------------Display Battery-------------------------*/
void Display_Battery_Status(uint8_t temp)
{
	/*if(temp==Display_Full)
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
	if(temp==Display_Full)
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
//	switch(temp)
//	{
//		case Display_Clean : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;  			                            
//		case Display_Wall  : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,1,0,1,0,0); break;              
//		case Display_Zizag : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;               
//		case Display_Remote: Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(0,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;
//		case Display_Test  : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(1,0,1,1,0,0);Set_LED(100,100,100,100,100,100); break;	
//		case Display_Home  : Set_LED_On_Blink(0,0,0,0,0,0);Set_LED_On_Switch(1,0,0,1,0,0); break;	
//		case Display_Spot  : Set_LED_On_Switch(0,0,0,1,0,0);Set_LED_On_Blink(0,1,0,0,0,0); break;	
//		case Display_Userinterface  
//											 : if(Error_GetCode()==ERROR_NONE)
//												 {
//													Set_LED_On_Switch(1,1,1,1,0,0);Set_LED_On_Blink(0,0,0,0,0,0);
//												 } 
//												 break;			
//		default:break;		
//	}		
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

//bat display
uint8_t g_green=0,g_red=0;
void Display_SetBattery(uint16_t bat)
{
	if(bat == LED_BAT_R)
	{
		g_green = 0;
		g_red   = 1;		
	}	
	else if(bat == LED_BAT_G)
	{
		g_green = 1;
		g_red   = 0;			
	}	
	else if(bat == LED_BAT_Y)
	{
		g_green = 1;
		g_red   = 1;			
	}
	Set_LED4_On_Switch(g_green);
	Set_LED5_On_Switch(g_red);	
}
uint8_t Get_Bat_Green(void)
{
	return g_green;
}
void Set_Bat_Green(uint8_t dat)
{
	g_green = dat;
}
uint8_t Get_Bat_Red(void)
{
	return g_red;
}

/*process display function*/
void Display_Process(CleanMode_t Mode)
{
	uint8_t breath_flag=0;
	switch(Mode)
	{
		case MODE_NAVIGATION 		: Set_LED_On_Blink(0,0,0,0,0,1);Set_LED_On_Switch(0,0,1,Get_Bat_Green(),Get_Bat_Red(),0); break;  			                            
		case MODE_WALL  				: Set_LED_On_Blink(0,0,0,0,0,1);Set_LED_On_Switch(0,1,0,Get_Bat_Green(),Get_Bat_Red(),0); break;                            
		case MODE_REMOTE				: Set_LED_On_Blink(0,0,0,0,0,1);Set_LED_On_Switch(0,0,1,Get_Bat_Green(),Get_Bat_Red(),0); break;
		case MODE_TEST					: Set_LED_On_Blink(0,0,0,0,0,1);Set_LED_On_Switch(1,0,1,Get_Bat_Green(),Get_Bat_Red(),0); break;	
		case MODE_HOME  				: Set_LED_On_Blink(0,0,0,0,0,1);Set_LED_On_Switch(1,0,0,Get_Bat_Green(),Get_Bat_Red(),0); break;	
		case MODE_SPOT  				: Set_LED_On_Blink(0,1,0,0,0,1);Set_LED_On_Switch(0,0,0,Get_Bat_Green(),Get_Bat_Red(),0); break;			
		case MODE_USERINTERFACE	: 
														if(Error_GetCode()==ERROR_NONE) 
														{
															Set_LED_On_Switch(1,1,1,Get_Bat_Green(),Get_Bat_Red(),0);
															Set_LED_On_Blink(0,0,0,0,0,1);															
														}	
														breath_flag=1;
														break;			
		default:break;		
	}
	if(breath_flag)
	{
		Display_Breath();
	}
	else
	{
		Set_LED(100,100,100,100,100,100); 
	}	
}
void Display_Breath(void)//20ms
{
	static int8_t breath_steps = 1,breath_cnt = 60;	
	if(breath_steps)
	{
		breath_cnt++;
		if(breath_cnt>97)
		{
			breath_steps=0;
		}	
	}
	else
	{
		breath_cnt--;
		if(breath_cnt<3)
		{
			breath_steps=1;
		}	
	}
	if(Is_ChargerOn())
	{
		Set_LED_Table(0,0,0,breath_cnt,breath_cnt,breath_cnt);
	}							
	else if(Mode_GetMode()==MODE_USERINTERFACE)
	{
		Set_LED_Table(breath_cnt,breath_cnt,breath_cnt,60,breath_cnt,breath_cnt);
//		Set_LED_Table(60,60,60,60,breath_cnt,breath_cnt);		
	}	
	else
	Set_LED_Table(breath_cnt,breath_cnt,breath_cnt,breath_cnt,breath_cnt,breath_cnt);	
}



