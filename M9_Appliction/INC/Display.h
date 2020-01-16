/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   Display Fuctions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "SysInitialize.h"
#include "Movement.h"


#define Display_Low            5
#define Display_Full           6

#define Plan                   ((uint16_t)0x0004)
#define Spot        					 ((uint16_t)0x0002)
#define Clean          				 ((uint16_t)0x0008)
#define Home        					 ((uint16_t)0x0001)
#define Dirt        					 ((uint16_t)0x0040)
#define Point       					 ((uint16_t)0x0020)
#define Sign        					 ((uint16_t)0x0010)

#define LED_BAT_R          ((uint16_t)0x0100)
#define LED_BAT_G          ((uint16_t)0x0200)
#define LED_BAT_Y	         ((uint16_t)0x0300)



void Display_Battery_Status(uint8_t temp);





void Display_Clean_Status(uint8_t temp);


void Set_LED(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6);
void Set_LED_Table(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6);



void Set_LED1_On_Switch(uint8_t led);
void Set_LED2_On_Switch(uint8_t led);
void Set_LED3_On_Switch(uint8_t led);
void Set_LED4_On_Switch(uint8_t led);
void Set_LED5_On_Switch(uint8_t led);
void Set_LED6_On_Switch(uint8_t led);
void Set_LED_On_Switch(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6);
void Set_LED1_On_Blink(uint8_t led);
void Set_LED2_On_Blink(uint8_t led);
void Set_LED3_On_Blink(uint8_t led);
void Set_LED4_On_Blink(uint8_t led);
void Set_LED5_On_Blink(uint8_t led);
void Set_LED6_On_Blink(uint8_t led);
void Set_LED_On_Blink(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4,uint8_t led5,uint8_t led6);
void LED_Display(void);
void Display_SetBattery(uint16_t bat);


void Display_Breath(void);
void Display_Process(CleanMode_t Mode);

uint8_t Get_Bat_Green(void);
void Set_Bat_Green(uint8_t dat);
uint8_t Get_Bat_Red(void);



#endif /* __DISPLAY_H */

