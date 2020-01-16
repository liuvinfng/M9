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


#define Error_Code_Gyro        0x10
#define Error_Code_LeftWheel   0x01
#define Error_Code_RightWheel  0x02
#define Error_Code_PickUp      0x03
#define Error_Code_Cliff       0x04
#define Error_Code_Bumper			 0x05
#define Error_Code_SideBrush   0x06
#define Error_Code_MainBrush   0x07
#define Error_Code_Stuck			 0x08
#define Error_Code_BTA				 0x09
#define Error_Code_Dustbin  	 0x0A
#define Error_Code_Encoder		 0x0B
#define Error_Code_Fan_H       0x0C
#define Error_Code_Fan_L       0x0D

#define LED_Plan  						 0x01
#define LED_Spot  						 0x02
#define LED_Clock  						 0x04
#define LED_Home  						 0x08
#define LED_Clean              0x10
#define LED_Colon 						 0x20
#define LED_Exclamation				 0x40
#define LED_Error              0x65

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
#endif /* __DISPLAY_H */

