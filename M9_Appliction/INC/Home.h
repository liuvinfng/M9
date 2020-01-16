/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __Home_H
#define __Home_H



#include "SysInitialize.h"

#define Searching   0x01
#define MoveAround  0x02
#define InPath      0x04
#define Near        0x08
#define InFront     0x10
#define Round_Left  0x01
#define Round_Right 0x02

#define Charge_Station_At_Left    (uint8_t)1
#define Charge_Station_At_Right   (uint8_t)2
#define Charge_Station_Near				(uint8_t)3
void GoHome(void);
void By_Path(void);
void Around_ChargerStation(uint8_t Dir);
void Search_For_ChargeStation(void);
uint8_t Home_Check_Current(void);
void Home_Motor_Set(void);
uint8_t Check_Top_Rcon(void);
uint8_t Check_Left_Rcon(void);
uint8_t Check_Right_Rcon(void);
void Tiny_Back(void);
void Minus_Right(uint16_t Steps);
uint8_t Is_InfrontOfStation(void);
uint8_t Check_Position(uint8_t Dir);
uint8_t Turn_Connect(void);
void SetHomeRemote(void);
void ResetHomeRemote(void);
uint8_t IsHomeRemote(void);
void Display_Home_LED(void);
void Turn_BackOnBase(void);
uint8_t Forward_Distance(uint8_t Speed,uint32_t Dis);
void Wake_Up_Adjust(void);

#endif /*----Behaviors------*/





