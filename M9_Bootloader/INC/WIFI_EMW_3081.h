/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    18-May-2016
  * @brief   WIFI EMW 3081 Header
  ******************************************************************************

  ******************************************************************************
  */  

#ifndef __WIFI_EMW_3081_H
#define __WIFI_EMW_3081_H

#include "gd32f30x.h"

typedef struct{
	volatile uint8_t APP_TRD;
	volatile uint8_t APP_DATA;
	volatile uint8_t APP_ADDCHK;
}Struct_App_Data;

typedef enum{
	Command_Easy_Link = 0xca,
	Command_Reset = 0xcb,
	Command_ProTest = 0xcc
}EWM_3081_Command;
typedef enum{
	App_WorkMode_Pause       = 0,
	App_WorkMode_Spot        = 0x01,
	App_WorkMode_Auto        = 0x02,
	App_WorkMode_Home        = 0x03,
	App_WorkMode_Wallfollow  = 0x04,
	App_WorkMode_Navigation  = 0x05,
}App_WorkMode_Enum;

#define App_WorkMode                  0xE1
#define App_OnOff_Direction_Forward   0xE2
#define App_OnOff_Direction_Backward  0xE3
#define App_OnOff_Direction_Left      0xE4
#define App_OnOff_Direction_Right     0xE5
#define App_DownAdjust                0xE6
#define App_SideAdjust                0xE7
#define App_CleaningSpeed             0xE8
#define App_RoomMode                  0xE9
#define App_Stop_Cleaning             0xEA

#define WIFI_Remote_Pause             (uint32_t)0X02550001
#define WIFI_Remote_Backward          (uint32_t)0X02550002
#define WIFI_Remote_BackwardPause     (uint32_t)0X02550003
#define WIFI_Remote_Forward           (uint32_t)0X02550006
#define WIFI_Remote_ForwardPause      (uint32_t)0X02550007
#define WIFI_Remote_Left              (uint32_t)0X02550008
#define WIFI_Remote_LeftPause         (uint32_t)0X02550009
#define WIFI_Remote_Right             (uint32_t)0X0255000a
#define WIFI_Remote_RightPause        (uint32_t)0X0255000b
#define WIFI_Remote_DirPause          (uint32_t)0X0255000c
#define WIFI_Remote_Auto              (uint32_t)0X0255000d
#define WIFI_Remote_Wallfollow        (uint32_t)0x0255000e
#define WIFI_Remote_SingleRoom        (uint32_t)0x0255000f
#define WIFI_Remote_Max               (uint32_t)0x02550010
#define WIFI_Remote_Normal            (uint32_t)0x02550020

uint8_t Get_WIFI_Turn_Mode(void);
void Set_WIFI_Turn_Mode(uint8_t code);

void Receive_WIFI_EMW3081(char data);
void Check_APP_Command(void);


void WIFI_Send_Status(void);
uint8_t Get_WIFI_WorkMode(void);
void WIFI_Set_WorkMode(char data);
void WIFI_Set_Forward(char data);
void WIFI_Set_Backward(char data);
void WIFI_Set_Left(char data);
void WIFI_Set_Right(char data);
void WIFI_Set_DownAdjust(char data);
void WIFI_Set_SideAdjust(char data);
void WIFI_Set_CleaningSpeed(char data);
void WIFI_Set_RoomMode(char data);
void WIFI_Set_BatteryCapacity(char data);
void WIFI_Set_ChargerStatus(char data);
void WIFI_Set_ErrorCode(char data);
void WIFI_Set_StopCleaning(char data);
uint8_t WIFI_GetBattery(void);
void Set_WIFIReport_Flag(void);
void Clear_WIFIReport_Flag(void);
uint8_t Is_WIFIReport_Flag(void);
void Period_WIFI_Report(uint8_t code);
void Bat_WIFI_Report(void);
void WIFI_Clear_Status(uint8_t code);
uint8_t Get_EM3081_Status(void);
void WIFI_Send_Setting(uint8_t Com);
void Clear_EM3081_Status(void);
void Set_EM3081_Status(uint8_t S);
void WIFI_Turn(uint16_t speed,uint16_t Dir);
void Display_WIFI_LED(void);
void Set_Display_WIFI_LED_Status(uint8_t code);
uint8_t Get_Display_WIFI_LED_Status(void);
void EMW3081_Vacuum_Power(void);

#endif




