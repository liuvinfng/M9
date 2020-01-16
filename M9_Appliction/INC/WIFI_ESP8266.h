/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    24-May-2016
  * @brief   WIFI ESP8266 Header
  ******************************************************************************

  ******************************************************************************
  */  

#ifndef __WIFI_ESP8266_H
#define __WIFI_ESP8266_H

#include "gd32f30x.h"
#include "RTC.h"

#ifdef WIFI_ESP8266

typedef enum{
	W_Null=0,
	W_GetBatteryInfo=2,
	W_GetChargeState=3,
	W_GetVersion=4,
	W_GetLog=6,
	W_GetSched=7,
	W_GetCleanState=8,
	W_AddSched=9,
	W_DelSched=10,
	W_ModSched=11,
	W_SetTime=12,
	W_GetLifeSpan_SideBrush=13,
	W_GetLifeSpan_Brush=14,
	W_GetLifeSpan_DustCaseHeap=15,
	W_Move_Forward=21,
	W_Move_Backward=22,
	W_Move_SpinRight=23,
	W_Move_SpinLeft=24,
	W_Move_Stop=25,
	W_Move_Brake=26,
	W_Charge_Go=31,
	W_Charge_StopGo=32,
	W_Charge_Going=33,
	W_Charge_SlotCharging=34,
	W_Charge_WireCharging=35,
	W_Charge_Idle=36,
	W_Clean_Auto=51,
	W_Clean_Spot=52,
	W_Clean_Border=53,
	W_Clean_SingleRoom=54,
	W_Clean_Stop=55,
	W_Clean_SpotArea=56,
	W_Clean_Idle=57,
	W_WIFIState_Idle=61,
	W_WIFIState_Smart=62,
	W_WIFIState_Connecting=63,
	W_WIFIState_Connected=64,
	W_Speed_Standard=70,
	W_Speed_Strong=71,
	W_Answer_Ok=91,
	W_Answer_Fail=92,
	W_GetDeviceCap=99,
	W_SetSideBrushLife=101,
	W_SetBrushLife=102,
	W_SetDustCaseHeapLife=103,
	W_DCStatus=104
}WIFI_Rec;

typedef enum{
	SD_OF=0,
	SD_H=1,
	SD_M=2
}WIFI_SD;

typedef enum{
	Log_Robot_Hang=1,
	Log_Robot_Wheel=2,
	Log_Robot_Stuck=3,
	Log_Robot_Cliff=6,
	Log_Robot_StartClean=7,
	Log_Robot_EndClean=8,
}Log_State;

typedef struct{        
	DT Time;   
	uint8_t Evt;
}Log_Type;

typedef enum{
	Life_Brush=1,
	Life_SideBrush=2,
	Life_DustCaseHeap=3
}Life_Span_Type;


#define RECEIVE_BUFFER_SIZE  (uint32_t)500
#define RECEIVE_SET_SIZE  (uint32_t)2//only can set to 2

#define ESP8266_FA 0x01
#define ESP8266_FB 0x02
#define ESP8266_FC 0x04

#define ESP8266_WIFIStatus_Idle 				(uint8_t)0x0
#define ESP8266_WIFIStatus_Connecting		(uint8_t)0x01
#define ESP8266_WIFIStatus_Connected    (uint8_t)0x02

void Send_ESP8266_Setting(char *data);
uint8_t Get_String_CRC(char *data);

void Set_ESP8266_Received(void);
void Clear_ESP8266_Receive(void);
uint8_t Is_ESP8266_Received(void);

void ESP8266_Check_Data(void);
void Receive_ESP8266(char data);

WIFI_Rec ESP8266_Data_Extract(char *data,uint32_t Length);

void TEST_ES(void);

void Reset_ESP8266(void);

void Add_String(char *Orig,char *St,char *name,char *output);
void Add_UserID_String(char *Orig,char *UD);

void ESP8266_Report_Battery(uint32_t V,uint8_t FAB);
void ESP8266_Report_ChargeState(WIFI_Rec S,uint8_t FAB);
void ESP8266_Report_CleanState(uint8_t S,uint8_t FAB);
void ESP8266_FeedDog(void);
void ESP8266_Report_Error(uint8_t E);
void ESP8266_Clear_Error(void);
void ESP8266_Report_Sleep(void);
uint8_t ESP8266_Set_Time(char *data);
void ESP8266_Report_Schedule(uint8_t FAB);
uint8_t ESP8266_Add_Schedule(char *data);
uint8_t ESP8266_Mod_Schedule(char *data);
uint8_t ESP8266_Del_Schedule(char *data);
void Remote_Add_Schedule(uint16_t Sched);
void Remote_Del_Schedule(void);
void ESP8266_Report_Log(void);
void ESP8266_Report_Version(void);
void ESP8266_Report_LifeSpan(Life_Span_Type Type);
void ESP8266_Reset_LifeSpan(WIFI_Rec T);
void ESP8266_Initialize_Module(void);

void Sort_Schedule_Data(void);
	
void Set_Charge_State(WIFI_Rec S);
WIFI_Rec Get_Charge_State(void);

void Set_Clean_State(uint8_t S);
uint8_t Get_Clean_State(void);

uint8_t Need_Feed_Dog(void);
void Set_Feed_Dog(void);
void Clear_Feed_Dog(void);

void Check_And_FeedDog(void);

void ESP8266_Get_ID(char *input);

void Report_Battery_Onchange(void);
void Report_CleanState_Onchange(void);
void Report_Error_Onchange(void);
uint8_t Get_ESP8266_ErrorCode(uint8_t Error);

void ESP8266_Event(void);

void Set_WIFI_Status(uint8_t S);
uint8_t Get_WIFI_Status(void);

void ESP8266_Answer_OF(uint8_t S);
void ESP8266_CEN55_Answer(uint8_t S);

void Store_SchedName(uint8_t Order,char *schdata);
//char *Get_SchedName_Position(char *data);

void Print_Scheduled_Time(void);
//void Temp_SID(char * data);
void Add_Sched_String(char *data);
void Add_Sched_Content(ESP8266_Sched data);
ESP8266_Sched Get_Scheduled_Data(uint8_t d);
void Check_and_SetAlarm(void);

void Log_Robot_Status(uint8_t St);
void Reset_Robot_Log(void);
Log_Type Get_Log_Content(uint8_t O);
uint32_t Get_LifeSpan(Life_Span_Type t);
void Clear_LifeSpan(WIFI_Rec T);

void Test_ESP8266_Module(void);
void ESP8266_Send_TestCommand(void);

void Get_WIFI_ID(char *data);
void Set_WIFI_ID(char *data);

void Receiving_WIFI_ID(void);
uint8_t Check_WIFI_ID(void);
void Get_Rec_ID(char *data);

#endif/*esp8266*/

#endif


