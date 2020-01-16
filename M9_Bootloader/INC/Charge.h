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

#ifndef __Charge_H
#define __Charge_H

#include "SysInitialize.h"

#define CC_Charge 1
#define CV_Charge 2
#define Finished_Charge 3
#define Battery_FullVoltage 1660
#define Charge_FinishCurrent 50
#define Charge_Target_Current 600

typedef enum
{
	CHARGE_IDLE = 0,
	CHARGE_INIT = 1,
	CHARGEING = 2,
	CHARGE_FINISH = 3,
	CHARGE_EXIT = 4,
}Charge_Step_t;


typedef enum
{
	CC_CHARGE = 1,
	CV_CHARGE = 2,
	FINISH_CHARGE = 3,
}Charge_Mode_t;

typedef enum
{
	CHARGE_STATION = 1,
	ADAPTER = 2,
}Charge_Type_t;



typedef struct
{
	uint8_t power_switch;
	Charge_Type_t type;        //充电座、线充
	Charge_Mode_t charge_mode; //恒流充、恒压充、充电完成
	Charge_Step_t step;
	uint32_t time;
}Charge_t;

extern volatile Charge_t g_charge;



uint16_t Get_Charge_Current(uint16_t BaselineADCV);
uint16_t Get_Sys_Current(uint16_t BaselineADCV);

void Charge_SetSwitch(uint8_t state);
void Charge_Process(void) ;

uint16_t GetBaseLineADCV(void);







#endif /*----Charge------*/





