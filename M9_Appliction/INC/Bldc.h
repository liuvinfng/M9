/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   Movement
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __BLDC_H
#define __BLDC_H

#include "SysInitialize.h"



#define VACUUM_STALL_LIMIT       		(uint16_t)18000 // 1500mA
#define VACUUM_STALL_NOLOAD       	(uint16_t)130 //400mA
#define VACUUM_STALL_SHORTCIRCUIT  	(uint16_t)2482 // 2a


extern volatile int32_t g_vac_speed;
extern volatile int32_t g_vac_temp_pwm;
extern volatile int32_t g_vac_pulse_cnt;

void Set_Vac_Speed(void);
void Vacuum_TurnOff(void);
void Vacuum_SetTempPWM(uint16_t P);
void Vacuum_SetSpeed(uint32_t S);
uint32_t Vacuum_GetSpeed(void);

uint8_t Get_VacMode(void);
void Set_VacMode(uint8_t data);
void Switch_VacMode(void);
void Vacuum_TuneProcess(void);

uint8_t Vacuum_IsFail(void);
void Vacuum_SetFailFlag(void);
void Vacuum_ResetFailFlag(void);

void Vacuum_SetCurrent(int16_t current);
int16_t Vacuum_GetCurrent(void);
uint16_t Vacuum_GetCurrentAdc(void);

uint8_t Vacuum_CheckCurrent(void); 

#endif /* __BLDC_H */






