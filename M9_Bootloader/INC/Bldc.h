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
#include "config.h"




extern volatile int32_t BLDC_Speed;
extern volatile int32_t BLDC_Temp_PWM;
extern volatile int32_t BLDC_Pulse_Counter;

void Set_Vac_Speed(void);
void Turn_BLDC_Off(void);
void Set_BLDC_TPWM(uint16_t P);
void Set_BLDC_Speed(uint32_t S);
void Set_BLDC_Fail(void);
void Clear_BLDC_Fail(void);
uint8_t Is_BLDC_Fail(void);
uint8_t Get_VacMode(void);
void Set_VacMode(uint8_t data);
void Switch_VacMode(void);
void Bldc_Loop(void);


#endif /* __BLDC_H */






