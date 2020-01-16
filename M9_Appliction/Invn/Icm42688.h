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

#ifndef __ICM42688_H
#define __ICM42688_H

#include "SysInitialize.h"

uint64_t inv_icm426xx_get_time_us(void);
void inv_icm42688_time_us_increase(void);

void inv_icm426xx_sleep_us(uint32_t us);
void inv_icm426xx_sleep_us_increase(void);

void inv_helper_disable_irq(void);
void inv_helper_enable_irq(void);


void Icm42688_Init(void);

void Set_Icm42688_Interr(uint8_t dat);
uint8_t Get_Icm42688_Interr(void);
void Icm42688_Loop(void);

#endif




