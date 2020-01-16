/**
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H
#define __RTC_H

#include "SysInitialize.h"




void Set_RemoteClock(uint64_t data);
uint64_t Read_RemoteClock(void);
void Set_ClockReceived(void);
void Clear_ClcokReceive(void);
uint8_t Is_ClockReceived(void);
void Set_Remote_Schedule(void);



void RTC_Configuration(void);


void RTC_SetCurrentTime(uint32_t Hour,uint32_t Minute);
void RTC_SetAlarmTime(uint32_t Hour,uint32_t Minute);



void RTC_ResetAlarm(void);
uint8_t RTC_IsAlarm(void);

void RTC_DisableAlarm(void);

uint32_t RTC_GetCurrentMinutes(void);



extern volatile uint8_t g_alarm_flag;



#endif



