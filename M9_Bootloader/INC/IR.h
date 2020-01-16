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
#ifndef __IR_H
#define __IR_H

#include "SysInitialize.h"

#define IR_Count_Range          (uint16_t)10  //38K
#define IR_Prescaler            (uint16_t)111  

#define IR_H  //{TIM15->CCR1=3;TIM15->CCR2=3;}
#define IR_L  //{TIM15->CCR1=0;TIM15->CCR2=0;}

void IR_Transmite_Data(uint32_t data);
void IR_Timer_Configure(void);
void IR_Transmite_Header(void);
void IR_Transmite_Source(uint32_t data);
void IR_Transmite_Status(uint8_t Step,uint16_t data);
void IR_Transmite_Error(uint8_t Step,uint16_t code,uint16_t data);
void IR_Transmite_Cycling(uint8_t Step,uint16_t data);

#endif /* __ */


