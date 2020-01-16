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
#ifndef __USART_H
#define __USART_H

#include "SysInitialize.h"


#define Dev_USART0   0x01
#define Dev_USART1   0x02
#define Dev_USART2   0x04


#define USART_120M_115200                 (uint32_t)0X00000411 
#define USART_60M_115200                  (uint32_t)0X00000208 
#define USART_30M_115200                  (uint32_t)0X00000104

#define USART_120M_9600                   (uint32_t)0X000030D4 
#define USART_60M_9600                    (uint32_t)0X0000186A 
#define USART_30M_9600                    (uint32_t)0X00000C35

typedef struct
{
	uint8_t idx;
	uint8_t length;
}US_Queue_t;


void USART0_Configuration(void);
void USART1_Configuration(void);
void USART2_Configuration(void);

void USART0_Transmit_Byte(char Data);
void USARTMY_Transmit_Byte(char Data);
void USART1_Transmit_Byte(unsigned char Data);
void USART2_Transmit_Byte(char Data);

void USART0_Transmit_String(uint16_t Length,char *data);
void USART1_Transmit_String(uint16_t Length,char *data);
void USART2_Transmit_String(uint16_t Length,char *data);

void USART0_DMA_String(uint16_t length,char *data);
void USART1_DMA_String(uint16_t Length,char *data);
void USART2_DMA_String(uint16_t Length,char *data);

void Debug_Print(char *data,uint8_t Dev);
void USART3_Print(char *data);
void USART1_DMA_Numbers(int32_t numbers);
void Debug_DMA_Numbers(int32_t numbers);

void USARTX_DMA_Numbers(uint8_t x,int32_t numbers);


void ups(char * Data);
void upn(int32_t data);

void ups3(char * Data);

void Debug_DMA_Write_String(uint16_t length,char *Data);
void Debug_Write_String(uint16_t length,char *Data);
void Debug_Write_Byte(char ch);
void Gyro_DMA_Write_String(uint16_t length,char *Data);
void WIFI_DMA_Write_String(uint16_t length,char *Data);


void USART3_CreateTask(void);
void USART3_PrintTask(void *p_agument);
void USART3_PrintStringByLength(char *p_data,uint8_t length);

void USART0_PrintfDebugInfo(char *pStr,...);
 




#endif /* __USART_H */



