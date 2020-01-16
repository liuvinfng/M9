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
#ifndef __I2C_H
#define __I2C_H

#include "SysInitialize.h"

//#define I2C_SDA_OUT  GPIOA->CRH=(uint32_t)0x44441519;
//#define I2C_SDA_IN   GPIOA->CRH=(uint32_t)0x44441419;

//#define I2C_SDA_OUT   {GPIOE->CRH &=~(uint32_t)0xf0000000;GPIOE->CRH |= (uint32_t)0x50000000;}
//#define I2C_SDA_IN    {GPIOE->CRH &=~(uint32_t)0xf0000000;GPIOE->CRH |= (uint32_t)0x40000000;}

#define I2C_SDA_OUT   //{GPIOE->CRH &=~(uint32_t)0xf0000000;GPIOE->CRH |= (uint32_t)0x50000000;}
#define I2C_SDA_IN    //{GPIOE->CRH &=~(uint32_t)0xf0000000;GPIOE->CRH |= (uint32_t)0x40000000;}


#define DS1307_Address_Read   0xD1
#define DS1307_Address_Write  0xD0
#define I2C_READ  0x01
#define I2C_WRITE 0x00
#define DS1338_MARK	0xAA

void I2C_Start(void);
void I2C_Stop(void);
void I2C_Master_ACK(void);
uint8_t Check_Acknowledge(void);

void I2C_Transfer_Byte(uint8_t Data);
uint8_t I2C_Receive_Byte(void);

uint8_t I2C_Read_Data(uint8_t Address,uint8_t *Data);
uint8_t I2C_Read_16bit(uint8_t Address,volatile uint32_t *Data);
uint32_t I2C_Read_32bit(uint8_t Address);

uint8_t I2C_Write_Data(uint8_t Address,uint8_t Data);
uint8_t I2C_Write_16Bit(uint8_t Address,uint16_t Data);



#endif /* __USART2_H */


