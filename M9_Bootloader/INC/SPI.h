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
#ifndef __SPI_H
#define __SPI_H

#include "SysInitialize.h"

#define SPI_I2S_FLAG_RXNE               ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE                ((uint16_t)0x0002)
#define I2S_FLAG_CHSIDE                 ((uint16_t)0x0004)
#define I2S_FLAG_UDR                    ((uint16_t)0x0008)
//#define SPI_FLAG_CRCERR                 ((uint16_t)0x0010)
#define SPI_FLAG_MODF                   ((uint16_t)0x0020)
#define SPI_I2S_FLAG_OVR                ((uint16_t)0x0040)
#define SPI_I2S_FLAG_BSY                ((uint16_t)0x0080)



u8 SPI1_ReadWriteByte(u8 txData);
u8 SPI2_ReadWriteByte(u8 txData);



#endif

