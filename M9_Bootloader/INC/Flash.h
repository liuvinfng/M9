/**
  ******************************************************************************
  * @file    stm32f07x_flash.h
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
#ifndef __FLASH_H
#define __FLASH_H

#include "SysInitialize.h"

void Unlock_Flash(void);
void Lock_Flash(void);
void Write_Flash(uint32_t add,uint16_t data);
uint32_t Read_Flash(uint32_t add);
void Flash_Erase_Page(uint32_t add);
void Write_Flash_Half(uint32_t add,uint16_t data);

void Add_Life(uint32_t T);
void Clear_Life(void);


#endif /* __USART2_H */


