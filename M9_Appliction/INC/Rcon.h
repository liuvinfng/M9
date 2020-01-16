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

#ifndef __RCON_H
#define __RCON_H

#include "SysInitialize.h"




#define Charge_Home_Left_Side     (uint8_t)0xb0
#define Charge_Home_Right_Side    (uint8_t)0xb2
#define Charge_Home_Left          (uint8_t)0xb4
#define Charge_Home_Right         (uint8_t)0xb8
#define Charge_Home_Top           (uint8_t)0xf0
#define Vitual_Wall_Code          (uint8_t)0xaa
#define Vitual_Wall_Code_Top      (uint8_t)0xaa



#define RconL_HomeL          (uint32_t)0x00000080
#define RconL_HomeR          (uint32_t)0x00000040

#define RconFL_HomeL         (uint32_t)0x00000020
#define RconFL_HomeR         (uint32_t)0x00000010

#define RconFR_HomeL         (uint32_t)0x00000008
#define RconFR_HomeR         (uint32_t)0x00000004

#define RconR_HomeL          (uint32_t)0x00000002
#define RconR_HomeR          (uint32_t)0x00000001

#define RconR_HomeT          (uint32_t)0x00000100
#define RconFR_HomeT         (uint32_t)0x00000200
#define RconFL_HomeT         (uint32_t)0x00000400
#define RconL_HomeT          (uint32_t)0x00000800

#define RconBR_HomeL         (uint32_t)0x00001000
#define RconBR_HomeR         (uint32_t)0x00002000
#define RconBR_HomeT         (uint32_t)0x00004000

#define RconBL_HomeL         (uint32_t)0x00010000
#define RconBL_HomeR         (uint32_t)0x00020000
#define RconBL_HomeT         (uint32_t)0x00040000

#define RconR_LEFT		       (uint32_t)0x00100000
#define RconFR_LEFT		       (uint32_t)0x00200000
#define RconFL_LEFT		       (uint32_t)0x00400000
#define RconL_LEFT		       (uint32_t)0x00800000
#define RconBL_LEFT		       (uint32_t)0x01000000
#define RconBR_LEFT		       (uint32_t)0x02000000

#define RconR_RIGHT          (uint32_t)0x04000000
#define RconFR_RIGHT         (uint32_t)0x08000000
#define RconFL_RIGHT         (uint32_t)0x10000000
#define RconL_RIGHT          (uint32_t)0x20000000
#define RconBL_RIGHT         (uint32_t)0x40000000
#define RconBR_RIGHT         (uint32_t)0x80000000


#define RconR_Wall					 (uint32_t)0xffffffff
#define RconFR_Wall					 (uint32_t)0xffffffff
#define RconFL_Wall					 (uint32_t)0xffffffff
#define RconL_Wall					 (uint32_t)0xffffffff
#define RconBL_Wall					 (uint32_t)0xffffffff
#define RconBR_Wall					 (uint32_t)0xffffffff
#define RconR_Wall_T         (uint32_t)0xffffffff
#define RconFR_Wall_T        (uint32_t)0xffffffff
#define RconFL_Wall_T        (uint32_t)0xffffffff
#define RconL_Wall_T         (uint32_t)0xffffffff
#define RconBL_Wall_T        (uint32_t)0xffffffff
#define RconBR_Wall_T        (uint32_t)0xffffffff

#define Remote_None                   (uint32_t)0
#define Remote_Power                  (uint32_t)0x00ff00fe
#define Remote_Wifi					  			  (uint32_t)0x00ff10ee	
#define Remote_Clean                  (uint32_t)0x00ff40be
#define Remote_Forward                (uint32_t)0x00ffc03e
#define Remote_Left                   (uint32_t)0x00ff20de
#define Remote_Max                    (uint32_t)0x00ffe01e
#define Remote_Right                  (uint32_t)0x00ff609e
#define Remote_Home                   (uint32_t)0x00ff48b6
#define Remote_Random	                (uint32_t)0x00ff807e
#define Remote_Mop                    (uint32_t)0x00ff28d6	
#define Remote_Vac_Quiet              (uint32_t)0x00ff906e	
#define Remote_Vac_Standard           (uint32_t)0x00ff08f6	
#define Remote_Vac_Strong             (uint32_t)0x00ff8876	
#define Remote_Shut_Up                (uint32_t)0x00ffa05e		

#define Remote_Navigation             (uint32_t)0x02AA11ee	
#define Remote_Plan                   (uint32_t)0x02AA11ee
#define Remote_Zigzag                 (uint32_t)0x02AA11ee
#define Remote_Wall              			(uint32_t)0x02AA11ee
#define Remote_Spot                   (uint32_t)0x02AA11ee	



#define Rcon_Signal_LRFLFR_T		      (uint32_t)0x00000F00
#define RconFront_Home_T              (uint32_t)0x00000600


typedef struct{
	volatile uint8_t Remote_Flag;
  volatile uint16_t Temp_Counter;
	volatile uint32_t Temp_Code;
  volatile uint8_t Receive_Start;
	volatile uint8_t BitCounter;
	volatile uint16_t Time_Counter;
}Rcon_Element;

uint32_t Wifi_GetRemoteCode(void);
void Wifi_SetRemoteCode(uint32_t Code);
	
uint32_t Rcon_GetRemoteCode(void);
void Rcon_SetRemoteCode(uint32_t Code);


uint32_t Rcon_GetStatus(void);
void Rcon_ResetStatus(void);



uint8_t Rcon_IsNearStation(void);
uint8_t Rcon_IsGotStationSignal(void);


void Rcon_ResetRemoteCode(void);


uint8_t Is_Turn_Remote(void);

uint8_t Rcon_RemoteKey(uint32_t Key);

uint8_t Is_Meet_Station(void);
uint8_t Is_Left_Meet_Station(void);
uint8_t Is_Right_Meet_Station(void);

void Rcon_Timer(void);
void Rcon_BR(void);
void Rcon_L(void);
void Rcon_FL(void);
void Rcon_R(void);
void Rcon_FR(void);

#endif /* __USART2_H */


