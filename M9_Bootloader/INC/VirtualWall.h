/**
  ******************************************************************************
  * @file    VirtualWall.h
  * @author  Wfliu
  * @version V0.0
  * @date    19-May-2016
  * @brief   Virtual Wall Header
  ******************************************************************************

  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __VIRTUALWALL_H
#define __VIRTUALWALL_H

#include "gd32f30x.h"

uint8_t WalkAlongVirtualWall(uint32_t VW_Status);

uint8_t Head_To_Virtualwall(uint8_t Dir, uint32_t Vr);

uint8_t Move_While_VirtualWall(void);

uint8_t VirtualWall_TurnRight(uint32_t VW_St);

#endif

