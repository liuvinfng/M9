/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   Display Fuctions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __TouchPad_H
#define __TouchPad_H

#include "SysInitialize.h"


#define KEY_SPOT     0x01
#define KEY_CLEAN    0x02
#define KEY_HOME     0x04

uint8_t Remote_Key(uint32_t Key);

uint8_t Key_GetStatus(void);
uint8_t Touch_Detect(void);

void Set_Touch(void);
uint8_t Is_Key_Press(uint8_t Key);
uint8_t Get_Touch_Status(void);

uint8_t Key_GetPressKey(void);

#endif /* __DISPLAY_H */


