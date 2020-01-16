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

#ifndef __UserInterface_H
#define __UserInterface_H

#include "SysInitialize.h"

#define Set_Clock_Flag	0x01
#define Set_Plan_Flag   0x02
#define Set_Hours				0x03
#define Set_Minutes     0x04

#define Status_Hour   1
#define Status_Minute 2

#define Edit_Add      1
#define Edit_Sub      2


void User_Interface(void);

typedef struct
{
	uint8_t Hours;
	uint8_t Minutes;
}Time_Struct;

typedef struct
{
	uint32_t Mon;
	uint32_t Tue;
	uint32_t Wed;
	uint32_t Thu;
	uint32_t Fri;
	uint32_t Sat;
	uint32_t Sun;
}Plan_Struct;


#endif 

