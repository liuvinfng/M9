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
#include "movement.h"
 
 
void 		User_Interface(void);
void 		UserInterFace_RemoteToMode(uint32_t remote_code);
uint8_t UserInterFace_IsWorkMode(CleanMode_t mode);
uint8_t UserInterFace_IsRobotPickedUp(void);
void		UserInterFace_BeginToWork(void);
uint8_t UserInterFace_IsLowPower(void);
void Task_ReportStackTask(void *argument);
void Task_CreateReportTask(void);



//void Draw_A_House(void);
//void Draw_A_Walked(void);

//void Draw_B_House(void);
//void Draw_B_Walked(void);

//void Draw_C_House(void);
//void Draw_C_Walked(void);

//void Draw_D_House(void);

//void Draw_Map(int16_t x,int16_t y,uint8_t dir, int16_t length,CellState_t value);
//void Draw_W_Map(void);



#endif /* __DISPLAY_H */

