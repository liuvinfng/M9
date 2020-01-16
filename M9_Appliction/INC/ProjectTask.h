/**
  ******************************************************************************
  * @file    Ilife Cleaning Robot
  * @author  Wfliu
  * @version Ver 00
  * @date    18-Sep-2016
  * @brief   Task Functions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __PROJECTTASK_H
#define __PROJECTTASK_H

#include "SysInitialize.h"


void Task_ControlTask(void *argument);
void Task_SensorsTask(void *argument);
void Task_CalculatePathTask(void *agument);
void Task_UpdatePositionTask(void *agument);

void Task_CreateMainTask(void);
void Task_CraeteSensorsTask(void);
void Task_CraeteControlTask(void);
void Task_CreateCalculatePathTask(void);

#endif /* __PROJECTTASK_H */



