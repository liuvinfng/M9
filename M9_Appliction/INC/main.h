#ifndef __MAIN_H
#define __MAIN_H


#include "SysInitialize.h"

typedef enum
{
	TURN_LEFT,
	TURN_RIGHT,	
}TurnDir_t;


void Task_MainTask(void *argument);
void Test_GyroFunction(uint8_t code);



#endif /* __MAIN_H */

