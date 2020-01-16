/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   The touch pad and remote functions which related to the user input
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/

#include "SysInitialize.h"
#include "TouchPad.h"
#include "Speaker.h"
#include "Rcon.h"



/*------------------------------------------------Get Key time------------------------------------*/
uint8_t Remote_Key(uint32_t Key)
{
  if(Rcon_GetRemoteCode()==Key)
	{
	  Rcon_ResetRemoteCode();
		Beep(2);
		return 1;
	}
	else return 0;
}
/*------------------------------------------------Check Key ------------------------------------*/
uint8_t Is_Key_Press(uint8_t Key)
{
  if(Key_GetStatus()&Key)
	{
	  Beep(2);
	  return 1;
	}
	return 0;
}
/*------------------------------------------------Get Key Press------------------------------------*/
uint8_t Key_GetStatus(void)
{
  uint8_t Key_Status=0;
	if(!Is_Key_Clean())
	{
		if(!Is_Key_Clean())
		{
			Key_Status |= KEY_CLEAN;
		}
	}
	if(!Is_Key_Spot())
	{
		if(!Is_Key_Spot())
		{
			Key_Status |= KEY_SPOT;
		}
	}	
	if(!Is_Key_Home())
	{
		if(!Is_Key_Home())
		{
			Key_Status |= KEY_HOME;
		}
	}	
	return Key_Status;
}
/*------------------------------------------------Get Key time------------------------------------*/
uint8_t Get_Key_Time(uint16_t Key)
{
  uint8_t time=0;
	while(1)
	{
	  time++;
		if(time>200)break;
	  delay(100);
		if(Key_GetStatus()!=Key)break;
	}
	return time;
}

/*Touch Detect*/
uint8_t g_key_press_status=0;
uint8_t Touch_Detect(void)
{		
	if(Get_Touch_Status())
	{
		return g_key_press_status;
	}
	
	if(Key_GetStatus())
	{
		  g_key_press_status = 1;
	}
	
  if(Remote_Key(Remote_Clean)) g_key_press_status = 1;
	
	return g_key_press_status;
}

uint8_t Get_Touch_Status(void)
{
	return g_key_press_status;
}

void Set_Touch(void)
{
  g_key_press_status=1;
}

