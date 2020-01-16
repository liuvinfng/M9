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
#include "include.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"




/*------------------------------------------------Get Key time------------------------------------*/
uint8_t Remote_Key(uint32_t Key)
{
  if(Rcon_GetRemoteCode()==Key)
	{
	  Rcon_ResetRemoteCode();
		return 1;
	}
	else return 0;
}
/*------------------------------------------------Check Key ------------------------------------*/
uint8_t Is_Key_Press(uint8_t Key)
{
  if(Key_GetStatus()&Key)
	{
	  return 1;
	}
	return 0;
}
uint8_t Key_GetPressKey(void)
{
	static uint8_t key_cnt = 0;
  uint8_t ret_key_status = 0;

	if(Key_GetStatus() & KEY_CLEAN)
	{
		if(key_cnt)
		{
			ret_key_status |= KEY_CLEAN;
			Usprintf("%s(%d):Clean Key Pressed!!\n",__FUNCTION__, __LINE__);
		}
		key_cnt=0;
	}
	else if(Key_GetStatus() & KEY_SPOT)
	{
		if(key_cnt)
		{
			ret_key_status |= KEY_SPOT;
			Usprintf("%s(%d):Spot Key Pressed!!\n",__FUNCTION__, __LINE__);
		}
		key_cnt=0;	
	}
	else if(Key_GetStatus() & KEY_HOME)
	{
		if(key_cnt)
		{
			ret_key_status |= KEY_HOME;
			Usprintf("%s(%d):Home Key Pressed!!\n",__FUNCTION__, __LINE__);
		}
		key_cnt=0;	
	}		
	else
	{
		key_cnt=1;
	}
	return ret_key_status;
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

