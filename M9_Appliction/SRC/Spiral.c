 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Spiral Path cleaning mode
	           move in a mode of a circle path by increasing the Radius
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Spiral.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Spot.h"
#include "Rcon.h"
#include "Home.h"


///*----------------------------------------------------------- Spining Event -----------------------*/
uint8_t Spiral(void)
{
	uint32_t Motor_Check_Code=0;
	uint8_t Temp_Dirt_Status=0;
	uint32_t Radius=0;
	uint32_t Max_Radius=0;
	int32_t Right_Wheel_Speed=0;
	uint8_t First_Round_Flag=1;
	uint8_t Vac_Mode_Buffer=0;
	
	Max_Radius = 180 + Get_Random_Factor();

	Set_Wheel_Step(0,0);
	Move_Forward(RUN_SPEED_17,0);
	while(1)
  {
	  Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  return 0;	  
		}
		/*-------------------------------------------------------Check Battery ------------------*/
	  if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
      return 0;
    }
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
		if(Get_Rcon_Remote()!=0)
		{
			if(Remote_Key(Remote_Spot))
			{
        Vac_Mode_Buffer = Get_VacMode();				
			  Temp_Dirt_Status=Random_Dirt_Event();
				Set_VacMode(Vac_Mode_Buffer);
				Set_Vac_Speed();
				if(Temp_Dirt_Status==1)
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 1;
				}
				Display_Clean_Status(Display_Clean);
				return 0;
			}
			if(Remote_Key(Remote_Left))
			{
			  Turn_Left(Turn_Speed,700);
				Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
				Set_Clean_Mode(Clean_Mode_RandomMode);
				Reset_Rcon_Remote();
				return 1;
			}
			if(Remote_Key(Remote_Right))
			{
			  Turn_Right(Turn_Speed,560);
				Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
				Set_Clean_Mode(Clean_Mode_RandomMode);
        Reset_Rcon_Remote();
				return 1;
			}
			if(Remote_Key(Remote_Home)) //                                    Check Key Home
			{
				Display_Content(LED_Home,100,100,0,7);
				Set_Clean_Mode(Clean_Mode_GoHome);
        Reset_Rcon_Remote();
				SetHomeRemote();
				return 1;
			}
#if 0
			if(Remote_Key(Remote_Wallfollow)) //                                    Check Key Home
			{
			  Set_Clean_Mode(Clean_Mode_WallFollow);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				return 1;
			}
#endif
		}

		Right_Wheel_Speed = (42*Radius)/(Radius+100);
		if(Right_Wheel_Speed>RUN_SPEED_17)Right_Wheel_Speed=RUN_SPEED_17;
		if(Right_Wheel_Speed<0)Right_Wheel_Speed=0;
		
		Set_Wheel_Speed(RUN_SPEED_17,Right_Wheel_Speed);
 
    if(First_Round_Flag)
		{
			if(Get_LeftWheel_Step()>3000)First_Round_Flag=0;
		}
    else
		{
			if(Get_LeftWheel_Step()>(Radius))
			{
				Set_Wheel_Step(0,0);
	//			Reset_Wheel_Step();
				if(Radius<100)
				{
					Radius+=1;
				}
				else
				{
					Radius+=2;
				}
			}
		}

		if(Radius>Max_Radius)
		{
		  return 0;
		}
		
    if(Get_Bumper_Status()||Get_Cliff_Trig()||Get_OBS_Status())
		{ 
		  return 0;
		}
  }
	//return 0;
}



