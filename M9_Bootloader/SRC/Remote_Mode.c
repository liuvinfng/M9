
 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   this mode the robot follows the command of the remote ,
	           Upkey : move forward untill stop command or obstacle event
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "Movement.h"
#include "Speaker.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Remote_Mode.h"
#include "Rcon.h"
#include "Home.h"

 
extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;


void Remote_Mode(void)
{
	uint32_t Moving_Speed=0;
  uint16_t No_Command_Counter=0;
	#ifndef SCREEN_REMOTE
	uint8_t Forward_Flag=0;
	uint8_t Dec_Counter=0;
	uint32_t OBS_Stop=0;
	#endif
	#ifndef STANDARD_REMOTE
	uint32_t dir=0,old_dir=0;
	#endif

  Display_Clean_Status(Display_Remote);
  Reset_Wheel_Step();
	Reset_Touch();
  Set_Vacuum_PWM(40);
	delay(1000);
  Set_Vac_Speed();

  while(1)
	{
		#ifndef SCREEN_REMOTE
		if(Remote_Key(Remote_Forward))
		{
			Forward_Flag=1-Forward_Flag;
			Reset_Rcon_Remote();
			No_Command_Counter=0;
			if(Forward_Flag)
			{

			}
			else
			{

			}
		}

		if(Forward_Flag)
		{			
			if(Get_OBS_Status())
			{
				Dec_Counter++;
				if(Moving_Speed>10)Moving_Speed--;
				Move_Forward(Moving_Speed,Moving_Speed);
				OBS_Stop++;
				if(OBS_Stop>15)Forward_Flag=0;
			}
			else
			{
				Moving_Speed=(Get_RightWheel_Step()/80)+25;
				if(Moving_Speed<18)Moving_Speed=18;
				if(Moving_Speed>30)Moving_Speed=30;
				Move_Forward(Moving_Speed,Moving_Speed);
				OBS_Stop=0;
			}
			No_Command_Counter=0;
		}
		else
		{
			Deceleration();
			Stop_Brifly();
		}

		if(Remote_Key(Remote_Left))
		{
			Deceleration();
			Work_Motor_Configure();
			Turn_Left(Turn_Speed,400);
			Set_SideBrush_PWM(30,30);
			Set_MainBrush_PWM(30);
			Set_Vacuum_PWM(30);
			No_Command_Counter=0;
			Reset_TempPWM();
			Forward_Flag=0;
		}
		if(Remote_Key(Remote_Right))
		{
			Deceleration();
			Work_Motor_Configure();
			Turn_Right(Turn_Speed,320);
			Set_SideBrush_PWM(30,30);
			Set_MainBrush_PWM(30);
			Set_Vacuum_PWM(30);
			No_Command_Counter=0;
			Reset_Wheel_Step();
			Forward_Flag=0;
		}
		if(Remote_Key(Remote_Max))
		{
			Deceleration();
			Reset_Rcon_Remote();
			Work_Motor_Configure();
			Turn_Right(Turn_Speed,1800);
			Set_SideBrush_PWM(30,30);
			Set_MainBrush_PWM(30);
			Set_Vacuum_PWM(30);
			Reset_TempPWM();
			No_Command_Counter=0;
			Forward_Flag=0;
			Reset_Rcon_Remote();
			Reset_Wheel_Step();
		}
		#endif
		#ifndef STANDARD_REMOTE
		if(old_dir!=Get_Rcon_Remote())
		{
			Deceleration();
			Moving_Speed=0;
		}
		dir=Get_Rcon_Remote();
		switch(dir)
		{
			case Remote_Forward:  Reset_Rcon_Remote();
														Moving_Speed+=10;
														if(Moving_Speed>30)Moving_Speed=30;		
														Move_Forward(Moving_Speed,Moving_Speed);
														
														delay(1000);
														No_Command_Counter=0;
														break;
			case Remote_Left:     Reset_Rcon_Remote();
														
														RW_DIR_FORWARD();
														LW_DIR_BACKWARD();
														Wheel_Configure(Enable,Enable);
														Moving_Speed+=10;
														if(Moving_Speed>30)Moving_Speed=30;					
														Set_Wheel_Speed(Moving_Speed,Moving_Speed);

														
														delay(1000);
														No_Command_Counter=0;
														break;
			case Remote_Right:    Reset_Rcon_Remote();
														
														RW_DIR_BACKWARD();
														LW_DIR_FORWARD();
														Wheel_Configure(Enable,Enable);
														Moving_Speed+=10;
														if(Moving_Speed>30)Moving_Speed=30;					
														Set_Wheel_Speed(Moving_Speed,Moving_Speed);
														
														delay(1000);
														No_Command_Counter=0;
														break;
			case Remote_Max:      Reset_Rcon_Remote();
														
														RW_DIR_BACKWARD();
														LW_DIR_BACKWARD();
														Wheel_Configure(Enable,Enable);
														Moving_Speed+=10;
														if(Moving_Speed>30)Moving_Speed=30;					
														Set_Wheel_Speed(Moving_Speed,Moving_Speed);
														
														delay(1000);
														No_Command_Counter=0;
														break;
			default: 
														Deceleration();
														Moving_Speed=0;
														Wheel_Configure(Disable,Disable);
														Move_Forward(0,0);														 
														 break;
		}
    old_dir=dir;				
		#endif
		
		//-------------------normal command-----------------------//
	  delay(100);
		Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power);
    No_Command_Counter++;
    if(No_Command_Counter>100)
    {
      No_Command_Counter=0;
			Deceleration();
			Set_Clean_Mode(Clean_Mode_Userinterface);
		  break;
    }

		if(Remote_Key(Remote_Spot))
		{
			Deceleration();
		  Disable_Motors();
		  Set_Clean_Mode(Clean_Mode_Spot);
			Initialize_Motor();
			Set_MoveWithRemote();
			return;
		}
		if(Remote_Key(Remote_Clean))
		{
			Deceleration();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Reset_Rcon_Remote();
			return;
		}
		if(Remote_Key(Remote_Random))
		{
//			Deceleration();
//		  Disable_Motors();
//		  Set_Clean_Mode(Clean_Mode_RandomMode);
//			Initialize_Motor();
//			return;
			Deceleration();
		  Disable_Motors();
		  Set_Clean_Mode(Clean_Mode_Spot);
			Initialize_Motor();
			Set_MoveWithRemote();
			return;			
		}
		if(Remote_Key(Remote_Navigation))
		{
			Deceleration();
		  Disable_Motors();
		  Set_Clean_Mode(Clean_Mode_Navigation);
			Initialize_Motor();
			return;
		}		
		if(Remote_Key(Remote_Home))
		{
			Deceleration();
			Set_Clean_Mode(Clean_Mode_GoHome);
			SetHomeRemote();
			return;
		}

	  /*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
			Deceleration();
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}

		/*------------------------------------------------------Check Battery-----------------------*/
    if(Check_Battery())
		{
			Deceleration();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
    }
		/*------------------------------------------------------Bumper Event-----------------------*/
		OBS_Dynamic_Base(5);

		#ifdef VIRTUAL_WALL
    if(Get_Bumper_Status()||Get_Cliff_Trig()||Get_OBS_Status()||(Get_Rcon_Status()&(Rcon_Signal_All_T))||Is_VirtualWall())
		#else
		if(Get_Bumper_Status()||Get_Cliff_Trig()||Get_OBS_Status()||(Get_Rcon_Status()&(Rcon_Signal_All_T)))
		#endif
    {
      Set_Wheel_Speed(0,0);
      Set_Dir_Backward();
	    delay(300);
			Cliff_Move_Back();		
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}

	}
	Disable_Motors();
}
