/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    19-May-2016
  * @brief   Virtual Wall Functions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "RCON.h"
#include "Movement.h"
#include "TouchPad.h"
#include "VirtualWall.h"
#include "Speaker.h"



uint8_t WalkAlongVirtualWall(uint32_t VW_Status)
{
//	uint32_t Out_Counter=0;
	uint32_t WD_Counter=0;
	int32_t R=0;

	Stop_Brifly();
	delay(2000);
	Turn_Right(Turn_Speed-8,700);
	Reset_Wall_Step();
	Reset_Move_Distance();
	Set_WallAccelerate(2000);
	while(1)
	{
		if(Get_Rcon_Status()&
			(RconFL_Wall|RconFL_Wall_T))
		{
			Reset_VirtualWall_Value(RconFL_Wall|RconFL_Wall_T);
			Move_Forward(RUN_SPEED_9,RUN_SPEED_3);
			WD_Counter=300;
			while(WD_Counter--)
			{
				delay(10);
				if(Get_Cliff_Trig())break;
				if(Get_Bumper_Status())break;
				if(Touch_Detect())break;
				if(Get_Rcon_Status()&(RconFR_Wall|RconFR_Wall_T))break;
			}
		}
		else if(Get_Rcon_Status()&(RconL_Wall|RconL_Wall_T))
		{
			Reset_VirtualWall_Value(RconL_Wall|RconL_Wall_T);
			Move_Forward(RUN_SPEED_9,RUN_SPEED_3);
			WD_Counter=330;
			while(WD_Counter--)
			{
				delay(10);
				if(Get_Cliff_Trig())break;
				if(Get_Bumper_Status())break;
				if(Touch_Detect())break;
				if(Get_Rcon_Status()&(RconFR_Wall|RconFR_Wall_T))break;
			}
		}
		else
		{
			Move_Forward(RUN_SPEED_4,RUN_SPEED_9);
		}
    
		if(Get_Rcon_Remote())break;
		
		if(Touch_Detect())return 1;
		
		if(Get_Cliff_Trig())
		{
			Set_Wheel_Speed(0,0);
      Reset_TempPWM();
      delay(150);
			Move_Back();
			if(Get_Cliff_Trig()==(Status_Cliff_All))return 0;
			Turn_Right(Turn_Speed,900);
			Stop_Brifly();
		}
		
		if(Get_Bumper_Status()&LeftBumperTrig)
		{
			Set_Wheel_Speed(0,0);
      Reset_TempPWM();
      delay(300);
			Wall_Move_Back();
			if(Get_Bumper_Status())return 0;
			Turn_Right(Turn_Speed,300);
			Stop_Brifly();
			Reset_VirtualWall();
			Reset_WallAccelerate();
		}
		if(Get_Bumper_Status()&RightBumperTrig)
		{
			Set_Wheel_Speed(0,0);
      Reset_TempPWM();
      delay(300);
			Wall_Move_Back();
			if(Get_Bumper_Status())return 0;
			Turn_Right(Turn_Speed,900);
			Stop_Brifly();
			Reset_VirtualWall();
			Reset_WallAccelerate();
		}
		if((Get_WallAccelerate()>2000)||(Get_Rcon_Status()&(RconFR_Wall|RconFR_Wall_T)))
		{
			if(Get_Rcon_Status()&(RconR_Wall|RconR_Wall_T|RconFR_Wall|RconFR_Wall_T))
			{
				Reset_VirtualWall();
				Deceleration();
				Stop_Brifly();
				OBS_Turn_Right(Turn_Speed-8,300);
//				Out_Counter=0;
				if(Get_Rcon_Status()&(RconFR_Wall|RconFR_Wall_T))
				{
					Reset_VirtualWall();
					OBS_Turn_Right(Turn_Speed-8,100);
				}
				if(Get_Rcon_Status()&(RconR_Wall|RconR_Wall_T))
				{
					Reset_VirtualWall();
					OBS_Turn_Right(Turn_Speed-8,200);
				}
//				while(Get_Rcon_Status()&(RconR_Wall|RconFR_Wall))
//				{
//					if(Get_Rcon_Status()&RconR_Wall)
//					{
//						Reset_VirtualWall();
//						OBS_Turn_Right(Turn_Speed-8,200);
//					}
//					if(Get_Rcon_Status()&RconFR_Wall)
//					{
//						Reset_VirtualWall();
//						OBS_Turn_Right(Turn_Speed-8,100);
//					}
//					delay(500);
//					if(Touch_Detect())break;
//					Out_Counter++;
//					if(Out_Counter>40)break;
//				}
				//OBS_Turn_Right(Turn_Speed-5,200);
				Stop_Brifly();
			}
		}
		else if(Get_Rcon_Status()&(RconR_Wall|RconR_Wall_T))
		{
			Reset_VirtualWall_Value(RconR_Wall|RconR_Wall_T);
			Move_Forward(RUN_SPEED_9,0);
			WD_Counter=330;
			while(WD_Counter--)
			{
				delay(10);
				if(Get_Cliff_Trig())break;
				if(Get_Bumper_Status())break;
				if(Touch_Detect())break;
			}
		}
		else
		{
			Reset_VirtualWall_Value(RconR_Wall|RconR_Wall_T|RconFR_Wall|RconFR_Wall_T);
		}
		
		if(Get_RightWall_Step()>Get_LeftWall_Step())
		{
			R=Get_RightWall_Step()-Get_LeftWall_Step();
			if(R>TURN_AROUND_CNT)//turn over 3600 degree
			{
				break;
			}
		}
		if((Is_Move_Finished(6000))||(Get_Rcon_Status()&(RconL_Wall_T|RconR_Wall_T|RconFL_Wall_T|RconFR_Wall_T)))
		{
			Stop_Brifly();
			R=Get_LeftWall_Step() - Get_RightWall_Step();
			if(R>TURN_AROUND_CNT)//turn over 3600 degree
			{
				break;
			}
			else
			{
				if(Get_WallAccelerate()>2000)Turn_Right(Turn_Speed,800);
			}
			Move_While_VirtualWall();
			break; 
		}
	}
	
	Set_Direction_Flag(Direction_Flag_Right);

	Reset_VirtualWall();
	return 0;
}

uint8_t Head_To_Virtualwall(uint8_t Dir,uint32_t Vr)
{
	uint32_t Vir_Count=0,Vir_NG=0;
	Set_Wheel_Speed(RUN_SPEED_7,RUN_SPEED_7);
	Reset_Wheel_Step();
	
	Reset_VirtualWall();
	if(Dir==Virtualwall_Left)Set_Dir_Left();
	else Set_Dir_Right();
	while(Get_LeftWheel_Step()<1800)
	{
		if(Get_Rcon_Status()&Vr)
		{
			Reset_VirtualWall();
			Vir_Count++;
			if(Vir_Count>1)break;
		}
		else
		{
			Vir_NG++;
			if(Vir_NG>20)
			{
				Vir_NG=0;
				Vir_Count=0;
			}
		}
		if(Touch_Detect())return 1;
		if(Get_Cliff_Trig())return 0;
		delay(100);
	}
	if(Get_LeftWheel_Step()>1799)return 2;
	Stop_Brifly();
	return 0;
}


uint8_t Move_While_VirtualWall(void)
{
	uint16_t No_VirSignal_Counter=0;
	Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
	No_VirSignal_Counter=0;
	Reset_Wheel_Step();
	Reset_VirtualWall();
	while(1)
	{
		delay(200);
		if(Get_Bumper_Status())break;
		if(Get_Cliff_Trig())break;
		if(Get_OBS_Status())break;
		if(Touch_Detect())return 1;
		
		No_VirSignal_Counter++;
		if(Get_Rcon_Status()&(RconR_Wall|RconL_Wall|RconBR_Wall|RconBL_Wall|RconR_Wall_T|RconL_Wall_T|RconBR_Wall_T|RconBL_Wall_T))
		{
			Reset_VirtualWall();
			No_VirSignal_Counter=0;
		}
		if(No_VirSignal_Counter>50)break;
		if(Is_LeftWheel_Reach(1000))break;
	}
	Reset_VirtualWall();
	Set_RightWheel_Step(400);
	return 0;
}

uint8_t VirtualWall_TurnRight(uint32_t VW_St)
{
	static uint8_t BR_RCON_Counter=0;
	uint32_t WD_Counter=0;
	static uint32_t Out_Counter=0;
	#ifdef VIRTUAL_WALL
	if(VW_St&(RconBR_Wall|RconL_Wall_T|RconR_Wall_T|RconFL_Wall_T|RconFR_Wall_T))
	{
		Reset_VirtualWall_Value(RconBR_Wall|RconL_Wall_T|RconR_Wall_T|RconFL_Wall_T|RconFR_Wall_T);
		BR_RCON_Counter++;
		if(BR_RCON_Counter>5)
		{
			BR_RCON_Counter=0;
			Deceleration();
			Stop_Brifly();
			Turn_Right(Turn_Speed-8,500);
			Stop_Brifly();
			Move_Forward(RUN_SPEED_3,RUN_SPEED_9);
			return 1;
		}
	}
	else
	{
		BR_RCON_Counter=0;
	}
	
	if(VW_St&(RconFR_Wall|RconFR_Wall_T))
	{
		Reset_VirtualWall_Value(RconFR_Wall|RconFR_Wall_T);
		Move_Forward(RUN_SPEED_9,0);
		WD_Counter=300;
		while(WD_Counter--)
		{
			delay(10);
			if(Get_Cliff_Trig())break;
			if(Get_Bumper_Status())break;
			if(Touch_Detect())break;
			if(Get_Rcon_Status()&(RconR_Wall|RconR_Wall_T))break;
		}
	}
	if(VW_St&(RconFL_Wall|RconFL_Wall_T))
	{
		Reset_VirtualWall_Value(RconFL_Wall|RconFL_Wall_T);
		Move_Forward(RUN_SPEED_9,RUN_SPEED_3);
		WD_Counter=300;
		while(WD_Counter--)
		{
			delay(10);
			if(Get_Cliff_Trig())break;
			if(Get_Bumper_Status())break;
			if(Touch_Detect())break;
			if(Get_Rcon_Status()&(RconR_Wall|RconR_Wall_T))break;
		}
	}
	else if(VW_St&(RconL_Wall|RconL_Wall_T))
	{
		Reset_VirtualWall_Value(RconL_Wall|RconL_Wall_T);
		Move_Forward(RUN_SPEED_9,RUN_SPEED_3);
		WD_Counter=300;
		while(WD_Counter--)
		{
			delay(10);
			if(Get_Cliff_Trig())break;
			if(Get_Bumper_Status())break;
			if(Touch_Detect())break;
			if(Get_Rcon_Status()&(RconR_Wall|RconR_Wall_T))break;
		}
	}
	if(VW_St&(RconR_Wall|RconR_Wall_T|RconBR_Wall))
	{
		Reset_VirtualWall();
		Deceleration();
		Stop_Brifly();
		OBS_Turn_Right(Turn_Speed-8,300);
		Out_Counter=0;
		while(Get_Rcon_Status()&(RconR_Wall|RconBR_Wall|RconR_Wall_T))
		{
			if(Get_Rcon_Status()&(RconR_Wall|RconBR_Wall|RconR_Wall_T))
			{
				Reset_VirtualWall();
				OBS_Turn_Right(Turn_Speed-8,200);
			}
//			if(Get_Rcon_Status()&RconFR_Wall)
//			{
//				Reset_VirtualWall();
//				OBS_Turn_Right(Turn_Speed-8,100);
//			}
			delay(500);
			if(Touch_Detect())break;
			Out_Counter++;
			if(Out_Counter>40)break;
		}
		OBS_Turn_Right(Turn_Speed-5,200);
		Stop_Brifly();
		return 1;
	}
//		if(Is_VirtualWall())
//		{
//			Beep(1);
//			Stop_Brifly();
//			break;
//		}
	#endif
	return 0;
}









