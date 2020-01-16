 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Move near the wall on the left in a certain distance
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "WallFollowShort.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Spot.h"
#include "Rcon.h"
#include "VirtualWall.h"
#include "WIFI_EMW_3081.h"


const uint32_t Wall_Distance_Array[10]={1000,1500,1800,3000,3500,2500,5500,6000,4000,2200};

const uint32_t Wall_Room_Array[5]={20000,10000,6000,4500,3000};


uint8_t Wall_Follow_Short(uint32_t dis)
{
	static volatile uint8_t Motor_Check_Code=0;
  static volatile int32_t Proportion=0;
	static volatile int32_t Delta=0;
	static volatile int32_t Previous=0;
  static volatile int32_t Wall_Distance=1000;
	static volatile int32_t Wall_Straight_Distance=2*DISTANCE_1CM;
	static volatile int32_t Left_Wall_Speed=0;
	static volatile int32_t Right_Wall_Speed=0;
  static uint8_t Temp_Random_Factor=0;
  static uint32_t Temp_Rcon_Status = 0;
	static uint8_t Temp_Counter=0;
	static uint8_t Jam=0;
	static int32_t R=0;
  static uint8_t Bumper_Limit = 0;
	static int16_t Left_Wall_Buffer[3]={0};
  static uint32_t Temp_Mobility_Distance=0;
  static uint8_t Mobility_Temp_Error=0;
	static int32_t Follow_Distance = 0;
  static uint32_t SWall_B_Counter=0;

	Motor_Check_Code=0;
	Proportion=0;
	Delta=0;
	Previous=0;
	Wall_Distance=1000;
	Wall_Straight_Distance=2*DISTANCE_1CM;
	Left_Wall_Speed=0;
	Right_Wall_Speed=0;
	Temp_Random_Factor=0;
	Temp_Rcon_Status = 0;
	Temp_Counter=0;
	Jam=0;
	R=0;
//	Bumper_Limit = 0;
	Left_Wall_Buffer[0]=0;
	Left_Wall_Buffer[1]=0;
	Left_Wall_Buffer[2]=0;
	Temp_Mobility_Distance=0;
	Mobility_Temp_Error=0;
	Follow_Distance = 0;
	
	if(Is_WorkTime_150min())
	{
		Follow_Distance=30000;
		Bumper_Limit=13;
	}
	else
	{
		Temp_Random_Factor = Get_Random_Factor()/10;
		
		if(Temp_Random_Factor>9)Temp_Random_Factor=9;
		
		
		dis/=2000;
		if(dis>4)dis=4;
		
		Follow_Distance = Wall_Room_Array[dis];
		
		Follow_Distance += (Wall_Distance_Array[Temp_Random_Factor]);

		Bumper_Limit = Get_Random_Factor()/25;
		
		Bumper_Limit +=6;
	}

	Reset_Bumper_Error();

	Work_Motor_Configure();
	Set_Vac_Speed();
	Set_RightWheel_Speed(15);
	Reset_Wall_Step();
	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  Reset_Rcon_Status();
  Reset_Wheel_Step();
  Set_Mobility_Step(1000);
	Reset_Average_Counter();
  Reset_WallAccelerate();
	Wall_Straight_Distance=5*DISTANCE_1CM;
	Display_Clean_Status(Display_Wall);
	SWall_B_Counter=0;

  while(1)
  {

    if(Get_LeftWheel_Step()<500)
    {
      Mobility_Temp_Error=0;
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        if(Get_Mobility_Step()<1)
        {
          Mobility_Temp_Error++;
          if(Mobility_Temp_Error>5)
          {
            break;
          }
        }
        else
        {
          Mobility_Temp_Error=0;
        }
        Reset_Mobility_Step();
      }
    }
	  /*------------------------------------------------------Check Current-----------------------*/
	   Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
        Set_Clean_Mode(Clean_Mode_Userinterface);
			  return 1;
			}
      Reset_TempPWM();
			Initialize_Motor();
      break;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
		  return 1;
		}
		#ifdef BLDC_INSTALL
		#ifdef WIFI_EMW3081
		EMW3081_Vacuum_Power();
		#endif
		if(Remote_Key(Remote_Max))
		{
			Switch_VacMode();
		}
		else if(Get_Rcon_Remote()!=0)
		{
			break;
		}
		#else
		if(Get_Rcon_Remote()!=0)
		{
			break;
		}
		#endif
		/*------------------------------------------------------Check Battery-----------------------*/
		
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
			break;
 		}	
		/*------------------------------------------------------Virtual Wall Event-----------------------*/
    Temp_Rcon_Status = Get_Rcon_Status();
    if(Temp_Rcon_Status)
    {
      Reset_Rcon_Status();
      if(Temp_Rcon_Status&0X0FFF)
      {
        if(Is_WorkFinish(Get_Room_Mode()))
        {
          break;
        }
      }
			if(Temp_Rcon_Status&0X0F00)
      {
				Stop_Brifly();
				if(Temp_Rcon_Status & RconFR_HomeT)
				{
					Turn_Right(Turn_Speed,1300);
				}
				else if(Temp_Rcon_Status & RconFL_HomeT)
				{
					Turn_Right(Turn_Speed,1200);
				}
				else if(Temp_Rcon_Status & RconL_HomeT)
				{
					Turn_Right(Turn_Speed,900);
				}
				else if(Temp_Rcon_Status & RconR_HomeT)
				{
					Turn_Right(Turn_Speed,1500);
				}
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_Rcon_Status();
				Wall_Straight_Distance=2*DISTANCE_1CM;
				Reset_WallAccelerate();
			}
			/*Virtual wall event*/
			#ifdef VIRTUAL_WALL
			if(VirtualWall_TurnRight(Temp_Rcon_Status))
			{
				Wall_Straight_Distance=3*DISTANCE_1CM;
				Reset_WallAccelerate();
			}
			#endif
    } 
		/*---------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status()&RightBumperTrig)
    {
			Stop_Brifly();
			Wall_Move_Back();
		  if(Get_WallAccelerate()>80)
			{
				if(Is_Bumper_Jamed())break;
			}
			if(Get_WallAccelerate()<2000)
			{
				Jam++;
			}
			else
			{
			  Jam=0;
			}

	    Turn_Right(Turn_Speed,720);
	    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			Reset_WallAccelerate();
			Wall_Straight_Distance=5*DISTANCE_1CM;

			for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
			{
			  Left_Wall_Buffer[Temp_Counter]=0;
			}	
      Stop_Brifly();
      Reset_Wheel_Step();
			SWall_B_Counter+=1;
    }
		if(Get_Bumper_Status()&LeftBumperTrig)
    {	
      Set_Wheel_Speed(0,0);
      Reset_TempPWM();
      delay(300);
			if(Get_LWall_ADC()>(Wall_Low_Limit)){
				Wall_Distance = Get_LWall_ADC()/3;
			}
			else {
					Wall_Distance+=200;
			}

			if (Wall_Distance < Wall_Low_Limit)Wall_Distance = Wall_Low_Limit;
			if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;		
			
			if(Get_Bumper_Status()&RightBumperTrig)
			{
				Wall_Move_Back();
				if(Is_Bumper_Jamed())break;;
				Turn_Right(Turn_Speed,600);
				Wall_Straight_Distance=3*DISTANCE_1CM;
			}
			else
			{
				Wall_Move_Back();
				if(Is_Bumper_Jamed())break;
				if(Jam<3)
				{
					if(Wall_Distance<200)
					{
						if(Get_LeftOBS()>(Get_LeftOBST_Value()-200)){
							Wall_Distance=Wall_High_Limit;
							Turn_Right(Turn_Speed , 300);
						}
						else{
							Turn_Right(Turn_Speed , 150);
						}
					}
					else
					{
						Turn_Right(Turn_Speed , 300);
					}
				}
				else
				{
					Turn_Right(Turn_Speed,150);
				}
				Wall_Straight_Distance=4*DISTANCE_1CM;
			}

			if(Get_WallAccelerate()<2000)
			{
			  Jam++;
			}
			else
			{
				Jam=0;
			}
			
			Reset_WallAccelerate();
      Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
			{
			  Left_Wall_Buffer[Temp_Counter]=0;
			}	
      //Stop_Brifly();
      Reset_Wheel_Step();
			SWall_B_Counter+=1;
    }
    

		/*------------------------------------------------------Cliff Event-----------------------*/
    if(Get_Cliff_Trig())
    {
		  Set_Wheel_Speed(0,0);
      Set_Dir_Backward();
	    delay(150);
//			if(Get_Cliff_Trig())
//			{
			  Cliff_Move_Back();
				if(Get_Cliff_Trig()==(Status_Cliff_All))
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
				  break;
				}
				if(Get_Cliff_Trig())
			  {
				  if(Cliff_Escape())
					{
					  Set_Clean_Mode(Clean_Mode_Userinterface);
            return 1;
					}
				}
			
				Turn_Right(Turn_Speed,900);
				Stop_Brifly();
				Reset_WallAccelerate();
				Reset_Wheel_Step();
//				break;
//			}
    }
		/*---------------------------------------------------Jam---------------------------------------------*/
		if(Jam>12)
		{
			break;
		}
		if(SWall_B_Counter>Bumper_Limit)break;
    /*---------------------------------------------------Jam---------------------------------------------*/
    if(Wall_Distance>=200)
		{
			Left_Wall_Buffer[2]=Left_Wall_Buffer[1];
			Left_Wall_Buffer[1]=Left_Wall_Buffer[0];
			Left_Wall_Buffer[0]=Get_LWall_ADC();
			if(Left_Wall_Buffer[0]<100)
			{
			  if((Left_Wall_Buffer[1]-Left_Wall_Buffer[0])>(Wall_Distance/25))
				{
				  if((Left_Wall_Buffer[2]-Left_Wall_Buffer[1])>(Wall_Distance/25))
					{
					  if(Get_WallAccelerate()>300)
						{
						  if((Get_RightWheel_Speed()-Get_LeftWheel_Speed())>=-3)
							{
								Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
								delay(1000);
								Reset_WallAccelerate();
								Wall_Straight_Distance=5*DISTANCE_1CM;
							}
						}
					}
				}
			}
		}
		
		/*------------------------------------------------------Short Distance Move-----------------------*/
		if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance)
		{
      if(Get_LeftWheel_Step()<500)
      {
        if(Get_WallAccelerate()<100)
  			{
  			  Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  			}
  			else
  			{
  			  Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
  			}
      }
      else
      {
        Move_Forward(RUN_SPEED_17,RUN_SPEED_17);
      }
		}
		else
		{
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
	    if(!Is_FrontOBS_Trig())
	    {
			  Proportion = Get_LWall_ADC();

				Proportion = Proportion * 100 / Wall_Distance;

				Proportion -= 100;

				Delta = Proportion - Previous;

				if (Wall_Distance > 300)//over left
				{
					Left_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
					Right_Wall_Speed = 25 - Proportion / 10 - Delta / 5;
					if (Right_Wall_Speed > 33) {
						Left_Wall_Speed = 9;
						Right_Wall_Speed = 33;
					}
				} else if (Wall_Distance > 150){//over left
					Left_Wall_Speed = 22 + Proportion / 15 + Delta / 7;
					Right_Wall_Speed = 22 - Proportion / 12 - Delta / 7;

					if (Right_Wall_Speed > 27) {
						Left_Wall_Speed = 8;
						Right_Wall_Speed = 30;
					}
				}
			  else{
					Left_Wall_Speed = 15 + Proportion / 22 + Delta / 10;
					Right_Wall_Speed = 15 - Proportion / 18 - Delta / 10;

					if (Right_Wall_Speed > 18) {
						Left_Wall_Speed = 5;
						Right_Wall_Speed = 18;
					}

					if(Left_Wall_Speed>20)Left_Wall_Speed=20;
					if(Left_Wall_Speed<4)Left_Wall_Speed=4;
					if(Right_Wall_Speed<4)Right_Wall_Speed=4;
					if((Left_Wall_Speed-Right_Wall_Speed)>5)
					{
						Left_Wall_Speed = Right_Wall_Speed+5;
					}
				}
			  /*slow move if left obs near a wall*/
				if(Get_LeftOBS()>Get_LeftOBST_Value()){
					if(Wall_Distance<Wall_High_Limit)Wall_Distance++;
				}
				if(Is_WallOBS_Near()){
						Left_Wall_Speed = Left_Wall_Speed/2;
						Right_Wall_Speed = Right_Wall_Speed/2;
				}

				Previous = Proportion;

				if (Left_Wall_Speed < 0) {
					Left_Wall_Speed = 0;
				}
				if (Left_Wall_Speed > 40) {
					Left_Wall_Speed = 40;
				}
				if (Right_Wall_Speed < 0) {
					Right_Wall_Speed = 0;
				}

				//Set_Dir_Forward();
				Move_Forward(Left_Wall_Speed, Right_Wall_Speed);

	      if(Get_RightWall_Step()>Get_LeftWall_Step())
				{
					R=Get_RightWall_Step()-Get_LeftWall_Step();
					if(R>TURN_AROUND_CNT)//turn over 3600 degree
					{
						break;
					}
				}
	      
	      if((Get_RightWall_Step()>Follow_Distance)||(Get_LeftWall_Step()>Follow_Distance))//about 5 Meter
	      {
          break; 
	      }
				if(Get_WallAccelerate()>750)
				{
					Set_Left_Brush(ENABLE);
			    Set_Right_Brush(ENABLE);
				}
	    }
	    else
	    {
        Stop_Brifly();
			  if(Get_LeftWheel_Step()<12500)
				{
					  if(Is_FrontOBS_Trig())
						{
						  if(Get_WallAccelerate()<2000)
							{
								Jam++;
							}
						  Turn_Right(Turn_Speed,800);
				      Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
						}
						else
						{
				      Turn_Right(Turn_Speed,500);
				      Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
						}
				}
				else
				{
				  Turn_Right(Turn_Speed,900);
			    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				}
        Reset_Wheel_Step();
				Wall_Distance = Wall_High_Limit;
	    }
		}
  }
	Set_Direction_Flag(Direction_Flag_Right);
	//Stop_Brifly();
	Wheel_Stop();
  delay(200);
	Display_Clean_Status(Display_Clean);
  return 0;
}



