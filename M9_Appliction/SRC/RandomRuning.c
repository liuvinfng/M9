 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Random Path Cleaning Function
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "RandomRuning.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Spot.h"
#include "Rcon.h"
#include "WallFollowShort.h"  
#include "WallFollow.h"
#include "Spiral.h"
#include "Home.h"
#include "VirtualWall.h"


extern volatile OBS_ADC OBS_Status;

volatile uint8_t Half_Turn_Flag=0;


/* --------------------------------------------------Random Runnincg mode----------------------*/
void Random_Running_Mode(void)
{
	volatile uint8_t Stuck = 0, Motor_Check_Code = 0, Temp_Cliff_Status = 0, Temp_OBS_Status = 0, Random_Factor = 0;
	volatile uint8_t Bumper_Counter=0,Wall_Bumper_Counter=0;
	volatile uint32_t Left_Wheel_Step_Buffer=0;
  uint32_t Moving_Speed=20;
	#ifdef MOBILITY
  uint32_t Temp_Mobility_Distance=0;
	#endif
	uint8_t Disacc=0;
	uint8_t Temp_Dirt_Status=0;
	uint8_t Low_Power_Counter=0;
  uint8_t Base_Wall_On=1;
  uint32_t Temp_Rcon_Status=0;
  uint8_t Temp_Bumper_Status=0;   
  uint8_t Wall_Small_Counter = 0;
  uint8_t Wall_Mid_Counter = 0;
  uint8_t Wall_Bumper_Factor = 0;
	uint32_t OBS_Distance_Counter=0;
  uint8_t OBS_Cycle=0;
  uint8_t On_TrapOut_Flag=0;
	static uint8_t Vac_Mode_Buffer=0;
	static uint8_t N_H_T=0;
	static uint8_t Avoid_Flag=0;

	volatile uint32_t OBS_Delay=0;

#ifdef VIRTUAL_WALL
	uint8_t  Virtual_Wall_NG = 0;
#endif

  Wall_Bumper_Factor = Get_Random_Factor()/15;
	Reset_MoveWithRemote();
	Reset_Bumper_Error();
  Display_Clean_Status(Display_Clean);
	Reset_Touch();

	USART_Print("\nRandom Mode!",Dev_USART3);
  
	if(Is_AtHomeBase())
	{	  
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Beep(2);
		delay(4000);
		Reset_Rcon_Remote();
	  Beep(2);
	  Set_SideBrush_PWM(30,30);
		Set_MainBrush_PWM(0);
    Set_Vacuum_PWM(30);
    Stop_Brifly();
		Quick_Back(RUN_SPEED_11,10*DISTANCE_1CM);
		if(Touch_Detect()||Remote_Key(Remote_Clean)||Is_ChargerOn())return;
		Beep(2);
		Quick_Back(RUN_SPEED_11,10*DISTANCE_1CM);
		if(Touch_Detect()||Remote_Key(Remote_Clean))return;
		Beep(2);
		Quick_Back(RUN_SPEED_11,10*DISTANCE_1CM);
		if(Touch_Detect()||Remote_Key(Remote_Clean))return;
		Beep(2);
		Turn_Right(Turn_Speed,1120+Get_Random_Factor()*10);
		if(Touch_Detect()||Remote_Key(Remote_Clean))return;
		Stop_Brifly();
		Initialize_Motor();
		Base_Wall_On=0;
	}

	Set_Clean_Mode(Clean_Mode_RandomMode);
	
	Work_Motor_Configure();
	Reset_Move_Distance();
	Reset_Wheel_Step();
	Reset_Touch();
	Wall_Bumper_Counter=0;
	Reset_Rcon_Remote();
	Set_Direction_Flag(Direction_Flag_Right);
	Stuck=0;
	Low_Power_Counter=0;
  Reset_Rcon_Status();
	Set_Vac_Speed();
  while(1)
  {

		Wall_Dynamic_Base(200);
		OBS_Dynamic_Base(50);

		if(Get_Room_Mode())
		{
			if(WorkFinish_ByRoom(Get_Room_Mode()))
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
		}
		#ifdef MOBILITY
    /*-------------------------------------Mobility----------------------------------------------*/
    if(Get_LeftWheel_Step()<500)
    {
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        Check_Mobility();
        Reset_Mobility_Step();
      }
    }
		#endif
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
			  Set_Clean_Mode(Clean_Mode_Userinterface);
				Display_Content(LED_Exclamation,100,100,0,7);
				break;
			}
			Initialize_Motor();	  
		}
		/*------------------------------------------------------Check Battery-----------------------*/
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
		  Low_Power_Counter++;
			if(Low_Power_Counter>10)
			{
			  Display_Battery_Status(Display_Low);//display low
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
 		}
		else
		{
		  Low_Power_Counter=0;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(Get_Rcon_Remote())
		{
			#ifndef STANDARD_REMOTE
			if(Remote_Key(Remote_Left))
			{
        Stop_Brifly();
				Set_Dir_Left();
				Set_Wheel_Speed(RUN_SPEED_10,RUN_SPEED_10);
				Set_SideBrush_PWM(60,60);
        delay(1000);
				while(Get_Rcon_Remote()==Remote_Left)
				{
				  Reset_Rcon_Remote();
				  delay(1000);
				  if(Touch_Detect())break;
				}
        Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
			if(Remote_Key(Remote_Right))
			{
        Stop_Brifly();
				Set_Dir_Right();
				Set_Wheel_Speed(RUN_SPEED_10,RUN_SPEED_10);
				Set_SideBrush_PWM(60,60);
        delay(1000);
				while(Get_Rcon_Remote()==Remote_Right)
				{
				  Reset_Rcon_Remote();
				  delay(1000);
				  if(Touch_Detect())break;
				}
        Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
			#endif
			#ifndef SCREEN_REMOTE
			if(Remote_Key(Remote_Left))
			{
				Deceleration();
        Stop_Brifly();
        Turn_Left(Turn_Speed,500);
        Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
			if(Remote_Key(Remote_Right))
			{
				Deceleration();
        Stop_Brifly();
        Turn_Right(Turn_Speed,400);
        Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
			#ifdef BLDC_INSTALL
			if(Remote_Key(Remote_Max))
			{
				Switch_VacMode();
			}
			#else
			if(Remote_Key(Remote_Max))
			{
        Stop_Brifly();
        Turn_Right(Turn_Speed,1800);
        Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
			#endif
			#endif
			if(Remote_Key(Remote_Home))
			{
			  delay(500);
				Deceleration();
        Display_Content(LED_Home,100,100,0,7);
				Set_Clean_Mode(Clean_Mode_GoHome);
				SetHomeRemote();
				return;
			}
#if 0
			if(Remote_Key(Remote_Wallfollow)) 
			{
			  Set_Clean_Mode(Clean_Mode_WallFollow);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Set_RemoteRandomWall();
				break;
			}
#endif
      if(Remote_Key(Remote_Spot))
  		{
				Deceleration();
				Vac_Mode_Buffer = Get_VacMode();
  		  Temp_Dirt_Status=Random_Dirt_Event();
				Set_VacMode(Vac_Mode_Buffer);
				Set_Vac_Speed();
  			if(Temp_Dirt_Status==1)
  			{
  				Set_Clean_Mode(Clean_Mode_Userinterface);
  				break;
  			}
        Reset_Wheel_Step();
				Display_Clean_Status(Display_Clean);
  		}
		}
		/*------------------------------------------------------Virtual wall Event-----------------------*/
    if(Is_WorkFinish(Get_Room_Mode()))
		{
			if(Is_NearStation())
			{
				Set_Clean_Mode(Clean_Mode_GoHome);
				break;
			}
		}
		else if(Base_Wall_On)
    {
      Temp_Rcon_Status = Get_Rcon_Status();
			if(Temp_Rcon_Status & Rcon_Signal_All_T)
			{
				OBS_OFF();
				Stop_Brifly();
				Stop_Brifly();
				if(Get_LeftWheel_Step()<1000)
				{
					if(Is_Direction_Right())
					{
						Turn_Right(Turn_Speed,1200);
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else
					{
						Turn_Left(Turn_Speed,1200);
						Set_Direction_Flag(Direction_Flag_Left);
					}
				}
				else
				{
					if(Temp_Rcon_Status & RconL_HomeT)
					{
						Turn_Right(Turn_Speed,800);
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else if(Temp_Rcon_Status & RconFL_HomeT)
					{
						Random_Back();
						Turn_Right(Turn_Speed,1120);
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else if(Temp_Rcon_Status & RconFR_HomeT)
					{
						Random_Back();
						Turn_Left(Turn_Speed,1120);
						Set_Direction_Flag(Direction_Flag_Left);
					}
					else if(Temp_Rcon_Status & RconR_HomeT)
					{
						Turn_Left(Turn_Speed,999);
						Set_Direction_Flag(Direction_Flag_Left);
					}
				}
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_Rcon_Status();
				Base_Wall_On=0;
				Reset_Wheel_Step();
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
			}
    }

		/*------------------------------------------------------Virtual Wall--------------------*/
		#ifdef VIRTUAL_WALL
		Temp_Rcon_Status = Get_Rcon_Status();
		if(Is_VirtualWall())
		{
			OBS_OFF();
			Reset_VirtualWall();
			{
				Bumper_Counter++;
				Wall_Bumper_Counter++;
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
				Stuck++;
				Stop_Brifly();
				if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor+3))||(Stuck>4))
        {
          Turn_Right(Turn_Speed,300);
          Stop_Brifly();
          if(Wall_Follow_Short(3000))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
        }
				else
				{
					WalkAlongVirtualWall(Temp_Rcon_Status);
					Temp_Rcon_Status=0;
				}

				Set_Wheel_Step(0,0);
				Set_Mobility_Step(0);
			}
		}
		else
		{
			Virtual_Wall_NG++;
			if(Virtual_Wall_NG>100)
			{
				Virtual_Wall_NG=0;
			}
		}
    #endif

		/*------------------------------------------------------Cliff Event-----------------------*/
		Temp_Cliff_Status=Get_Cliff_Trig();
    if(Temp_Cliff_Status)
    {
			OBS_ON();
      Set_Wheel_Speed(0,0);
      Set_Dir_Backward();
	    delay(300);
			if(Get_Cliff_Trig()||(Get_LeftWheel_Step()<200))
			{
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
				    break;
					}
				}
				if(Cliff_Event(Temp_Cliff_Status))
				{
					Set_Direction_Flag(Direction_Flag_Left);
				}
				else
				{
					Set_Direction_Flag(Direction_Flag_Right);
				}
				Reset_Wheel_Step();
				Reset_TempPWM();
				Stuck++;
				Bumper_Counter++;
				Wall_Bumper_Counter+=2;
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				delay(100);
			}
			Reset_LeftWheel_Step();
			Set_Dir_Forward();
    }	
		/*------------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status()&LeftBumperTrig)
    {
			OBS_ON();
			Avoid_Flag=0;
		  Left_Wheel_Step_Buffer=Get_LeftWheel_Step();
			Add_Average(Get_LeftWheel_Step());
      if(Get_LeftWheel_Step()>14000)
      {
			  Wall_Bumper_Counter+=2;
      }
      else
      {
        Wall_Bumper_Counter+=3;
      }
     
      Wheel_Stop();
      delay(100);
      Temp_Bumper_Status = Get_Bumper_Status();
			Random_Back();
			if(Is_Bumper_Jamed())break;
			
			Stuck++;
			if(Stuck>7)
			{
			  Turn_Right(Turn_Speed,240);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			  if(Out_Trap_Left())break;
				Stuck=0;
				Set_Direction_Flag(Direction_Flag_Right);
				Turn_Right(Turn_Speed,80);
			}
		  else
			{
				if((Wall_Small_Counter>30)&&(!Is_Move_Finished(300000)))
				{
					Stop_Brifly();
          Turn_Right(Turn_Speed,400);
          Stop_Brifly();
          if(Wall_Follow_Short(3000))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Wall_Small_Counter=0;
					Wall_Mid_Counter=0;
				  Reset_Move_Distance();
				}
				else if((Wall_Mid_Counter>40)||(Is_Move_Finished(460000)))
				{
					Stop_Brifly();
          Turn_Right(Turn_Speed,400);
          Stop_Brifly();
          if(Wall_Follow_Short(1000))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
					Reset_Move_Distance();
				}
        else if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,400);
          Stop_Brifly();
          if(Wall_Follow_Short(Get_Average_Move()))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
        }
        else
        {
  				if(Left_Wheel_Step_Buffer<1000)
  				{
  				  if(Is_Direction_Right())
  					{
  					  Turn_Right(Turn_Speed,660);
  						Set_Direction_Flag(Direction_Flag_Right);
  					}
  					else
  					{
  					  Turn_Left(Turn_Speed,660);
  						Set_Direction_Flag(Direction_Flag_Left);
  					}
  				}
  				else
  				{
            if(Temp_Bumper_Status == AllBumperT)
            {
  						 Half_Turn_Right(Turn_Speed,800+Get_Random_Factor()*7);
							 Avoid_Flag=1;
            }
            else
            {
							if(Get_Random_Factor()<60)
							{
								Turn_Right(Turn_Speed,800);
								if(Left_Bumper_Avoiding())Avoid_Flag=1;
							}
							else
							{
								Half_Turn_Right(Turn_Speed,700+Get_Random_Factor()*9);
								Avoid_Flag=1;
							}
            }
  					Set_Direction_Flag(Direction_Flag_Right);
  				}
        }
			}
      if(!Avoid_Flag)
			{
				Reset_TempPWM();
				Set_Wheel_Step(0,0);
			}
			else
			{
				if(Get_LeftWheel_Step()>Get_RightWheel_Step())
				{
					Set_RightWheel_Step(Get_LeftWheel_Step());
				}
				Set_RightWheel_Step(Get_RightWheel_Step()/2); 
			}
			Set_Mobility_Step(0);
			Bumper_Counter++;
      Wall_Small_Counter++;
      Wall_Mid_Counter++;
    }

/*---------------------------------------------------------Right Bumper ----------------------------------*/
		if(Get_Bumper_Status()&RightBumperTrig)
		{
			OBS_ON();
			Avoid_Flag=0;
		  Left_Wheel_Step_Buffer=Get_LeftWheel_Step();
			Add_Average(Get_LeftWheel_Step());
      if(Get_LeftWheel_Step()>14000)
      {
			  Wall_Bumper_Counter+=1;
      }
      else
      {
        Wall_Bumper_Counter+=2;
      }
      Wheel_Stop();
      delay(100);
      Temp_Bumper_Status = Get_Bumper_Status();
			Random_Back();
			if(Is_Bumper_Jamed())break;
			Stuck++;
			if(Stuck>7)
			{
			  Turn_Left(Turn_Speed,240);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			  if(Out_Trap_Right())break;
				Stuck=0;
				Set_Direction_Flag(Direction_Flag_Left);
				Turn_Left(Turn_Speed,80);
			}
			else
			{
        if((Wall_Small_Counter>30)&&(!Is_Move_Finished(300000)))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
          if(Wall_Follow_Short(4000))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Reset_Move_Distance();
					Wall_Small_Counter=0;
        }
        else if((Wall_Mid_Counter>40)||(Is_Move_Finished(460000)))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
          if(Wall_Follow_Short(1000))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
          Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
					Reset_Move_Distance();
        }
        else if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
          if(Wall_Follow_Short(Get_Average_Move()))return;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
        }
        else
        {
  				if(Left_Wheel_Step_Buffer<1000)
  				{
  				  if(Is_Direction_Left())
  					{
  				    Turn_Left(Turn_Speed,660);
  						Set_Direction_Flag(Direction_Flag_Left);
  					}
  					else
  					{
  				    Turn_Right(Turn_Speed,660);
  						Set_Direction_Flag(Direction_Flag_Right);
  					}
  				}
  				else 
  				{
            if(Temp_Bumper_Status == AllBumperT)
            {
              Half_Turn_Left(Turn_Speed,800+Get_Random_Factor()*6);
							Avoid_Flag=1;
            }
            else
            {
							if(Get_Random_Factor()<60)
							{
								Turn_Left(Turn_Speed,800);
								if(Right_Bumper_Avoiding())Avoid_Flag=1;
							}
							else
							{
								Half_Turn_Left(Turn_Speed,700+Get_Random_Factor()*8);
								Avoid_Flag=1;
							}
            }
  					Set_Direction_Flag(Direction_Flag_Left);
  				}
        }
			}		 
			if(!Avoid_Flag)
			{
				Reset_TempPWM();
				Set_Wheel_Step(0,0);
			}
			else
			{
				if(Get_LeftWheel_Step()>Get_RightWheel_Step())
				{
					Set_RightWheel_Step(Get_LeftWheel_Step());
				}
				Set_RightWheel_Step(Get_RightWheel_Step()/2);
			}
			Set_Mobility_Step(0);
			Bumper_Counter++;
      Wall_Small_Counter++;
      Wall_Mid_Counter++;
		}	
		
		/*------------------------------------------------------OBS_Status-----------------------*/
		if(Temp_OBS_Status)
		{
		  Temp_OBS_Status=Get_OBS_Status();
			Left_Wheel_Step_Buffer=Get_LeftWheel_Step();
      Add_Average(Get_LeftWheel_Step());
			Set_Wheel_Speed(0,0);
			Reset_TempPWM();
			delay(100);
			N_H_T=0;
			Reset_HalfTurn_Flag();
			if(Is_LeftWheel_Reach(30))
			{
        if(Left_Wheel_Step_Buffer>14000)
        {
  			  Wall_Bumper_Counter+=1;
        }
        else
        {
          Wall_Bumper_Counter+=2;
        }
        Stop_Brifly();
				Bumper_Counter++;
			}

			if((Wall_Small_Counter>30)&&(!Is_Move_Finished(300000)))
      {                            
        if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
        {
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
        }
		  	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
        if(Wall_Follow_Short(4000))return;
        Stuck=0;
        Wall_Bumper_Counter=0;
				Wall_Small_Counter=0;
				Reset_Move_Distance();
				Wall_Mid_Counter=0;
      }
      else if((Wall_Mid_Counter>40)||(Is_Move_Finished(460000)))
      {                           
        if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
        {
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
        }
		  	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
        if(Wall_Follow_Short(1000))return;
        Stuck=0;
        Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
				Reset_Move_Distance();
      }
      else if((Wall_Bumper_Counter> (Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
     	{                            
        if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
        {
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
        }
		  	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
        if(Wall_Follow_Short(Get_Average_Move()))return;
        Stuck=0;
        Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
      }
      else
      {
				Random_Back();
  			Stuck++;
        if(Get_Bumper_Status())
        {
          Random_Back();
  			  if(Is_Bumper_Jamed())break;
          Stop_Brifly();
        }
  		  if(Left_Wheel_Step_Buffer<1000)
  			{
  			  if(Is_Direction_Left())
  				{
  				  if(Stuck>10)
  					{
  					  if(Out_Trap_Right())break;
  					  Stuck=0;
  						Turn_Left(Turn_Speed,240);
  					}
  					else
  					{
							if(Left_Wheel_Step_Buffer<300)
							{
								Turn_Left(Turn_Speed,400);
							}
							else
							{
								Turn_Left(Turn_Speed,400);
							}
  					}
						N_H_T=1;
  					Set_Direction_Flag(Direction_Flag_Left);
  				}
  				else
  				{
  				  if(Stuck>10)
  					{
  					  if(Out_Trap_Left())break;
  					  Stuck=0;
  						Turn_Right(Turn_Speed,240);
  					}
  					else
  					{
							if(Left_Wheel_Step_Buffer<300)
							{
								Turn_Right(Turn_Speed,400);
							}
							else
							{
								Turn_Right(Turn_Speed,400);
							}
  					}
						N_H_T=1;
  					Set_Direction_Flag(Direction_Flag_Right);
  				}
  			}
  			else
  			{
  			  Random_Factor=Left_Wheel_Step_Buffer%2;
          if(Temp_OBS_Status==0xA2)// LFR
  				{
    				if(Random_Factor)
    				{
    					Half_Turn_Left(Turn_Speed,750+Get_Random_Factor()*8);
    					Set_Direction_Flag(Direction_Flag_Left);
    				}
    				else
    				{
    					Half_Turn_Right(Turn_Speed,800+Get_Random_Factor()*8);
    					Set_Direction_Flag(Direction_Flag_Right);
    				}
          }
  				else if(Temp_OBS_Status==0x72)//LF
  				{
    				if(Random_Factor)
    				{
    				  Half_Turn_Right(Turn_Speed,1200);
    					Set_Direction_Flag(Direction_Flag_Right);
    				}
    				else 
    				{ 
    					Half_Turn_Left(Turn_Speed,1200);
    					Set_Direction_Flag(Direction_Flag_Left);
    				}
  				}
  			  else if(Temp_OBS_Status&0x02)//L
  				{
  				  if(Stuck>10)
  					{
  					  if(Out_Trap_Left())break;
  				    Stuck=0;
  						Half_Turn_Right(Turn_Speed,240);
  					}
  					else
  					{
    	        if((Bumper_Counter%3)==0)Half_Turn_Right(Turn_Speed,800+Get_Random_Factor()*7);
    					else Half_Turn_Right(Turn_Speed,750+Get_Random_Factor()*8);
  					}
  					Set_Direction_Flag(Direction_Flag_Right);
  				}
  				else
  				{
  				  if(Stuck>10)
  					{
  					  if(Out_Trap_Right())break;
  						Stuck=0;
  						Turn_Left(Turn_Speed,240);
  					}
  					else
  					{
  					  Temp_OBS_Status>>=4;
  						if(Temp_OBS_Status==0x08)
  						{
    					  if(Random_Factor)
    						{
    						  Half_Turn_Left(Turn_Speed,750+Get_Random_Factor()*7);
    							Set_Direction_Flag(Direction_Flag_Left);
    						}
    						else
    						{
    						  Half_Turn_Right(Turn_Speed,750+Get_Random_Factor()*7);
    							Set_Direction_Flag(Direction_Flag_Right);
    						}
      				}
  						else if(Temp_OBS_Status>8)
  						{
  							Half_Turn_Left(Turn_Speed,800+Get_Random_Factor()*7);
                Set_Direction_Flag(Direction_Flag_Left);
  						}
  						else
  						{
  							if((Bumper_Counter%3)==0)Half_Turn_Left(Turn_Speed,850+Get_Random_Factor()*7);
  							else Half_Turn_Left(Turn_Speed,800+Get_Random_Factor()*7);
                Set_Direction_Flag(Direction_Flag_Left);
  						}
  					}
  				}
  			}
      }
      OBS_Cycle = 0;
      On_TrapOut_Flag=0;
			if(!Is_Remote())
			{
				if(Get_OBS_Status())
				{
					if(!N_H_T)
					{
						Stop_Brifly();
						Random_Back();
						Stop_Brifly();
					}
					
					do
					{
						OBS_Cycle++;
						Stuck++;
						if(OBS_Cycle>6)
						{
							Adjust_OBST_Value();
							Stuck=Stuck-OBS_Cycle;
							break;
						}
						if(Is_Direction_Left())
						{
							if((Stuck>10)||(OBS_Cycle>7))
							{
								Stuck = 0;
								On_TrapOut_Flag=1;
								break;
							}
							else
							{
								OBS_Turn_Left(Turn_Speed,400);
							}
							Set_Direction_Flag(Direction_Flag_Left);
						}
						else
						{
							if((Stuck>10)||(OBS_Cycle>7))
							{
								Stuck = 0;
								On_TrapOut_Flag=2;
								break;
							}
							else
							{
								OBS_Turn_Right(Turn_Speed,400);
							}
							Set_Direction_Flag(Direction_Flag_Right);
						}
						delay(100);
					}
					while(Get_OBS_Status());
					Reset_HalfTurn_Flag();
				}
			}
      if(On_TrapOut_Flag==1)
      {
        if(Out_Trap_Right())break;
        Turn_Right(Turn_Speed,240);
        Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_TempPWM();
      }
      else if(On_TrapOut_Flag==2)
      {
        if(Out_Trap_Left())break;
        Turn_Right(Turn_Speed,240);
        Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_TempPWM();
      }
			if(!Is_HalfTurn_Flag())
			{
				Move_Forward(0,0);
				Reset_TempPWM();
				Set_Wheel_Step(0,0);
				delay(100);
			}
			else
			{
				if(Get_LeftWheel_Step()>Get_RightWheel_Step())
				{
					Set_RightWheel_Step(Get_LeftWheel_Step());
				}
				Set_RightWheel_Step(Get_RightWheel_Step()/2);
			}
			Set_Mobility_Step(0);
      Wall_Small_Counter++;
      Wall_Mid_Counter++;
			Temp_OBS_Status=0;
		}
		/* -----------------------------Speed up ----------------------------------*/
    if(Wall_Mid_Counter>70)
    {
      Wall_Mid_Counter=0;
      Reset_Move_Distance();
    }   
		if(Is_LeftWheel_Reach(29000))
		{
      if(Spiral())break;
			Set_Wheel_Step(0,0);
			Set_HalfTurn_Flag();
		}
    if(Is_LeftWheel_Reach(24000))
    {
      Wall_Small_Counter=0;
    }
		if(Is_LeftWheel_Reach(2500))
		{
			Stuck=0;
			Reset_Bumper_Error();
			if(Get_LeftBrush_Stall())Set_LeftBrush_Stall(0);
			if(Get_RightBrush_Stall())Set_RightBrush_Stall(0);
		}
		else if(Is_LeftWheel_Reach(10*DISTANCE_1CM))
		{
      Base_Wall_On=1;
			Set_Left_Brush(ENABLE);
			Set_Right_Brush(ENABLE);
		}
    
	/*-------------------------------------------------------------------------------------------------------------------*/	
    if((Get_Cliff_Trig()==0)&&(Get_Bumper_Status()==0))
    {
			if(Is_OBS_Near())//Get_OBS_Status()
			{
				if(Moving_Speed>RUN_SPEED_3)
				{
					Disacc++;
					if(Disacc>0)
					{
						Disacc=0;
						Moving_Speed-=5;
					}
				}
				OBS_Distance_Counter++;
				OBS_Delay = Moving_Speed*OBS_Distance_Counter;
				if(OBS_Delay>600)
				{
					OBS_Distance_Counter=0;
					Temp_OBS_Status = Get_OBS_Status();
				}
				Set_RightWheel_Step(200);
			}
			/*else if(Is_OBS_Near())
      {
        if(Moving_Speed>RUN_SPEED_5)
				{
					Moving_Speed-=1;
				}
				Set_RightWheel_Step(400);
      }*/			
			else
			{ 
				OBS_Distance_Counter=0;
				Temp_OBS_Status=0;
				Moving_Speed=(Get_RightWheel_Step()/80)+20;
				if(Is_HalfTurn_Flag())
				{
					Reset_HalfTurn_Flag();
					if(Moving_Speed<RUN_SPEED_14)Moving_Speed=RUN_SPEED_14;
				}
				else
				{
					if(Moving_Speed<RUN_SPEED_10)Moving_Speed=RUN_SPEED_10;
				}
				if(Moving_Speed>Max_Speed)Moving_Speed=Max_Speed;
			}
			Move_Forward(Moving_Speed,Moving_Speed);
    }
  }
}

/*------------------------------------------------------Out Trap Right--------------------------------------------------------*/
uint8_t Out_Trap_Right(void)
{
	int32_t R=0;
  uint8_t Motor_Check_Code=0;
	uint32_t Bump_Counter=0;
  Reset_Wheel_Step();
  Reset_Rcon_Status();
  Reset_Wall_Step();
  while(1)
	{
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
			  Set_Clean_Mode(Clean_Mode_Userinterface);
        Display_Content(LED_Exclamation,100,100,0,7);
				return 1;
			}
			Initialize_Motor();	  
		}
		/*-------------------------------------------------------Wheel ---------------------------------------*/
		if(Get_LeftWall_Step() - Get_RightWall_Step())
		{
			R=Get_LeftWall_Step() - Get_RightWall_Step();
			if(R>TURN_AROUND_CNT)//turn over 3600 degree
			{
				return 0;
			}
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
		
		if(Remote_Key(Remote_Left))
		{
		  Turn_Right(Turn_Speed,240);
			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  		Reset_Rcon_Remote();
			return 0;
		}
		if(Get_Rcon_Status()&Rcon_Signal_All_T)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			return 0;
		}
#endif

		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
 		}
		if(Get_Bumper_Status()&LeftBumperTrig)
		{
      Stop_Brifly();
			Wall_Move_Back();
      if(Is_Bumper_Jamed())return 1;
			Turn_Left(Turn_Speed,800);
      Stop_Brifly();
      Reset_LeftWheel_Step();
			Bump_Counter++;
		}

    if(Get_Bumper_Status()&RightBumperTrig)
		{
      Stop_Brifly();
      Wall_Move_Back();
      if(Is_Bumper_Jamed())return 1;
      Turn_Left(Turn_Speed,150);
      Stop_Brifly();
      Reset_LeftWheel_Step();
			Bump_Counter++;
		}
		if(Bump_Counter>15)return 0;
    if(Is_Front_Close())
    {
      Stop_Brifly();
      Turn_Left(Turn_Speed,640);
      Stop_Brifly();
      Reset_LeftWheel_Step();
    }
    if(Is_LeftWheel_Reach(5000))return 0;
		if(Get_RightWall_Step()>12000)return 0;

		if(Get_Cliff_Trig())
		{
		  return 0;
		}
    if(Get_LeftWheel_Step()<130)
		{
		  Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
		}
		else
		{
		  Move_Forward(RUN_SPEED_17,RUN_SPEED_3);
		}
	}
}
/*------------------------------------------------------Out Trap Left--------------------------------------------------------*/
uint8_t Out_Trap_Left(void)
{
	int32_t R=0;
  uint8_t Motor_Check_Code=0;
	uint32_t Bump_Counter=0;
  Reset_Wheel_Step();             
  Reset_Rcon_Status();
  Reset_Wall_Step();
  while(1)
	{
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
			  Set_Clean_Mode(Clean_Mode_Userinterface);
        Display_Content(LED_Exclamation,100,100,0,7);
				return 1;
			}
			Initialize_Motor();	  
		}
		/*-------------------------------------------------------Wheel ---------------------------------------*/
		if(Get_RightWall_Step()>Get_LeftWall_Step())
		{
			R=Get_RightWall_Step()-Get_LeftWall_Step();
			if(R>TURN_AROUND_CNT)//turn over 3600 degree
			{
				return 0;
			}
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
		
		if(Remote_Key(Remote_Right))
		{
		  Turn_Right(Turn_Speed,300);
			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
      Reset_Rcon_Remote();
			return 0;
		}	
    if(Get_Rcon_Status()&Rcon_Signal_All_T)return 0;
		#ifdef VIRTUAL_WALL
		if(VirtualWall_TurnRight(Get_Rcon_Status()))
		{
			return 0;
		}
		#endif

		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
 		}
		if(Get_Bumper_Status()&RightBumperTrig)
		{
      Stop_Brifly();
      Wall_Move_Back();
      if(Is_Bumper_Jamed())return 1;
			Turn_Right(Turn_Speed,1025);
      Stop_Brifly();
			Reset_RightWheel_Step();
			Bump_Counter++;
		}

    if(Get_Bumper_Status()&LeftBumperTrig)
		{
      Stop_Brifly();
      Wall_Move_Back();
      if(Is_Bumper_Jamed())return 1;
      Turn_Right(Turn_Speed,150);
      Stop_Brifly();
			Reset_RightWheel_Step();
			Bump_Counter++;
		}
		if(Bump_Counter>15)return 0;
    if(Is_Front_Close())
    {
      Stop_Brifly();
      Turn_Right(Turn_Speed,800);
      Stop_Brifly();
      Reset_RightWheel_Step();
    }
    if(Is_RightWheel_Reach(5000))return 0;
    if(Get_LeftWall_Step()>12000)return 0;

		if(Get_Cliff_Trig())
		{
		  return 0;
		}
    if(Get_RightWheel_Step()<130)
		{
		  Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
		}
		else
		{
		  Move_Forward(RUN_SPEED_3,RUN_SPEED_17);
		}
	}
}


uint8_t Left_Bumper_Avoiding(void)
{
  uint16_t Counter_Watcher=0;
	uint32_t Temp_A_Speed=0;
	uint8_t motor_check=0;
	Reset_Wheel_Step();
  Move_Forward(RUN_SPEED_3,RUN_SPEED_10); 
	while(Get_RightWheel_Step()<3400)
	{
	  delay(1);
		Temp_A_Speed = Get_RightWheel_Step()/8 + 20;
		if(Temp_A_Speed<RUN_SPEED_10)Temp_A_Speed=RUN_SPEED_10;
		if(Temp_A_Speed>RUN_SPEED_17)Temp_A_Speed=RUN_SPEED_17;
		Move_Forward(Temp_A_Speed/4,Temp_A_Speed); 
		
		Counter_Watcher++;
	  if(Counter_Watcher>50000)
		{
		  if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
				return 0;
			}
			return 0;
		}
		if(Touch_Detect())
		{
			return 0;
		}
		if(Get_Rcon_Remote())return 0;
		if(Get_Bumper_Status())break;
		if(Get_OBS_Status())break;
		if(Get_Cliff_Trig())break;
		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return 0;
		}
    if(Get_Rcon_Status()&Rcon_Signal_All_T)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			break;
		}
#endif

	} 
	if(Get_RightWheel_Step()>=3400)return 1;
	Stop_Brifly();
	Stop_Brifly();
	return 0;
}

uint8_t Right_Bumper_Avoiding(void)
{
  uint16_t Counter_Watcher=0;
	uint32_t Temp_A_Speed=0;
	uint8_t motor_check=0;
	Reset_Wheel_Step();
  Move_Forward(RUN_SPEED_10,RUN_SPEED_3); 
	while(Get_LeftWheel_Step()<3400)
	{
	  delay(1);
		Temp_A_Speed = Get_LeftWheel_Step()/8 + 20;
		if(Temp_A_Speed<RUN_SPEED_10)Temp_A_Speed=RUN_SPEED_10;
		if(Temp_A_Speed>RUN_SPEED_17)Temp_A_Speed=RUN_SPEED_17;
		Move_Forward(Temp_A_Speed,Temp_A_Speed/4); 
		Counter_Watcher++;
	  if(Counter_Watcher>50000)
		{
		  if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
				return 0;
			}
			return 0;
		}
		if(Touch_Detect())
		{
			return 0;
		}
		if(Get_Rcon_Remote())return 0;
		if(Get_Bumper_Status())break;
		if(Get_OBS_Status())break;
		if(Get_Cliff_Trig())break;
		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return 0;
		}
    if(Get_Rcon_Status()&Rcon_Signal_All_T)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			break;
		}
#endif

	} 
	if(Get_LeftWheel_Step()>=3400)return 1;
	Stop_Brifly();
	Stop_Brifly();
	return 0;
}

/*-------- Turn Left ------------------------*/
void Half_Turn_Left(uint16_t speed,uint16_t angle)
{
	uint8_t H_S=0;
  uint16_t Counter_Watcher=0;
  uint8_t Temp_H_Flag=0;
	uint8_t motor_check=0;
	angle = angle*ANGLE_MUL;
	Turn_Left(speed,angle/2);
	if(Get_Rcon_Remote())return;	
	Set_Wheel_Step(0,0);
	Reset_TempPWM();
	delay(100);
	Move_Forward(0,speed);
	Counter_Watcher=0;
	Reset_HalfTurn_Flag();
	while(Get_RightWheel_Step()<angle)
	{
		delay(1);
		Counter_Watcher++;
	  if(Counter_Watcher>40000)
		{
			if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return;
		}
		if(Is_Turn_Remote())Temp_H_Flag=1;
		if(Get_Bumper_Status())Temp_H_Flag=1;
		if(Is_Front_Close())
		{
			/*Set_Wheel_Speed(0,RUN_SPEED_10);*/
		}
		else
		{
			H_S = Get_RightWheel_Step()/8 + speed;
			if(H_S>RUN_SPEED_14)H_S=RUN_SPEED_14;
			Set_Wheel_Speed(0,H_S);
		}
    if(Get_Cliff_Trig())Temp_H_Flag=1;
	  if(Touch_Detect())
		{
			Temp_H_Flag=1;
		}
		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			Temp_H_Flag=1;
		}
		if(Temp_H_Flag)break;
		#ifdef VIRTUAL_WALL
		if(Is_VirtualWall())Temp_H_Flag=1;
    #endif

	}
	if(Temp_H_Flag)
	{
		Set_Wheel_Speed(0,0);
		return;
	}
	Set_HalfTurn_Flag();
}

/*-------- Tur_Right ------------------------*/
void Half_Turn_Right(uint16_t speed,uint16_t angle)
{
	uint16_t H_S=0;
  uint16_t Counter_Watcher=0;
	uint8_t Temp_H_Flag=0;
	uint8_t motor_check=0;
	angle = angle*ANGLE_MUL;	
  Turn_Right(speed,angle/2);
	if(Get_Rcon_Remote())return;
	Set_Wheel_Step(0,0);
	Reset_TempPWM();
	delay(100);
	Move_Forward(speed,0);
	Counter_Watcher=0;
	Reset_HalfTurn_Flag();
	while(Get_LeftWheel_Step()<angle)
	{
		delay(1);
		Counter_Watcher++;
	  if(Counter_Watcher>40000)
		{
		  if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return;
		}
    if(Is_Turn_Remote())Temp_H_Flag=1;
		if(Get_Bumper_Status())Temp_H_Flag=1;
		if(Is_Front_Close())
		{
			/*Set_Wheel_Speed(RUN_SPEED_10,0);*/
		}
		else
		{
			H_S = Get_LeftWheel_Step()/8 + speed;
			if(H_S>RUN_SPEED_14)H_S=RUN_SPEED_14;
			Set_Wheel_Speed(H_S,0);
		}
    if(Get_Cliff_Trig())Temp_H_Flag=1;
	  if(Touch_Detect())
		{
			Temp_H_Flag=1;
		}		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {			
			Temp_H_Flag=1;
		}
		
		if(Temp_H_Flag)break;
		#ifdef VIRTUAL_WALL
		if(Is_VirtualWall())Temp_H_Flag=1;
    #endif

	}
	if(Temp_H_Flag)
	{
		Set_Wheel_Speed(0,0);
		return;
	}
	Set_HalfTurn_Flag();
}

void Set_HalfTurn_Flag(void)
{
	Half_Turn_Flag=1;
}
void Reset_HalfTurn_Flag(void)
{
	Half_Turn_Flag=0;
}
uint8_t Is_HalfTurn_Flag(void)
{
	return Half_Turn_Flag;
}



