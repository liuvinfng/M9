 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Random Path Cleaning Function
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "HomeStraight.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Spot.h"
#include "Rcon.h"
#include "Home.h"
#include "HomeWall.h"
#include "RandomRuning.h"
#include "WallFollow.h"
#include "WallFollowShort.h"
#include "VirtualWall.h"


#define Home_ShortWall_Trig 8 

/* --------------------------------------------------Random Runnincg mode----------------------*/
void HomeStraight_Mode(void)
{

	volatile uint8_t Stuck = 0, Motor_Check_Code = 0,
	                 Temp_Cliff_Status=0,Temp_OBS_Status=0,Random_Factor=0;
	volatile uint8_t Bumper_Counter=0,Wall_Bumper_Counter=0;
	volatile uint32_t Left_Wheel_Step_Buffer=0;
  uint32_t Moving_Speed=10;
	#ifdef MOBILITY
  uint32_t Temp_Mobility_Distance=0;
	#endif
	uint8_t Low_Power_Counter=0;
	uint8_t Disacc=0;

	uint32_t OBS_Distance_Counter=0;
  uint8_t OBS_Cycle=0;
  uint8_t Temp_Bumper_Status=0;
  uint8_t Wall_Small_Counter = 0;
  uint8_t Wall_Mid_Counter = 0;
  uint8_t Wall_Bumper_Factor = 0;
  uint8_t On_TrapOut_Flag=0;
	uint8_t Avoid_Flag=0;
	uint8_t N_H_T=0;
	volatile uint32_t OBS_Delay=0;
	
	
  Wall_Bumper_Factor = Get_Random_Factor()/15;
	Display_Clean_Status(Display_Home);
	Reset_MoveWithRemote();

	Reset_Touch();

	Set_Clean_Mode(Clean_Mode_GoHome);

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
	
  Display_Home_LED();
	Set_BLDC_Speed(Vac_Speed_NormalL);

  while(1)
  {
		
		Wall_Dynamic_Base(200);
		OBS_Dynamic_Base(40);
		
		if(Is_Station())
    {			
			OBS_OFF();
      GoHome();
			OBS_ON();
			if(Get_Clean_Mode()!=Clean_Mode_GoHome)
			{
				if(Get_Clean_Mode()!=Clean_Mode_Charging)
				{
					Speaker(EXIT_RECHARGE_MODE);
				}				
				return;			
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
		if(Check_Bat_SetMotors(Home_Vac_Power,Home_SideBrush_Power,Home_MainBrush_Power))//Low Battery Event
    {
		  Low_Power_Counter++;
			if(Low_Power_Counter>10)
			{
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
			Speaker(EXIT_RECHARGE_MODE);	
			break;
		}
		if(Get_Rcon_Remote())
		{
			#ifndef STANDARD_REMOTE
			if(Remote_Key(Remote_Left))
			{
        Stop_Brifly();
				Set_Dir_Left();
				Set_Wheel_Speed(RUN_SPEED_13,RUN_SPEED_13);
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
				Set_Wheel_Speed(RUN_SPEED_13,RUN_SPEED_13);
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
#if 0
			if(Remote_Key(Remote_Max))
			{
				//Switch_VacMode();
        Stop_Brifly();
        Turn_Right(Turn_Speed,1800);
        Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
#endif
			#endif
#if 0
			if(Remote_Key(Remote_Wallfollow)) //                                    Check Key Home
			{
				if(Home_Wall_Follow(Homel_Wall_Distance))return;
				Wall_Small_Counter=0;
				Reset_Move_Distance();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
#endif
		}

                /*------------------------------------------------------Virtual Wall--------------------*/
		#ifdef VIRTUAL_WALL
		if(Is_VirtualWall())
		{
			OBS_OFF();
			Reset_VirtualWall();
			Stop_Brifly();
			WalkAlongVirtualWall(0);
			Bumper_Counter++;
			Wall_Bumper_Counter+=2;
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
		}
    #endif

		/*------------------------------------------------------Cliff Event-----------------------*/
		Temp_Cliff_Status=Get_Cliff_Trig();
    if(Temp_Cliff_Status)
    {
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
				Stop_Brifly();
				Reset_Wheel_Step();
				Stuck++;
				Bumper_Counter++;
				Wall_Bumper_Counter+=2;
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			}
			Reset_LeftWheel_Step();
			Set_Dir_Forward();
    }	
		/*------------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status()&LeftBumperTrig)
    {
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
			  if(Home_OutTrap_Left())break;
				Stuck=0;
				Set_Direction_Flag(Direction_Flag_Right);
				Turn_Right(Turn_Speed,80);
			}
		  else
			{
				if((Wall_Small_Counter>20)&&(!Is_Move_Finished(300000)))
				{
					Stop_Brifly();
          Turn_Right(Turn_Speed,400);
          Stop_Brifly();
					OBS_OFF();
					if(Home_Wall_Follow(Homel_Wall_Distance)){OBS_ON();return;}
					OBS_ON();
					Wall_Small_Counter=0;
					Stop_Brifly();
					Stuck=0;
					Reset_Move_Distance();
					Wall_Bumper_Counter=0;
					Set_Direction_Flag(Direction_Flag_Right);
				}
        else if((Wall_Bumper_Counter>(Home_ShortWall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<50))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,400);
          Stop_Brifly();
					OBS_OFF();
					if(Home_Wall_Follow(Get_Average_Move())){OBS_ON();return;}
					OBS_ON();
					Wall_Small_Counter=0;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Reset_Move_Distance();
          Set_Direction_Flag(Direction_Flag_Right);
        }
        else
        {
  				if(Left_Wheel_Step_Buffer<1000)
  				{
  				  if(Is_Direction_Right())
  					{
  					  Turn_Right(Turn_Speed,560);
  						Set_Direction_Flag(Direction_Flag_Right);
  					}
  					else
  					{
  					  Turn_Left(Turn_Speed,560);
  						Set_Direction_Flag(Direction_Flag_Left);
  					}
  				}
  				else
  				{
            if(Temp_Bumper_Status == AllBumperT)
            {
  						 Half_Turn_Right(Turn_Speed,960+Get_Random_Factor()*8);
							 Avoid_Flag=1;
            }
            else
            {
              Turn_Right(Turn_Speed,800);
  						if(Left_Bumper_Avoiding())Avoid_Flag=1;
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
			Reset_Rcon_Status();
    }

/*---------------------------------------------------------Right Bumper ----------------------------------*/
		if(Get_Bumper_Status()&RightBumperTrig)
		{
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
			  Turn_Left(Turn_Speed,240);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			  if(Home_OutTrap_Right())break;
				Stuck=0;
				Set_Direction_Flag(Direction_Flag_Left);
				Turn_Left(Turn_Speed,80);
			}
			else
			{
        if((Wall_Small_Counter>20)&&(!Is_Move_Finished(300000)))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
    		  Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
					OBS_OFF();
					if(Home_Wall_Follow(Homel_Wall_Distance)){OBS_ON();return;}
					OBS_ON();
					Wall_Small_Counter=0;
					Stop_Brifly();
					Stuck=0;
					Wall_Bumper_Counter=0;
					Reset_Move_Distance();
					Set_Direction_Flag(Direction_Flag_Right);
        }
        else if((Wall_Bumper_Counter>(Home_ShortWall_Trig+Wall_Bumper_Factor-3))&&(Get_Random_Factor()<50))
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
					OBS_OFF();
					if(Home_Wall_Follow(Get_Average_Move())){OBS_ON();return;}
					OBS_ON();
					Wall_Small_Counter=0;
          Stuck=0;
          Wall_Bumper_Counter=0;
					Reset_Move_Distance();
          Set_Direction_Flag(Direction_Flag_Right);
        }
        else
        {
  				if(Left_Wheel_Step_Buffer<1000)
  				{
  				  if(Is_Direction_Left())
  					{
  				    Turn_Left(Turn_Speed,560);
  						Set_Direction_Flag(Direction_Flag_Left);
  					}
  					else
  					{
  				    Turn_Right(Turn_Speed,560);
  						Set_Direction_Flag(Direction_Flag_Right);
  					}
  				}
  				else 
  				{
            if(Temp_Bumper_Status == AllBumperT)
            {
              Half_Turn_Left(Turn_Speed,960+Get_Random_Factor()*8);
							Avoid_Flag=1;
            }
            else
            {
              Turn_Left(Turn_Speed,800);
  						if(Right_Bumper_Avoiding())Avoid_Flag=1;
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
			Reset_Rcon_Status();
		}	
		
		/*------------------------------------------------------OBS_Status-----------------------*/

    if(Temp_OBS_Status)
		{
		  Temp_OBS_Status=Get_OBS_Status();
			Left_Wheel_Step_Buffer=Get_LeftWheel_Step();
      Add_Average(Get_LeftWheel_Step());
			//Random_Back();
			Stop_Brifly();
			N_H_T=0;
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
				OBS_OFF();
				if(Home_Wall_Follow(Homel_Wall_Distance)){OBS_ON();return;}
				OBS_ON();
        Stuck=0;
        Wall_Bumper_Counter=0;
				Wall_Small_Counter=0;
				Reset_Move_Distance();
				Wall_Mid_Counter=0;
      }
      else if((Wall_Bumper_Counter> (Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
     	{                            
        if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
        {
          Turn_Right(Turn_Speed,900);
          Stop_Brifly();
        }
		  	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				OBS_OFF();
				if(Home_Wall_Follow(Get_Average_Move())){OBS_ON();return;}
				OBS_ON();
        Stuck=0;
        Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
      }
      else
      {
				Random_Back();
			  Stop_Brifly();
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
  					  if(Home_OutTrap_Right())break;
  					  Stuck=0;
  						Turn_Left(Turn_Speed,240);
  					}
  					else
  					{
							if(Left_Wheel_Step_Buffer<300)
							{
								Turn_Left(Turn_Speed-10,400);
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
  					  if(Home_OutTrap_Left())break;
  					  Stuck=0;
  						Turn_Right(Turn_Speed,240);
  					}
  					else
  					{
							if(Left_Wheel_Step_Buffer<300)
							{
								Turn_Right(Turn_Speed-10,400);
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
								OBS_Turn_Left(Turn_Speed-5,400);
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
								OBS_Turn_Right(Turn_Speed-5,400);
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
        if(Home_OutTrap_Right())break;
        Turn_Right(Turn_Speed-5,240);
        Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_TempPWM();
      }
      else if(On_TrapOut_Flag==2)
      {
        if(Home_OutTrap_Left())break;
        Turn_Right(Turn_Speed-5,240);
        Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_TempPWM();
      }
			if(!Is_HalfTurn_Flag())
			{
				Stop_Brifly();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_TempPWM();
				//Beep(1);
				delay(500);
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
      Wall_Small_Counter++;
      Wall_Mid_Counter++;
			Temp_OBS_Status=0;
		}
		/* -----------------------------Speed up ----------------------------------*/
    if(Wall_Small_Counter>40)Wall_Small_Counter=0;
    if(Wall_Mid_Counter>70)
    {
      Wall_Mid_Counter=0;
      Reset_Move_Distance();
    }
        
		if(Is_LeftWheel_Reach(31500))
		{
      Stop_Brifly();
      Stop_Brifly();
      if(Get_Random_Factor()<50)
      {
		    Turn_Right(Turn_Speed,1280+Get_Random_Factor()*10);
      }
      else
      {
        Turn_Left(Turn_Speed,640+Get_Random_Factor()*10);
      }
			Reset_Wheel_Step();
		}
    if(Is_LeftWheel_Reach(24000))
    {
      Wall_Small_Counter=0;
    }
		if(Is_LeftWheel_Reach(2500))
		{
			Stuck=0;
			Reset_Bumper_Error();
		}
		else if(Is_LeftWheel_Reach(10*DISTANCE_1CM))
		{
//      Base_Wall_On=1;
			Set_Left_Brush(ENABLE);
			Set_Right_Brush(ENABLE);
		}
    /*-------------------------------------------------------------------------------------------------------------------*/	
    if((Get_Cliff_Trig()==0)&&(Get_Bumper_Status()==0))
    {
			if(OBS_SLOW||Is_OBS_Near())
      {
        if(Moving_Speed>30)
				{
					Moving_Speed--;
				}
				Set_RightWheel_Step(400);
      }
//      if(Is_OBS_Near2())
//			{
//				if(Moving_Speed>25)
//				{
//					Moving_Speed--;
//				}
//				Set_RightWheel_Step(300);
//			}
			if(Get_OBS_Status())
			{
				if(Moving_Speed>10)
				{
					Disacc++;
					if(Disacc>3)
					{
						Disacc=0;
						Moving_Speed--;
					}
				}
				OBS_Distance_Counter++;
				OBS_Delay = Moving_Speed*OBS_Distance_Counter;
				if(OBS_Delay>1000)
				{
					OBS_Distance_Counter=0;
					Temp_OBS_Status = Get_OBS_Status();
				}
				Set_RightWheel_Step(200);
			}
			else
			{ 
				OBS_Distance_Counter=0;
				Temp_OBS_Status=0;
				Moving_Speed=(Get_RightWheel_Step()/80)+20;
				if(Is_HalfTurn_Flag())
				{
					Reset_HalfTurn_Flag();
					if(Moving_Speed<35)Moving_Speed=35;
				}
				else
				{
					if(Moving_Speed<25)Moving_Speed=25;
				}
				if(Moving_Speed>Max_Speed)Moving_Speed=Max_Speed;
			}
			if(Get_Rcon_Status()&(RconFL_LEFT|RconFR_LEFT|RconL_LEFT))
			{
				Reset_Rcon_Status();
				Move_Forward(Moving_Speed,Moving_Speed/2);
				delay(2000);
			}
			else if(Get_Rcon_Status()&(RconFL_RIGHT|RconFR_RIGHT|RconR_RIGHT))
			{
				Reset_Rcon_Status();
				Move_Forward(Moving_Speed/2,Moving_Speed);
				delay(2000);
			}
			else
			{
				Move_Forward(Moving_Speed,Moving_Speed);
			}			
		}
  }
}

/*------------------------------------------------------Out Trap Right--------------------------------------------------------*/
uint8_t Home_OutTrap_Right(void)
{
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

		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
 		}
		if(Get_Bumper_Status()&LeftBumperTrig)
		{
      Stop_Brifly();
			Move_Back();
      if(Is_Bumper_Jamed())return 1;
			Turn_Left(Turn_Speed,800);
      Stop_Brifly();
      Reset_LeftWheel_Step();
			Bump_Counter++;
		}

    if(Get_Bumper_Status()&RightBumperTrig)
		{
      Stop_Brifly();
      Move_Back();
      if(Is_Bumper_Jamed())return 1;
      Turn_Left(Turn_Speed,150);
      Stop_Brifly();
      Reset_LeftWheel_Step();
			Bump_Counter++;
		}
		if(Bump_Counter>10)return 0;
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
		  Move_Forward(RUN_SPEED_10,RUN_SPEED_10);
		}
		else
		{
		  Move_Forward(RUN_SPEED_17,RUN_SPEED_5);
		}
	}
}
/*------------------------------------------------------Out Trap Left--------------------------------------------------------*/
uint8_t Home_OutTrap_Left(void)
{
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

		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
 		}
		if(Get_Bumper_Status()&RightBumperTrig)
		{
      Stop_Brifly();
      Move_Back();
      if(Is_Bumper_Jamed())return 1;
			Turn_Right(Turn_Speed,1025);
      Stop_Brifly();
			Reset_RightWheel_Step();
			Bump_Counter++;
		}

    if(Get_Bumper_Status()&LeftBumperTrig)
		{
      Stop_Brifly();
      Move_Back();
      if(Is_Bumper_Jamed())return 1;
      Turn_Right(Turn_Speed,150);
      Stop_Brifly();
			Reset_RightWheel_Step();
			Bump_Counter++;
		}
		if(Bump_Counter>10)return 0;
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
		  Move_Forward(RUN_SPEED_10,RUN_SPEED_10);
		}
		else
		{
		  Move_Forward(RUN_SPEED_5,RUN_SPEED_17);
		}
	}
}




