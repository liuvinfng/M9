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
#include "HomeWall.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Spot.h"
#include "Rcon.h"
#include "Home.h"
#include "VirtualWall.h"

extern volatile OBS_ADC OBS_Status;

extern uint16_t moveToPointTimeCount;

uint8_t Home_Wall_Follow(uint32_t dis)
{
	volatile uint8_t Motor_Check_Code = 0;
  volatile int32_t Proportion=0;
	int32_t Delta=0;
	int32_t Previous=0;
  volatile int32_t Wall_Distance=1000;
	volatile int32_t Move_Distance = 100;
	volatile int32_t Left_Wall_Speed=0,Right_Wall_Speed=0;

//  uint32_t Temp_Rcon_Status = 0;

//	uint8_t Temp_Counter=0;
	uint8_t Jam=0;
	int32_t R=0;

//  uint32_t TempTime = 0;

  uint8_t Bumper_Limit = 0;
//	uint8_t Temp_Dirt_Status=0;

	uint8_t Wall_BH_Counter=0;

//	int16_t Left_Wall_Buffer[3]={0};

  uint32_t Temp_Mobility_Distance=0;
  uint8_t Mobility_Temp_Error=0;
//  uint8_t Mobility_Error=0;

//	int16_t Temp_Slow=0;

  int32_t Follow_Distance = 8000 + dis*4 + ((int32_t)Get_Random_Factor()*260);



  Bumper_Limit = Get_Random_Factor()/25;
  Bumper_Limit +=8;

	Reset_Bumper_Error();

	Work_Motor_Configure();

	Set_RightWheel_Speed(15);
	Reset_Wall_Step();
	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  Reset_Rcon_Status();
  Reset_Wheel_Step();
  Set_Mobility_Step(1000);
	Reset_Average_Counter();
	Display_TM1618(Bumper_Limit,0);
	Display_Home_LED();
	Set_BLDC_Speed(Vac_Speed_NormalL);
	if(Is_Wall_GoHome())
	{
		moveToPointTimeCount = 0;
	}
  while(1)
  {
    //TempTime = Get_Mobility_Step();
    //if(TempTime>9999)Reset_Mobility_Step();
   // TempTime = Get_Move_Distance()/600;
                                  
    //TempTime = Get_MainBrush_Current();
   ////Display_Content(0,TempTime/100,TempTime%100,0,2);
				
		
    if(Is_Station())
    {
      //Stop_Brifly();
      //Turn_Right(Turn_Speed,400);
 
      OBS_OFF();
      GoHome();
			OBS_ON();
      return 1;
    }
		
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
            return 0;
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
      return 0;
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
			return 0;
		}
		/*------------------------------------------------------Check Battery-----------------------*/
		
		if(Check_Bat_SetMotors(Home_Vac_Power,Home_SideBrush_Power,Home_MainBrush_Power))//Low Battery Event
    {
			return 0;
 		}	

		/*------------------------------------------------------Virtual Wall-----------------------*/
		#ifdef VIRTUAL_WALL
    if(VirtualWall_TurnRight(Get_Rcon_Status()))
		{
			Move_Distance=150;
			Reset_WallAccelerate();
		}
		#endif

		/*---------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status()&RightBumperTrig)
    {
		  Wall_BH_Counter++;
		  if(Get_WallAccelerate()>80)
			{
        Stop_Brifly();
				Wall_Move_Back();
				if(Is_Bumper_Jamed())return 0;
        //Stop_Brifly();
			}
			if(Get_WallAccelerate()<2000)
			{
				Jam++;
			}
			else
			{
			  Jam=0;
			}

	    Turn_Right(Turn_Speed-5,720);
	    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			Reset_WallAccelerate();
			Move_Distance=375;

//			for(Temp_Counter=0;Temp_Counter<5;Temp_Counter++)
//			{
//			  Left_Wall_Buffer[Temp_Counter]=0;
//			}	
      Stop_Brifly();
      Reset_Wheel_Step();
    }
		if(Get_Bumper_Status()&LeftBumperTrig)
    {
		  Wall_BH_Counter++;
      Set_Wheel_Speed(0,0);
      Reset_TempPWM();
      delay(300);
			
      Wall_Distance-=100;
			
			if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;		
			
			if(Get_Bumper_Status()&RightBumperTrig)
			{
				Wall_Move_Back();
				if(Is_Bumper_Jamed())break;;
				Turn_Right(Turn_Speed-5,400);
				Move_Distance=100;
			}
			else
			{
				Wall_Move_Back();
				if(Is_Bumper_Jamed())break;
				Turn_Right(Turn_Speed-5,300);
				Move_Distance=150;
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

//			for(Temp_Counter=0;Temp_Counter<5;Temp_Counter++)
//			{
//			  Left_Wall_Buffer[Temp_Counter]=0;
//			}	
      Stop_Brifly();
      Reset_Wheel_Step();	

    }
    

		/*------------------------------------------------------Cliff Event-----------------------*/
    if(Get_Cliff_Trig())
    {
		  Move_Back();
			if(Get_Cliff_Trig())
			{
			  Move_Back();
				if(Get_Cliff_Trig()==(Status_Cliff_All))
				{
					  Set_Clean_Mode(Clean_Mode_Userinterface);
						//Set_Error_Code(Error_Code_Cliff);
				    break;
				}
				if(Get_Cliff_Trig())
			  {
				  if(Cliff_Escape())
					{
					  Set_Clean_Mode(Clean_Mode_Userinterface);
						//Set_Error_Code(Error_Code_Cliff);
            return 1;
					}
				}
			}
      Turn_Right(Turn_Speed,900);
      Stop_Brifly();
      return 0;
    }
		
		
		if((Is_Wall_GoHome()==0)||moveToPointTimeCount>1200)
		{
			if(Wall_BH_Counter>12)return 0;
			
			if(Jam>Bumper_Limit)
			{
				return 0;
			}
		}
		



//		delta_counter++;
//		if(delta_counter>10)
//		{
//		  delta_counter=0;
//			Left_Wall_Buffer[2]=Left_Wall_Buffer[1];
//			Left_Wall_Buffer[1]=Left_Wall_Buffer[0];
//			Left_Wall_Buffer[0]=OBS_Status.Left_Wall;
//			if(Left_Wall_Buffer[0]<300)
//			{
//			  if((Left_Wall_Buffer[1]-Left_Wall_Buffer[0])>(Wall_Distance/10))
//				{
//				  if((Left_Wall_Buffer[2]-Left_Wall_Buffer[1])>(Wall_Distance/20))
//					{
//					  if(WallFollowAccelerate>300)
//						{
//						  if(Get_RightWheel_Speed()>=Get_LeftWheel_Speed())
//							{
//								Move_Forward(30,9);
//								delay(1000);
//								WallFollowAccelerate=0;
//								Move_Distance=100;
//							}
//						}
//					}
//				}
//			}
//		}
		
		/*------------------------------------------------------Short Distance Move-----------------------*/
		if (Get_WallAccelerate() < (uint32_t) Move_Distance)
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
        Move_Forward(RUN_SPEED_10,RUN_SPEED_10);
      }
		}
		else
		{
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
	    if(!Is_FrontOBS_Trig())
	    {
			  	Proportion = Get_LWall_ADC();
					
					Proportion = Proportion*100/Wall_Distance;
					
					Proportion-=100;
				
				  Delta = Proportion - Previous;
					
					if(Wall_Distance>400)//over left
					{
						Left_Wall_Speed = 25 + Proportion/12 + Delta/5;
						Right_Wall_Speed = 25 - Proportion/10 - Delta/5;
						if(Right_Wall_Speed>33)
						{
							Left_Wall_Speed=7;
							Right_Wall_Speed=40;
						}
					}
					else 
					{
						Left_Wall_Speed = 22 + Proportion/16 + Delta/5;
						Right_Wall_Speed = 22 - Proportion/11 - Delta/5;
						if(Right_Wall_Speed>28)
						{
							Left_Wall_Speed=6;
							Right_Wall_Speed=30;
						}
					}

					Previous = Proportion;
					
					if(Left_Wall_Speed<0)Left_Wall_Speed=0;
					if(Left_Wall_Speed>40)Left_Wall_Speed=40;
					if(Right_Wall_Speed<0)Right_Wall_Speed=0;
					
					Move_Forward(Left_Wall_Speed,Right_Wall_Speed);

	
	      if(Get_RightWall_Step()>Get_LeftWall_Step())R=Get_RightWall_Step()-Get_LeftWall_Step();
			 				
				if(R>TURN_AROUND_CNT)
				{
					return 0;
				}
				
				if((Is_Wall_GoHome()==0)||moveToPointTimeCount>1200)
				{	
					if((Get_RightWall_Step()>Follow_Distance)||(Get_LeftWall_Step()>Follow_Distance))//about 5 Meter
					{
						return 0; 
					}
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
				  Turn_Right(Turn_Speed,1200);
			    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				}
        Reset_Wheel_Step();
				Wall_Distance+=200;
				if(Wall_Distance>1000)Wall_Distance=1000;
	    }
		}
  }
  return 0;
}



