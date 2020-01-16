
 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Robot move in a certain path like ZZZZZ patant
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
#include "Zigzag.h"
#include "Spot.h"
#include "Rcon.h"

extern volatile ADC_Value_Struct ADC_Value;

extern volatile OBS_ADC OBS_Status;


extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;

extern volatile uint32_t Mobility_Error;


void Zigzag(void)
{
  uint8_t Direction=1,Motor_Check_Code=0,Long_Distance_Counter=0;
	uint8_t Temp_Dirt_Status=0;
	uint16_t Angle=0;
	uint32_t Moving_Speed=0;
	uint8_t Low_Power_Counter=0;
  uint32_t Temp_Mobility_Distance=0;
  uint32_t Mobility_Temp_Error=0;

//  Display_Clean_Status(Display_Zizag);//                      __________
                          //                      |_________
                          //                      __________|

	Work_Motor_Configure();
	Move_Forward(5,5);
	Reset_Wheel_Step();
	Reset_Touch();

  if(!Is_Dustbin_Install())
  {
    Set_Error_Code(Error_Code_Dustbin);
    Set_Clean_Mode(Clean_Mode_Userinterface);
    Disable_Motors();
    return;
  }

  while(1)
	{
   // TempTime = Get_Mobility_Step();

    if(Get_Move_Distance()<300)
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
          if(Mobility_Temp_Error>3)
          {
            Mobility_Error++;
            if(Mobility_Error>3)
            {
              Move_Back();
              Turn_Right(50,1800);
              Reset_Wheel_Step();
              Mobility_Temp_Error=0;
              Set_Clean_Mode(Clean_Mode_RandomMode);
              break;
            }
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
		Motor_Check_Code=0;
		Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
			  Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
			if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
			else
			{
				 Set_Clean_Mode(Clean_Mode_RandomMode);
				 Initialize_Motor();
			}
      break;  
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
		  if(Remote_Key(Remote_Spot))
			{
			  Temp_Dirt_Status=Random_Dirt_Event();
				if(Temp_Dirt_Status==1)
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
				}
				else
				{
					Set_Clean_Mode(Clean_Mode_RandomMode);			
				}
				break;
			}
			if(Remote_Key(Remote_Left))
			{
			  Turn_Left(Turn_Speed,560);
				Move_Forward(30,30);
        Reset_Rcon_Remote();
				Reset_Wheel_Step();
			}
			if(Remote_Key(Remote_Right))
			{
			  Turn_Right(Turn_Speed,560);
				Move_Forward(30,30);
        Reset_Rcon_Remote();
				Reset_Wheel_Step();
			}
			if(Remote_Key(Remote_Home)) //                                    Check Key Home
			{
					Reset_Rcon_Remote();
					return;
			}
			if(Remote_Key(Remote_Wallfollow)) //                                    Check Key Home
			{
			  Set_Clean_Mode(Clean_Mode_WallFollow);
				Move_Forward(10,10);
				break;
			}
		}
		/*------------------------------------------------------Virtual wall Event-----------------------*/
    
    if(Get_Rcon_Status()&0x0f00)
    {
        Stop_Brifly();
        Reset_Rcon_Status();
        Stop_Brifly();
        Turn_Right(Turn_Speed,1800);
        Stop_Brifly();
        Move_Forward(2,2);
        Reset_Rcon_Status();
        Reset_Move_Distance();
        Set_Clean_Mode(Clean_Mode_RandomMode);
        break;
    }

		/*------------------------------------------------------Check Battery-----------------------*/
		
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
		  Low_Power_Counter++;
			if(Low_Power_Counter>10)
			{
			  Display_Battery_Status(Display_Low);//display low
				if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
				else Set_Clean_Mode(Clean_Mode_GoHome);
				break;
			}
 		}
		else
		{
		  Low_Power_Counter=0;
		}

				
		/*--------------------------------------------Cliff Event----------------------------------------*/
		if(Get_Cliff_Trig())
    {
			if(Is_LeftWheel_Reach(900))
			{
        Set_Wheel_Speed(0,0);
        Reset_TempPWM();
        Set_Dir_Backward();
  	    delay(150);
  		  Move_Back();
  			if(Get_Cliff_Trig())
  			{
  			  Move_Back();
					if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
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
  						Set_Error_Code(Error_Code_Cliff);
  				    break;
  					}
  				}
  			}
        Stop_Brifly();
				if(Direction)Turn_Right(Turn_Speed,900);
				else Turn_Left(Turn_Speed,900);
				Angle+=1;
				Reset_Wheel_Step();
			}
			else
			{
			  if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
			  else Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
    }
		/*--------------------------------------------OBS Event -------------------------------*/
		
		if(Get_OBS_Status())
		{
			  if(Is_LeftWheel_Reach(900))
				{
          Stop_Brifly();
					if(Direction)Turn_Right(Turn_Speed,1150);
					else Turn_Left(Turn_Speed,1150);
					Angle+=1;
					Reset_Wheel_Step();
          Stop_Brifly();
				}
				else
				{
				  if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
			    else Set_Clean_Mode(Clean_Mode_RandomMode);
					break;
				}
		}
		
		/*------------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status())
    {
		  if(Is_LeftWheel_Reach(900))
			{
        Stop_Brifly();
			  Random_Back();
        if(Is_Bumper_Jamed())break;
				if(Direction)Turn_Right(Turn_Speed,900);
				else Turn_Left(Turn_Speed,900);
				Angle+=1;
				Reset_Wheel_Step();
        Stop_Brifly();
			}
			else
			{
			  if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
		  	else Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
		}
		/*------------------------------------------------------Distance Check-----------------------*/
		if(Left_Wheel_Step>48000)
		{
      Stop_Brifly();
      Stop_Brifly();
		  Turn_Left(Turn_Speed,1150);
			Direction=1-Direction;
			Angle=0;
			Long_Distance_Counter++;
			if(Long_Distance_Counter>1)
			{
			  Long_Distance_Counter=0;
			  if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
			  else Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
      Reset_Wheel_Step();
		}

		/*--------------------------------------------------Check Angle--------------------------------*/
		if(Angle>1)
		{
		  Direction=1-Direction;
			Angle=0;
		}
		else if(Angle>0)
		{
		  if(Is_LeftWheel_Reach(1200))
			{
				if(Direction)Turn_Right(Turn_Speed,900);
				else Turn_Left(Turn_Speed,900);
				Angle+=1;
				Reset_Wheel_Step();
			}
		}
    if(Is_OBS_Near())
    {
      if(Moving_Speed>35)Set_RightWheel_Step(800);
    }
    if(Is_OBS_Near2())Reset_RightWheel_Step();
		Moving_Speed=(Get_RightWheel_Step()/80)+25;
		if(Moving_Speed<25)Moving_Speed=25;
		if(Moving_Speed>50)Moving_Speed=50;
		Move_Forward(Moving_Speed,Moving_Speed);
	}
}






