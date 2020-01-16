 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Searching the charging home base then move slowly to attach the charge pin
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  
/* Includes ------------------------------------------------------------------*/
#include "Home.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "RandomRuning.h"
#include "Rcon.h"


extern volatile OBS_ADC OBS_Status;

volatile uint8_t R_H_Flag=0;


/*-------------------------------***********---------------------------------GO Home  ----------------*/
void GoHome(void)
{
  uint32_t Receive_Code=0;

  Deceleration();	
//	Stop_Brifly();	
	Display_Home_LED();
  Receive_Code = Get_Rcon_Status();
	Reset_Rcon_Status();

  /*if(Receive_Code&(RconR_RIGHT|RconFR_RIGHT|RconFL_RIGHT))//fr ht
  {
    Turn_Left(Turn_Speed,300);
    Stop_Brifly();
    Around_ChargerStation(0);
    return;
  }	
  if(Receive_Code&(RconL_LEFT|RconFR_LEFT|RconFL_LEFT))//fl ht
  {
    Turn_Right(Turn_Speed,300);
    Stop_Brifly();
    Around_ChargerStation(1);
    return;
  }*/		
	
  if(Receive_Code&RconFR_HomeT)//fr ht
  {
    Turn_Left(Turn_Speed,900);
    Stop_Brifly();
    Around_ChargerStation(0);
    return;
  }	
	
  if(Receive_Code&RconFL_HomeT)//fl ht
  {
    Turn_Right(Turn_Speed,900);
    Stop_Brifly();
    Around_ChargerStation(1);
    return;
  }
		
  if(Receive_Code&RconL_HomeT)//l t
  {
    Turn_Right(Turn_Speed,400);
    Stop_Brifly();
    Around_ChargerStation(1);
    return;
  }
  if(Receive_Code&RconR_HomeT)//r t
  {
    Turn_Left(Turn_Speed,400);
    Stop_Brifly();
    Around_ChargerStation(0);
    return;
  }
	
	/*if(Receive_Code&RconBL_HomeT)//bl t
  {
    Turn_Right(Turn_Speed,300);
    Stop_Brifly();
    Around_ChargerStation(1);
    return;
  }
  if(Receive_Code&RconBR_HomeT)//br t
  {
    Turn_Left(Turn_Speed,300);
    Stop_Brifly();
    Around_ChargerStation(0);
    return;
  }*/
	
	if(Receive_Code&RconR_HomeR)//r r//edit
  {
    Turn_Left(Turn_Speed,400);
    Stop_Brifly();		
    Around_ChargerStation(0);
    return;
  }
	if(Receive_Code&RconL_HomeL)//l l//edit
  {
    Turn_Right(Turn_Speed,400);
    Stop_Brifly();		
    Around_ChargerStation(1);
    return;
  }


	/*-----------------------------------------*/
	if(Receive_Code&RconFL_HomeL)//fl l
  {
    Turn_Right(Turn_Speed,500);
    Stop_Brifly();
    Around_ChargerStation(1);	

//		By_Path();	
//		Reset_Rcon_Status();		
    return;
  }
	if(Receive_Code&RconFL_HomeR)//fl r
  {
    Turn_Left(Turn_Speed,500);
    Stop_Brifly();
    Around_ChargerStation(0);

//		By_Path();	
//		Reset_Rcon_Status();		
    return;
  }		
	
	if(Receive_Code&RconFR_HomeR)//fr r
  {
    Turn_Left(Turn_Speed,500);
    Stop_Brifly();
    Around_ChargerStation(0);

//		By_Path();	
//		Reset_Rcon_Status();		
    return;
  }
	if(Receive_Code&RconFR_HomeL)//fr l
  {
    Turn_Right(Turn_Speed,500);
    Stop_Brifly();
    Around_ChargerStation(1);

//		By_Path();	
//		Reset_Rcon_Status();		
    return;
  }	
	/*-----------------------------------------*/	


  /*if((Receive_Code&0x000000ff))// infront of : straight
  {
		Receive_Code&=0x000000ff;
    switch(Receive_Code)
		{
			case 0x00000008:Turn_Right(Turn_Speed,900);break;//		if(Receive_Code==RconFR_HomeL)
      case 0x00000010:Turn_Left(Turn_Speed,900);break;//		if(Receive_Code==RconFL_HomeR)
			case 0x0000000c:Turn_Right(Turn_Speed,400);break;//		if(Receive_Code==(RconFR_HomeR|RconFR_HomeL))
			case 0x00000030:Turn_Left(Turn_Speed,400);break;//		if(Receive_Code==(RconFL_HomeR|RconFL_HomeL))
			case 0x00000014:Turn_Left(Turn_Speed,700);break;//		if(Receive_Code==(RconFL_HomeR|RconFR_HomeR))
			case 0x00000028:Turn_Right(Turn_Speed,700);break;//		if(Receive_Code==(RconFL_HomeL|RconFR_HomeL))
			case 0x00000020:Turn_Right(Turn_Speed,700);break;//		if(Receive_Code==RconFL_HomeL)
			case 0x00000004:Turn_Left(Turn_Speed,700);break;//		if(Receive_Code==RconFR_HomeR)
			case 0x0000002c:Turn_Right(Turn_Speed,700);break;//		if(Receive_Code==(RconFR_HomeL|RconFL_HomeL|RconFR_HomeR))
			case 0x00000034:Turn_Left(Turn_Speed,700);break;//		if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
			//case 0x00000001:Turn_Left(Turn_Speed,700);break;//	if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
			case 0x00000002:Turn_Right(Turn_Speed,1300);break;//	if(Receive_Code==(RconR_HomeL))
			case 0x00000040:Turn_Left(Turn_Speed,1300);break;//		if(Receive_Code==(RconL_HomeR))
			//case 0x00000080:Turn_Left(Turn_Speed,700);break;//	if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
			default:Turn_Right(Turn_Speed,900);break;
	  }
		Stop_Brifly();
 	
		By_Path();	
		Reset_Rcon_Status();
    return;
  }*/
		
	if((Receive_Code&(RconBR_HomeL|RconBR_HomeR)))//br hlr
  {
    Turn_Right(Turn_Speed,1800);
//		Stop_Brifly();
//		delay(1500);
//		Receive_Code = Get_Rcon_Status();
//		Receive_Code&=0x0000003C;
//		if(Receive_Code)
		{
			By_Path();
		}
		Reset_Rcon_Status();
    return;
  }
	
	/*if((Receive_Code&(RconBL_HomeL|RconBL_HomeR)))//bl hlr
  {
    Turn_Left(Turn_Speed,900);
//    Stop_Brifly();
//		delay(1500);
//		Receive_Code = Get_Rcon_Status();
//		Receive_Code&=0x0000003C;
//		if(Receive_Code)
		{
			By_Path();
		}
		Reset_Rcon_Status();
    return;
  }*/
  
}
/*--------------------------------------------------------------------Move Around Station----------------*/
void Around_ChargerStation(uint8_t Dir)
{
	uint8_t Temp_Position=0,cycle_cnt=0;
	#ifdef MOBILITY
  uint8_t Mobility_Temp_Error=0;
  uint32_t Temp_Mobility_Distance=0;
	#endif
  uint32_t Temp_Rcon_Status=0;
  uint8_t Signal_Counter=0;
	uint32_t No_Signal_Counter=0;
	uint32_t N_Around_LRSignal=0;
  uint8_t Bumper_Counter=0;
  Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  Set_SideBrush_PWM(30,30);
  Set_MainBrush_PWM(30);
	Set_BLDC_Speed(Vac_Speed_NormalL);
  Reset_Rcon_Status();
	Display_Home_LED();
  Reset_Wheel_Step();
  Reset_Move_Distance();
  while(1)
  {
		/*------------------------------------------------------Check Battery-----------------------*/
		Check_Bat_SetMotors(Home_Vac_Power,Home_SideBrush_Power,Home_MainBrush_Power);//Low Battery Event
		
		#ifdef MOBILITY
    if(Get_LeftWheel_Step()<500)
    {
      Mobility_Temp_Error=0;
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
				Set_Left_Brush(ENABLE);
				Set_Right_Brush(ENABLE);
        Temp_Mobility_Distance = Get_Move_Distance();
        if(Get_Mobility_Step()<1)
        {
          Mobility_Temp_Error++;
          if(Mobility_Temp_Error>3)
          {
            Set_Clean_Mode(Clean_Mode_GoHome);
            return;
          }
        }
        else
        {
          Mobility_Temp_Error=0;
        }
        Reset_Mobility_Step();
      }
    }
		#endif
		
    if(Get_Cliff_Trig())
	  {
      Set_Wheel_Speed(0,0);
      Set_Dir_Backward();
	    delay(300);			
		  Move_Back();
      Move_Back();
			Turn_Left(Turn_Speed,1750);
			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
      Set_Clean_Mode(Clean_Mode_GoHome);
      return;
		}

		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		if(GetBatteryVoltage()<1150)
	  {
		  Display_Battery_Status(Display_Low);
		  delay(10000);
			Set_Clean_Mode(Clean_Mode_Sleep);
		  return;
	  }

    if(Home_Check_Current())return;


    if(Get_Bumper_Status())
    {
      Bumper_Counter++;
      Random_Back();
			if(Is_Bumper_Jamed())return;
      if(Dir)Turn_Left(Turn_Speed,1800);
      else Turn_Right(Turn_Speed,1800);
			Reset_Rcon_Status();
      Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
      Dir = 1-Dir;
      if(Bumper_Counter>1)
      {
        Set_Clean_Mode(Clean_Mode_GoHome);
        return ;
      }
      Reset_Wheel_Step();
			No_Signal_Counter=0;
    }
    Temp_Rcon_Status = Get_Rcon_Status();
    if(Temp_Rcon_Status)
    {
			No_Signal_Counter=0;
      Reset_Rcon_Status();
    }
		else
		{
			No_Signal_Counter++;
			if(No_Signal_Counter>80)
			{
				Set_Clean_Mode(Clean_Mode_GoHome);
        return ;
			}
		}
    /*
    if(Temp_Rcon_Status&0x4000)
    {
      Turn_Right(30,2200);
 
      Dir=1-Dir;
    }*/
		cycle_cnt++;
		if(cycle_cnt>0)
		{
			cycle_cnt=0;
			if(Dir == 1)//10.30
			{
				if(Get_RightWheel_Step()>20000)
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,2200);
					Set_Clean_Mode(Clean_Mode_GoHome);
					return ;
				}
				if(Temp_Rcon_Status&RconL_HomeT)
				{
					Move_Forward(RUN_SPEED_11,RUN_SPEED_6);
					delay(1500);
				}
				else if(Temp_Rcon_Status&RconL_HomeL)
				{
					Move_Forward(RUN_SPEED_12,RUN_SPEED_3);

				}
				/*else if(Temp_Rcon_Status&RconL_LEFT)
				{
					Move_Forward(RUN_SPEED_9,RUN_SPEED_5);
					delay(1000);
				}*/				
				else if(Temp_Rcon_Status&(RconFL_HomeL|RconFR_HomeL))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,600);
					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				}
//				else if(Temp_Rcon_Status&RconFL_HomeT)
//				{
//					Stop_Brifly();
//					Turn_Right(Turn_Speed,700);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				}
//				else if(Temp_Rcon_Status&RconFR_HomeT)
//				{
//					Stop_Brifly();
//					Turn_Right(Turn_Speed,800);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				}
//				else if(Temp_Rcon_Status&RconR_HomeT)
//				{
//					Stop_Brifly();
//					Turn_Right(Turn_Speed,1000);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				}
				else
				{
					Move_Forward(RUN_SPEED_5,RUN_SPEED_10);
				}


				if(Temp_Rcon_Status&(RconFR_HomeR))
				{
					//Stop_Brifly();
					//Turn_Left(Turn_Speed,300);
					By_Path();
					return;
				}
				if(Temp_Rcon_Status&(RconFL_HomeR))
				{
					//Stop_Brifly();
//					Turn_Left(Turn_Speed,300);
					By_Path();
					return;
				}
//				if(Temp_Rcon_Status&(RconBR_HomeR))
//				{
//					Stop_Brifly();
//					Turn_Right(Turn_Speed,2000);
//					Reset_Rcon_Status();
//					Dir=0;
//				}				
				
				if((Temp_Rcon_Status&(RconL_HomeR)))
				{
					Signal_Counter++;
	//				N_Around_LRSignal=0;
					if(Signal_Counter>1)//6
					{
						Signal_Counter=0;
						Stop_Brifly();
						Temp_Position = Check_Position(Round_Left);
						if(Temp_Position==1)
						{
							Reset_Error_Code();
							Reset_Touch();
							Set_Clean_Mode(Clean_Mode_Userinterface);
							return;
						}
						if(Temp_Position==2)
						{
							Move_Forward(1,1);
							By_Path();
							return;
						}
					}
				}
//				else
//				{
//					N_Around_LRSignal++;
//					if(N_Around_LRSignal>4)
//					{
//						if(Signal_Counter>0)Signal_Counter--;
//					}
//				}
			}
			else//30.10
			{
				if(Get_LeftWheel_Step()>20000)
				{
					Stop_Brifly();
					Turn_Left(Turn_Speed,2200);
					Set_Clean_Mode(Clean_Mode_GoHome);
					return ;
				}
				
				if(Temp_Rcon_Status&RconR_HomeT)
				{
					Move_Forward(RUN_SPEED_6,RUN_SPEED_11);
					delay(1500);
				}
				else if(Temp_Rcon_Status&RconR_HomeR)
				{
					Move_Forward(RUN_SPEED_6,RUN_SPEED_9);
					delay(1500);
				}
				/*else if(Temp_Rcon_Status&RconR_RIGHT)
				{
					Move_Forward(RUN_SPEED_6,RUN_SPEED_11);
					delay(1000);
				}*/				
				else if(Temp_Rcon_Status&(RconFR_HomeR|RconFL_HomeR))
				{
					Stop_Brifly();
					Turn_Left(Turn_Speed,400);
					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				}
//				else if(Temp_Rcon_Status&RconFR_HomeT)
//				{
//					Stop_Brifly();
//					Turn_Left(Turn_Speed,800);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				}
//				else if(Temp_Rcon_Status&RconFL_HomeT)
//				{
//					Stop_Brifly();
//					Turn_Left(Turn_Speed,800);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				}
//				else if(Temp_Rcon_Status&RconL_HomeT)
//				{
//					Stop_Brifly();
//					Turn_Left(Turn_Speed,1000);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//					Dir=1;
//				}
				else
				{
					Move_Forward(RUN_SPEED_12,RUN_SPEED_3);
				}


				if(Temp_Rcon_Status&(RconFL_HomeL))
				{
					//Stop_Brifly();
					//Turn_Right(30,500);
					By_Path();
					return;
				}
				if(Temp_Rcon_Status&(RconFR_HomeL))
				{
					//Stop_Brifly();
					//Turn_Right(Turn_Speed,300);
					By_Path();
					return;
				}
//				if(Temp_Rcon_Status&(RconBR_HomeR))
//				{
//					Stop_Brifly();
//					Turn_Left(Turn_Speed,2000);
//					Dir=1;
//				}	
				
				if((Temp_Rcon_Status&RconR_HomeL))
				{
//					N_Around_LRSignal=0;
					Signal_Counter++;
					if(Signal_Counter>5)//5
					{
						Stop_Brifly();
						Signal_Counter=0;
						Temp_Position = Check_Position(Round_Right);
						if(Temp_Position==1)
						{
							Reset_Error_Code();
							Reset_Touch();
							Set_Clean_Mode(Clean_Mode_Userinterface);
							return;
						}
						if(Temp_Position==2)
						{
							Move_Forward(1,1);
							By_Path();
							return;
						}
					}
				}
//			else
//			{
//				N_Around_LRSignal++;
//				if(N_Around_LRSignal>4)
//				{
//					N_Around_LRSignal=0;
//					if(Signal_Counter>0)Signal_Counter--;
//				}
//			}
			}		
		}

			delay(500);
  }
}
/*--------------------------------------------------------------------By Path ----------------*/
void By_Path(void)
{
  uint8_t Cycle=0,Ch_Cy=0,SPEED_MUL=1,SPEED_N=1;
	#ifdef MOBILITY
  uint8_t Mobility_Temp_Error=0;
  uint32_t Temp_Mobility_Distance=0;
	#endif
	uint32_t Receive_Code=0;
  uint32_t Temp_Code =0 ;
  uint8_t Position_Far=1;
  uint16_t NoSignal_Counter=0;
  uint8_t Temp_Check_Position=0;
  uint8_t Near_Counter=0;
  uint8_t Bumper_Counter=0;
	uint8_t Side_Counter=0;
	Reset_Wheel_Step();

	Reset_Touch();
	Display_Content(LED_Home,100,100,0,7);
	
	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  Set_SideBrush_PWM(30,30);
  Set_MainBrush_PWM(30);
	Set_BLDC_Speed(Vac_Speed_NormalL);
	Display_Home_LED();
  while(1)
	{
		/*------------------------------------------------------Check Battery-----------------------*/
		Check_Bat_SetMotors(Home_Vac_Power,Home_SideBrush_Power,Home_MainBrush_Power);//Low Battery Event
		
		Receive_Code=0;
		
		#ifdef MOBILITY
    if(Get_LeftWheel_Step()<500)
    {
      Mobility_Temp_Error=0;
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
				Set_Left_Brush(ENABLE);
				Set_Right_Brush(ENABLE);
        Temp_Mobility_Distance = Get_Move_Distance();
        if(Get_Mobility_Step()<1)
        {
          Mobility_Temp_Error++;
          if(Mobility_Temp_Error>3)
          {
            Set_Clean_Mode(Clean_Mode_GoHome);
            return;
          }
        }
        else
        {
          Mobility_Temp_Error=0;
        }
        Reset_Mobility_Step();
      }
    }
    #endif
		
    Cycle=10;
    while(Cycle--)
    {
		  if(Is_ChargerOn())
			{
        Stop_Brifly();
				Set_SideBrush_PWM(0,0);
				Set_MainBrush_PWM(0);
				delay(2000);
				if(Is_ChargerOn())
				{
					Ch_Cy=50;
					while(Ch_Cy--)
					{
						delay(100);
						if(!Is_ChargerOn())break;
						if(Touch_Detect())break;
					}

					if(Is_ChargerOn())
					{
						Reset_Error_Code();
						Set_Base_C();
						Set_Clean_Mode(Clean_Mode_Charging);
						return;
					}
				}
				else if(Turn_Connect())
				{
					Set_Base_C();
					Set_Clean_Mode(Clean_Mode_Charging);
					return;
				}
				else
				{
          Set_SideBrush_PWM(30,30);
          Set_MainBrush_PWM(0);
				  Back(30,800);
					Set_MainBrush_PWM(30);
          Stop_Brifly();
				}
				Set_SideBrush_PWM(30,30);
        Set_MainBrush_PWM(30);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				break;
			}
			/*----------------------------------------------OBS------------------Event---------------*/

			if(Get_Bumper_Status()&LeftBumperTrig)//waiting for modify
			{
        Reset_Rcon_Status();
        if(!Position_Far)
        {
					Stop_Brifly();
					if(Turn_Connect())
					{
						Set_Clean_Mode(Clean_Mode_Charging);
					  return;
					}
					Set_SideBrush_PWM(30,30);
          Set_MainBrush_PWM(0);
          Back(30,2500);//waiting
					Set_MainBrush_PWM(30);
          Stop_Brifly();
					Stop_Brifly();
          if(Bumper_Counter>1)
          {
            Move_Forward(0,0);
            Set_Clean_Mode(Clean_Mode_GoHome);
            return;
          }
        }
        else if((Get_Rcon_Status()&Rcon_Signal_ALL_L_R)==0)
        {
					Random_Back();
          Turn_Right(Turn_Speed,1100);
          Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
          Set_Clean_Mode(Clean_Mode_GoHome);
          return;
        }
        else
        {
					Random_Back();
          Turn_Right(Turn_Speed,1100);
					Set_SideBrush_PWM(30,30);
          Set_MainBrush_PWM(30);
          Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
        }
				if(Is_Bumper_Jamed())return;
        Bumper_Counter++;
			}
			if(Get_Bumper_Status()&RightBumperTrig)
			{
        Reset_Rcon_Status();
        if(!Position_Far)
        {
					Stop_Brifly();
					if(Turn_Connect())
					{
						Set_Clean_Mode(Clean_Mode_Charging);
					  return;
					}
					Set_SideBrush_PWM(30,30);
          Set_MainBrush_PWM(0);
          Back(30,2500);
					Set_MainBrush_PWM(30);
          Stop_Brifly();
					Stop_Brifly();
          if(Bumper_Counter>1)
          {
            Move_Forward(0,0);
            Set_Clean_Mode(Clean_Mode_GoHome);
            return;
          }
        }
        else if((Get_Rcon_Status()&Rcon_Signal_ALL_L_R)==0)
        {
					Random_Back();
          Turn_Left(Turn_Speed,1100);
          Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
          Set_Clean_Mode(Clean_Mode_GoHome);
          return;
        }
        else
        {
					Random_Back();
          Turn_Left(Turn_Speed,1100);
					Set_SideBrush_PWM(30,30);
          Set_MainBrush_PWM(30);
          Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
        }
				if(Is_Bumper_Jamed())return;
        Bumper_Counter++;
			}
      if(Position_Far)
      {
  			if(Get_Cliff_Trig())
  		  {
  			  Move_Back();
          Move_Back();
  				Turn_Left(Turn_Speed,1750);
  				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
          Set_Clean_Mode(Clean_Mode_GoHome);
          return;
  			}
      }
			else
			{
  			if(Get_Cliff_Trig())
  		  {
					Set_Wheel_Speed(0,0);
					Set_Dir_Backward();
					delay(300);
					if(Get_Cliff_Trig())
					{
						Move_Back();
						Move_Back();
						Turn_Left(Turn_Speed,1750);
						Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
						Set_Clean_Mode(Clean_Mode_GoHome);
						return;
					}
					Set_Dir_Forward();
					break;
  			}
			}

			/*------------------------------------------------------Touch and Remote event-----------------------*/
			if(Touch_Detect())
			{
			  Reset_Touch();
			  Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}
			if(GetBatteryVoltage()<1100)
	    {
			  Display_Battery_Status(Display_Low);
			  delay(10000);
				Set_Clean_Mode(Clean_Mode_Sleep);
			  return;
		  }

			if(Home_Check_Current())return;
      delay(100);
   }

#if 1
    Receive_Code = Get_Rcon_Status();		 
		Temp_Code = Receive_Code;
    Temp_Code&=0x000330ff;
    if(Receive_Code)
    {
      if((Receive_Code&0x00000600)==0x00000600)Position_Far=0;
      if(Receive_Code&0x00000900)Position_Far=0;
      if(Receive_Code&0x00000f00)
      {
        Near_Counter++;
        if(Near_Counter>1)
        {
          Position_Far=0;
        }
				if((Receive_Code&0x000000ff)==0)
				{
					Side_Counter++;
					if(Side_Counter>5)
					{
						Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
						Set_Clean_Mode(Clean_Mode_GoHome);
						return;
					}
				}
				else
				{
					Side_Counter=0;
				}
      }
      if((Receive_Code&0x00000024)==0x00000024)Position_Far=0;
      Reset_Rcon_Status();
      NoSignal_Counter=0;
    }
    else
    {
      Near_Counter=0;
      NoSignal_Counter++;
      if(NoSignal_Counter>50)
      {
        NoSignal_Counter=0;
        Stop_Brifly();
        Temp_Check_Position = Check_Position(Round_Left);
        if(Temp_Check_Position==1)
        {
          Set_Clean_Mode(Clean_Mode_Userinterface);
          return;
        }
        else if(Temp_Check_Position == 0)
        {
          Stop_Brifly();
          Turn_Right(Turn_Speed,1000);
          Stop_Brifly();
          Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
          Set_Clean_Mode(Clean_Mode_GoHome);
          return;
        }
      }
    } 
			
		
    if(Position_Far)
    {
			SPEED_MUL=3/2;
			switch(Temp_Code)
			{
				case 0x0000003c:Move_Forward(14*SPEED_MUL,14*SPEED_MUL);break;//00 11 11 00
				case 0x00000024:Move_Forward(13*SPEED_MUL,13*SPEED_MUL);break;//00 10 01 00
				
				case 0x000000ff:Move_Forward(12*SPEED_MUL,12*SPEED_MUL);break;//11 11 11 11*
				
				case 0x000000fc:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break;//11 11 11 00*
				case 0x0000003f:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break;//00 11 11 11*
				
				case 0x000000fd:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break;//11 11 11 01*
				case 0x000000bf:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break;//10 11 11 11*
				
				case 0x000000f0:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break;//11 11 00 00*
				case 0x0000000f:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break;//00 00 11 11*
					
				case 0x00000034:Move_Forward(10*SPEED_MUL,13*SPEED_MUL);break;//00 11 01 00
				case 0x00000014:Move_Forward(10*SPEED_MUL,15*SPEED_MUL);break;//00 01 01 00
				case 0x00000010:Move_Forward(5*SPEED_MUL,15*SPEED_MUL);break; //00 01 00 00
				
				case 0x00000074:Move_Forward(11*SPEED_MUL,15*SPEED_MUL);break;//01 11 01 00*
				case 0x000000f4:Move_Forward(11*SPEED_MUL,15*SPEED_MUL);break;//11 11 01 00*
				case 0x000000d4:Move_Forward(10*SPEED_MUL,15*SPEED_MUL);break;//11 01 01 00*
				case 0x000000d0:Move_Forward(3*SPEED_MUL,8*SPEED_MUL);break;  //11 01 00 00*

				case 0x0000002c:Move_Forward(13*SPEED_MUL,10*SPEED_MUL);break;//00 10 11 00
				case 0x00000028:Move_Forward(15*SPEED_MUL,10*SPEED_MUL);break;//00 10 10 00
				case 0x00000008:Move_Forward(15*SPEED_MUL,5*SPEED_MUL);break; //00 00 10 00
				
				case 0x0000002e:Move_Forward(15*SPEED_MUL,11*SPEED_MUL);break;//00 10 11 10*
				case 0x0000002f:Move_Forward(15*SPEED_MUL,11*SPEED_MUL);break;//00 10 11 11*
				case 0x0000002b:Move_Forward(15*SPEED_MUL,10*SPEED_MUL);break;//00 10 10 11*
				case 0x0000000b:Move_Forward(8*SPEED_MUL,3*SPEED_MUL);break;  //00 00 10 11*			
				//right
				case 0x00000001:Move_Forward(15*SPEED_MUL,0*SPEED_MUL);break;      //00 00 00 01#
//          case 0x0001:Turn_Right(20,300);       //00 00 00 01
//                      Stop_Brifly();
//                      Move_Forward(30*SPEED_MUL,0*SPEED_MUL);
//                      break;                    //00 00 00 10
				case 0x00000002:Turn_Right(20,500);
										    Stop_Brifly();
										    Move_Forward(30*SPEED_MUL,0*SPEED_MUL);
										    break; 
				case 0x00000003:Move_Forward(15*SPEED_MUL,0*SPEED_MUL);break;       //00 00 00 11

				case 0x00000080:Move_Forward(0*SPEED_MUL,15*SPEED_MUL);break;        //10 00 00 00
//          case 0x0080:Turn_Left(Turn_Speed,300);        //10 00 00 00
//                      Stop_Brifly();
//                      Move_Forward(0*SPEED_MUL,30*SPEED_MUL);
//                      break; 
				case 0x00000040:Turn_Left(Turn_Speed,500);        //01 00 00 00
										    Stop_Brifly();
										    Move_Forward(0*SPEED_MUL,30*SPEED_MUL);
										    break; 
				case 0x000000c0:Move_Forward(0*SPEED_MUL,15*SPEED_MUL);break;        // 11 00 00 00
					
				/*----------------------*/  
				case 0x00000009:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break; //00 00 10 01
				case 0x00000090:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break; //10 01 00 00

				case 0x00000094:Move_Forward(8*SPEED_MUL,13*SPEED_MUL);break; //10 01 01 00
				case 0x00000029:Move_Forward(13*SPEED_MUL,8*SPEED_MUL);break; //00 01 10 01

				case 0x0000000d:Move_Forward(14*SPEED_MUL,10*SPEED_MUL);break; //00 00 11 01
				case 0x000000b0:Move_Forward(10*SPEED_MUL,14*SPEED_MUL);break; //10 11 00 00

				case 0x0000000A:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break; //00 00 10 10
				case 0x00000050:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break; //01 01 00 00

				case 0x0000002A:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break; //00 10 10 10
				case 0x00000054:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break; //01 01 01 00

				case 0x0000002D:Move_Forward(12*SPEED_MUL,9*SPEED_MUL);break; //00 10 11 01
				case 0x000000B4:Move_Forward(9*SPEED_MUL,12*SPEED_MUL);break; //10 11 01 00

				case 0x00000041:Move_Forward(15*SPEED_MUL,9*SPEED_MUL);break; //01 00 00 01
				case 0x00000082:Move_Forward(9*SPEED_MUL,15*SPEED_MUL);break; //10 00 00 10

				case 0x0000000c:Move_Forward(12*SPEED_MUL,8*SPEED_MUL);break; //00 00 11 00
				case 0x00000030:Move_Forward(8*SPEED_MUL,12*SPEED_MUL);break; //00 11 00 00

				case 0x00000005:Move_Forward(15*SPEED_MUL,4*SPEED_MUL);break; //00 00 01 01
				case 0x000000a0:Move_Forward(4*SPEED_MUL,15*SPEED_MUL);break; //10 10 00 00

				case 0x00000004:Move_Forward(8*SPEED_MUL,15*SPEED_MUL);break; //00 00 01 00*
				case 0x00000020:Move_Forward(15*SPEED_MUL,8*SPEED_MUL);break; //00 10 00 00*

				case 0x00000095:Move_Forward(11*SPEED_MUL,13*SPEED_MUL);break;//10 01 01 01
				case 0x000000A9:Move_Forward(13*SPEED_MUL,11*SPEED_MUL);break;//10 10 10 01

				case 0x000000B5:Move_Forward(12*SPEED_MUL,14*SPEED_MUL);break;//10 11 01 01
				case 0x000000AD:Move_Forward(14*SPEED_MUL,12*SPEED_MUL);break;//10 10 11 01

				case 0x0000001c:Move_Forward(10*SPEED_MUL,8*SPEED_MUL);break; //00 01 11 00
				case 0x00000038:Move_Forward(8*SPEED_MUL,10*SPEED_MUL);break; //00 11 10 00
				
				case 0x00001000: Turn_Right(Turn_Speed,1100);         //00 00 00 00 00 10
									    	 Stop_Brifly();
										     Move_Forward(12*SPEED_MUL,12*SPEED_MUL);
										     break;
				case 0x00020000:Turn_Left(Turn_Speed,1100);         //01 00 00 00 00 00
									    	Stop_Brifly();
										    Move_Forward(12*SPEED_MUL,12*SPEED_MUL);
										    break;
				
				case 0x00002000:Move_Forward(10*SPEED_MUL,0*SPEED_MUL);break; //00 00 00 00 00 01
				case 0x00010000:Move_Forward(0*SPEED_MUL,10*SPEED_MUL);break; //10 00 00 00 00 00
				

				default: Move_Forward(12*SPEED_MUL,15*SPEED_MUL);         
								 break;
	     }
			
     }
     else
     {    
			  SPEED_MUL=1;//3/2
			  SPEED_N=1;
        Temp_Code&=0x000000ff;			 
				switch(Temp_Code)
				{
					case 0x0000003c:Move_Forward(10*SPEED_MUL/SPEED_N,10*SPEED_MUL/SPEED_N);break;// 00 11 11 00
          case 0x00000024:Move_Forward(9*SPEED_MUL/SPEED_N,9*SPEED_MUL/SPEED_N);break;// 00 10 01 00
					
					case 0x000000ff:Move_Forward(8*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break;//11 11 11 11*
					
					case 0x000000fc:Move_Forward(3*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;//11 11 11 00*
					case 0x0000003f:Move_Forward(7*SPEED_MUL/SPEED_N,3*SPEED_MUL/SPEED_N);break;//00 11 11 11*
					
					case 0x000000fd:Move_Forward(5*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;//11 11 11 01*
					case 0x000000bf:Move_Forward(7*SPEED_MUL/SPEED_N,5*SPEED_MUL/SPEED_N);break;//10 11 11 11*
					
					case 0x000000f0:Move_Forward(3*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;//11 11 00 00*
					case 0x0000000f:Move_Forward(7*SPEED_MUL/SPEED_N,3*SPEED_MUL/SPEED_N);break;//00 00 11 11*			
          	
					case 0x00000034:Move_Forward(5*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;// 00 11 01 00   4-7
          case 0x00000014:Move_Forward(2*SPEED_MUL/SPEED_N,6*SPEED_MUL/SPEED_N);break;// 00 01 01 00
          case 0x00000010:Move_Forward(2*SPEED_MUL/SPEED_N,6*SPEED_MUL/SPEED_N);break;// 00 01 00 00
					
					
					case 0x00000074:Move_Forward(4*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;//01 11 01 00*
					case 0x000000f4:Move_Forward(4*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;//11 11 01 00*
					case 0x000000d4:Move_Forward(6*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break;//11 01 01 00*
					case 0x000000d0:Move_Forward(3*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break; //11 01 00 00*

          case 0x0000002c:Move_Forward(7*SPEED_MUL/SPEED_N,5*SPEED_MUL/SPEED_N);break; // 00 10 11 00    
          case 0x00000028:Move_Forward(6*SPEED_MUL/SPEED_N,2*SPEED_MUL/SPEED_N);break; // 00 10 10 00
          case 0x00000008:Move_Forward(6*SPEED_MUL/SPEED_N,2*SPEED_MUL/SPEED_N);break;// 00 00 10 00
					
					case 0x0000002e:Move_Forward(7*SPEED_MUL/SPEED_N,5*SPEED_MUL/SPEED_N);break;//00 10 11 10*
					case 0x0000002f:Move_Forward(7*SPEED_MUL/SPEED_N,4*SPEED_MUL/SPEED_N);break;//00 10 11 11*
					case 0x0000002b:Move_Forward(8*SPEED_MUL/SPEED_N,6*SPEED_MUL/SPEED_N);break;//00 10 10 11*
					case 0x0000000b:Move_Forward(8*SPEED_MUL/SPEED_N,3*SPEED_MUL/SPEED_N);break; //00 00 10 11*
          //right
					
					case 0x00000001:Move_Forward(12*SPEED_MUL/SPEED_N,0*SPEED_MUL/SPEED_N);break; // 00 00 00 01#
//          case 0x0001:Turn_Right(20,300);      // 00 00 00 01
//                      Stop_Brifly();
//                      Move_Forward(14*SPEED_MUL,0*SPEED_MUL);
//                      break;
          case 0x00000002:Turn_Right(20,500);      // 00 00 00 10
                          Stop_Brifly();
                          Move_Forward(14*SPEED_MUL/SPEED_N,0*SPEED_MUL/SPEED_N);
                          break;
          case 0x00000003:Turn_Right(20,400);      // 00 00 00 11
                          Stop_Brifly();
                          Move_Forward(14*SPEED_MUL/SPEED_N,0*SPEED_MUL/SPEED_N);
                          break;
				  case 0x00000080:Move_Forward(0*SPEED_MUL/SPEED_N,12*SPEED_MUL/SPEED_N);break;   // 10 00 00 00#
//          case 0x0080:Turn_Left(Turn_Speed,300);        // 10 00 00 00
//                      Stop_Brifly();
//                      Move_Forward(0*SPEED_MUL,14*SPEED_MUL);
//                      break;
          case 0x00000040:Turn_Left(Turn_Speed,500);        // 01 00 00 00
                          Stop_Brifly();
                          Move_Forward(0*SPEED_MUL/SPEED_N,14*SPEED_MUL/SPEED_N);
                          break;
          case 0x000000c0:Turn_Left(Turn_Speed,400);        // 11 00 00 00
                          Stop_Brifly();
                          Move_Forward(0*SPEED_MUL/SPEED_N,14*SPEED_MUL/SPEED_N);
                          break;
          /*----------------------*/  
          case 0x00000009:Move_Forward(10*SPEED_MUL/SPEED_N,3*SPEED_MUL/SPEED_N);break;// 00 00 10 01
          case 0x00000090:Move_Forward(3*SPEED_MUL/SPEED_N,10*SPEED_MUL/SPEED_N);break;// 10 01 00 00

          case 0x00000094:Move_Forward(4*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break;// 10 01 01 00
          case 0x00000029:Move_Forward(8*SPEED_MUL/SPEED_N,4*SPEED_MUL/SPEED_N);break;// 00 10 10 01

          case 0x0000000d:Move_Forward(12*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break; // 00 00 11 01
          case 0x000000b0:Move_Forward(7*SPEED_MUL/SPEED_N,12*SPEED_MUL/SPEED_N);break; // 10 11 00 00

          case 0x0000000A:Move_Forward(12*SPEED_MUL/SPEED_N,5*SPEED_MUL/SPEED_N);break; // 00 00 10 10
          case 0x00000050:Move_Forward(5*SPEED_MUL/SPEED_N,12*SPEED_MUL/SPEED_N);break; // 01 01 00 00

          case 0x0000002A:Move_Forward(8*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break; // 00 10 10 10
          case 0x00000054:Move_Forward(7*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break; // 01 01 01 00

          case 0x0000002D:Move_Forward(9*SPEED_MUL/SPEED_N,5*SPEED_MUL/SPEED_N);break;  // 00 10 11 01
          case 0x000000B4:Move_Forward(5*SPEED_MUL/SPEED_N,9*SPEED_MUL/SPEED_N);break;  // 10 11 01 00

          case 0x00000041:Move_Forward(9*SPEED_MUL/SPEED_N,5*SPEED_MUL/SPEED_N);break; // 01 00 00 01
          case 0x00000082:Move_Forward(5*SPEED_MUL/SPEED_N,9*SPEED_MUL/SPEED_N);break; // 10 00 00 10

          case 0x0000000c:Move_Forward(9*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break; // 00 00 11 00
          case 0x00000030:Move_Forward(6*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break;// 00 11 00 00

          case 0x00000005:Move_Forward(11*SPEED_MUL/SPEED_N,6*SPEED_MUL/SPEED_N);break;// 00 00 01 01
          case 0x000000a0:Move_Forward(6*SPEED_MUL/SPEED_N,11*SPEED_MUL/SPEED_N);break; // 10 10 00 00

          case 0x00000004:Move_Forward(7*SPEED_MUL/SPEED_N,9*SPEED_MUL/SPEED_N);break; // 00 00 01 00*
          case 0x00000020:Move_Forward(8*SPEED_MUL/SPEED_N,6*SPEED_MUL/SPEED_N);break; // 00 10 00 00*

          case 0x00000095:Move_Forward(7*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break; // 10 01 01 01
          case 0x000000A9:Move_Forward(8*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break; // 10 10 10 01

          case 0x000000B5:Move_Forward(7*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break; // 10 11 01 01
          case 0x000000AD:Move_Forward(8*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break; // 10 10 11 01

          case 0x0000001c:Move_Forward(7*SPEED_MUL/SPEED_N,8*SPEED_MUL/SPEED_N);break;// 00 01 11 00*
          case 0x00000038:Move_Forward(8*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);break;// 00 11 10 00*

					default: Move_Forward(7*SPEED_MUL/SPEED_N,7*SPEED_MUL/SPEED_N);; 
                  break;
		    }
						
    } 
#endif
		
  delay(500);                    
	}
}

/*------------------------------------------------*/
uint8_t Home_Check_Current(void)
{
	uint8_t Motor_Check_Code=Check_Motor_Current();
	if(Motor_Check_Code)
	{
		if(Self_Check(Motor_Check_Code))
		{
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
//		else
//		{
		  Home_Motor_Set();
			Set_Clean_Mode(Clean_Mode_GoHome);
//			Initialize_Motor();
			return 1;
//		}	    
  }
	return 0;
}
/*-------------------Turn OFF the Vaccum-----------------------------*/
void Home_Motor_Set(void)
{
  Set_Vacuum_PWM(0);
	Set_MainBrush_PWM(20); 
  Set_SideBrush_PWM(20,20);
	Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
	Reset_WheelSLow();
	Reset_Bumper_Error();

}

/*------------------------------------------------*/
uint8_t Is_InfrontOfStation(void)
{
  uint16_t Receive_Code = 0;
  Reset_Rcon_Status();
  Stop_Brifly();
  Stop_Brifly();
  Receive_Code = (Get_Rcon_Status()&Rcon_Signal_ALL_L_R);
  switch(Receive_Code)
  {
    case 0x003c:return 1;
		case 0x0034:return 1;
    case 0x0014:return 1;
    case 0x002c:return 1;;
    case 0x0028:return 1;;
    default:return 0;
  }
}
/*------------------------------------------------*/
uint8_t Check_Position(uint8_t Dir)
{
  uint32_t Counter_Watcher=0;
  uint16_t Receive_Code = 0;

  if(Dir == Round_Left)
  {
    Set_Dir_Left();
  }
  else if(Dir == Round_Right)
  {
    Set_Dir_Right();
  }
  Reset_LeftWheel_Step();
  Reset_TempPWM();   
	Set_Wheel_Speed(RUN_SPEED_5,RUN_SPEED_5);
	Counter_Watcher=0;
  Reset_Touch();
  while(Get_LeftWheel_Step()<3600)
	{
	  delay(1);
		Counter_Watcher++;
	  if(Counter_Watcher>150000)
		{
			if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return 1;
		}
    Receive_Code = (Get_Rcon_Status()&Rcon_Signal_ALL_L_R);
    if(Receive_Code)Reset_Rcon_Status();
    if(Dir == Round_Left)
    {
      if(Receive_Code & (RconFR_HomeL|RconFR_HomeR))//0x0c
      {
				Stop_Brifly();
        return 2;
      }
    }
    if(Dir == Round_Right)
    {
      if(Receive_Code & (RconFL_HomeL|RconFL_HomeR))//0x30
      {
				Stop_Brifly();
        return 2;
      }
    }
    //if(Is_Remote())return 1;
		if(Touch_Detect())
		{
			return 1;
		}
		//if((Check_Motor_Current()==Check_Left_Wheel)||(Check_Motor_Current()==Check_Right_Wheel))return 1;
	}
  Reset_TempPWM();
  return 0;
}
/*------------------------------------------------*/

uint8_t Turn_Connect(void)
{
	Wheel_Configure(ENABLE,ENABLE);
	Set_Dir_Right();
	Set_Wheel_Speed(RUN_SPEED_4,RUN_SPEED_4);
	Reset_Wheel_Step();
	while(Get_LeftWheel_Step()<100)
	{
		if(Is_ChargerOn())
		{
			Disable_Motors();
			Stop_Brifly();
			if(Is_ChargerOn())
			{
				return 1;
			}
			break;
		}
		if(Touch_Detect())
		{
			Disable_Motors();
			return 0;
		}
	}
	Stop_Brifly();
	Wheel_Configure(ENABLE,ENABLE);
	Set_Dir_Left();
	Set_Wheel_Speed(RUN_SPEED_4,RUN_SPEED_4);
	Reset_Wheel_Step();
	while(Get_LeftWheel_Step()<200)
	{
		if(Is_ChargerOn())
		{
			Disable_Motors();
			Stop_Brifly();
			if(Is_ChargerOn())
			{
				return 1;
			}
			return 0;
		}
		if(Touch_Detect())
		{
			Disable_Motors();
			return 0;
		}
	}
	Disable_Motors();
	return 0;
}

void SetHomeRemote(void)
{
	R_H_Flag=1;
}
void ResetHomeRemote(void)
{
	R_H_Flag=0;
}
uint8_t IsHomeRemote(void)
{
	return R_H_Flag;
}
void Display_Home_LED(void)
{
	if(IsHomeRemote())
	{
		Set_LED_On_Switch(1,0,0,1,0,0);//Set_LED(100,100,0);
	}
	else
	{
		Set_LED_On_Switch(1,0,0,1,0,0);//Set_LED(100,0,100);
	}
}
/*----------------------------------------Wake UP ------------------------------------*/
void Wake_Up_Adjust(void)
{
	uint32_t Rcon_Buffer=0;
	Reset_Rcon_Status();
	delay(3000);
	Rcon_Buffer=Get_Rcon_Status();
	
	if(Rcon_Buffer&(RconFL_HomeL|RconFL_HomeR|RconFL_HomeT
		             |RconFR_HomeL|RconFR_HomeR|RconFR_HomeT
	               |RconL_HomeT|RconL_HomeL|RconL_HomeR
	               |RconR_HomeT|RconR_HomeL|RconR_HomeR))
	{
		Set_MainBrush_PWM(0);
		Back(20,500);
	}

	Turn_BackOnBase();
	if(Forward_Distance(20,1500))
	{
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return;
	}
	Stop_Brifly();
	Turn_Left(Turn_Speed,1900);
	OBS_OFF();
	GoHome();
	OBS_ON();
	return;
}

void Turn_BackOnBase(void)
{
	Reset_Touch();
	Set_Wheel_Speed(RUN_SPEED_5,RUN_SPEED_5);
	Set_SideBrush_PWM(30,30);
	Set_Left_Brush(ENABLE);
	Set_Right_Brush(ENABLE);
	if(Get_Rcon_Status()&(RconFL_HomeT|RconL_HomeT|RconBL_HomeT))
	{
		Reset_Rcon_Status();
		Set_Dir_Left();
		while(Get_LeftWheel_Step()<3600)
		{
			if(Touch_Detect())
			{
				return;
			}
			if(Get_Rcon_Status()&RconFR_HomeT)
			{
				Turn_Left(Turn_Speed,1800);
				Stop_Brifly();
				return;
			}
		}
	}
	Reset_Rcon_Status();
	Set_Dir_Right();
	while(Get_LeftWheel_Step()<3600)
	{
		if(Touch_Detect())
		{
			return;
		}
		if(Get_Rcon_Status()&RconFL_HomeT)
		{
			Turn_Right(20,1800);
			Stop_Brifly();
			return;
		}
	}
	Stop_Brifly();
	return;
}

uint8_t Forward_Distance(uint8_t Speed,uint32_t Dis)
{
	uint8_t motor_check=0;
	Move_Forward(Speed,Speed);
	Reset_Wheel_Step();
	while(Get_LeftWheel_Step()<Dis)
	{
		if(Is_Turn_Remote())return 1;
    if(Get_Cliff_Trig())return 1;
		if(Touch_Detect())
		{
			return 1;
		}
		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return 1;
		}
		if(Get_Bumper_Status())return 0;
		if(Get_OBS_Status())
		{
			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
		}
		else
		{
			Move_Forward(Speed,Speed);
		}
		delay(10);
	}
	return 0;
}




