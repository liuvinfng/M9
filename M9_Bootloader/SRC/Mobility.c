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
#include "Mobility.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Rcon.h"
#include "IR.h"
#include "TestMode.h"
#include "Flash.h"
#include "PathPlanning.h"
#include "Map.h"
#include "Gyro.h"
#include "WallFollowMulti.h"
#include "CorMove.h"
#include "WIFI_EMW_3081.h"

extern volatile int32_t WallFollowRightWheelStep,WallFollowLeftWheelStep;

extern volatile Cliff_ADC Cliff_Status;
extern volatile OBS_ADC OBS_Status;
extern volatile ADC_Value_Struct ADC_Value;

extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;

extern volatile ADC_Value_Struct ADC_Value;
/* --------------------------------------------------Random Runnincg mode----------------------*/
void Mobility_Mode(void)
{
	uint8_t Temp_Cliff=0;
  uint32_t Test_Status=0;
 
	volatile int32_t Wall_Distance = 800;
	volatile int32_t Move_Distance = 100;
	int32_t Temp_Counter=0;
	int32_t Proportion=0;
	int32_t Previous=0;
	int32_t Delta=0;
	int32_t Wheel_Dif=0;
	uint32_t Rcon_Delay=0;
  uint32_t Wall_BH_Counter=0;
  uint32_t Motor_Check_Code=0;
	int32_t BaseCurrent=0;
	uint32_t Time_Counter=0;
	int32_t Current_Summary=0;
	uint32_t Current_SUC=0;
	int32_t Left_Wall_Speed=0,Right_Wall_Speed=0;
 
 
	Display_Clean_Status(Display_Test); 
	Beep(1);
	Beep(2);
	Beep(3);

	Display_Content(0,88,88,0,7);

  Sound(1);
  /*-------------------------------------------------Get Base Current------------------------------------------------*/
	Time_Counter=10;
	BaseCurrent=0;
  while(Time_Counter--)
	{
			BaseCurrent+=Get_Current_Current();
      delay(20);
	}
	BaseCurrent/=10;

  Reset_MoveWithRemote();
	Reset_Bumper_Error();
	Reset_Touch();
	Reset_Wheel_Step();
	Reset_Move_Distance();
  Set_MainBrush_PWM(50);
  Set_SideBrush_PWM(60,60);
  Set_Vacuum_PWM(40);
	#ifdef BLDC_INSTALL
	BLDC_OFF;
  delay(5000);
	Set_VacMode(Vac_Max);
	Set_BLDC_TPWM(60);
	Set_Vac_Speed();
	#endif
	
	Reset_Rcon_Remote();
  Reset_Rcon_Status();

  Stop_Brifly();
  Rcon_Delay=0;
  Reset_Rcon_Status();
	Set_Dir_Right();
	Set_Wheel_Speed(20,20);
	Work_Motor_Configure();
	Reset_Wheel_Step();
	Left_Wheel_Step=0;
	Temp_Counter=1;
 
	
  while(1)
	{
	  if(Left_Wheel_Step>4500)
		{
			Disable_Motors();
			Display_Content(LED_Exclamation,100,ME_CS,0,1);// 
			if(Get_Rcon_Status()&RconFL_HomeT){Test_Status|=Test_Status_FLRCON;USART_Print("\n\rFL! ",Dev_USART3);}else{IR_Transmite_Cycling(0,ME_FLRcon);}
			if(Get_Rcon_Status()&RconFR_HomeT){Test_Status|=Test_Status_FRRCON;USART_Print("\n\rFR! ",Dev_USART3);}else{IR_Transmite_Cycling(0,ME_FRRcon);}
			if(Get_Rcon_Status()&RconR_HomeT){Test_Status|=Test_Status_RRCON;USART_Print("\n\rR! ",Dev_USART3);}else{IR_Transmite_Cycling(0,ME_RRcon);}
			if(Get_Rcon_Status()&RconL_HomeT){Test_Status|=Test_Status_LRCON;USART_Print("\n\rL! ",Dev_USART3);}else{IR_Transmite_Cycling(0,ME_LRcon);}
			if(Get_Rcon_Status()&RconBR_HomeT){Test_Status|=Test_Status_BRRCON;USART_Print("\n\rBR! ",Dev_USART3);}else{IR_Transmite_Cycling(0,ME_BRRcon);}
			if(Get_Rcon_Status()&RconBL_HomeT){Test_Status|=Test_Status_BLRCON;USART_Print("\n\rBL! ",Dev_USART3);}else{IR_Transmite_Cycling(0,ME_BLRcon);}
			//IR_Transmite_Cycling(0,ME_CS);
		}
    if(Get_Rcon_Status()&RconFR_HomeT)Test_Status|=Test_Status_FRRCON;
		if(Get_Rcon_Status()&RconFL_HomeT)Test_Status|=Test_Status_FLRCON;
		if(Get_Rcon_Status()&RconL_HomeT)Test_Status|=Test_Status_LRCON;
		/*if(Get_Rcon_Status()&RconBL_HomeT)Test_Status|=Test_Status_BLRCON;*/
		if(Get_Rcon_Status()&RconBR_HomeT)Test_Status|=Test_Status_BRRCON;
		if(Get_Rcon_Status()&RconR_HomeT)Test_Status|=Test_Status_RRCON;
		
		
		if(Temp_Counter)
		{
			if(Left_Wheel_Step>290)
			{
				Temp_Counter=0;	
				Set_Wheel_Speed(20,20);
				Reset_Rcon_Status();
			}			
		}
		if(Left_Wheel_Step>3600)
		{
			if(Get_Rcon_Status()&RconBR_HomeT)//finish
			{
				if((Test_Status&0x37000)==0x37000)
				{
					Wheel_Dif=Left_Wheel_Step-Right_Wheel_Step;
					if((Wheel_Dif>100)||(Wheel_Dif<-100))
					{
						Disable_Motors();
						Display_Content(LED_Exclamation,100,ME_Encoder,0,1);// 
						IR_Transmite_Cycling(0,ME_Encoder);
					}
					USART_Print("\n\rRCON! ",Dev_USART3);
					break;
				}
			}
		}
	}
	
	Sound(2);
	Reset_Rcon_Status();
	Test_Status=0;
	//------------------------OBS LOW DATA------------------
	if(OBS_Status.Right_OBS >800)Test_Status|=Test_Status_Right_OBS;
	if(OBS_Status.Front_OBS >800)Test_Status|=Test_Status_Front_OBS;
	if(OBS_Status.Left_OBS >800)Test_Status|=Test_Status_Left_OBS;
	if((Test_Status&Test_Status_Right_OBS))
	{
		Disable_Motors();
		 
 
		while(1)
		{
			IR_Transmite_Error(0,ME_RROBS,OBS_Status.Right_OBS);
			delay(500);
		}
	}
	if((Test_Status&Test_Status_Front_OBS))
	{
		Disable_Motors();		 
 
		while(1)
		{
			IR_Transmite_Error(0,ME_FFOBS,OBS_Status.Front_OBS);
			delay(500);
		}
	}
	if((Test_Status&Test_Status_Left_OBS))
	{
		Disable_Motors();
 
		while(1)
		{
			IR_Transmite_Error(0,ME_LLOBS,OBS_Status.Left_OBS);
			delay(500);
		}		 
	}	
	
	/*---------------------------------------------------Move Around base-------------------------------------*/
	while(1)
	{
		Move_Forward(30,30);
		if(Get_Rcon_Status()&RconFR_HomeT)
		{
			Stop_Brifly();
			Turn_Right(Turn_Speed,920);
			Reset_Rcon_Status();
		}
		if(Get_Rcon_Status()&RconFL_HomeT)
		{
			Stop_Brifly();
			Turn_Right(Turn_Speed,800);
			Reset_Rcon_Status();
		}
		if(Get_Rcon_Status()&RconL_HomeT)
		{
			Stop_Brifly();
			Turn_Right(Turn_Speed,320);
			Reset_Rcon_Status();
		}
		if(Get_Bumper_Status())
		{
      break;
		}
		delay(100);
	}
  Stop_Brifly();
	Set_SideBrush_PWM(60,60);
	Set_MainBrush_PWM(50);

  /*-----------------------------------------------------Test OBS---------------------------------------*/
  while(1)
  {
	  Move_Forward(30,30);
		if(Get_Bumper_Status()&LeftBumperTrig)
    {
		  Test_Status|=Test_Status_Left_Bumper;
		  Move_Back();
			Set_Dir_Left();
			Set_Wheel_Speed(20,20);
		  Right_Wheel_Step=0;
		  while(Right_Wheel_Step<1080)
			{
				if(OBS_Status.Right_OBS >1000)Test_Status|=Test_Status_Right_OBS;
				if(OBS_Status.Front_OBS >1000)Test_Status|=Test_Status_Front_OBS;
				if(OBS_Status.Left_OBS >1000)Test_Status|=Test_Status_Left_OBS;	
			}
			if(!(Test_Status&Test_Status_Right_OBS))
			{
			  Disable_Motors();
				Display_Content(LED_Exclamation,100,ME_ROBS,0,1);//r obs
				IR_Transmite_Cycling(0,ME_ROBS);
			}
			if(!(Test_Status&Test_Status_Front_OBS))
			{
			  Disable_Motors();
				Display_Content(LED_Exclamation,100,ME_FOBS,0,1);//f obs
				IR_Transmite_Cycling(0,ME_FOBS);
			}
			if(!(Test_Status&Test_Status_Left_OBS))
			{
			  Disable_Motors();
				Display_Content(LED_Exclamation,100,ME_LOBS,0,1);//l obs
				IR_Transmite_Cycling(0,ME_LOBS);
			}
		}
    Move_Forward(20,20);
		if(Get_Bumper_Status()&RightBumperTrig)
    {
		  if(Test_Status&Test_Status_Left_Bumper)
			{
				Test_Status|=Test_Status_Right_Bumper ;
				Move_Back();
				
				break;
			}
			else
			{
			  Disable_Motors();
        Display_Content(LED_Exclamation,100,ME_RBumper,0,1);//right bumper
			  IR_Transmite_Cycling(0,ME_RBumper);
			}
		}
	}
	/*Reset_TempPWM();
  Turn_Left(Turn_Speed,300);
  Stop_Brifly();
	Move_Forward(10,10);
  Wall_BH_Counter=0;
  Reset_Mobility_Step();
	Clear_BLDC_Fail();
	Reset_Wall_Step();
	//-------------------------------------------------------------------------------------------------------Wall follow---------------------------------------//
	while(1)
	{
//		OBS_Dynamic_Base(50);
		
		if(Is_BLDC_Fail())
		{
			IR_Transmite_Cycling(0,ME_FStall);
		}
		if(Is_Vacuum_Stall())
		{
			IR_Transmite_Cycling(0,ME_FStall);
		}
		
    Motor_Check_Code=Check_Motor_Current();
//		USART_Print("\n\r RightWheel ",Dev_USART3);USART_Print("    ",Dev_USART3);
//		USART3_Transmit_Numbers(ADC_Value.Right_Wheel_Current);
//		USART_Print("\n\r LeftWheel ",Dev_USART3);USART_Print("    ",Dev_USART3);
//		USART3_Transmit_Numbers(ADC_Value.Left_Wheel_Current);
    if(Motor_Check_Code)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,1,Motor_Check_Code,0,1);//motor stall 
			if(Motor_Check_Code&Check_Left_Wheel)IR_Transmite_Cycling(0,ME_LWStall);
			if(Motor_Check_Code&Check_Right_Wheel)IR_Transmite_Cycling(0,ME_RWStall);
			if(Motor_Check_Code&Check_Left_Brush)IR_Transmite_Cycling(0,ME_LBStall);
			if(Motor_Check_Code&Check_Right_Brush)IR_Transmite_Cycling(0,ME_RBStall);
			if(Motor_Check_Code&Check_Main_Brush)IR_Transmite_Cycling(0,ME_MBStall);
			if(Motor_Check_Code&Check_Vacuum)IR_Transmite_Cycling(0,ME_FStall);
    }

//	  Temp_Cliff = Get_Cliff_Trig();
//	  if(Temp_Cliff)
//		{
//      Set_Wheel_Speed(0,0);
//      Reset_TempPWM();
//			//--------------------------------------------------------------Cliff------------------------------//
//			if(Get_RightWall_Step()<6000)//about 1 Meter    6000 = 1 m
//			{
//				Disable_Motors();
//				if(Temp_Cliff&Status_Cliff_Left)IR_Transmite_Cycling(0,ME_LCliff);
//				if(Temp_Cliff&Status_Cliff_Front)IR_Transmite_Cycling(0,ME_FCliff);
//				if(Temp_Cliff&Status_Cliff_Right)IR_Transmite_Cycling(0,ME_RCliff);
//			}
//		  break;
//		}
	 //---------------------------------------------------Bumper Event-----------------------//
    if(Get_Bumper_Status()==LeftBumperTrig)
    {
      Wall_BH_Counter++;
		  if(Get_WallAccelerate()>100)
			{
        Stop_Brifly();
				Wall_Move_Back();
				if(Is_Bumper_Jamed())
        {
          Disable_Motors();
          Display_Content(LED_Exclamation,100,ME_LBumper,0,1);// Bumper jamed
    			IR_Transmite_Cycling(0,ME_LBumper);
        }
			}
	    Turn_Left(Turn_Speed,720);
	    Move_Forward(15,15);
	  	Reset_WallAccelerate();
			Move_Distance=375;
      Set_Wheel_Step(0,0);
			Reset_TempPWM();
    }

		if(Get_Bumper_Status()&RightBumperTrig)
    {
      Stop_Brifly();       
			Reset_TempPWM();
			
			if(Get_RightWall_Step()<1800)//about 0.3 Meter    2000 = 0.3 m
			{
				Disable_Motors();USART3_Transmit_Numbers(Get_RightWall_Step());
        Display_Content(LED_Exclamation,100,ME_RBumper,0,1);//right bumper
			  IR_Transmite_Cycling(0,ME_RBumper);
			}
			Turn_Right(Turn_Speed,1600);
			break;
    }
    if(Wall_BH_Counter>4)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_RWSensor,0,1);//wall sensor
      IR_Transmite_Cycling(0,ME_RWSensor);
    }

    //------------------------------------------------------Short Distance Move-----------------------//
		if (Get_WallAccelerate() < (uint32_t) Move_Distance)
		{
      if(Get_RightWheel_Step()<500)
      {
        if(Get_WallAccelerate()<100)
  			{
  			  Move_Forward(20,20);
  			}
  			else
  			{
  			  Move_Forward(25,25);
  			}
      }
      else
      {
        Move_Forward(30,30);
      }
		}
		else
		{
			//------------------------------------------------------Wheel Speed adjustment-----------------------//
	    //if(OBS_Status.Front_OBS <1500)
			if((Cormove_Get_OBSStatus()&Status_Front_OBS)==0)				
	    {
				
          Proportion = Get_RWall_ADC();
					
					
					Proportion = Proportion*100/Wall_Distance;
					
					Proportion-=100;
				
				  Delta = Proportion - Previous;
					
					if(Wall_Distance>400)//over left
					{
						Right_Wall_Speed = 25 + Proportion/12 + Delta/5;
						Left_Wall_Speed = 25 - Proportion/10 - Delta/5;
						if(Left_Wall_Speed>33)
						{
							Right_Wall_Speed=7;
							Left_Wall_Speed=40;
						}
					}
					else 
					{
						Right_Wall_Speed = 22 + Proportion/16 + Delta/10;
						Left_Wall_Speed = 22 - Proportion/10 - Delta/8;
						if(Left_Wall_Speed>28)
						{
							Right_Wall_Speed=6;
							Left_Wall_Speed=30;
						}
					}
          Current_Summary += ADC_Value.System_Current;
					Current_SUC++;
					
					Previous = Proportion;
					
					if(Right_Wall_Speed<0)Right_Wall_Speed=0;
					if(Right_Wall_Speed>40)Right_Wall_Speed=40;
					if(Left_Wall_Speed<0)Left_Wall_Speed=0;
					
					Move_Forward(Left_Wall_Speed,Right_Wall_Speed);
	    }
	    else
	    {
        Stop_Brifly();       
			  Reset_TempPWM();
				
				if(Get_RightWall_Step()<1800)//about 0.5 Meter    2000 = 0.3 m
				{
					Disable_Motors();
//					USART3_Transmit_Numbers(Get_RightWall_Step());
					Display_Content(LED_Exclamation,100,ME_RWSensor,0,1);//right bumper
					IR_Transmite_Cycling(0,ME_RWSensor);
				}
				Turn_Right(Turn_Speed,1600);
				break;
	    }
		}
	}*/

  Reset_TempPWM();  
  Stop_Brifly();
	Move_Forward(10,10);
  Wall_BH_Counter=0;
  Reset_Mobility_Step();
	Clear_BLDC_Fail();
	Reset_Wall_Step();
	/*-------------------------------------------------------------------------------------------------------Wall follow---------------------------------------*/
	while(1)
	{
//		OBS_Dynamic_Base(50);
		
		
		if(Is_BLDC_Fail())
		{
			IR_Transmite_Cycling(0,ME_FStall);
		}
		if(Is_Vacuum_Stall())
		{
			IR_Transmite_Cycling(0,ME_FStall);
		}
    Motor_Check_Code=Check_Motor_Current();
//		USART_Print("\n\r RightWheel ",Dev_USART3);USART_Print("    ",Dev_USART3);
//		USART3_Transmit_Numbers(ADC_Value.Right_Wheel_Current);
//		USART_Print("\n\r LeftWheel ",Dev_USART3);USART_Print("    ",Dev_USART3);
//		USART3_Transmit_Numbers(ADC_Value.Left_Wheel_Current);
    if(Motor_Check_Code)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,1,Motor_Check_Code,0,1);//motor stall 
			if(Motor_Check_Code&Check_Left_Wheel)IR_Transmite_Cycling(0,ME_LWStall);
			if(Motor_Check_Code&Check_Right_Wheel)IR_Transmite_Cycling(0,ME_RWStall);
			if(Motor_Check_Code&Check_Left_Brush)IR_Transmite_Cycling(0,ME_LBStall);
			if(Motor_Check_Code&Check_Right_Brush)IR_Transmite_Cycling(0,ME_RBStall);
			if(Motor_Check_Code&Check_Main_Brush)IR_Transmite_Cycling(0,ME_MBStall);
			if(Motor_Check_Code&Check_Vacuum)IR_Transmite_Cycling(0,ME_FStall);
    }

	  Temp_Cliff = Get_Cliff_Trig();
	  if(Temp_Cliff)
		{
      Set_Wheel_Speed(0,0);
      Reset_TempPWM();
			/*--------------------------------------------------------------Cliff------------------------------*/
			if(Get_RightWall_Step()<8000)//about 1 Meter    6000 = 1 m
			{
				Disable_Motors();
				if(Temp_Cliff&Status_Cliff_Left)IR_Transmite_Cycling(0,ME_LCliff);
				if(Temp_Cliff&Status_Cliff_Front)IR_Transmite_Cycling(0,ME_FCliff);
				if(Temp_Cliff&Status_Cliff_Right)IR_Transmite_Cycling(0,ME_RCliff);
			}
		  break;
		}
	 /*---------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status()&RightBumperTrig)
    {
      Wall_BH_Counter++;
		  if(Get_WallAccelerate()>100)
			{
        Stop_Brifly();
				Wall_Move_Back();
				if(Is_Bumper_Jamed())
        {
          Disable_Motors();
          Display_Content(LED_Exclamation,100,ME_RBumper,0,1);//Right Bumper jamed
    			IR_Transmite_Cycling(0,ME_RBumper);
        }
			}
			else
			{
				Wall_Move_Back();
			}
	    Turn_Right(Turn_Speed,720);
	    Move_Forward(15,15);
	  	Reset_WallAccelerate();
			Move_Distance=375;
      Set_Wheel_Step(0,0);
			Reset_TempPWM();
    }

		if(Get_Bumper_Status()&LeftBumperTrig)
    {
//      Wall_BH_Counter=0;
      Set_Wheel_Speed(0,0);
      Reset_TempPWM();
      delay(300);
			
      if(Get_LWall_ADC()>200)
			{
				Wall_Distance-=100;
			}
			else
			{
				Wall_Distance-=100;
			}
			if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;	
			
      if(Get_Bumper_Status()&RightBumperTrig)
      {
        Wall_Move_Back();
        if(Is_Bumper_Jamed())
        {
          Disable_Motors();
          Display_Content(LED_Exclamation,100,ME_LBumper,0,1);//Left Bumper
    			IR_Transmite_Cycling(0,ME_LBumper);
        }
        Turn_Right(Turn_Speed,600);
        Move_Distance=150;
      }
      else
      {
        Wall_Move_Back();
        if(Is_Bumper_Jamed())
        {
          Disable_Motors();
          Display_Content(LED_Exclamation,100,ME_LBumper,0,1);//Left Bumper
    			IR_Transmite_Cycling(0,ME_LBumper);
        }
        Turn_Right(Turn_Speed,300);
        Move_Distance=250;
      }

			Reset_WallAccelerate();
      Move_Forward(10,10);
      Set_Wheel_Step(0,0);
			Reset_TempPWM();
    }
    if(Wall_BH_Counter>8)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_WSensor,0,1);//wall sensor
      IR_Transmite_Cycling(0,ME_WSensor);
    }

    /*------------------------------------------------------Short Distance Move-----------------------*/
		if (Get_WallAccelerate() < (uint32_t) Move_Distance)
		{
      if(Get_LeftWheel_Step()<500)
      {
        if(Get_WallAccelerate()<100)
  			{
  			  Move_Forward(20,20);
  			}
  			else
  			{
  			  Move_Forward(25,25);
  			}
      }
      else
      {
        Move_Forward(30,30);
      }
		}
		else
		{
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
	    //if(OBS_Status.Front_OBS <1500)
			if((Cormove_Get_OBSStatus()&Status_Front_OBS)==0)
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
						Left_Wall_Speed = 22 + Proportion/16 + Delta/10;
						Right_Wall_Speed = 22 - Proportion/10 - Delta/8;
						if(Right_Wall_Speed>28)
						{
							Left_Wall_Speed=6;
							Right_Wall_Speed=30;
						}
					}
          Current_Summary += ADC_Value.System_Current;
					Current_SUC++;
					
					Previous = Proportion;
					
					if(Left_Wall_Speed<0)Left_Wall_Speed=0;
					if(Left_Wall_Speed>40)Left_Wall_Speed=40;
					if(Right_Wall_Speed<0)Right_Wall_Speed=0;
					
					Move_Forward(Left_Wall_Speed,Right_Wall_Speed);
	    }
	    else
	    {
        Stop_Brifly();
//			  if(Left_Wheel_Step<12500)
//				{
//					if(Is_FrontOBS_Trig())
//					{
//						 Turn_Right(Turn_Speed,1025);
//				     Move_Forward(15,15);
//					}
//					else
//					{
//				     Turn_Right(Turn_Speed,500);
//				     Move_Forward(15,15);
//					}
//				}
//				else
//				{
//				  Turn_Right(Turn_Speed,1500);
//			    Move_Forward(15,15);
//				}
				Turn_Right(Turn_Speed,500);
				Move_Forward(15,15);
        Set_Wheel_Step(0,0);
			  Reset_TempPWM();
	    }
		}
	}
	/*--------------------------------------------------------------Run Current--------------------*/
	Current_Summary = Current_Summary/Current_SUC;
	Current_Summary = (Current_Summary*10*ReferenceVoltage)/4096;
	Current_Summary = Current_Summary-BaseCurrent;
	#ifdef BLDC_INSTALL
	if((Current_Summary)<800||(Current_Summary>1470))//pending 410(1361 1353 1393 1348) 420(1250)
	{
		Disable_Motors();
    Display_Content(LED_Exclamation,100,ME_RunCurrent,0,1);
		Display_TM1618(Current_Summary,0);
    IR_Transmite_Cycling(0,ME_RunCurrent);
		//IR_Transmite_Cycling(0,Current_Summary);
	}
	#else
	if((Current_Summary)<400||(Current_Summary>700))
	{
		Disable_Motors();
    Display_Content(LED_Exclamation,100,ME_RunCurrent,0,1);
		Display_TM1618(Current_Summary,0);
    IR_Transmite_Cycling(0,ME_RunCurrent);
		//IR_Transmite_Cycling(0,Current_Summary);
	}
	#endif

	/*--------------------------------------------------------------Mobility Sensor fail--------------------*/
  if(Get_Mobility_Step()<150)              //Mobility Sensor fail
  {
    Disable_Motors();
    Display_Content(LED_Exclamation,100,ME_MobSensor,0,1);//Mobility Sensor
    IR_Transmite_Cycling(0,ME_MobSensor);
  }

	delay(1000);
	if(Cliff_Status.Front_Cliff<100)
  {
    delay(100);
    if(Cliff_Status.Front_Cliff<100)
    {
      Test_Status |= Test_Status_Front_Cliff; 
    }
  }
	
	Cliff_Turn_Left(Turn_Speed,450);
  Set_Wheel_Speed(0,0);
  Reset_TempPWM();
	
	Rcon_Delay=0;
	while(1)
	{
	  Rcon_Delay++;
		if(Rcon_Delay>15)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_RCliff,0,1);//Right Cliff
    	IR_Transmite_Cycling(0,ME_RCliff);
    }
    if(Cliff_Status.Right_Cliff  <100)
		{
      delay(100);
      if(Cliff_Status.Right_Cliff  <100)
		  {
			  Test_Status |= Test_Status_Right_Cliff;
			  break;
      }
		}
		delay(1000);
	}
	Cliff_Turn_Right(Turn_Speed,200);
  Move_Back();
//	Move_Back();
	Move_Back();
	Turn_Right(Turn_Speed,900);
  Stop_Brifly();
	Move_Forward(20,20);
  Rcon_Delay=0;
	while(1)
	{
    Rcon_Delay++;
    if(Rcon_Delay > 300)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_LCliff,0,1);//Left Cliff
      IR_Transmite_Cycling(0,ME_LCliff);
    }
		if(Cliff_Status.Left_Cliff  <100)
		{
      delay(100);
      if(Cliff_Status.Left_Cliff  <100)
		  {
        Stop_Brifly();
		    Move_Back();
			  Test_Status |= Test_Status_Left_Cliff;
			  break;
      }
		}
		if(Cliff_Status.Front_Cliff<100)
    {
      delay(100);
      if(Cliff_Status.Front_Cliff<100)
      {
        break;
      }
    }
		if(Cliff_Status.Right_Cliff  <100)
    {
      delay(100);
      if(Cliff_Status.Right_Cliff  <100)
      {
        break;
      }
    }
		delay(100);
	}
	if(!(Test_Status&Test_Status_Left_Cliff))
	{
		Disable_Motors();
		Display_Content(LED_Exclamation,100,ME_LCliff,0,1);//left cliff
		IR_Transmite_Cycling(0,ME_LCliff);
	}
	if(!(Test_Status&Test_Status_Front_Cliff))
	{
		Disable_Motors();
	  Display_Content(LED_Exclamation,100,ME_FCliff,0,1);//front cliff 1
		while(1)
		{
			IR_Transmite_Error(0,ME_FCliff,0);
		}
	}
	if(!(Test_Status&Test_Status_Right_Cliff))
	{
		Disable_Motors();
		Display_Content(LED_Exclamation,100,ME_RCliff,0,1);  //right cliff
		IR_Transmite_Cycling(0,ME_RCliff);
	}
	Turn_Right(Turn_Speed,800);
  Move_Forward(20,20);
  Reset_Wheel_Step();
  Rcon_Delay=0;
  while(1)
  {
    Rcon_Delay++;
    delay(100);
    if(Rcon_Delay>300)
    {
      Disable_Motors();
  		Display_Content(LED_Exclamation,100,ME_Encoder,0,1);//left wheel counter
  		IR_Transmite_Cycling(0,ME_Encoder);
    }
    if(Left_Wheel_Step>600)
    {
      if((Right_Wheel_Step>500)&&(Right_Wheel_Step<700))
      {
        break;
      }
      else
      {
        Disable_Motors();
        Display_Content(LED_Exclamation,100,ME_Encoder,0,1);//Right right wheel counter
        IR_Transmite_Cycling(0,ME_Encoder);
      }
    }
  }
  //move a certan distance
  Stop_Brifly();

  
  Rcon_Delay=0;
  Reset_Rcon_Status();
  while(1)
  {
    Rcon_Delay++;
    if(Rcon_Delay>10)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_LRcon,0,1);// left rcon
      IR_Transmite_Cycling(0,ME_LRcon);
    }
    if(Get_Rcon_Status()&(RconL_HomeL|RconL_HomeR))break;//check left rcon
    delay(1000);
  }

  Turn_Right(Turn_Speed,800);//Turn right 90
  Stop_Brifly();
  Rcon_Delay=0;
  Reset_Rcon_Status();
  while(1)
  {
    Rcon_Delay++;
    if(Rcon_Delay>10)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_BLRcon,0,1);//back left rcon
      IR_Transmite_Cycling(0,ME_BLRcon);
    }
    if(Get_Rcon_Status()&(RconBR_HomeL|RconBR_HomeR))break;//check Back Left rcon
    delay(1000);
  }

  Turn_Right(Turn_Speed,900);//Turn right 90
  Stop_Brifly();
  Rcon_Delay=0;
  Reset_Rcon_Status();
  while(1)
  {
    Rcon_Delay++;
    if(Rcon_Delay>10)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_BRRcon,0,1);// back right rcon
      IR_Transmite_Cycling(0,ME_BRRcon);
    }
    if(Get_Rcon_Status()&(RconBR_HomeL|RconBR_HomeR))break;//check Back Right rcon
    delay(1000);
  }

  Turn_Right(Turn_Speed,800);
  Stop_Brifly();
  Rcon_Delay=0;
  Reset_Rcon_Status();
  
  while(1)
  {
    Rcon_Delay++;
    if(Rcon_Delay>15)
    {
      Disable_Motors();
      Display_Content(LED_Exclamation,100,ME_RRcon,0,1);//R rcon
      IR_Transmite_Cycling(0,ME_RRcon);
    }
    if(Get_Rcon_Status()&(RconR_HomeL|RconR_HomeR))break;//check  R rcon
    delay(1000);
  }

  Turn_Right(Turn_Speed,250);
  Stop_Brifly();
  Rcon_Delay=0;
  Reset_Rcon_Status();
	
	Set_LED_On_Switch(1,1,1);
	Set_LED(100,0,0);
	IR_Transmite_Status(8,8888);
	Beep(10);
	Beep(0);
	Beep(1);
	Beep(0);
	Beep(3);
	Beep(2);
	Beep(0);
	Beep(5);
//	Clear_Life();
	IR_Transmite_Status(8,8888);
	Set_VacMode(Vac_Normal);
  //testing all rcons
	Set_SideBrush_PWM(0,0);
	Set_Clean_Mode(Clean_Mode_GoHome);
  Reset_Rcon_Status();
}



/*-------------------------------------------------wrong & pending*/
/*
  No charger signal detect                    01
  Encoder                                     02
  Right OBS                                   03
  Front OBS                                   04
  Left OBS                                    05
  Right Bumper                                06
  Motor Current                               07
  Right Bumper Jamed                          08
  Left Bumper Jamed                           09
  Wall Sensor                                 10
  Mobility Sensor                             11
  Right Cliff                                 12
  Left  Cliff                                 13
  Front Cliff                                 14
  Wheel Speed                                 15
  Left Rcon                                   16
  Back Left Rcon                              17
  Back Right Rcon                             18
  Right Rcon                                  19
  Run Current                                 20
	OBS_Error                                   21*/
	/*-------------------------------------------------wrong & pending V2*/


void Run_Gyro_Mobility(uint8_t code)
{
	uint8_t Blink_LED=0;
	Point32_t	Home_Point;
	Point32_t Next_P;
	Home_Point.X=0;
	Home_Point.Y=0;
	uint16_t temp_angle=0;
	uint32_t temp_moving=0;
	uint8_t error=0;
	
	Enable_PPower();
	Beep(3);
	Beep(5);
	delay(200);
	Gyro_Cmd(DISABLE);
	delay(300);
	Gyro_Cmd(ENABLE);
	Work_Motor_Configure();
	Blink_LED=5;
	Set_LED_On_Blink(0,0,0);
	Set_LED_On_Switch(1,0,1);
	while(Blink_LED--)
	{
		Set_LED(100, 0, 0);
		delay(2000);
		Set_LED(0, 0, 0);
		delay(2000);
	}
	Reset_OBST_Value();
	
	#ifdef GYRO_XV7011
	if((Gyro_GetCalibration()&0xA8)!=0xA0)
	{
		Display_Content(LED_Exclamation,100,10,0,8);//Main brush c detect
		USART_Print("#GyroFail!",Dev_USART3);
    while(1)
		{
			IR_Transmite_Error(0,11,0);
      delay(2000);
		}
	}
	#endif
	
	if(code==0)
	{
		Map_Initialize();
		PathPlanning_Initialize(&Home_Point.X, &Home_Point.Y);
		Map_SetCell(MAP, cellToCount(0), cellToCount(-10), CLEANED);
		Map_SetCell(MAP, cellToCount(10), cellToCount(0), CLEANED);
		Map_SetCell(MAP, cellToCount(10), cellToCount(-10), CLEANED);
		Map_SetCell(MAP, cellToCount(0), cellToCount(0), CLEANED);
		Reset_Rcon_Remote();
		Set_LED_On_Switch(1,0,1);
		Set_LED(100,100, 0);

		for(Blink_LED=0;Blink_LED<4;Blink_LED++)
		{
			Next_P.X=cellToCount(3);
			Next_P.Y=cellToCount(0);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

			Next_P.X=cellToCount(3);
			Next_P.Y=cellToCount(-2);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

			Next_P.X=cellToCount(0);
			Next_P.Y=cellToCount(-2);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

			Next_P.X=cellToCount(0);
			Next_P.Y=cellToCount(0);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

		}
		Next_P.X=cellToCount(3);
		Next_P.Y=cellToCount(0);
		CM_MoveToPoint(Next_P, 0);
		Set_LED(100,100, 0);
		Beep(1);
		Set_LED(0,0, 0);
		Beep(5);
		Set_LED(100,100, 0);
		Beep(1);
		Set_LED(0,0, 0);
		Beep(5);
		Set_LED(100,100, 0);
		while(1)
		{
			if(Remote_Key(Remote_Max))// wheel test
			{
				break;
			}
		}
		Reset_Rcon_Remote();
		for(Blink_LED=0;Blink_LED<4;Blink_LED++)
		{

			Next_P.X=cellToCount(3);
			Next_P.Y=cellToCount(2);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

			Next_P.X=cellToCount(0);
			Next_P.Y=cellToCount(2);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

			Next_P.X=cellToCount(0);
			Next_P.Y=cellToCount(0);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);

			Next_P.X=cellToCount(3);
			Next_P.Y=cellToCount(0);
			CM_MoveToPoint(Next_P, 0);
			Reset_Rcon_Remote();
			Reset_Touch();
			Set_BLDC_Speed(Vac_Speed_Normal);
		}
		/*---------------*/
		CM_HeadToCourse(Rotate_TopSpeed,0);
		Disable_Motors();
		Stop_Brifly();
		Set_LED_On_Switch(1,1,1);
		Set_LED(100,0, 0);
		Beep(3);
		Beep(5);
		Beep(1);
		Beep(2);
		Beep(3);
		while(1);
	}
	else if(code==1)
	{		
		Set_LED(100,0, 0);
		temp_angle = Gyro_GetAngle(0);
				
		if((ADC_Value.Battery_Voltage%2)==0)
		{
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}					
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}					
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}					
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}					
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}					
			
//			CM_HeadToCourse(Rotate_TopSpeed, 900);
//			CM_HeadToCourse(Rotate_TopSpeed, 1800);
//			CM_HeadToCourse(Rotate_TopSpeed, 2700);
//			CM_HeadToCourse(Rotate_TopSpeed, 3600);
		}
		else
		{
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 2700);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 1800);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 900);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}			
			Left_Wheel_Step = 0;
			Right_Wheel_Step = 0;
			if(error==0)CM_HeadToCourse(Rotate_TopSpeed, 3600);
			temp_moving = (Left_Wheel_Step+Right_Wheel_Step)/2;
			temp_moving = temp_moving*ANGLE_P/ANGLE_D;
			USPRINTF("left%d:\n", Left_Wheel_Step);
			USPRINTF("right%d:\n", Right_Wheel_Step);
			USPRINTF("moving%d:\n", temp_moving);	
			if((temp_moving>950)||(temp_moving<850))
			{
				error = 1;
			}				
			
//			CM_HeadToCourse(Rotate_TopSpeed, 2700);
//			CM_HeadToCourse(Rotate_TopSpeed, 1800);
//			CM_HeadToCourse(Rotate_TopSpeed, 900);
//			CM_HeadToCourse(Rotate_TopSpeed, 3600);
		}
		
		/*---------------*/
		CM_HeadToCourse(Rotate_TopSpeed,temp_angle);
		Disable_Motors();
		Stop_Brifly();
		if(error==0)
		{
			Set_LED_On_Blink(0,0,0);
			Set_LED_On_Switch(1,1,1);
			Set_LED(100,0,0);
		}
		else
		{
			Set_LED_On_Switch(0,0,0);
			Set_LED_On_Blink(1,1,1);
		}
		Beep(3);
		Beep(5);
		Beep(1);
		Beep(2);
		Beep(3);
		while(1);
		
	}

}







