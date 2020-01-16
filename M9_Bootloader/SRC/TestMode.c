 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Test mode for customer to check the robot's seperated funtion/module
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "SysInitialize.h"
#include "TestMode.h"
#include "Speaker.h"
#include "USART.h"
#include "Charge.h"
#include "Rcon.h"
#include "RTC.h"
#include "IR.h"
#include "Flash.h"
#include "Gyro.h"
#include "W25Q16.h"




/*----------------------------------------------------------- Spining Event -----------------------*/
void Test_Mode(void)
{
		
  uint16_t Temp_Display=0,Time_Counter=500;
	
  uint8_t Status_Flag=1,blink=0,blinkplus=0;
	
	uint32_t Current_Current=0;
	
	uint16_t Temp_Base_Line=0,Test_PWM=0;
	
	uint32_t Temp_Value=0,Test_Result=0;
	
	uint16_t Temp_PWM=0,Charge_Time=0;

  uint32_t Temp_Rcon_Status=0;
	
  uint8_t Display_Delay=0,Retest_Flag=0;

	
	Display_Clean_Status(Display_Test);
	Speaker(SPK_DON);
	delay(800);
	
 	Debug_Print("@Test Start!",Dev_USART0);
  Debug_Print("@Version",Dev_USART0);
	Debug_Print("#20190912",Dev_USART0);
	
	while(Key_GetStatus()&KEY_CLEAN){}//wait until clean key release
  delay(2000);

	#if 0
	while(1)
	{

		delay(10000); 
//		USPRINTF("Left_Wall:%d\n",g_obs_adc.Left_Wall);
//		USPRINTF("Left_OBS:%d\n",g_obs_adc.Left_OBS);		
//		USPRINTF("Front_OBS:%d\n",g_obs_adc.Front_OBS);
//		USPRINTF("Right_OBS:%d\n",g_obs_adc.Right_OBS);
//		USPRINTF("Right_Wall:%d\n",g_obs_adc.Right_Wall);
//		USPRINTF("Xp_Wall:%d\n\n",g_obs_adc.Xp_Wall);		

	
		USPRINTF("Front_Left_Cliff:%d\n",g_cliff_adc.Front_Left_Cliff);
		USPRINTF("Front_Right_Cliff:%d\n",g_cliff_adc.Front_Right_Cliff);	
		USPRINTF("Back_Left_Cliff:%d\n",g_cliff_adc.Back_Left_Cliff);
		USPRINTF("Back_Right_Cliff:%d\n\n",g_cliff_adc.Back_Right_Cliff);	

//		USPRINTF("Front_Left_Cliff:%d\n",g_cliff_sunlight.Front_Left_Cliff);
//		USPRINTF("Front_Right_Cliff:%d\n",g_cliff_sunlight.Front_Right_Cliff);	
//		USPRINTF("Back_Left_Cliff:%d\n",g_cliff_sunlight.Back_Left_Cliff);
//		USPRINTF("Back_Right_Cliff:%d\n\n",g_cliff_sunlight.Back_Right_Cliff);	
//		USPRINTF("Front_Left_Cliff:%d\n",Cliff_SunLight2.Front_Left_Cliff);
//		USPRINTF("Front_Right_Cliff:%d\n",Cliff_SunLight2.Front_Right_Cliff);	
//		USPRINTF("Back_Left_Cliff:%d\n",Cliff_SunLight2.Back_Left_Cliff);
//		USPRINTF("Back_Right_Cliff:%d\n\n",Cliff_SunLight2.Back_Right_Cliff);	

		
//		USPRINTF("Battery_Voltage:%d\n",g_adc_value.Battery_Voltage);		
//		USPRINTF("Main_Brush_Current:%d\n",g_adc_value.Main_Brush_Current);		
//		USPRINTF("System_Current:%d\n",g_adc_value.System_Current);	
//		USPRINTF("Front_Left_Cliff:%d\n",g_adc_value.Front_Left_Cliff);
//		USPRINTF("Back_Left_Cliff:%d\n",g_adc_value.Back_Left_Cliff);
//		USPRINTF("OBS_CH1:%d\n",g_adc_value.OBS_CH1);		
//		USPRINTF("OBS_CH2:%d\n",g_adc_value.OBS_CH2);
//		USPRINTF("OBS_CH3:%d\n",g_adc_value.OBS_CH3);
//		USPRINTF("Xp_Wall:%d\n",g_adc_value.Xp_Wall);
//		USPRINTF("Left_Wheel_Current:%d\n",g_adc_value.Left_Wheel_Current);		
//		USPRINTF("Right_Wheel_Current:%d\n",g_adc_value.Right_Wheel_Current);
//		USPRINTF("Right_Brush_Current:%d\n",g_adc_value.Right_Brush_Current);
//		USPRINTF("Vacuum_Current:%d\n",g_adc_value.Vacuum_Current);	
//		USPRINTF("Front_Right_Cliff:%d\n",g_adc_value.Front_Right_Cliff);		
//		USPRINTF("Back_Right_Cliff:%d\n\n",g_adc_value.Back_Right_Cliff);				
	}
	#endif		
		
  /*----------------------------------------------- Wait for start key  ------------------------------------------*/
	while(1)
	{			
		if(Key_GetStatus()==KEY_CLEAN)
		{
			Speaker(SPK_DON);
			Status_Flag = 1;//1
      while(Key_GetStatus()==KEY_CLEAN){}			
		  break;
		}		
		delay(100);
	}
	Gyro_Cmd(ENABLE);
	
	/*===============================================Self Test Start====================================*/

		
  /*----------------------------------------------- Check Base Line Voltage ------------------------01-------------*/
	Temp_Base_Line = g_baselineadc*ReferenceVoltage/4096;
	if((Temp_Base_Line<220)||(Temp_Base_Line>240))//2.2V~2.4V
	{
		USPRINTF("#BaseLineFail:%d\n",Temp_Base_Line);
		while(1)
		{
			delay(2000);
		}
	}
	/*----------------------------------------------- Check RTC ------------------------02-------------*/
//  RTC_Configuration();
//	RTC_SetAlarmTime(10,15);	
//	RTC_SetCurrentTime(10,10);

//  delay(100);
//	
//	if(RTC_GetCurrentMinutes()!=610)
//	{
//		RTC_Configuration();
//		RTC_SetCurrentTime(10,10);
//		RTC_SetAlarmTime(10,15);
//		delay(100);
//	}
//	
//	if(RTC_GetCurrentMinutes()!=610)
//	{	
//		Debug_Print("#RTC Fail",Dev_USART0);
//		delay(100);
//		USPRINTF("#Time:%d\n",RTC_GetCurrentMinutes());
//		while(1)
//		{
//			delay(10000);
//		}	
//	}
//	RTC_DisableAlarm();
		
  /*----------------------------------------------- Check Battery Voltage -------------------------------03-----------*/
	Temp_Value=	GetBatteryVoltage();
	if((Temp_Value<1350)||(Temp_Value>1700)) 
	{
//		Display_Battery_Status(Display_Low);
//		Debug_Print("#BatFail!",Dev_USART0);
//		USPRINTF("#BatV:%d\n",Temp_Value);
//		while(1)
//		{
//      delay(2000);
//    }
	}
	
	/*---------------------------------- Check systemt Current   ---------------------------------*/
	Motor_Test_Set();
	delay(1000);
	Temp_Value = Get_Sys_Current(g_baselineadc);	
	if((Temp_Value>180)||(Temp_Value<20))
	{
		USPRINTF("#CurFail!:%d\n",Temp_Value);	
    while(1)
    {
      delay(2000);
    }
	}
		
	/*-----------------------------------------------------Check Gyro------------------------*/
	Gyro_ResetUpdateFlag(0);
	delay(3000);	
	if(!Gryo_GetUpdateFlag())
	{
//		USPRINTF("GYRO ID read ERR!!");
//		while(1)
//		{
//      delay(2000);
//			if(Gryo_GetUpdateFlag())break;
//    }		
	}
	
	/*-----------------------------------------------------spi flash check------------------------*/
	Usprintf("flash ID:%x\n",W25QXX_ReadID());
	if(W25QXX_ReadID()!=0x1515)
	{
		USPRINTF("SPI flash read ERR!!");
		while(1)
		{
			delay(1000);
			if(W25QXX_ReadID()!=0x1515)break;
			Usprintf("flash ID:%x\n",W25QXX_ReadID());
    }		
	}	
	
  Debug_Print("@PASS",Dev_USART0);
	Speaker(SPK_DON);	
	/*===============================================Self Test End====================================*/	
	

	#define MAX_STEP 14
	#define MIN_STEP 1
 /*---------***************************---------Test Start---------***************************--------*/
	while(Key_GetStatus()&KEY_CLEAN);
	while(1)
	{
		Test_Result=0;
    /*------------------------------------------Test Display----------------------------------*/
		while(Status_Flag == 1)
		{
			Set_LED_On_Switch(1,1,1,1,1,1);
      for(Display_Delay=0;Display_Delay<100;Display_Delay++)
			{
				delay(100);
				if(Is_Key_Press(KEY_HOME))
				{
					Test_Result|=0x01;
				}
				else
				{
					Test_Result|=0x02;
				}
				if(Is_Key_Press(KEY_SPOT))
				{
					Test_Result|=0x04;
				}
				else
				{
					Test_Result|=0x08;
				}				
				if(Is_Key_Press(KEY_CLEAN))
				{
					Test_Result|=0x10;
				}
				else
				{
					Test_Result|=0x20;
				}
				Set_LED_On_Switch(1,1,1,1,1,1);
				Set_LED_Table(Display_Delay,Display_Delay,Display_Delay,Display_Delay,Display_Delay,Display_Delay);
				
				if(Test_Result==0x3f)
				{
					Status_Flag=Switch_Step(Status_Flag,1);
					Set_LED_On_Switch(1,0,1,0,0,0);
					Set_LED(100,100,100,100,100,100);
					break;
				}
			}
			if(Status_Flag!=1)break;
			if(Remote_Key(Remote_Max))
			{
				Status_Flag=Switch_Step(Status_Flag,0);
        break;
			}
    }
		while(Key_GetStatus()&KEY_CLEAN){}
			
  	/*------------------------------------------Test OBS----------------------------------*/
	  Test_Result=0;
		Temp_Display=0;
		blink=50;
	  while(Status_Flag == 2)
		{
			delay(500);

			if(g_obs_adc.Left_Wall > Obs_Max)
			{
			  Test_Result|=0x0001;  
			}
			else if(g_obs_adc.Left_Wall < Obs_Limit)
			{
			  Test_Result|=0x0002;
			}
	
			if(g_obs_adc.Left_OBS > Obs_Max)
			{
			  Test_Result|=0x0004;
			}
			else if(g_obs_adc.Left_OBS < Obs_Limit)
			{
			  Test_Result|=0x0008;
			}
			
		  if(g_obs_adc.Front_OBS > Obs_Max)
			{
			  Test_Result|=0x0010;
			}
			else if(g_obs_adc.Front_OBS < Obs_Limit)
			{
			  Test_Result|=0x0020;
			}
			
			if(g_obs_adc.Right_OBS > Obs_Max)
			{
			  Test_Result|=0x0040;
			}
			else if(g_obs_adc.Right_OBS < Obs_Limit) 
			{
			  Test_Result|=0x0080;
			}
			
			if(g_obs_adc.Right_Wall > Obs_Max)
			{
			  Test_Result|=0x0100;  
			}
			else if(g_obs_adc.Right_Wall < Obs_Limit)
			{
			  Test_Result|=0x0200;
			}

			if(g_obs_adc.Xp_Wall > Obs_Max)
			{
			  Test_Result|=0x0400;  
			}
			else if(g_obs_adc.Xp_Wall < Obs_Limit)
			{
			  Test_Result|=0x0800;
			}
									
      if(Remote_Key(Remote_Max))//back to previous test
			{
				Status_Flag=Switch_Step(Status_Flag,0);
        break;
			}
      if(Test_Result==0xfff)//test pass
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }
						
			if(Is_Key_Press(KEY_CLEAN))
			{
				if((Test_Result&0x003)!=0x003)Debug_Print("#LWall ",Dev_USART0);
        if((Test_Result&0x00c)!=0x00c)Debug_Print("#LOBS ",Dev_USART0);
        if((Test_Result&0x030)!=0x030)Debug_Print("#FOBS ",Dev_USART0);
        if((Test_Result&0x0c0)!=0x0c0)Debug_Print("#ROBS ",Dev_USART0);				
				if((Test_Result&0x300)!=0x300)Debug_Print("#RWall ",Dev_USART0);				
				if((Test_Result&0xc00)!=0xc00)Debug_Print("#XWall ",Dev_USART0);
			}
		}
		
		/*------------------------------------------Test Switches----------------------------------*/
	  Test_Result=0;
		Temp_Display=0;
		while(Status_Flag == 3)
		{
		  delay(100);
			Temp_Display=0;
		  if(Get_Bumper_Status()&LLeftBumperTrig)
			{
			  Test_Result|=0x01;
			}
			else 
			{
				Test_Result|=0x02;
			}
			
		  if(Get_Bumper_Status()&FLeftBumperTrig)
			{
			  Test_Result|=0x04;
			}
			else 
			{
				Test_Result|=0x08;
			}
			
			if(Get_Bumper_Status()&FRightBumperTrig)
			{
				Test_Result|=0x10;
			}
			else 
			{
				Test_Result|=0x20;
			}
			
			if(Get_Bumper_Status()&RRightBumperTrig)
			{
				Test_Result|=0x40;
			}
			else 
			{
				Test_Result|=0x80;
			}	
			
			blink++;
			if(blink>50)
			{
				blink = 0;
			}
			
			if(Remote_Key(Remote_Max))//back to previous test
			{
				Status_Flag=Switch_Step(Status_Flag,0);
        break;
			}

      if(Test_Result==0xff)
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }
			if(Is_Key_Press(KEY_CLEAN))
			{
				if((Test_Result&0x0003)!=0x0003)Debug_Print("#LBUMPER",Dev_USART0);
				if((Test_Result&0x000c)!=0x000c)Debug_Print("#FLBUMPER",Dev_USART0);				
				if((Test_Result&0x0030)!=0x0030)Debug_Print("#FRBUMPER ",Dev_USART0);
				if((Test_Result&0x00c0)!=0x00c0)Debug_Print("#RBUMPER ",Dev_USART0);
			}
		}
		
		/*------------------------------------------Test Cliff----------------------------------*/
	  Test_Result=0;
		Temp_Display=0;
		while(Status_Flag == 4)
		{
			Temp_Display=0;
      /*--------------Front Left Cliff-----------------------*/
      if(g_cliff_adc.Front_Left_Cliff < Cliff_Limit)
    	{
    	  delay(50);
    	  if(g_cliff_adc.Front_Left_Cliff < Cliff_Limit)
    	  {
           Test_Result|=0x0001;
    		}
    	}
      else if(g_cliff_adc.Front_Left_Cliff > Cliff_Max)
      {
        delay(50);
        if(g_cliff_adc.Front_Left_Cliff > Cliff_Max)
        {
           Test_Result|=0x0002;
        }
      }
      /*--------------Front Right Cliff-----------------------*/
    	if(g_cliff_adc.Front_Right_Cliff < Cliff_Limit)
    	{
    	  delay(50);
    	  if(g_cliff_adc.Front_Right_Cliff < Cliff_Limit)
    	  {
           Test_Result|=0x0004;
    		}
     	}
      else if(g_cliff_adc.Front_Right_Cliff > Cliff_Max)
    	{
    	  delay(50);
    	  if(g_cliff_adc.Front_Right_Cliff > Cliff_Max)
    	  {
           Test_Result|=0x0008;
    		}
     	}
      /*--------------Back Left Cliff-----------------------*/
    	if(g_cliff_adc.Back_Left_Cliff < Cliff_Limit)
    	{
    	  delay(50);
    		if(g_cliff_adc.Back_Left_Cliff < Cliff_Limit)
    		{
           Test_Result|=0x0010;
    		}
    	}
      else if(g_cliff_adc.Back_Left_Cliff > Cliff_Max)
    	{
    	  delay(50);
    		if(g_cliff_adc.Back_Left_Cliff > Cliff_Max)
    		{
           Test_Result|=0x0020;
    		}
    	}
      /*--------------Back Right Cliff-----------------------*/
    	if(g_cliff_adc.Back_Left_Cliff < Cliff_Limit)
    	{
    	  delay(50);
    		if(g_cliff_adc.Back_Left_Cliff < Cliff_Limit)
    		{
           Test_Result|=0x0040;
    		}
    	}
      else if(g_cliff_adc.Back_Left_Cliff > Cliff_Max)
    	{
    	  delay(50);
    		if(g_cliff_adc.Back_Left_Cliff > Cliff_Max)
    		{
           Test_Result|=0x0080;
    		}
    	}

			if(Is_Right_Wheel_Drop())
			{
				Test_Result|=0x0100;
			}
			else
			{
				Test_Result|=0x0200;
			}
			if(Is_Left_Wheel_Drop())
			{
				Test_Result|=0x0400;
			}
			else
			{
				Test_Result|=0x0800;
			}			
			
			blink++;
			if(blink>50)
			{
				blink = 0;
			}
			
      if(Remote_Key(Remote_Max))//back to previous test
			{
				Status_Flag=Switch_Step(Status_Flag,0);
        break;
			}
			
      if(Test_Result==0xfff)//test pass jump to next test
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }
			if(Is_Key_Press(KEY_CLEAN))
			{
         if((Test_Result&0x0003)!=0x0003)Debug_Print("#FL_CLIFF",Dev_USART0);
         if((Test_Result&0x000c)!=0x000c)Debug_Print("#FR_CLIFF",Dev_USART0);
				 if((Test_Result&0x0030)!=0x0030)Debug_Print("#BL_CLIFF",Dev_USART0);
				 if((Test_Result&0x00c0)!=0x00c0)Debug_Print("#BR_CLIFF",Dev_USART0);	
				 if((Test_Result&0x0300)!=0x0300)Debug_Print("#R_DROP",Dev_USART0);
				 if((Test_Result&0x0c00)!=0x0c00)Debug_Print("#L_DROP",Dev_USART0);					
			}
		}

		/*------------------------------------------Test Rcon----------------------------------*/
	  Test_Result=0;
		Temp_Display=0;
		while(Status_Flag == 5)
		{		
		  delay(500);
      Temp_Rcon_Status = Rcon_GetStatus();

      if(Temp_Rcon_Status)Rcon_ResetStatus();
      Temp_Display = 0;
      if(Temp_Rcon_Status & (RconR_HomeL|RconR_HomeR))//right
	    {
				Test_Result|=0x0001;
        Temp_Display +=1;
	    }
			else	
			{
			  Test_Result|=0x0002;
			}

	    if(Temp_Rcon_Status & (RconFR_HomeL|RconFR_HomeR))// front right
	    {
				Test_Result|=0x0004;
        Temp_Display +=10;
	    }
			else	
			{
			  Test_Result|=0x0008;
			}

      if(Temp_Rcon_Status & (RconFL_HomeL|RconFL_HomeR))// front Left
	    {
				Test_Result|=0x0010;
        Temp_Display +=100;
	    }
			else	
			{
			  Test_Result|=0x0020;
			}
		
      if(Temp_Rcon_Status & (RconL_HomeL|RconL_HomeR))// Left
	    {
				Test_Result|=0x0040;
        Temp_Display +=1000;
	    }
			else	
			{
			  Test_Result|=0x0080;
			}

			
			if(Temp_Rcon_Status & (RconBR_HomeL|RconBR_HomeR))// Back Right
	    {
				Test_Result|=0x0100;
        Temp_Display +=2;
	    }
			else	
			{
			  Test_Result|=0x0200;
			}
     			
      if(Remote_Key(Remote_Max))//back to previous test
			{
				Status_Flag=Switch_Step(Status_Flag,0);
        break;
			}
			if(Test_Result==0x03ff)
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }
			if(Is_Key_Press(KEY_CLEAN))
			{
         if((Test_Result&0x0003)!=0x0003)Debug_Print("#RRCON  ",Dev_USART0);
         if((Test_Result&0x000c)!=0x000c)Debug_Print("#FRRCON ",Dev_USART0);
				 if((Test_Result&0x0030)!=0x0030)Debug_Print("#FLRCON ",Dev_USART0);
				 if((Test_Result&0x00c0)!=0x00c0)Debug_Print("#LRCON  ",Dev_USART0);
				 if((Test_Result&0x0300)!=0x0300)Debug_Print("#BRRCON ",Dev_USART0);
			}		
		}

		/*-----------------------------------------Test Water/Dustbin Box-----------------------------------*/
		Test_Result =0;
		while(Status_Flag == 6)
		{							
		  delay(100);		
      if(Remote_Key(Remote_Max))//back to previous test
			{
				Status_Flag=Switch_Step(Status_Flag,0);
        break;
			}

			if(Is_Water_Tank())
			{
				Test_Result|=0x01;				
			}
			else
			{
				Test_Result|=0x02;
			}
			if(Is_Dustbin())
			{
				Test_Result|=0x04;
			}
			else
			{
				Test_Result|=0x08;
			}

			if(Test_Result==0x0f)
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }	
			if(Is_Key_Press(KEY_CLEAN))
			{
				if((Test_Result&0x0003)!=0x0003)Debug_Print("#WATER",Dev_USART0);
				if((Test_Result&0x000c)!=0x000c)Debug_Print("#DUSTBIN",Dev_USART0);
			}			
		}

		/*---------------------------------------Test Left Wheel Current-------------------------------------*/
		Test_Result=0;
		while(Status_Flag == 7)
		{   				
		  Set_Dir_Forward();
			Set_Wheel_Speed(RUN_SPEED_15,0);
			delay(10000);
			Time_Counter=40;
			Current_Current=0;		
			while(Time_Counter--)
			{
			  delay(50);
				Current_Current += g_adc_value.Left_Wheel_Current;
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}
      if(Status_Flag !=7)break;
      Test_PWM = TIMER_CH1CV(TIMER0);
			Current_Current/=40;
			Current_Current-=	g_leftwheel_baseline;
			Current_Current *= 5;
			Current_Current = Current_Current*ReferenceVoltage/4096;
			if((Current_Current>70)||(Current_Current<20))
			{
				Debug_Print("#Fail",Dev_USART0);			
				USPRINTF("#Left W C1=:%d\n",Current_Current);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0001;
        delay(3000);
      }
      //check the run pwm
      if(((Test_PWM<50)||(Test_PWM>90)))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Left W P1=:%d\n",Test_PWM);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0002;
      }
			
		  Set_Dir_Backward();
			Set_Wheel_Speed(RUN_SPEED_15,0);
			delay(10000);
			Time_Counter=40;
			Current_Current=0;			
		  while(Time_Counter--)
			{
			  delay(50);
				Current_Current += g_adc_value.Left_Wheel_Current;
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}
      if(Status_Flag !=7)break;
			Test_PWM = TIMER_CH0CV(TIMER0);
			Current_Current/=40;
			Current_Current-=	g_leftwheel_baseline;
			Current_Current *= 5;
			Current_Current = Current_Current*ReferenceVoltage/4096;//we will check
			if((Current_Current>70)||(Current_Current<20))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Left W C2=:%d\n",Current_Current);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0004;
        delay(2000);
      }
      //check the run pwm
      if(((Test_PWM<50)||(Test_PWM>90)))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Left W P2=:%d\n",Test_PWM);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0008;
      }

      if(Test_Result==0xf)
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }
		}
    
    /*---------------------------------------Test Left Wheel stall Sensors-------------------------------------*/
    while(Status_Flag == 8)
    { 				
      Move_Forward(RUN_SPEED_17,0);
      while(1)
			{
        if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			  if(g_adc_value.Left_Wheel_Current>800)
				{
				  delay(100);
					if(g_adc_value.Left_Wheel_Current>800)
				  {
					  Current_Current=(g_adc_value.Left_Wheel_Current - g_leftwheel_baseline)*ReferenceVoltage*5/4096;
					  if(Current_Current<500)
						{
							Debug_Print("#Fail",Dev_USART0);
							USPRINTF("#Left Wheel Stall=:%d\n",Current_Current);	
              delay(5000);
						}
            else
            {
              delay(2000);
              Status_Flag=Switch_Step(Status_Flag,1);
              break;
            }
					}
				}
        delay(500);
			}
			break;
    }
		
	  /*-------------------------------------------Test Right Wheel Current ---------------------------------*/
    Test_Result = 0;
		while(Status_Flag == 9)
		{		  
		  Set_Dir_Forward();
			Set_Wheel_Speed(0,RUN_SPEED_15);
			delay(10000);
			Time_Counter=40;
			Current_Current=0;
			
			while(Time_Counter--)
			{
			  delay(50);
				Current_Current += g_adc_value.Right_Wheel_Current;
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}

      if(Status_Flag!=9)break;
      Test_PWM = TIMER_CH3CV(TIMER0);
			Current_Current/=40;
			Current_Current-=	g_rightwheel_baseline;
			Current_Current *= 5;
			Current_Current = Current_Current*ReferenceVoltage/4096;
			if((Current_Current>70)||(Current_Current<20))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Right W C1=:%d\n",Current_Current);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0001;
        delay(2000);
      }
      if(((Test_PWM<50)||(Test_PWM>90)))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Right W P1=:%d\n",Test_PWM);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0002;
      }

      //test right wheel run forward current and pwm			
		  Set_Dir_Backward();
			Set_Wheel_Speed(0,RUN_SPEED_15);
			delay(10000);
			Time_Counter=40;
			Current_Current=0;
			
		  while(Time_Counter--)
			{
			  delay(50);
				Current_Current += g_adc_value.Right_Wheel_Current;
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}
					
      if(Status_Flag!=9)break;
			Test_PWM = TIMER_CH2CV(TIMER0);
			Current_Current/=40;
			Current_Current-=	g_rightwheel_baseline;
			Current_Current *= 5;
			Current_Current = Current_Current*ReferenceVoltage/4096;
			if((Current_Current>70)||(Current_Current<20))//current fail
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Right W C2=:%d\n",Current_Current);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0004;
        delay(2000);
      }
      //check pwm 
			if(((Test_PWM<50)||(Test_PWM>90)))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Right W P2=:%d\n",Test_PWM);	
        delay(7000);
			}
      else
      {
        Test_Result|=0x0008;
      }
      if(Test_Result==0x000f)
      {
        Status_Flag=Switch_Step(Status_Flag,1);
        break;
      }
		}
		
    /*-------------------------------------------------------------Test Right wheel stall sensor-----------11111111111111111111---*/
    while(Status_Flag == 10)
    {				
      Move_Forward(0,RUN_SPEED_17);
      while(1)
			{
        if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			  if(g_adc_value.Right_Wheel_Current>800)
				{
				  delay(100);
					if(g_adc_value.Right_Wheel_Current>800)
				  {
					  Current_Current=(g_adc_value.Right_Wheel_Current-g_rightwheel_baseline)*ReferenceVoltage*5/4096;
					  if(Current_Current<500)
						{
							Debug_Print("#Fail",Dev_USART0);
							USPRINTF("#Right Stall=:%d\n",Current_Current);	
              delay(5000);
						}
            else
            {
              delay(2000);
              Status_Flag=Switch_Step(Status_Flag,1);
              break;
            }
					}
				}
        delay(100);
			}
      break;
    }
    
		/*-------------------------------------------------------Test Right Brush Sensors--------------------------------------*/
    Test_Result=0;
		while(Status_Flag == 11)
		{				
	    Test_Result=0;
			Temp_PWM=(uint8_t)((uint32_t)Clean_SideBrush_Power/(uint32_t)GetBatteryVoltage());
      Set_SideBrush_PWM(0,Temp_PWM);
			delay(10000);
			Time_Counter=40;
			Current_Current=0;
		  while(Time_Counter--)
			{
			  Temp_PWM=(uint8_t)((uint32_t)Clean_SideBrush_Power/(uint32_t)GetBatteryVoltage());
        Set_SideBrush_PWM(0,Temp_PWM);
			  delay(100);
				Current_Current += g_adc_value.Right_Brush_Current;
				if(Remote_Key(Remote_Max))
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}
      if(Status_Flag != 11)break;
			Current_Current/=40;
			Current_Current-=	g_rightbrush_baseline;
			Current_Current *= 1;
			Current_Current = Current_Current*ReferenceVoltage/4096;	
			if((Current_Current>30)||(Current_Current<6))				
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#RBrush C=:%d\n",Current_Current);	
        delay(5000);
			}
      else
      {
        Test_Result=0x0001;
				Debug_Print("@Pass",Dev_USART0);
        delay(2000);
      }
			
      Speaker(SPK_DON);
			while(1)
			{
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			  if(g_adc_value.Right_Brush_Current>600)
				{
				  delay(100);
					if(g_adc_value.Right_Brush_Current>600)
				  {
					  Current_Current=(g_adc_value.Right_Brush_Current-g_rightbrush_baseline)*ReferenceVoltage*1/4096;
					  if(Current_Current<70)
						{
							Debug_Print("#Fail",Dev_USART0);
							USPRINTF("#RBrushStall=:%d\n",Current_Current);	
              delay(5000);
						}
            else
            {
              delay(2000);
              Status_Flag=Switch_Step(Status_Flag,1);
              break;
            }
					}
				}
        delay(100);				
			}
			break;		
		}
		Set_SideBrush_PWM(0,0);
		
		/*------------------------------------------------Test Vacuum Sensors------------------------------*/
    Test_Result=0;
		while(Status_Flag == 12)
		{			
      Set_BLDC_Speed(1200);
			BLDC_ON;
			Set_BLDC_TPWM(40);
			delay(30000);
      while(1)
      {
  			Time_Counter=20;
  			Current_Current=0;
  		  while(Time_Counter--)
  			{
  			  delay(100);
  				Current_Current += g_adc_value.Vacuum_Current;
  			}
        if(Status_Flag!=12)break;
  			Current_Current/=20;
				Current_Current -=	g_vac_baseline;
				Current_Current *= 10;
				Current_Current = Current_Current*ReferenceVoltage/4096;
				
  			if((Current_Current>1500)||(Current_Current<800))
  			{
					Debug_Print("#Fail",Dev_USART0);
					USPRINTF("#Vac C=:%d\n",Current_Current);	
          Test_Result=0;
  			}
        else
        {
          Test_Result++;
        }
				Test_PWM =	BLDC_PWM;
				if((Test_PWM<30)||(Test_PWM>90))
				{
					Debug_Print("#Fail",Dev_USART0);
					USPRINTF("#Vac P=:%d\n",Test_PWM);	
          Test_Result=0;
				}

				while(Test_Result>0)
				{					
					if(Key_GetStatus()==KEY_CLEAN)
					{
						Status_Flag=Switch_Step(Status_Flag,1);
						break;
					}
				}
        if(Test_Result>1)
        {					
					break;
        }
      }
      break;
		}
		BLDC_OFF;
		
		/*------------------------------------------------Test Main Brush-----------------------------*/
		Test_Result=0;
		Set_Main_Brush(ENABLE);
		while(Status_Flag == 13)
		{
	    Test_Result=0;
			Temp_PWM=(uint8_t)((uint32_t)Clean_MainBrush_Power/(uint32_t)GetBatteryVoltage());
      Set_MainBrush_PWM(Temp_PWM);
			delay(10000);
			Time_Counter=20;
			Current_Current=0;
		  while(Time_Counter--)
			{
			  Temp_PWM=(uint8_t)((uint32_t)Clean_MainBrush_Power/(uint32_t)GetBatteryVoltage());
        Set_MainBrush_PWM(Temp_PWM);
			  delay(100);
				Current_Current += g_adc_value.Main_Brush_Current;
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}
      if(Status_Flag != 13)break;
			Current_Current/=20;
			Current_Current -=	g_mainbrush_baseline;
			Current_Current *= 5;
			Current_Current = Current_Current*ReferenceVoltage/4096;
			if((Current_Current>350)||(Current_Current<100))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#MB C=:%d\n",Current_Current);	
        delay(5000);
			}
      else
      {
        Test_Result=0x0001;
				Debug_Print("@Pass",Dev_USART0);
        delay(2000);
      }
			
			Speaker(SPK_DON);
			while(1)
			{
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
				if(g_adc_value.Main_Brush_Current > 800)
				{
				  delay(50);
					if(g_adc_value.Main_Brush_Current > 800)
					{
				    Current_Current = (g_adc_value.Main_Brush_Current-g_mainbrush_baseline)*ReferenceVoltage*5/4096;
				    if(Current_Current<500)
					  {
							Debug_Print("#Fail",Dev_USART0);
							USPRINTF("#MB Stall=:%d\n",Current_Current);	
              delay(5000);
						}
            else
            {
              delay(2000);
              if(Test_Result)
              {
                Status_Flag=Switch_Step(Status_Flag,1);
                break;
              }
            }
					}
				}
        delay(100);
			}
			break;		
		}
		Set_MainBrush_PWM(0);
		
		/*------------------------------------------------Charger-----------------------------*/
		Retest_Flag=0;
		while(Status_Flag == 14)
		{
		  Current_Current=0;
		  while(1)	// Wait for charger plugin
			{
			  delay(500);
				Debug_Print("Wait for charger plugin",Dev_USART0);
				if(Is_ChargerOn())break;
        if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}

      if(Status_Flag!=14)break;
			Charge_Configuration();
			Charge_PWM=1;			
			Temp_PWM=300;
			Current_Current=0;
			Charge_Time=0;
			blink=0;
			while(1)
			{
			  delay(20);
				Charge_Time++;
				if(Charge_Time>600)break;
				if(g_adc_value.System_Current<g_baselineadc)//Check if on Charging 
				{
				  Current_Current=Get_Charge_Current(g_baselineadc);
				}
				else Current_Current=0;
				
        if(Current_Current>300)//charge current over
  			{
					if(Current_Current>400)
					{
						if(Temp_PWM<10)Temp_PWM=10;
						Temp_PWM-=20;
					}
					else
					{
						if(Temp_PWM<2)Temp_PWM=2;
						Temp_PWM-=2;
					}
  			}
  			else
  			{
          if(Current_Current<280)
          {
            Temp_PWM+=30;
          }
          else if(Current_Current<290)
          {
            Temp_PWM+=10;
          }
          else
          {
            Temp_PWM++;
          }
  				if(Temp_PWM>1000)Temp_PWM=1000;
  			}
        Charge_PWM = Temp_PWM;
				
									
				if(Charge_Time%30==0)
				{
					USPRINTF("Time:%d\n", Charge_Time);
					USPRINTF("I_AD:%d\n", g_adc_value.System_Current);
					USPRINTF("I:%d\n", Current_Current);				
					USPRINTF("P:%d\n", Charge_PWM);						
					blink=1-blink;
					if(!blink)
					{
						blinkplus++;
						if(blinkplus>3)blinkplus=0;
						if(blinkplus==0)Set_LED_On_Switch(0,0,0,0,0,0);
						else if(blinkplus==1)Set_LED_On_Switch(1,0,0,0,0,0);
						else if(blinkplus==2)Set_LED_On_Switch(1,1,0,0,0,0);
						else if(blinkplus==3)Set_LED_On_Switch(1,1,1,0,0,0);
						Set_LED(100,100,100,100,100,100);
					}				
				}
				
				if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
			}
			if(Status_Flag!=14)break;
			Set_LED_On_Blink(0,0,0,0,0,0);
			Set_LED_On_Switch(1,0,1,0,0,0);
			Charge_PWM=0;
			
			if((Temp_PWM<250)||(Temp_PWM>850))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#ChargePWM=:%d\n",Temp_PWM);	
				Set_LED_On_Blink(1,1,1,0,0,0);
        while(1)
				{
					if(Get_Key_Time(KEY_CLEAN)>100)
					{
						Status_Flag=14; 
						Retest_Flag=1;
						break;
					}
				}
			}
      else
      {
        delay(2000);
      }
			if(Retest_Flag)break;

			if((Current_Current<230)||(Current_Current>390))
			{
				Debug_Print("#Fail",Dev_USART0);
				USPRINTF("#Charge C=:%d\n",Current_Current);	
				Set_LED_On_Blink(1,1,1,0,0,0);
			  while(1)
				{
					if(Get_Key_Time(KEY_CLEAN)>100)
					{
						Status_Flag=14; 
						Retest_Flag=1;
						break;
					}
				}
			}
			if(Retest_Flag)break;
      Debug_Print("@Pass",Dev_USART0);			
			while(1)
			{
			  delay(500);
        if(!Is_ChargerOn())break;
			}
			
			Debug_Print("@Pass",Dev_USART0);
			Set_LED_On_Switch(1,1,1,1,1,1);
      Set_LED(100,100,100,100,100,100);			
      while(1)
      {				
        if(Remote_Key(Remote_Max))//back to previous test
				{
					Status_Flag=Switch_Step(Status_Flag,0);
					break;
				}
				if(Get_Key_Time(KEY_CLEAN)>100)
				{
					Status_Flag=14;
					Retest_Flag=1;
					break;
				}				
      }
      break;
		}
		while(1); 
	}
}


/*------------------------------------------------------------Switch Steps---------------*/
uint8_t Switch_Step(uint8_t Step,uint8_t dir)
{
  uint8_t temp_step=0;
  if(dir)
	{
	  temp_step=Step+1;
		if(temp_step > MAX_STEP)temp_step=MIN_STEP;
	}
	else
	{
	  temp_step=Step-1;
		if(temp_step < MIN_STEP)temp_step=MAX_STEP;
	}
	Speaker(SPK_DON);
	Display_Clean_Status(Display_Test);
	Debug_Print("@PASS",Dev_USART0);
	USPRINTF("Step: %d\n",temp_step);		
	Disable_Motors();
	delay(1000);
	return temp_step;
}
/*------------------------------------------------------------Get_Current_Voltage---------------*/
uint32_t Get_Current_Voltage(void)
{
	uint32_t SystemVoltageSum=0;
  uint8_t temp=0;
  for(temp=0;temp<10;temp++)
  {
    delay(10);
    SystemVoltageSum+=g_adc_value.System_Current;
  }
	SystemVoltageSum/=10;
	SystemVoltageSum=SystemVoltageSum*ReferenceVoltage/4096;
	return SystemVoltageSum;
}
uint32_t Get_Current_Adc(void)
{
	uint32_t SystemVoltageSum=0;
  uint8_t temp=0;
  for(temp=0;temp<10;temp++)
  {
    delay(10);
    SystemVoltageSum+=g_adc_value.System_Current;
  }
	SystemVoltageSum/=10;
	return SystemVoltageSum;
}
/*System Tick Configuration ----------------------------------------*/
void Motor_Test_Set(void)
{
	LW_DIR_BACKWARD();
	RW_DIR_BACKWARD();
	Set_Wheel_Speed(0,0);
	Set_SideBrush_PWM(0,0);
	Set_MainBrush_PWM(0);
	BLDC_OFF;
	delay(3000);
}

 
