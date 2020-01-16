/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   UserInterface Fuction
	           Display Button lights and waiting for user to select cleaning mode
						 Plan setting , set hours and minutes
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Display.h"
#include "SysInitialize.h"
#include "Speaker.h"
#include "Movement.h"
#include "UserInterface.h"
#include "TouchPad.h"
#include "Charge.h"
#include "I2C.h"
#include "Rcon.h"
#include "RTC.h"
#include "USART.h"
#include "Home.h"
#include "CorMove.h"
#include "IR.h"
#include "gyro.h"

/*------------------------------------------------------------User Interface ----------------------------------*/
void User_Interface(void)
{
  static volatile uint16_t Press_time=0;
	static volatile uint8_t Temp_Mode=0,Press_time2=0;
	static volatile uint16_t Error_Show_Counter=400;
	static volatile uint16_t TimeOutCounter=0;
	
	uint16_t Temp_Battery_Voltage=1600;
	uint8_t BTA_Power_Dis=0;
	
#ifdef STANDARD_DISPLAY
	uint8_t LED_Add=0;
	uint8_t LED_Brightness=0;
	uint8_t Display_Counter=0;
#endif

#ifdef ONE_KEY_DISPLAY
  uint8_t ONE_Display_Counter=60;
#endif

	Press_time=0;
	Temp_Mode=0;
	Error_Show_Counter=400;
	TimeOutCounter=0;

  Disable_Motors();
  Display_Clean_Status(Display_Userinterface);	
	Beep(3);
	delay(1000);

	Reset_Encoder_Error();

  Reset_Rcon_Remote();

	USART_Print("\n\rUser Interface! ",Dev_USART3);

	Reset_Rcon_Status();
	Clear_Clcok_Receive();
	Set_Room_Mode(Room_Mode_Large);
	ResetHomeRemote();
	Set_VacMode(Vac_Normal);

	Store_Motor_BaseLine();
	
	while(1)
	{
		if(Into_Test_Mode())
		{
			Set_Clean_Mode(Clean_Mode_Test);
			return;
		}
				
		if(Get_Rcon_Remote()==Remote_Forward||Get_Rcon_Remote()==Remote_Right
		    ||Get_Rcon_Remote()==Remote_Left||Get_Rcon_Remote()==Remote_Max)
	  {
			Set_Clean_Mode(Clean_Mode_Remote);
			return;
	  }


		#ifdef SCREEN_REMOTE
		if(Remote_Clock_Received())
		{
			Set_Remote_Schedule();
		}
		#endif
		/*--------------------------------------------------------Check if on the station--------------*/
		if(Is_ChargerOn())
		{
		  if(Get_CleanKeyDelay()<1)
	  	{
			  Set_Clean_Mode(Clean_Mode_Charging);
				break;
			}
		}
		else
		{
			/* -----------------------------Check if spot event ----------------------------------*/
			if(Remote_Key(Remote_Spot))
			{
			  Set_MoveWithRemote();
				Temp_Mode=Clean_Mode_Spot;
			}

			/* -----------------------------Check if Home event ----------------------------------*/
			if(Remote_Key(Remote_Home)) 
			{
			  Display_Content(LED_Home,100,100,0,7);
				Temp_Mode=Clean_Mode_GoHome;
				Reset_MoveWithRemote();
				SetHomeRemote();
			}
			/* -----------------------------Check if wall follow event ----------------------------------*/
			if(Remote_Key(Remote_Random))
			{
//			  Set_MoveWithRemote();
//				Temp_Mode=Clean_Mode_RandomMode;
			  Set_MoveWithRemote();
				Temp_Mode=Clean_Mode_Spot;				
			}
		}
		/* -----------------------------Check if Clean event ----------------------------------*/
		if(Is_Alarm())
		{
			Reset_Alarm();
			if(Get_AlarmSet_Minutes()==Get_Time_Minutes())
			{
				Temp_Mode=Clean_Mode_Navigation;
				Reset_MoveWithRemote();
			}
		}
		if(Remote_Key(Remote_Clean))
		{
			Reset_Rcon_Remote();
			Set_Room_Mode(Room_Mode_Large);
			Press_time=10;
			while(Press_time--)
			{
				/*if(Remote_Key(Remote_Clean))
				{
					Set_Room_Mode(Room_Mode_Auto);
					break;
				}*/
				delay(500);
			}
      Temp_Mode=Clean_Mode_Navigation;
			Reset_Rcon_Remote();
			Reset_MoveWithRemote();
		}
		if(Get_Key_Press()==KEY_CLEAN)
		{
			Beep(2);
			Set_LED(100,100,100,100,100,100);
			while(Get_Key_Press()==KEY_CLEAN)
			{
				if(Remote_Key(Remote_Forward))
				{
					Set_Clean_Mode(Clean_Mode_Test);
					Display_Clean_Status(Display_Test);
					return;
				}
				Press_time++;
				if(Press_time>100)
				{
					Press_time=0;
					Press_time2++;
					if(Press_time2>3)
					{
						Press_time2=0;
						Set_Clean_Mode(Clean_Mode_Sleep);
						Beep(3);
						Beep(6);							
						return;
					}
				}
			}			

			Set_Room_Mode(Room_Mode_Large);
			Press_time=5;
			while(Press_time--)
			{
				if(Get_Key_Press()==KEY_CLEAN)
				{
					Beep(2);
					/*Set_Room_Mode(Room_Mode_Auto);*/
					break;
				}
				delay(500);
			}
			Temp_Mode=Clean_Mode_Navigation;
			Reset_WorkTimer();
			
			Reset_MoveWithRemote();
			Reset_Error_Code();
		}
		if(Get_Key_Press()==KEY_SPOT)
		{
			Beep(2);
			Set_LED(100,100,100,100,100,100);
			Set_MoveWithRemote();
			Temp_Mode=Clean_Mode_Spot;		
		}
		if(Get_Key_Press()==KEY_HOME)
		{
			while(Get_Key_Press());
			Beep(2);		
			Display_Content(LED_Home,100,100,0,7);
			Temp_Mode=Clean_Mode_GoHome;
			Reset_MoveWithRemote();
			SetHomeRemote();
		}
		/* ----------------------------- ----------------------------------*/
		if(Temp_Mode)
		{
      Reset_Error_Code();
		  if(Is_ChargerOn())
			{
			  if(!Is_AtHomeBase())
				{
				  Temp_Mode=0;
				}
			}
			if((Temp_Mode==Clean_Mode_GoHome)||(Temp_Mode==Clean_Mode_Sleep))
			{
				Reset_Bumper_Error();
				Reset_Error_Code();
				Set_Clean_Mode(Temp_Mode);
				Set_CleanKeyDelay(0);
				return;
			}
			if((Temp_Mode==Clean_Mode_WallFollow)||(Temp_Mode==Clean_Mode_Spot)||(Temp_Mode==Clean_Mode_RandomMode)||(Temp_Mode==Clean_Mode_Navigation))
			{
				if(Get_Cliff_Trig()==(Status_Cliff_All))
				{
					Set_Error_Code(Error_Code_PickUp);
					Error_Show_Counter=400;
					Temp_Mode=0;
				}
				else if(GetBatteryVoltage() < 1300)
				{
					Set_LED_On_Switch(0,0,0,0,0,0);
					Set_LED_On_Blink(0,0,0,0,1,0);
					Temp_Mode=0;
				}
				else
				{
					Reset_Error_Code();					
					if(Temp_Mode==Clean_Mode_WallFollow)
					{
						Display_Clean_Status(Display_Wall);
					}
					else if(Temp_Mode==Clean_Mode_Spot)
					{
						Display_Clean_Status(Display_Spot);
						Speaker(AREA_CLEANING_START);
						delay(10000);								
					}
					else if(Temp_Mode==Clean_Mode_RandomMode)
					{
						Display_Clean_Status(Display_Clean);
					}
					else if(Temp_Mode==Clean_Mode_Navigation)
					{
						Display_Clean_Status(Display_Zizag);
						Speaker(ENTER_GLOBAL_SWEEP_MODE);
						delay(10000);								
					}
					
					if(Get_Room_Mode()==Room_Mode_Auto)
					{
						Beep(1);
						Beep(3);
						Beep(5);
					}
					else if(Get_Room_Mode()==Clean_Mode_Navigation)
					{				
						Beep(5);
						Beep(3);
					}
					else
					{						
						Beep(5);
						Beep(3);
					}
					if(!Is_ChargerOn()&&(Temp_Mode!=Clean_Mode_Navigation))Initialize_Motor();
					Set_Clean_Mode(Temp_Mode);
					Set_CleanKeyDelay(0);
					Reset_Rcon_Remote();
					Reset_Bumper_Error();
					return;
				}
			}
			Disable_Motors();
			Temp_Mode=0;
		}

		Error_Show_Counter++;
	  if(Error_Show_Counter>500)
	  {
			Error_Show_Counter=0;
			if(Get_Error_Code())
			{
				Sound_Out_Error(Get_Error_Code());				
				IR_Transmite_Error(0,Get_Error_Code(),0);
			}
		}

		#ifdef ONE_KEY_DISPLAY
		delay(80);
		
		ONE_Display_Counter++;
		if(ONE_Display_Counter>99)
		{
			ONE_Display_Counter=0;
			TimeOutCounter++;
			if(TimeOutCounter>16)//15=45se 
			{
				TimeOutCounter=0;
				Set_Clean_Mode(Clean_Mode_Sleep);
				Beep(3);
				Beep(6);					
				break;
			}
			if(TimeOutCounter>0)//on base but miss charging , adjust position to charge
			{
				if(Is_Base_C())
				{
					Reset_Base_C();
					if(Get_Rcon_Status()&Rcon_Signal_ALL_L_R)
					{
						Reset_Rcon_Status();
						if(Get_Cliff_Trig()==0)
						{
							Set_LED(100,100,100,100,100,100);
							if(Turn_Connect())
							{
								Set_Clean_Mode(Clean_Mode_Charging);
								break;
							}
							Disable_Motors();
						}

					}
				}
			}
		}
		
		Temp_Battery_Voltage = GetBatteryVoltage();		
		if(Temp_Battery_Voltage<1350)
		{
			BTA_Power_Dis=1;
		}
		if(Temp_Battery_Voltage>1380)
		{
			BTA_Power_Dis=0;
		}
		
		if(Get_Error_Code())
		{
			Set_LED_On_Blink(1,1,1,0,0,0);
			Set_LED_On_Switch(0,0,0,1,0,0);
		}
		else if(BTA_Power_Dis)
		{
			Set_LED_On_Switch(1,1,1,0,0,0);
			Set_LED_On_Blink(0,0,0,0,1,0);
		}
		else
		{
			Set_LED_On_Switch(1,1,1,1,0,0);
			Set_LED_On_Blink(0,0,0,0,0,0);
		}
		Set_LED_Table(ONE_Display_Counter,ONE_Display_Counter,ONE_Display_Counter,60,ONE_Display_Counter,ONE_Display_Counter);
		#endif

 #ifdef STANDARD_DISPLAY
		{
			Display_Counter++;
			if(Display_Counter>0x000a)
		  {
			  if(Temp_Charge_CleangKey_Delay<1)Temp_Charge_CleangKey_Delay=1;
			  Temp_Charge_CleangKey_Delay--;
				Set_CleanKeyDelay(Temp_Charge_CleangKey_Delay);
			  TimeOutCounter++;
				if(TimeOutCounter>160)
				{
			  	TimeOutCounter=0;
				  Set_Clean_Mode(Clean_Mode_Sleep);
				  break;
				}
				Display_Counter=0;
			  if(flag)
				{
				  LED_Add++;
					if(LED_Add>1)
					{
					  LED_Add=0;
					  LED_Brightness++;
						if(LED_Brightness>=7)flag=0;
					}
				}
				else
				{
					if(LED_Brightness>0)
					{
					  LED_Brightness--;
					}
					if(LED_Brightness==0)
					{
					  LED_Add++;
						if(LED_Add>1)
						{
						  LED_Add=0;
					    flag=1;
						}
					}
				}
				if(Get_Error_Code())
				{
				  if((LED_Brightness==0)&&(LED_Add==0))Display_Content(LED_Exclamation,LED_Error,100,0,LED_Brightness);
				  else Display_Content(LED_Spot|LED_Plan|LED_Clean|LED_Home|LED_Clock|LED_Exclamation,LED_Error,Get_Error_Code(),0,LED_Brightness);					
				}
				else
				{
          if(GetBatteryVoltage() < 1350)
          {
            Display_Low_BTA(LED_Brightness);
          }
          else
          {
  				  if((LED_Brightness==0)&&(LED_Add==0))Display_Content(0,100,100,0,LED_Brightness);
  					else Display_Content(LED_Spot|LED_Plan|LED_Clean|LED_Home|LED_Clock,100,100,0,LED_Brightness);
          }
				}
			}
		}
		#endif
	}
	Set_CleanKeyDelay(0);
}




