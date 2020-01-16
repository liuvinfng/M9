 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Test mode for customer to check the robot's seperated funtion/module
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "TestMode.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "I2C.h"
#include "UserInterface.h"
#include "Rcon.h"
#include "RTC.h"
#include "FixtureTestMode.h"
//#include "RTC.h"

extern volatile OBS_ADC OBS_Status;
extern volatile ADC_Value_Struct ADC_Value;
extern volatile Cliff_ADC Cliff_Status;
extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;


//extern volatile uint32_t Schedule_Minutes;



/*----------------------------------------------------------- Spining Event -----------------------*/
void Fixture_Test(uint32_t BaseCurrent)
{
  uint16_t Test_Result=0;
  uint8_t Temp_Display=0;
  int32_t Current_Current=0;
  uint32_t Charge_Time=0;
  uint16_t Temp_PWM=0;
  uint32_t Time_Counter=0;
  uint32_t Dirt_Counter=0;
//  uint32_t Temp_Count=0;
  uint32_t Temp_Rcon_Status=0;
  uint16_t Test_PWM=0;
 /*---------***************************---------Menul Test ---------******************************************************--------*/
	while(1)
	{
    /*------------------------------------------Test Display---------------------111111111111111111------------*/
		/*------------------------------------------Test Keys--------------------------222222222222222222222-------*/
    Sound(1);
    USART_DMA_String(2,"@01");
    Test_Result=0;
		while(1)//------------------------------------------Test Keys
		{
		  if(Is_Key_Press(KEY_PLAN))
			{
			  //Display_Content(LED_Plan,100,1,2,7);
				Test_Result|=0x0001;
			}
			else Test_Result|=0x0010;

		  if(Is_Key_Press(KEY_SPOT))
			{
			  //Display_Content(LED_Spot,100,1,2,7);
				Test_Result|=0x0002;
			}
			else Test_Result|=0x0020;

			if(Is_Key_Press(KEY_HOME))
			{
			  //Display_Content(LED_Home,100,1,2,7);
				Test_Result|=0x0004;
			}
			else Test_Result|=0x0040;

      if(Is_Key_Press(KEY_CLOCK))
			{
			  //Display_Content(LED_Clock,100,1,2,7);
				Test_Result|=0x0100;
			}
			else Test_Result|=0x0200;

      if(Is_Key_Press(KEY_CLEAN))
			{
				//Display_Content(LED_Clean,100,1,2,7);
				Test_Result|=0x0008;
        if(Test_Result==0x03ff)
        {
          USART_DMA_String(5,"@PASS");
          break;
        }
        else
        {
          Beep(6);
          //Display_Content(LED_Clean|LED_Exclamation,100,1,2,7);
          USART_DMA_String(5,"#Fail");
          if((Test_Result&0x0011)!=0x0011)USART_DMA_String(6,"#PLAN ");
          if((Test_Result&0x0022)!=0x0022)USART_DMA_String(6,"#SPOT ");
          if((Test_Result&0x0044)!=0x0044)USART_DMA_String(6,"#HOME ");
          if((Test_Result&0x0300)!=0x0300)USART_DMA_String(6,"#CLOCK");
          if((Test_Result&0x0088)!=0x0088)USART_DMA_String(6,"#CLEAN");
          while(1);
        }
			}
			else Test_Result|=0x0080;

      //Display_Content(LED_Plan|LED_Spot|LED_Clean|LED_Home|LED_Clock,100,1,2,2);
		}
  	/*------------------------------------------Test OBS--------------------------3333333333333333333333-------*/
    Sound(2);
    USART_DMA_String(2,"@02");
	  Test_Result=0;
	  while(1)
		{
			delay(100);
			//Display_Content(Temp_Display,OBS_Status.Left_Wall/100,OBS_Status.Left_Wall%100,3,7);

			if(OBS_Status.Left_Wall>2000)
			{
			  Test_Result|=0x0040;  
        Temp_Display|=LED_Plan;
			}
			if(OBS_Status.Left_Wall<150)
			{
			  Test_Result|=0x0080;
        Temp_Display&=~LED_Plan;
			}

		  if(OBS_Status.Left_OBS  >500)
			{
			  Test_Result|=0x0001;
			  Temp_Display|=LED_Spot;
			}
			else if(OBS_Status.Left_OBS  <50)
			{
			  Test_Result|=0x0002;
			  Temp_Display&=~LED_Spot;
			}

		  if(OBS_Status.Front_OBS >500)
			{
			  Test_Result|=0x0004;
			  Temp_Display|=LED_Clean;
			}
			else if(OBS_Status.Front_OBS <50)
			{
			  Test_Result|=0x0008;
				Temp_Display&=~LED_Clean;
			}

			if(OBS_Status.Right_OBS >500)
			{
			  Test_Result|=0x0010;
				Temp_Display|=LED_Home;
			}
			else if(OBS_Status.Right_OBS <50) 
			{
			  Test_Result|=0x0020;
				Temp_Display&=~LED_Home;
			}

			if(Is_Key_Press(KEY_CLEAN))
			{
        if(Test_Result==0x00ff)
        {
          USART_DMA_String(5,"@PASS");
          break;
        }
        else
        {
          Beep(6);
          //Display_Content(Temp_Display|LED_Exclamation,OBS_Status.Left_Wall/100,OBS_Status.Left_Wall%100,3,7);
          USART_DMA_String(5,"#Fail");
          if((Test_Result&0x0003)!=0x0003)USART_DMA_String(6,"#LOBS ");
          if((Test_Result&0x000c)!=0x000c)USART_DMA_String(6,"#FOBS ");
          if((Test_Result&0x0030)!=0x0030)USART_DMA_String(6,"#ROBS ");
          if((Test_Result&0x00c0)!=0x00c0)USART_DMA_String(6,"#LWALL");
          while(1);
        }
			}
		}
		/*------------------------------------------Test Switches-------------------4444444444444444--------------*/
    Sound(3);
    USART_DMA_String(2,"@03");
	  Test_Result=0;
		while(1)
		{
		  delay(100);
		  if(Get_Bumper_Status()&LeftBumperTrig)
			{
			  Test_Result|=0x01;
				Temp_Display|=LED_Plan;
			}
			else 
			{
				Test_Result|=0x02;
				Temp_Display&=~LED_Plan;
			}

			if(Get_Bumper_Status()&RightBumperTrig)
			{
				Test_Result|=0x04;
				Temp_Display|=LED_Clock;
			}
			else 
			{
				Test_Result|=0x08;
				Temp_Display&=~LED_Clock;
			}

			if(Is_Key_Press(KEY_CLEAN))
			{
        if(Test_Result==0x00ff)
        {
          USART_DMA_String(5,"@PASS");
          break;
        }
        else
        {
          Beep(6);
          //Display_Content(Temp_Display|LED_Exclamation,100,3,4,7);
          USART_DMA_String(5,"#Fail");
          if((Test_Result&0x0003)!=0x0003)USART_DMA_String(8,"#LBUMPER");
          if((Test_Result&0x000c)!=0x000c)USART_DMA_String(8,"#RBUMPER");
          if((Test_Result&0x0030)!=0x0030)USART_DMA_String(8,"#LDROP  ");
          if((Test_Result&0x00c0)!=0x00c0)USART_DMA_String(8,"#RDROP  ");
          while(1);
        }
			}
	    else
      {
			  //Display_Content(Temp_Display,100,3,4,7);
      }
		}
		/*------------------------------------------Test Cliff------------------------55555555555555555---------*/
    Sound(4);
    USART_DMA_String(2,"@04");
	  Test_Result=0;
		while(1)
		{
		  delay(100);
      /*--------------Left Cliff-----------------------*/
      if(Cliff_Status.Left_Cliff<Cliff_Limit)
    	{
    	  delay(50);
    	  if(Cliff_Status.Left_Cliff<Cliff_Limit)
    	  {
          Test_Result|=0x0001;
          Temp_Display|=LED_Plan;
    		}
    	}
      if(Cliff_Status.Left_Cliff>1000)
      {
        delay(50);
        if(Cliff_Status.Left_Cliff>1000)
        {
          Test_Result|=0x0002;
          Temp_Display&=~LED_Plan;
        }
      }
      /*--------------Right Cliff-----------------------*/
    	if(Cliff_Status.Right_Cliff<Cliff_Limit)
    	{
    	  delay(50);
    	  if(Cliff_Status.Right_Cliff<Cliff_Limit)
    	  {
          Test_Result|=0x0004;
          Temp_Display|=LED_Clock;
    		}
     	}
      if(Cliff_Status.Right_Cliff>1000)
    	{
    	  delay(50);
    	  if(Cliff_Status.Right_Cliff>1000)
    	  {
          Test_Result|=0x0008;
          Temp_Display&=~LED_Clock;
    		}
     	}
      /*--------------Front Cliff-----------------------*/
    	if(Cliff_Status.Front_Cliff<Cliff_Limit)
    	{
    	  delay(50);
    	  if(Cliff_Status.Front_Cliff<Cliff_Limit)
    	  {
          Test_Result|=0x0040;
          Temp_Display|=LED_Home;
    		}
    	}
      if(Cliff_Status.Front_Cliff>1000)
    	{
    	  delay(50);
    	  if(Cliff_Status.Front_Cliff>1000)
    	  {
          Test_Result|=0x0080;
          Temp_Display&=~LED_Home;
    		}
    	}

			if(Is_Key_Press(KEY_CLEAN))
			{
        if(Test_Result==0x00ff)
        {
          USART_DMA_String(5,"@PASS");
          break;
        }
        else
        {
          Beep(6);
          //Display_Content(Temp_Display|LED_Exclamation,100,4,5,7);
          USART_DMA_String(5,"#Fail");
          if((Test_Result&0x0003)!=0x0003)USART_DMA_String(8,"#LCLIFF ");
          if((Test_Result&0x000c)!=0x000c)USART_DMA_String(8,"#RCLIFF ");
          if((Test_Result&0x0030)!=0x0030)USART_DMA_String(8,"#FCLIFF1");
          if((Test_Result&0x00c0)!=0x00c0)USART_DMA_String(8,"#FCLIFF2");
          while(1);
        }
			}
      else
      {
			  //Display_Content(Temp_Display,100,4,5,7);
      }
		}

		/*------------------------------------------Test Rcon------------------------66666666666666666---------*/
    Sound(5);
    USART_DMA_String(2,"@05");
	  Test_Result=0;
		while(1)
		{
		  delay(1500);
      Temp_Rcon_Status = Get_Rcon_Status();

      if(Temp_Rcon_Status)Reset_Rcon_Status();

      if(Temp_Rcon_Status&0x0003)//right
	    {
				Test_Result|=0x0001;
        Temp_Display|=LED_Clock;
	    }
			else	
			{
			  Test_Result|=0x0002;
        Temp_Display &=~ LED_Clock;
			}

	    if(Temp_Rcon_Status&0x000c)// front right
	    {
				Test_Result|=0x0004;
        Temp_Display |= LED_Home;
	    }
			else	
			{
			  Test_Result|=0x0008;
        Temp_Display &=~ LED_Home;
			}

      if(Temp_Rcon_Status&0x0030)// front Left
	    {
				Test_Result|=0x0010;
        Temp_Display|=LED_Spot;
	    }
			else	
			{
			  Test_Result|=0x0020;
        Temp_Display &=~ LED_Spot;
			}

      if(Temp_Rcon_Status&0x00c0)// Left
	    {
				Test_Result|=0x0040;
        Temp_Display|=LED_Plan;
	    }
			else	
			{
			  Test_Result|=0x0080;
        Temp_Display&=~LED_Plan;
			}

      if(Temp_Rcon_Status&0x0F00)//Charge Top
	    {
//        Temp_Count = 8;
	    }
			else	
			{
//        Temp_Count = 0;
			}

      if(Temp_Rcon_Status&0x3000)// Back
	    {
        Temp_Display|=LED_Clean;
        Test_Result|=0x0100;
	    }
			else	
			{
        Temp_Display&=~LED_Clean;
        Test_Result|=0x0200;
			}

      //Display_Content(Temp_Display,100,5,Temp_Count,3);

			if(Is_Key_Press(KEY_CLEAN))
			{
        if(Test_Result==0x03ff)
        {
          USART_DMA_String(5,"@PASS");
          break;
        }
        else
        {
          Beep(6);
          //Display_Content(Temp_Display|LED_Exclamation,100,5,Temp_Count,3);
          USART_DMA_String(5,"#Fail");
          if((Test_Result&0x0003)!=0x0003)USART_DMA_String(7,"#RRCON ");
          if((Test_Result&0x000c)!=0x000c)USART_DMA_String(7,"#FRRCON");
          if((Test_Result&0x0030)!=0x0030)USART_DMA_String(7,"#FLRCON");
          if((Test_Result&0x00c0)!=0x00c0)USART_DMA_String(7,"#LRCON ");
          if((Test_Result&0x0300)!=0x0300)USART_DMA_String(7,"#BRCON ");
          while(1);
        }
			}
		}

		/*-------------------------------------------------------------Test Mobility Sensors-----------77777777777777777777777---*/
    Sound(6);
    USART_DMA_String(2,"@06");
		Dirt_Counter=0;
    Reset_Mobility_Step();
		while(1)
		{
		  delay(100);
      Dirt_Counter = Get_Mobility_Step();
      //Display_Content(0,Dirt_Counter/100,Dirt_Counter%100,0,2);
			if(Is_Key_Press(KEY_CLEAN))
			{
        if(Dirt_Counter>2)
        {
          USART_DMA_String(5,"@PASS");
          break;
        }
        else
        {
          Beep(6);
          //Display_Content(LED_Exclamation,Dirt_Counter/100,Dirt_Counter%100,0,2);
          USART_DMA_String(5,"#Fail");
          USART_DMA_String(9,"#MOBILITY");
          while(1);
        }
			}
		}

		/*-------------------------------------------------------------Test Left Wheel Current-----------888888888888888---*/
    Sound(7);
    USART_DMA_String(2,"@07");
		Test_Result=0;
		while(1)
		{
      //left wheel move backward test the run current
			//Display_Content(LED_Plan,100,100,0,7);
			
		  LW_DIR_BACKWARD;
			Set_Wheel_Speed(60,0);
			delay(10000);
			Time_Counter=20;
			Current_Current=0;
			while(Time_Counter--)
			{
			  delay(100);
				Current_Current+=Get_Current_Current();
			}
      Test_PWM =	Left_Wheel_PWM;
			Current_Current/=20;
			Current_Current-=	BaseCurrent;
			if((Current_Current>120)||(Current_Current<30))
			{
			  //Display_Content(LED_Plan|LED_Exclamation,Current_Current/100,Current_Current%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(10,"#Left W C=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
			}
      else
      {
        //Display_Content(LED_Plan,Current_Current/100,Current_Current%100,0,5);
      }
      //check the run pwm
      if((Test_PWM<20)||(Test_PWM>80))
			{
			  //Display_Content(LED_Colon|LED_Exclamation,100,Test_PWM%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(10,"#Left W P=");
        USART_DMA_Numbers(Test_PWM);
        Disable_Motors();
        while(1);
			}

      //left wheel move Forward test the run current
			//Display_Content(LED_Spot,100,100,0,7);
			
		  LW_DIR_FORWARD;
			Set_Wheel_Speed(60,0);
			delay(10000);
			Time_Counter=20;
			Current_Current=0;
		  while(Time_Counter--)
			{
			  delay(100);
				Current_Current += Get_Current_Current();
			}
			Test_PWM =	Left_Wheel_PWM;
			Current_Current/=20;
			Current_Current-=	BaseCurrent;
			if((Current_Current>120)||(Current_Current<30))
			{
			  //Display_Content(LED_Spot|LED_Exclamation,Current_Current/100,Current_Current%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(10,"#Left W C=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
			}
      else
      {
        //Display_Content(LED_Spot,Current_Current/100,Current_Current%100,0,5);
      }
      //check the run pwm
      if((Test_PWM<20)||(Test_PWM>80))
			{
			  //Display_Content(LED_Colon|LED_Exclamation,100,Test_PWM%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(10,"#Left W P=");
        USART_DMA_Numbers(Test_PWM);
        Disable_Motors();
        while(1);
			}
      USART_DMA_String(5,"@PASS");
      break;
		}
    
    /*-------------------------------------------------------------Test Left Wheel stall Sensors-----------9---*/
    Sound(8);
    USART_DMA_String(2,"@08");
    while(1)
    {
      Move_Forward(60,0);
      while(1)
			{
			  if(GPIOD->IDR&MCU_WHL_L_FAULT)
				{
				  delay(100);
					if(GPIOD->IDR&MCU_WHL_L_FAULT)
					{
					  Current_Current=Get_Current_Current()-BaseCurrent;
					  if(Current_Current<950)
						{
						  //Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,0,7);
							USART_DMA_String(5,"#Fail");
              USART_DMA_String(11,"#Left Stall=");
              USART_DMA_Numbers(Current_Current);
              Disable_Motors();
              while(1);
						}
            else
            {
              //Display_Content(LED_Clean,Current_Current/100,Current_Current%100,0,7);
              USART_DMA_String(5,"@PASS");
              break;
            }
					}
				}
        Current_Current=Get_Current_Current()-BaseCurrent;
				//Display_Content(0,Current_Current/100,Current_Current%100,0,7);
        delay(500);
			}
			break;
    }
		
	  /*-------------------------------------------------------------Test Right Wheel Current -----------10---*/
    Sound(9);
    USART_DMA_String(2,"@09");
		while(1)
		{
      //test right wheel run backward current and pwm
			//Display_Content(LED_Clock,100,100,0,7);
		  
		  RW_DIR_BACKWARD;
			Set_Wheel_Speed(0,60);
			delay(10000);
			Time_Counter=20;
			Current_Current=0;
			while(Time_Counter--)
			{
			  delay(100);
				Current_Current +=Get_Current_Current();
			}
      Test_PWM =	Right_Wheel_PWM;
			Current_Current/=20;
			Current_Current-=	BaseCurrent;
			if((Current_Current>120)||(Current_Current<30))
			{
			  //Display_Content(LED_Clock|LED_Exclamation,Current_Current/100,Current_Current%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(11,"#Right W C=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
			}
      else
      {
        //Display_Content(LED_Clock,Current_Current/100,Current_Current%100,0,7);
      }

      if((Test_PWM<20)||(Test_PWM>80))
			{
			  //Display_Content(LED_Colon|LED_Exclamation,100,Test_PWM%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(11,"#Right W P=");
        USART_DMA_Numbers(Test_PWM);
        Disable_Motors();
        while(1);
			}

      //test right wheel run forward current and pwm
			//Display_Content(LED_Home,100,100,0,7);
			
		  RW_DIR_FORWARD;
			Set_Wheel_Speed(0,60);
			delay(10000);
			Time_Counter=20;
			Current_Current=0;
		  while(Time_Counter--)
			{
			  delay(100);
				Current_Current += Get_Current_Current();
			}
			Test_PWM =	Right_Wheel_PWM;
			Current_Current/=20;
			Current_Current-=	BaseCurrent;
			//check current  
			if((Current_Current>120)||(Current_Current<30))
			{
			  //Display_Content(LED_Home|LED_Exclamation,Current_Current/100,Current_Current%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(11,"#Right W C=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
			}
      else
      {
        //Display_Content(LED_Home,Current_Current/100,Current_Current%100,0,7);
      }
      //check pwm 
			if((Test_PWM<20)||(Test_PWM>80))
			{
			  //Display_Content(LED_Colon|LED_Exclamation,100,Test_PWM%100,0,5);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(11,"#Right W P=");
        USART_DMA_Numbers(Test_PWM);
        Disable_Motors();
        while(1);
			}
      USART_DMA_String(5,"@PASS");
      break;
		}
		
    /*-------------------------------------------------------------Test Right wheel stall sensor-----------11111111111111111111---*/
    Sound(10);
    USART_DMA_String(2,"@10");
    while(1)
    {
      Move_Forward(0,60);
      while(1)
			{
			  if(GPIOD->IDR&MCU_WHL_R_FAULT)
				{
				  delay(100);
					if(GPIOD->IDR&MCU_WHL_R_FAULT)
				  {
					  Current_Current=Get_Current_Current()-BaseCurrent;
					  if(Current_Current<950)
						{
						  //Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,0,7);
							USART_DMA_String(5,"#Fail");
              USART_DMA_String(12,"#Right Stall=");
              USART_DMA_Numbers(Current_Current);
              Disable_Motors();
              while(1);
						}
            else
            {
              //Display_Content(LED_Clean,Current_Current/100,Current_Current%100,0,7);
              USART_DMA_String(5,"@PASS");
              break;
            }
					}
				}
        Current_Current=Get_Current_Current()-BaseCurrent;
				//Display_Content(0,Current_Current/100,Current_Current%100,0,7);
        delay(100);
			}
      break;
    }
    
		/*-------------------------------------------------------------Test Left Brush Sensors-----------12121212121212121212121212121212122222222--------------------*/
    Sound(11);
    USART_DMA_String(2,"@11");
		while(1)
		{
	    Test_Result=0;
			//Display_Content(LED_Plan,100,100,0,7);
			Temp_PWM=(uint8_t)((uint32_t)120000/(uint32_t)GetBatteryVoltage());
      Set_SideBrush_PWM(0,Temp_PWM);
			delay(10000);
			Time_Counter=20;
			Current_Current=0;
		  while(Time_Counter--)
			{
			  Temp_PWM=(uint8_t)((uint32_t)120000/(uint32_t)GetBatteryVoltage());
        Set_SideBrush_PWM(0,Temp_PWM);
			  delay(100);
				Current_Current += Get_Current_Current();
			}
			Current_Current/=20;
			Current_Current-=	BaseCurrent;
			if((Current_Current>90)||(Current_Current<30))
			{
			  //Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,0,7);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(10,"#SBrush C=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
			}
      USART_DMA_String(5,"@PASS");
			while(1)
			{
			  Current_Current=Get_Current_Current()-BaseCurrent;
        //Display_Content(0,Current_Current/100,Current_Current%100,0,7);
				if(GPIOE->IDR&MCU_L_SIDEBRUSH_I)
			  {
				  delay(50);
					if(GPIOE->IDR&MCU_L_SIDEBRUSH_I)
					{
				    Current_Current=Get_Current_Current()-BaseCurrent;
				    if(Current_Current<250)
					  {
							//Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,0,7);
              USART_DMA_String(5,"#Fail");
              USART_DMA_String(13,"#SBrushStall=");
              USART_DMA_Numbers(Current_Current);
              Disable_Motors();
              while(1);
						}
            else
            {
              USART_DMA_String(5,"@PASS");
              break;
            }
					}
				}
        delay(100);
			}
      Set_SideBrush_PWM(0,0);
			break;		
		}
		/*-------------------------------------------------------------Test Vacuum Sensors-----------13---*/
    Sound(12);
    USART_DMA_String(2,"@12");
    Test_Result=0;
		while(1)
		{
			Temp_PWM=(uint8_t)((uint32_t)120000/(uint32_t)GetBatteryVoltage());
      Set_Vacuum_PWM(Temp_PWM);
			//Display_Content(LED_Clock,100,100,0,7);
			delay(10000);

  		Time_Counter=20;
  		Current_Current=0;
  		while(Time_Counter--)
  		{
   		  Temp_PWM=(uint8_t)((uint32_t)120000/(uint32_t)GetBatteryVoltage());
        Set_Vacuum_PWM(Temp_PWM);
  		  delay(100);
  			Current_Current += Get_Current_Current();
  		}
  		Current_Current/=20;
  		Current_Current-=	BaseCurrent;
  		if((Current_Current>350)||(Current_Current<100))
  		{
        Test_Result=0;
  		  //Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,0,7);
        USART_DMA_String(5,"#Fail");
        USART_DMA_String(7,"#Vac C=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
  		}
      USART_DMA_String(5,"@PASS");
      Set_Vacuum_PWM(0);
      break;
		}
		
		/*-------------------------------------------------------------Test Main Brush------------14--*/
    Sound(13);
    USART_DMA_String(2,"@13");
		while(1)
		{
      Time_Counter=5;
  		Current_Current=0;
  	  while(Time_Counter--)
  		{
  		  Temp_PWM=(uint8_t)((uint32_t)120000/(uint32_t)GetBatteryVoltage());
        Set_MainBrush_PWM(Temp_PWM);
  		  delay(1000);
  		}


      /*
      Current_Current = Get_MainBrush_Speed();
      if((Current_Current>35)||(Current_Current<20))
  	  {
        Test_Result|=0x0200;
  		  //Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,8,7);
  		  USART_DMA_String(5,"#Fail");
        USART_DMA_String(10,"#MBrush V=");
        USART_DMA_Numbers(Current_Current);
        Disable_Motors();
        while(1);
  		} */
      Main_Brush_PWM=0;
      USART_DMA_String(5,"@PASS");
      break;
		}
		
		/*-------------------------------------------------------------Charger------------14--*/
    Sound(14);
    USART_DMA_String(2,"@14");
//    Set_Time_To_DS1338(1,0,0);
//	  Read_And_SetTime();
		while(1)
		{
      //Display_Content(LED_Colon,100,100,0,2);
		  Current_Current=0;
		  while(1)	// Wait for charger plugin
			{
			  delay(500);
				Current_Current=Get_Charger_Voltage();
        if((Current_Current>1700))break;
			}
      USART_DMA_String(6,"@PASS");
//      TIM3->PSC = Timer3_Prescaler_50K;//set charge frequency to 50 k
	    TIM3->ARR = Timer3_Count_Range1000;
	   
			Charge_PWM=1;
			Temp_PWM=0;
			Current_Current=0;
			Charge_Time=0;
			while(1)
			{
			  delay(10);
				Charge_Time++;
				if(Charge_Time>35)break;
				if(BaseCurrent>Get_Current_Current())// Check if on Charging 
				{
				  Current_Current=BaseCurrent-Get_Current_Current();
				}
				else Current_Current=0;

        if(Current_Current>300)//charge current over
  			{
  				if(Temp_PWM<2)Temp_PWM=2;
  				Temp_PWM-=2;
  			}
  			else
  			{
          if(Current_Current<270)
          {
            Temp_PWM+=10;
          }
          else if(Current_Current<295)
          {
            Temp_PWM+=5;
          }
          else
          {
            Temp_PWM++;
          }
  				if(Temp_PWM>720)Temp_PWM=720;
  			}
        Charge_PWM = Temp_PWM;
				//Display_Content(0,Current_Current/100,Current_Current%100,Charge_Time/5,2);
			}

			Charge_PWM=0;
      TIM3->PSC = Timer3_Prescaler_10K;
//	    TIM3->ARR = Timer3_Count_Range100;

			if((Temp_PWM<150)||(Temp_PWM>350))
			{
        //Display_Content(LED_Exclamation,Temp_PWM/100,Temp_PWM%100,1,2);
        USART_DMA_String(2,"@Fail");
        USART_DMA_String(11,"#ChargePWM=");
        USART_DMA_Numbers(Temp_PWM);
        while(1);
			}
      else
      {
        //Display_Content(0,Temp_PWM/100,Temp_PWM%100,0,2);
      }
      //USART_DMA_String(6,"@PASS2");
			if((Current_Current<280)||(Current_Current>320))
			{
        //Display_Content(LED_Exclamation,Current_Current/100,Current_Current%100,2,2);
			  USART_DMA_String(2,"@Fail");
        USART_DMA_String(10,"#Charge C=");
        USART_DMA_Numbers(Current_Current);
        while(1);
			}
      USART_DMA_String(6,"@PASS");
      //USART_DMA_String(6,"@PASS3");
			while(1)
			{
			  delay(500);
				Current_Current=Get_Charger_Voltage();
        //Display_Content(LED_Clean,Current_Current/100,Current_Current%100,3,2);
        if(Current_Current<1600)break;
			}
      USART_DMA_String(6,"@PASS");
      Display_Pass();
      while(1);
//      break;

			/*-------------------------------Check RTC Timing-------------------------------*/
	  }
  }
}
