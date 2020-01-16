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

#include "include.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"

void User_Interface(void)
{
  static volatile uint8_t key_press_time = 0,sound_flag=0;
	static volatile uint16_t usr_time_out_cnt = 0,g_error_sound_cnt=0;
	uint8_t error_report_cnt = 0;
	uint8_t  key_clean_on_cnt = 0,key_spot_on_cnt = 0,key_home_on_cnt = 0;
	uint16_t key_break_cnt = 0;
		
	uint32_t temp_remote_code = 0;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	Motor_DisableAll();
	Power_EnableAll();  
	Motor_ResetStatus();
	Battery_SetCheckState(DISABLE);	
	Rcon_ResetStatus();	
	Rcon_ResetRemoteCode();
	key_press_time = 0;

	/*error code*/
	Set_LED_On_Blink(0,0,0,0,0,1);
	Set_LED_On_Switch(1,1,1,Get_Bat_Green(),Get_Bat_Red(),0);	
	if(Error_GetCode()!=ERROR_NONE)
	{
		sound_flag=0;
		Set_LED_On_Blink(1,1,1,0,0,1);
		Set_LED_On_Switch(0,0,0,Get_Bat_Green(),Get_Bat_Red(),0);
	}

	while(Key_GetStatus())
	{
		key_break_cnt++;
		if(key_break_cnt >= 100)break;
		vTaskDelay(20/portTICK_RATE_MS);
	}	

	
	while(Mode_GetMode() == MODE_USERINTERFACE)
	{		
		Bumper_GetTrigStatus();
		if(sound_flag==0)
		{
			if(Error_GetCode())
			{
				Set_LED_On_Blink(1,1,1,0,0,1);
				Set_LED_On_Switch(0,0,0,Get_Bat_Green(),Get_Bat_Red(),0);	
				Set_LED_On_Blink(1,1,1,0,0,1);
				Set_LED_On_Switch(0,0,0,Get_Bat_Green(),Get_Bat_Red(),0);					
				SPK_Report_Error(Error_GetCode());
				Usprintf("error code: %d \n",Error_GetCode());	
				error_report_cnt=0;
				sound_flag=1;
			}			
		}
		else
		{
			if(Error_GetCode()==ERROR_NONE)sound_flag=0;
			/*error code*/
			g_error_sound_cnt++;		
			if(g_error_sound_cnt > 500)//report error every 10 seconds
			{
				g_error_sound_cnt=0;
				if(Error_GetCode())
				{
					Set_LED_On_Blink(1,1,1,0,0,1);
					Set_LED_On_Switch(0,0,0,Get_Bat_Green(),Get_Bat_Red(),0);				
					error_report_cnt++;
					if(error_report_cnt < 4)
					{
						SPK_Report_Error(Error_GetCode());
					}
					Usprintf("error code:: %d \n",Error_GetCode());	
				}			
				usr_time_out_cnt++;
				if(usr_time_out_cnt > 18)//no actions for 3 minutes then into sleep mode 18
				{
					usr_time_out_cnt = 0;			
					vTaskDelay(700/portTICK_RATE_MS);	
					if(!Is_ChargerOn())
					{
						Usprintf("%s(%d):Sleep mode!\n",__FUNCTION__, __LINE__);					
						Mode_SetMode(MODE_SLEEP);										
						return;				
					}
				}
			}		
		}
		
		
		/*check the remote events,and robot starts to work*/
		temp_remote_code = Rcon_GetRemoteCode();		
		if(temp_remote_code)
		{
			if(temp_remote_code==Remote_Forward||temp_remote_code==Remote_Right
					||temp_remote_code==Remote_Left||temp_remote_code==Remote_Max)
			{
				if (UserInterFace_IsLowPower())
				{
				  Rcon_ResetRemoteCode();
					Error_SetCode(ERROR_BATTERY);
				}
				else if (UserInterFace_IsRobotPickedUp())
				{
				  Rcon_ResetRemoteCode();
					Error_SetCode(ERROR_CLIFF);
				}
				else 
				{
					Error_ResetCode();					
					Mode_SetMode(MODE_REMOTE);										
				}
				return;	
			}	
			else
			{
				UserInterFace_RemoteToMode(temp_remote_code); 
			}			 			
		}							
	
    /*clean key events*/					
		if(Key_GetStatus() == KEY_CLEAN)
		{
			key_clean_on_cnt++;
			if(key_clean_on_cnt>150)
			{
				key_clean_on_cnt = 150;
			}
		}
		else if(Key_GetStatus() == KEY_SPOT)
		{
			key_spot_on_cnt++;
		}
		else if(Key_GetStatus() == KEY_HOME)
		{
			key_home_on_cnt++;
		}		
		else
		{
			if(!Is_ChargerOn())
			{
				if(key_clean_on_cnt > 140)//press for 3s to shut down	mode		
				{
					Error_ResetCode();				
					vTaskDelay(100/portTICK_RATE_MS);							
					Usprintf("%s(%d):Power OFF!\n",__FUNCTION__, __LINE__);
					Mode_SetMode(MODE_SLEEP);					
					return;			   			
				}			
			}
			if(key_home_on_cnt > 100)//2s
			{	
				Speaker(SPK_WIFI_CONNECT_START);	
				mcu_set_wifi_mode(SMART_CONFIG);		
			}				
			else if(key_clean_on_cnt > 1)//20ms			
			{			
				Mode_SetMode(MODE_NAVIGATION);		   			
			}
			else if(key_spot_on_cnt > 1)//20ms
			{
				Mode_SetMode(MODE_SPOT);	
			}
			else if(key_home_on_cnt > 1)//20ms
			{						
				Mode_SetMode(MODE_HOME);				
				return;		
			}			
			key_home_on_cnt = 0;
			key_spot_on_cnt = 0;
			key_clean_on_cnt = 0;		
		}

		/*wifi mode even*/
		#ifdef WIFI_TY
		if(Wifi_Get_Mode_Flag())
		{
			Wifi_Set_Mode_Flag(0);
			Mode_SetMode(Wifi_Mode_GetMode());
		}
		#endif

		/* mode change*/
		if(Mode_GetMode()!=MODE_USERINTERFACE)
		{			
      Error_ResetCode();
			if(Mode_GetMode() == MODE_SLEEP)
			{ 							
				Bumper_ResetErrorCnt();
				break;
			}
			/*if mode is "work mode",start to work!*/
			if(UserInterFace_IsWorkMode(Mode_GetMode()))
			{				
				/*check if the robot has been picked up*/
				if(UserInterFace_IsRobotPickedUp()) 
				{
					Usprintf("%s(%d):Cliff Error! %d\n",ERROR_CLIFF,__FUNCTION__, __LINE__);
					Error_SetCode(ERROR_PICK_UP);
					Mode_SetMode(MODE_USERINTERFACE);
				}
				else if(Is_Dustbin() == 0)
				{
					Error_SetCode(ERROR_DUSTBIN);
					Mode_SetMode(MODE_USERINTERFACE);					
				}
				/*check battery*/
				else if(UserInterFace_IsLowPower())
				{
					Usprintf("%s(%d):Low Battery! %d\n",__FUNCTION__, __LINE__,Battery_GetVoltage());
					Error_SetCode(ERROR_BATTERY);
					Display_SetBattery(LED_BAT_R);
					Mode_SetMode(MODE_USERINTERFACE);
					break;
				}
				/*start to work*/
				else
				{	
					UserInterFace_BeginToWork();
				}
				break;
			}
		}
				
		vTaskDelayUntil(&xLastWakeTime,20/portTICK_RATE_MS);
	}
}

void UserInterFace_RemoteToMode(uint32_t remote_code)
{
	Rcon_ResetRemoteCode();		
	/*mode selection from the remote*/ 
	if(remote_code == Remote_Wall)
	{
		Mode_SetModeAndSelection(MODE_WALL); 
	}	
	else if(remote_code == Remote_Spot)
	{
		Mode_SetModeAndSelection(MODE_SPOT);    
	}		
	else if(remote_code == Remote_Home)
	{
		Mode_SetModeAndSelection(MODE_HOME); 
	}	
	if(remote_code == Remote_Random)
	{
		Mode_SetModeAndSelection(MODE_RANDOM); 
	}
	else if(remote_code == Remote_Clean)
	{
		Mode_SetModeAndSelection(MODE_NAVIGATION); 
	}	
}

uint8_t UserInterFace_IsWorkMode(CleanMode_t mode)
{
	uint8_t reval = 0;
	if((mode == MODE_NAVIGATION)||(mode == MODE_SPOT)||(mode == MODE_REMOTE)
		||(mode == MODE_NAVIGATION2)||(mode == MODE_WALL)) 
	{
	  reval =1;	
	}
	return reval;
}


uint8_t UserInterFace_IsRobotPickedUp(void)
{
  uint8_t reval = 0;
	if(Cliff_GetInstantStatus() == (CLIFF_ALL_TRIG))
	{
		reval =1;	
	}
	return reval;	
}

uint8_t UserInterFace_IsLowPower(void)
{
	uint8_t reval = 0;

	if(Battery_GetVoltage() <= LOW_BATTERY_VOLTAGE)
	{
		reval =1;
	}
	return reval;		
}

void UserInterFace_BeginToWork(void)
{
	Battery_SetCheckState(ENABLE);
	Set_LED1_On_Blink(1);
	Usprintf("%s(%d):Clean Start!\n",__FUNCTION__, __LINE__);
	Bumper_ResetErrorCnt();
	vTaskDelay(20/portTICK_RATE_MS);
	Rcon_ResetRemoteCode();
}




void Task_CreateReportTask(void)
{
	portBASE_TYPE xTaskStatus;
	xTaskStatus = xTaskCreate(Task_ReportStackTask,"ReportTaskStack",100,NULL,2,NULL);
	if(xTaskStatus == pdPASS)
	{
		printf("Report Stack Task Created!!\n");
	}
}


portBASE_TYPE MainTask_Stack;
portBASE_TYPE SensorTask_Stack;
portBASE_TYPE ControlTask_Stack;
portBASE_TYPE CalculatePathTask_Stack;
portBASE_TYPE USART3Task_Stack;

extern TaskHandle_t Task_MainTask_Handler;
extern TaskHandle_t Task_SensorTask_Handler;
extern TaskHandle_t Task_ControlTask_Handler;
extern TaskHandle_t Task_CalculatePathTask_Handler;
extern TaskHandle_t g_usart3_print_task_handler;
void Task_ReportStackTask(void *argument)
{
	for(;;)
	{
		
		MainTask_Stack = uxTaskGetStackHighWaterMark(Task_MainTask_Handler);
		SensorTask_Stack = uxTaskGetStackHighWaterMark(Task_SensorTask_Handler);
		ControlTask_Stack = uxTaskGetStackHighWaterMark(Task_ControlTask_Handler);		
		CalculatePathTask_Stack = uxTaskGetStackHighWaterMark(Task_CalculatePathTask_Handler);		
		USART3Task_Stack = uxTaskGetStackHighWaterMark(g_usart3_print_task_handler);
		
//		Usprintf("\n\rMaintask stack =%d",MainTask_Stack);
//		Usprintf("\n\rSensorTask stack =%d",SensorTask_Stack);		
//		Usprintf("\n\rControlTask stack =%d",ControlTask_Stack);		
//		Usprintf("\n\rCalculatePathTask stack =%d",CalculatePathTask_Stack);		
//		Usprintf("\n\rUSART2Task stack =%d",USART3Task_Stack);		

/*---------------------------------- Dbug ---------------------------------*/
		
		Usprintf("Front_Left_Cliff= %d\n ",g_cliff_adc.Front_Left_Cliff);	
		Usprintf("Front_Right_Cliff= %d\n",g_cliff_adc.Front_Right_Cliff);			
		Usprintf("Back_Left_Cliff= %d\n ",g_cliff_adc.Back_Left_Cliff);	
		Usprintf("Back_Right_Cliff= %d\n ",g_cliff_adc.Back_Right_Cliff);
//		
//		Usprintf("Left_wall= %d\n",g_obs_adc.Left_Wall);	
//		Usprintf("left_obs= %d\n",g_obs_adc.Left_OBS);			
//		Usprintf("fromt_left_obs= %d\n",g_obs_adc.Front_Left_OBS);	
//		Usprintf("fromt_right_obs= %d\n",g_obs_adc.Front_Right_OBS);
//		Usprintf("right_obs= %d\n",g_obs_adc.Right_OBS);	
//		Usprintf("right_wall= %d\n",g_obs_adc.Right_Wall);			
//		Usprintf("BAT = %d\n",Battery_GetVoltage());
//		Usprintf("OBS_SLOW = %d\n",OBS_SLOW);	
//		Usprintf("Gyro angle = %d\n",Gyro_GetAngle(0));	
		
				
//		Usprintf(" wheel =%d",WheelSpeed_Stack);
//		Usprintf(" sensor =%d",SensorTask_Stack);
//		Usprintf(" move =%d",MoveTask_Stack);
//		Usprintf(" UpPositon =%d",UpdatePositonTask_Stack);
//		Usprintf("\n\r Current Time = %d",Time_GetCurrentTime());
		
//		Usprintf("\n\r\n\r Gyro angle =  %d",Gyro_GetAngle(0));
//		Usprintf("\n\r Robot Heading 8 =  %d",Path_GetRobotHeading8());
//		Usprintf("\n\r Robot Heading 4 =  %d",Path_GetRobotHeading4());
//      Usprintf("Obs_GetAck:%d \n",Obs_GetAck());
//		Usprintf("*");
    
    
    
    //Usprintf("Report Task tick %d!!\n",ti++);
		
		vTaskDelay(500/portTICK_RATE_MS);
		//taskYIELD();
	}
}


//void Draw_C_House(void)
//{
//	int32_t x=0,y=0;
//	
//	y=4;
//	for(x=-4;x<6;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=-5;
//	for(y=-2;y<5;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=4;
//	for(y=1;y<4;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=-2;
//	for(x=-4;x<-2;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	for(x=4;x<8;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=-4;
//	for(x=-1;x<2;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=-3;
//	for(x=2;x<4;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	Map_SetCell(-2,-3,BLOCKED_BUMPER);
//	Map_SetCell(7,-1,BLOCKED_BUMPER);
//	Map_SetCell(6,0,BLOCKED_BUMPER);
//}

//void Draw_C_Walked(void)
//{
//	int32_t x=0,y=0;

//	for(x=-4;x<5;x++)
//	{
//		for(y=-2;y<4;y++)
//		{
//			Map_SetCell(x,y,CLEANED);
//		}
//	}
//	Map_SetCell(5,0,CLEANED);
//	Map_SetCell(5,-1,CLEANED);
//	Map_SetCell(6,-1,CLEANED);
//	Map_SetCell(-1,-3,CLEANED);
//	Map_SetCell(0,-3,CLEANED);
//	Map_SetCell(1,-3,CLEANED);
//		
//}


//void Draw_B_House(void)
//{
//	int32_t x=0,y=0;
//	for(x=-3;x<-1;x++)
//	{
//		for(y=-1;y<1;y++)
//		{
//			Map_SetCell(x,y,BLOCKED_BUMPER);
//		}
//	}
//	for(x=-4;x<-2;x++)
//	{
//		for(y=1;y<4;y++)
//		{
//			Map_SetCell(x,y,BLOCKED_BUMPER);
//		}
//	}
//	
//	y=4;
//	for(x=-3;x<0;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	for(x=3;x<5;x++)
//	{
//		for(y=4;y<6;y++)
//		{
//			Map_SetCell(x,y,BLOCKED_BUMPER);
//		}
//	}
//	
//	x=5;
//	for(y=0;y<6;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	Map_SetCell(-2,3,BLOCKED_BUMPER);
//	Map_SetCell(4,1,BLOCKED_BUMPER);
//	Map_SetCell(3,2,BLOCKED_BUMPER);
//	Map_SetCell(6,0,BLOCKED_BUMPER);
//	Map_SetCell(6,1,BLOCKED_BUMPER);
//	
//	Map_SetCell(1,3,BLOCKED_BUMPER);
//	Map_SetCell(2,4,BLOCKED_BUMPER);
//	Map_SetCell(-1,2,BLOCKED_BUMPER);
//	Map_SetCell(0,2,BLOCKED_BUMPER);
//	Map_SetCell(1,2,BLOCKED_BUMPER);
//	
//	Map_SetCell(0,5,BLOCKED_BUMPER);
//	Map_SetCell(1,5,BLOCKED_BUMPER);
//}

//void Draw_B_Walked(void)
//{
//	int32_t x=0,y=0;

//	for(x=-2;x<5;x++)
//	{
//		for(y=0;y<4;y++)
//		{
//			Map_SetCell(x,y,CLEANED);
//		}
//	}
//}


//void Draw_A_House(void)
//{
//	int32_t x=0,y=0;
//	x=-21;
//	for(y=-24;y<39;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=18;
//	for(y=-24;y<39;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=39;
//	for(x=-20;x<18;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=27;
//	for(x=-18;x<0;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=27;
//	for(x=6;x<18;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=21;
//	for(x=0;x<18;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=15;
//	for(x=-12;x<0;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=-12;
//	for(y=12;y<19;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=6;
//	for(x=-9;x<9;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=9;
//	for(x=3;x<9;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=0;
//	for(x=-12;x<-3;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=-9;
//	for(y=-3;y<6;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=-12;
//	for(x=-18;x<-9;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	y=-3;
//	for(x=3;x<18;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=3;
//	for(y=-24;y<0;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	y=-20;
//	for(x=-13;x<0;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	y=35;
//	for(x=-13;x<11;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	y=8;
//	for(x=10;x<18;x++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	
//	x=9;
//	for(y=27;y<35;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//}

//void Draw_A_Walked(void)
//{
//	int32_t x=0,y=0;

//	for(x=-7;x<6;x++)
//	{
//		for(y=-5;y<5;y++)
//		{
//			Map_SetCell(x,y,CLEANED);
//		}
//	}
//}

//void Draw_W_Map(void)
//{
//	Draw_Map(-49,-49,0,100,BLOCKED);
//	Draw_Map(-49,-49,1,100,BLOCKED);
//	
//	Draw_Map(49,49,0,-100,BLOCKED);
//	Draw_Map(49,49,1,-100,BLOCKED);
//	
//	Draw_Map(-30,-49,1,80,BLOCKED);
//	Draw_Map(-10,49,1,-80,BLOCKED);
//	Draw_Map(10,-49,1,80,BLOCKED);
//	Draw_Map(20,49,1,-80,BLOCKED);
//	Draw_Map(30,-49,1,70,BLOCKED);
//	
//	Draw_Map(49,0,0,-15,BLOCKED);
//	Draw_Map(-49,20,0,15,BLOCKED);
//	Draw_Map(20,30,0,20,BLOCKED);
//}

//void Draw_Map(int16_t x,int16_t y,uint8_t dir, int16_t length,CellState_t value)
//{
//	int16_t x_temp=0,y_temp=0;
////	int16_t i=0;
//	
//	x_temp = x;
//	y_temp = y;
//	
//	if(dir)
//	{		
//		if(length>0)
//		{
//			for(;length>0;length--)
//			{
//				Map_SetCell((x_temp), (y_temp+length), value);
//			}
//		}
//		else
//		{
//			for(;length<0;length++)
//			{
//				Map_SetCell((x_temp), (y_temp+length), value);
//			}
//		}
//	}
//	else
//	{
//		if(length>0)
//		{
//			for(;length>0;length--)
//			{
//				Map_SetCell((x_temp+length), (y_temp), value);
//			}
//		}
//		else
//		{
//			for(;length<0;length++)
//			{
//				Map_SetCell((x_temp+length), (y_temp), value);
//			}
//		}		
//	}	
//}


//void Draw_D_House(void)
//{
//	int32_t x=0,y=0;
//	x=-13;
//	for(y=-1;y<2;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=8;
//	for(y=-1;y<6;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//  x=1;
//	for(y=4;y<6;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	x=9;
//	for(y=2;y<4;y++)Map_SetCell(x,y,BLOCKED_BUMPER);
//	Map_SetCell(0,4,BLOCKED_BUMPER);
//	Map_SetCell(2,6,BLOCKED_BUMPER);
//	Map_SetCell(2,7,BLOCKED_BUMPER);
//	Map_SetCell(3,7,BLOCKED_BUMPER);
//	Map_SetCell(4,8,BLOCKED_BUMPER);
//	Map_SetCell(4,9,BLOCKED_BUMPER);
//	Map_SetCell(5,8,BLOCKED_BUMPER);
//	Map_SetCell(7,7,BLOCKED_BUMPER);
//	Map_SetCell(-12,-11,BLOCKED_BUMPER);
//}
