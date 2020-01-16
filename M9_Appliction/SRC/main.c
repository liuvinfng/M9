/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version Ver 0918
  * @date    17-Nov-2018
  * @brief   Main program body
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
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
#include "wifiOTA.h"

void VectorTabs_setup(void)
{
	 __disable_irq() ;
	 __disable_fault_irq();
	 nvic_vector_table_set(NVIC_VECTTAB_FLASH, APPLICATION_OFFSET);
	 __enable_irq();
	 __enable_fault_irq();
}

int main(void)
{   	
	SystemInitialize();	
//	VectorTabs_setup();	
	Power_DisableAll();
	Set_5v(Power_On);	
	delay(500);
	System_SetCurrentBaselineAdc(GetBaseLineADCV());
	Power_EnableAll();
	BLDC_OFF;

	#ifdef WIFI_TY
	if(Get_Upgrade_Flag() == 0x55)
	{	
		Set_Upgrade_Flag(0);		
		delay(2000);
    NVIC_SystemReset();		
	}	
	Set_Wifi_Enable(0);
	#endif
	
	delay(10000);
	if(Key_GetStatus() == KEY_SPOT)
	{
		Mode_SetMode(MODE_GYRO_TURN_TEST);
	}
	else if(Key_GetStatus() == KEY_HOME)
	{
		Mode_SetMode(MODE_GYRO_RUN_TEST);
	}
	else
	{
		Mode_SetMode(MODE_USERINTERFACE);
	}

//	Task_CreateReportTask();		
	USART3_CreateTask();
	Task_CreateMainTask();
	Task_CraeteSensorsTask();
	Task_CraeteControlTask();
	Task_CreateCalculatePathTask();
	printf("OS Start!!\n");
	vTaskStartScheduler();	
	while(1);
}


/*Task Main Task*/
void Task_MainTask(void *argument)
{	
	System_StoreMotorBaseline();
	OBS_ResetTrigValue();
	Error_SetCode(ERROR_NONE);
	Speaker(SPK_SYS_MUSIC);
	#ifdef WIFI_TY
	Battery_Capacity_Reset();
	wifi_protocol_init();
	Wifi_All_Data_Init();	
	Set_Wifi_Enable(1);
	#endif

	while(1)
	{
	
		if(Mode_GetMode()==MODE_NAVIGATION)Speaker(SPK_CLEAN_START);
		else if(Mode_GetMode()==MODE_SPOT)Speaker(SPK_AREA_CLEAN_START);
		else if(Mode_GetMode()==MODE_WALL)Speaker(SPK_WALLFOLLOW_MODE_START);
		else if(Mode_GetMode()==MODE_HOME)Speaker(SPK_ENTER_RECHARGE_MODE);
		
		switch(Mode_GetMode()) 
		{
			/*User Interface mode ,waiting for user to select clean mode or report errors*/
			case MODE_USERINTERFACE	:										
																				User_SetQuitCheckState(DISABLE);	
																				printf("%s(%d):MODE_USERINTERFACE!\n",__FUNCTION__, __LINE__);																					
																				User_Interface();
																				Time_ResetCurrentTime();
																				Vacuum_ResetFailFlag();
																				break;
      /*Spot mode , robot turn all the power max to clean a certain area (1 m2),and stop to the start point */
			case MODE_SPOT	: 								User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_SPOT!\n",__FUNCTION__, __LINE__);	
																				Spot_Mode();
																				Motor_DisableAll();																				
																				break;
			/*Random mode , robot cleans with normal power in random path*/
			case MODE_RANDOM :								 
																				User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_RANDOM!\n",__FUNCTION__, __LINE__);
																				//Random_Running_Mode();
																				Motor_DisableAll();
																				break;
			/*Navigation mode , 
			robot cleans with normal power and run a zizag path to clean about 50 square meter floor , 
			then stop to the start point
			*/
			case MODE_NAVIGATION:							
																				User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_NAVIGATION!\n",__FUNCTION__, __LINE__);
																				CM_SetCMType(CM_NORMAL);
																				#ifdef WIFI_TY
																				mcu_get_system_time();
																				#endif
																				CM_MapTouring();
																				#ifdef WIFI_TY
																				Wifi_Stop_Stream_trans();
																				Wifi_Clean_Record_Report();
																				#endif
																				Motor_DisableAll();	
																				break;
			/*Double navigation mode , just run a navigation mode twice*/
			case MODE_NAVIGATION2:						User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_NAVIGATION2!\n",__FUNCTION__, __LINE__);
																				CM_SetCMType(CM_SPECIAL);
																				CM_MapTouring();
																				Motor_DisableAll();	
																				break;
			/*Remote mode , robot respond to user remote command*/
			case MODE_REMOTE	:								User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_REMOTE!\n",__FUNCTION__, __LINE__);	
																				Remote_Mode();																			
																				break;
			/*Home mode , robot respond to user remote command*/
			case MODE_HOME	:								  User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_HOME!\n",__FUNCTION__, __LINE__);	
																				HomeStraight_Mode();																			
																				break;																				
																				
			/* Sleep Mode */
			case MODE_SLEEP	:						      
																				printf("%s(%d):MODE_SLEEP!\n",__FUNCTION__, __LINE__);
																				Motor_DisableAll();
																				Standby_Mode();
																				break;			
			/*Follow Wall*/
			case MODE_WALL	:									User_SetQuitCheckState(ENABLE);
																				printf("%s(%d):MODE_WALL!\n",__FUNCTION__, __LINE__);
																				Wall_Follow_Mode();	
																				Motor_DisableAll();
																				break; 
			case MODE_GYRO_RUN_TEST	:							
																				Test_GyroFunction(0);
																				break;
			case MODE_GYRO_TURN_TEST	:							
																				Test_GyroFunction(1);
																				break;			
			default:													Mode_SetMode(MODE_USERINTERFACE);
																				printf("%s(%d):Clean Mode Out of range ,return to Userinterface\n",__FUNCTION__, __LINE__);
																				break;
		}
	}
}


/*--------------Gyro Test-----------------*/
void Test_GyroFunction(uint8_t code)
{
	PathList_t temp_pathlist;
	Point16_t P;
	int32_t angle_diff = 0;
	TurnDir_t turn_dir = TURN_LEFT;
	uint16_t target_angle = 900;
	uint8_t head2course_cnt = 0,turn_cycle_cnt = 0,test_result=1,i=0;
	portTickType xLastWakeTime = xTaskGetTickCount(); 

	
	if(!ICM42688_Config())
	{
		printf("%s(%d):Gyro Error!\n",__FUNCTION__, __LINE__);
		Motor_DisableAll();
		Set_LED_On_Blink(1,1,1,0,0,1);
		Set_LED_On_Switch(0,0,0,1,0,0);
		while(1);
	}
	Gyro_Cmd(DISABLE);
	osDelay(300);
	osDelay(300);
	Gyro_Cmd(ENABLE);
	osDelay(300);	
	osDelay(300);

	Motor_WorkConfigure();
  Map_Initialize();
	PathList_Clear();
	TargetList_Clear();
	PathPoint_ClearAllPoints();

	if(code==0)
	{
		for(i = 0;i<5;i++)
		{					
			P.X = 20;P.Y = 0;
			temp_pathlist.cell_pos = P;		
			CM_MoveToMap(temp_pathlist.cell_pos,temp_pathlist.status);	
			Usprintf("Gyro:%d\n\n\n",Gyro_GetAngle(0));
			P.X = 0;P.Y = 0;
			temp_pathlist.cell_pos = P;
			CM_MoveToMap(temp_pathlist.cell_pos,temp_pathlist.status);
			Usprintf("Gyro:%d\n\n\n",Gyro_GetAngle(0));		
		}	

			P.X = 0;P.Y = 0;
			temp_pathlist.cell_pos = P;
			CM_MoveToMap(temp_pathlist.cell_pos,temp_pathlist.status);
			Usprintf("Gyro:%d\n\n\n",Gyro_GetAngle(0));	
		
		Wheel_Stop();	
		Motor_DisableAll();
		Usprintf("stop!!\n");
		Set_LED_On_Blink(0,0,0,0,0,1);
		Set_LED_On_Switch(1,1,1,1,0,0);	
		while(1);		
	}
	else if(code==1)
	{
		(Battery_GetVoltage()%2)?(turn_dir = TURN_RIGHT):(turn_dir = TURN_LEFT);
		
		(turn_dir == TURN_LEFT)?(target_angle = 2700):(target_angle = 900);
		
		CM_HeadToCourse(ROTATE_TOP_SPEED,target_angle);//TURN_SPEED		

		Action_SetMove(MOVE_ACT_HEAD2COURCE);
		
		Wheel_ResetStep();
		while(1)
		{
			switch(Action_GetMove())
			{
				case MOVE_ACT_HEAD2COURCE://angle:900 step:890
																		
																		angle_diff = Math_Diff_int(Heading_GetTargetAngle(), Gyro_GetAngle(0));
																		head2course_cnt++;
																		if(head2course_cnt >= 5)
																		{
																			head2course_cnt = 0;
																			Usprintf("l step : %d target angle :%d  angle:%d angle_diff : %d\n",Wheel_GetLeftStep(),Heading_GetTargetAngle(),Gyro_GetAngle(0),angle_diff);																	
																		}
																	
																		if(angle_diff < 20)
																		{	
																			Usprintf("l step:%d\n",Wheel_GetLeftStep());																		
																			if(Wheel_GetLeftStep() < 850)
																			{
																				Usprintf("%s(%d):error_2\n",__FUNCTION__,__LINE__);
																				test_result=0;
																				Usprintf("less step, l step:%d!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
																			}
																			ActList_Switch();					
																			break;
																		}
																		else if(angle_diff < 400)
																		{
																			angle_diff /= 20;
																			if(angle_diff < 6)angle_diff = 6;
																			Wheel_SetTargetSpeed(angle_diff,angle_diff);
																		}
																		if(Wheel_GetLeftStep() > 1050)
																		{
																			test_result=0;
																			Usprintf("over steps, l step:%d!!!!!!!!!!!!!!!!!!!!!!!!!!\n",Wheel_GetLeftStep());
																		}
																		break;																						
				case MOVE_ACT_DECELERATE:	
																		if((Wheel_GetLeftSpeed() <= 15) && (Wheel_GetRightSpeed()) <= 15)
																		{															
																			ActList_Switch();
																			break;
																		}  
																		break;		

				case MOVE_ACT_STATIC:      
																		turn_cycle_cnt++;
																		if(turn_cycle_cnt >= 20)
																		{
																			Wheel_Stop();
																			test_result=2;
																		}
																		else
																		{ 
																			Action_Stop();
																			osDelay(200);
																			if(turn_dir == TURN_LEFT)
																			{
																				if((target_angle == 0)||(target_angle == 3600))
																				{
																					target_angle = 2700;
																				}
																				else
																				{
																					target_angle -= 900;																			
																				}																		
																			}
																			else
																			{
																				if((target_angle == 0)||(target_angle == 3600))
																				{
																					target_angle = 900;
																				}
																				else
																				{
																					target_angle += 900;																			
																				}																																					
																			}
																			Usprintf("target angle:%d\n",target_angle);
																			Wheel_ResetStep();			
																			CM_HeadToCourse(ROTATE_TOP_SPEED,target_angle);//TURN_SPEED	
																			Action_SetMove(MOVE_ACT_HEAD2COURCE);
																		}
																		break;
				
				case MOVE_ACT_HANDLER:
																		ActList_Switch();
																		break;						
				default:
																		break;
			}
			
			
			if(test_result==0 || test_result==2)
			{
				break;
			}				
			vTaskDelayUntil(&xLastWakeTime,50/portTICK_RATE_MS);
		}
		Wheel_Stop();	
		Motor_DisableAll();
		if(test_result==2)
		{
			Usprintf("gyro pass\n");
			Set_LED_On_Blink(0,0,0,0,0,1);
			Set_LED_On_Switch(1,1,1,1,0,0);	
		}
		else
		{
			Usprintf("gyro error\n");
			Set_LED_On_Blink(1,1,1,0,0,1);
			Set_LED_On_Switch(0,0,0,1,0,0);	
		}	
		while(1);		
	}
	
}








/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 ILife CO.LTD *****END OF FILE****/
