/**
  ******************************************************************************
  * @file    Ilife Cleaning Robot
  * @author  Wfliu
  * @version Ver 00
  * @date    18-Sep-2016
  * @brief   Task Functions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
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
#include "wifiota.h"
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

TaskHandle_t Task_MainTask_Handler;
TaskHandle_t Task_SensorTask_Handler;
TaskHandle_t Task_CalculatePathTask_Handler;
TaskHandle_t Task_ControlTask_Handler;

/*create main task*/
void Task_CreateMainTask(void)
{
	portBASE_TYPE xTaskStatus;
	xTaskStatus = xTaskCreate(Task_MainTask,"MainTask",MAIN_TASK_STK_SIZE,NULL,osPriorityNormal,&Task_MainTask_Handler);
	if(xTaskStatus==pdPASS)
	{
		printf("MainTask Created!!\n");
	}
}	
/*creat sensor task*/
void Task_CraeteSensorsTask(void)
{
	portBASE_TYPE xTaskStatus;
	xTaskStatus = xTaskCreate(Task_SensorsTask,"SensorTask",SENSORS_TASK_STK_SIZE,NULL,osPriorityNormal,&Task_SensorTask_Handler);
	if(xTaskStatus==pdPASS)
	{
		printf("SensorTask Created!!\n");
	}
}
/*create control task*/
void Task_CraeteControlTask(void)
{
	portBASE_TYPE xTaskStatus;
	xTaskStatus = xTaskCreate(Task_ControlTask,"ControlTask",CONTROL_TASK_STK_SIZE,NULL,osPriorityAboveNormal,&Task_ControlTask_Handler);
	if(xTaskStatus==pdPASS)
	{
		printf("ControlTask Created!!\n");
	}
}

/*create path calculte task*/
void Task_CreateCalculatePathTask(void)
{
	portBASE_TYPE xTaskStatus;
	xTaskStatus = xTaskCreate(Task_CalculatePathTask,"CalculatePathTask",CALCULATE_PATH_TASK_STK_SIZE,NULL,osPriorityLow,&Task_CalculatePathTask_Handler);
	if(xTaskStatus==pdPASS)
	{
		printf("CalculatePathTask Created!!\n");
	}
}

/*check if stack over flow*/
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	printf(pcTaskName);
	printf("  Stack Overflow!!!!");
	while(1)
	{
	}
}

/* Task Control task*/
void Task_ControlTask(void *argument)
{
	uint8_t whee_adj_cnt = 0,display_cnt=0,cnt_10ms=0,cnt_60ms=0,cnt_50ms=0,cnt_100ms=0,cnt_500ms=0,cnt_1s=0,cnt_map=0,cnt_1min=0;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)//5ms
	{	
		/* Wheel speed and BLDC speed adjust */
		whee_adj_cnt++;
		if(whee_adj_cnt>4)//4-25ms
		{
			whee_adj_cnt=0;
			
			if(Vacuum_GetSpeed()!=0)Vacuum_TuneProcess();//adjust bldc speed
			if(Motor_GetState()==ENABLE)Motor_TunePower();
			
			Wheel_TuneSpeedProcess(Wheel_GetLeftSpeed(),Wheel_GetRightSpeed());//adjust wheel speed			
			Time_25ms++;
		}
		cnt_60ms++;
		if(cnt_60ms>4)
		{
			cnt_60ms = 0;	
			Wheel_SpeedAccelerationProcess(); //wheel acceleration only work while wheels need to move
		}	
		cnt_10ms++;
		if(cnt_10ms>1)
		{
			cnt_10ms=0;

		}				
		/* Display */
		display_cnt++;
		if(display_cnt>3)//20 ms
		{
			display_cnt = 0;
			Display_Process(Mode_GetMode());//process display
			Charge_Process();			
		}
			
		/* Charge Proess */
		cnt_100ms++;
		if(cnt_100ms>19)//100 ms
		{			
			cnt_100ms = 0;
			if(Is_ChargerOn()&&(Mode_GetMode()!=MODE_NAVIGATION))
			{
				Error_ResetCode();
				Charge_SetSwitch(ENABLE);
				Motor_DisableAll();
				if(Mode_GetMode()!=MODE_USERINTERFACE)Mode_SetMode(MODE_USERINTERFACE);
			}
			else Charge_SetSwitch(DISABLE);
			

			/*low power*/
			if(Mode_GetMode()!=MODE_SLEEP)
			{
				if(UserInterFace_IsLowPower()||(Error_GetCode() == ERROR_BATTERY))
				{
					Display_SetBattery(LED_BAT_R);
				}
				else
				{
					if(!Is_ChargerOn())
					{
						Display_SetBattery(LED_BAT_G);
					}
				}			
			}

			/*dustbin check*/
			if(Is_Dustbin() == 0)
			{
				Error_SetCode(ERROR_DUSTBIN);
				if(Mode_GetMode()!=MODE_USERINTERFACE)Mode_SetMode(MODE_USERINTERFACE);
			}
			else
			{
				if(Error_GetCode() == ERROR_DUSTBIN)Error_SetCode(ERROR_NONE);
			}			
		}			
		/* charge time */
		cnt_1s++;
		if(cnt_1s>199)//1s
		{
			cnt_1s = 0;	
			cnt_map++;
			if(cnt_map>0)
			{
				cnt_map=0;
				#ifdef WIFI_TY				
				Wifi_Report_Map();
				#endif
			}
			cnt_1min++;
			if(cnt_1min>59)
			{
				cnt_1min=0;				
				g_minute_counter++;
				#ifdef WIFI_TY
				if(Get_ACFlag())
				{
					g_clean_time++;
				}
				#endif
			}				
		}
		cnt_50ms++;
		if(cnt_50ms>3)//20ms
		{
			cnt_50ms=0;				
			#ifdef WIFI_TY
			if(Get_Wifi_Enable())
			{
				wifi_uart_service();
			}			
			#endif
		}
		cnt_500ms++;
		if(cnt_500ms>99)
		{
			cnt_500ms=0;
			Battery_Current_Filter();
		}		
		vTaskDelayUntil(&xLastWakeTime,5/portTICK_RATE_MS);
	}
}

/* Sensor Task */
volatile uint32_t g_remote_delay_cnt=0;
void Task_SensorsTask(void *argument)
{
	uint32_t battery_voltage_sum=0;
	int32_t system_current_sum = 0,wall_left_adc_sum = 0, wall_right_adc_sum = 0, wall_x_adc_sum = 0;
	int32_t l_wheel_current_sum=0,r_wheel_current_sum=0,main_brush_current_sum=0,side_brush_current_sum=0,vac_current_sum=0;
	uint8_t cycle_cnt=10;
	uint16_t battery_display_cnt = 0;
	uint8_t clean_key_touch_cnt = 0;	
	static uint8_t timer_second_cnt=1,timer_200ms_cnt=5;
 	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	g_remote_delay_cnt=0;
	printf("SensorTask Start!!\n");		
	for(;;)
	{				
		battery_voltage_sum += Battery_GetAdcValue();
		system_current_sum += System_GetAdc();
		side_brush_current_sum += Side_Brush_GetCurrentAdc(); 
		main_brush_current_sum += Main_Brush_GetCurrentAdc();		
		vac_current_sum += Vacuum_GetCurrentAdc();
		l_wheel_current_sum += Wheel_GetLeftCurrentAdc();  
		r_wheel_current_sum += Wheel_GetRightCurrentAdc();				
		wall_right_adc_sum += Wall_GetRightAdcValue();
		wall_left_adc_sum += Wall_GetLeftAdcValue();
		wall_x_adc_sum += Wall_GetXAdcValue();
		
		cycle_cnt--;
		if(cycle_cnt == 0)//40 ms /25 Hz
		{
			cycle_cnt=8;
				
			/*clean key touch*/
			if(Key_GetStatus() == KEY_CLEAN)	
			{
				clean_key_touch_cnt++;
				if(clean_key_touch_cnt > 30)//75
				{
					clean_key_touch_cnt = 0;

				}								
			}	
			else
			{
				clean_key_touch_cnt = 0;
			}
				
			/*Remote delay*/
			if(g_remote_delay_cnt < 100)g_remote_delay_cnt++;

			/*second counter*/
			timer_second_cnt--;
			if(timer_second_cnt == 0)//1s
			{
				timer_second_cnt=25;
				Time_IncreaseCurrentTime();
				battery_display_cnt++;
				if(battery_display_cnt >= 180)//3 min
				{
					battery_display_cnt = 0;
//					Display_ShowBatteryState(Battery_GetCheckState());//display the battery state
				}
			}
			/*Battery Voltage*/
			battery_voltage_sum = (((battery_voltage_sum * 330)/4096)*1000)/180/8;
			Battery_SetVoltage(battery_voltage_sum);
			battery_voltage_sum = 0;
			/*System Current*/
			system_current_sum /= 8;
			system_current_sum = ((system_current_sum - System_GetCurrentBaselineAdc()) * 330) * 10 / 4096;
			System_SetAverageCurrent(system_current_sum);
			system_current_sum = 0;
			/*Side Brush Current*/
			side_brush_current_sum /= 8;
			side_brush_current_sum = ((side_brush_current_sum - g_rightbrush_baseline) * 330) / 4096;
			Side_Brush_SetCurrent(side_brush_current_sum);
      side_brush_current_sum = 0;
			/*Main Brush Current*/
			main_brush_current_sum /= 8;
			main_brush_current_sum = ((main_brush_current_sum - g_mainbrush_baseline) * 330) * 5  / 4096;
			Main_Brush_SetCurrent(main_brush_current_sum);
      main_brush_current_sum = 0;			
			/*Vaccum Current*/
	    vac_current_sum /= 8;
			vac_current_sum = ((vac_current_sum - g_vac_baseline) * 330) * 10 / 4096;
			Vacuum_SetCurrent(vac_current_sum);
			vac_current_sum = 0;	
      /*Left Wheel Current*/
      l_wheel_current_sum /= 8;
      l_wheel_current_sum = (l_wheel_current_sum * 330)*5 / 1024;  
      Wheel_SetLeftCurrent(l_wheel_current_sum);
      l_wheel_current_sum=0;
      /*Right Wheel Current*/
      r_wheel_current_sum /= 8;
      r_wheel_current_sum = (r_wheel_current_sum * 330)*5 / 1024;
      Wheel_SetRightCurrent(r_wheel_current_sum);
      r_wheel_current_sum = 0;				
			/*Set Wall ADC*/
			Wall_SetAverageAdcValue(wall_left_adc_sum/8,wall_right_adc_sum/8,wall_x_adc_sum/8);
			wall_left_adc_sum = 0;
			wall_right_adc_sum = 0;
			wall_x_adc_sum = 0;
			
			/*Touch and Remote event
				swtich to ui mode
			*/
			if(User_GetQuitCheckState()==ENABLE)
			{
				if(Key_GetPressKey())
				{
					printf("key press\n");
					Mode_SetMode(MODE_USERINTERFACE);
				}
				if(Rcon_RemoteKey(Remote_Clean))
				{
					printf("%s(%d):Rcon_RemoteKey = REMOTE_CLEAN Mode_SetMode(MODE_USERINTERFACE)\n",__FUNCTION__, __LINE__);
					Mode_SetMode(MODE_USERINTERFACE);
				}
			}
			
//			Motor_CheckCurrentProcess(Motor_GetCheckState());//only check motor current when cleanning/moving					
			timer_200ms_cnt--;				
			if(timer_200ms_cnt==0)//200 ms
			{
				timer_200ms_cnt=5;
				Wall_DynamicProcess(Wall_GetDynamicState());//adjust wall sensor bias
				OBS_DynamicProcess(OBS_GetDynamicState());//adjust obs sensor bias 			
				Bumper_CheckProcess(Bumper_GetCheckState());//check if the bumper is always pressed	
				#ifdef WIFI_TY
				Wifi_Even_Report();
				#endif
			}
			TurnSlip_DetectProcess(TurnSlip_GetCheckState());//check if robot slip while turning 
			CM_UpdatePosition(Gyro_GetAngle(0), Gyro_GetAngle(1), Wheel_GetLeftCount(), Wheel_GetRightCount());
 		}						
		Cliff_DetectionProcess(Cliff_GetDetectState());
		
		vTaskDelayUntil(&xLastWakeTime,5/portTICK_RATE_MS);
	}	
}

/* Task_CalculatePathTask
* this task only runs while robot being trapped
* calculates if there's any path can access to the unclean area
* parse a signal to CM_WallTolane() to quit wall following
*/
xSemaphoreHandle g_binary_wallcalculate_found;
xSemaphoreHandle g_binary_wallcalculate_start;
void Task_CalculatePathTask(void *agument)
{
	vSemaphoreCreateBinary(g_binary_wallcalculate_start);
	vSemaphoreCreateBinary(g_binary_wallcalculate_found);
	xSemaphoreTake(g_binary_wallcalculate_start,0);
	printf("Task_CalculatePathTask Start!\n");
	for(;;)
	{

		xSemaphoreTake(g_binary_wallcalculate_start,portMAX_DELAY);
//		if(g_next_t==1)
//		{
//			g_next_t = 0;
//			Path_Next_V2(PathList_ReadLastPath().cell_pos);
//		}
//		else if(g_next_t==2)	
		{
			if(Path_WallFineWay(Map_GetRobotCell()) ==  PATH_STATE_NORMAL_CLEAN) // new shortest path found
			{
				xSemaphoreGive(g_binary_wallcalculate_found);//give a signal to wall follow function
			}		
		}			
	}
}

