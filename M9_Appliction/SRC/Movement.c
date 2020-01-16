/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Basical Movement functions
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



/*work time functions*/
volatile uint32_t g_second_cnt=0;
uint32_t Time_GetCurrentTime(void)//return time by second
{
	return g_second_cnt;
}
void Time_ResetCurrentTime(void)
{
	g_second_cnt = 0;
}
void Time_IncreaseCurrentTime(void)
{
	g_second_cnt++;
}

/*motor check*/
volatile uint8_t g_check_motor_status =0;
volatile FunctionalState g_check_motor_current_state = DISABLE;
void Motor_CheckCurrentProcess(FunctionalState state)
{
	if(state == DISABLE)return;

	if(Wheel_CheckLeftCurrent()) 	g_check_motor_status  |=  CHECK_L_WHEEL;

	if(Wheel_CheckRightCurrent())	g_check_motor_status  |=  CHECK_R_WHEEL;

	if(g_left_wheel_slow > 100)  	g_check_motor_status  |=  CHECK_L_WHEEL;

	if(g_right_wheel_slow > 100) 	g_check_motor_status  |=  CHECK_R_WHEEL;
	
	if(Side_Brush_CheckCurrent())	g_check_motor_status  |=  CHECK_MAIN_BRUSH;

//edit by vin	
//	if(Main_Brush_CheckCurrent())	g_check_motor_status  |=  CHECK_SIDE_BRUSH;
//	if(Vacuum_CheckCurrent())    	g_check_motor_status  |=  CHECK_VACUUM;
//	if(Vacuum_IsFail())					 	g_check_motor_status  |=  CHECK_VACUUM;	

	if(Bumper_IsFailed())        	g_check_motor_status  |=  CHECK_BUMPER;	
}
uint8_t Motor_GetStatus(void)
{
	return g_check_motor_status;
}
void Motor_ResetStatus(void)
{
	g_check_motor_status = 0;
}

void Motor_SetCheckState(FunctionalState state)
{
	g_check_motor_current_state = state;
}
FunctionalState Motor_GetCheckState(void)
{
	return g_check_motor_current_state;
}

/*user quit functions*/
volatile FunctionalState g_userquit_check_state = DISABLE;
void User_SetQuitCheckState(FunctionalState state)
{
	g_userquit_check_state = state;
}
FunctionalState User_GetQuitCheckState(void)
{
	return g_userquit_check_state;
}

/*battery voltage*/
volatile int32_t g_battery_voltage=0;
void Battery_SetVoltage(uint16_t voltage)
{
	g_battery_voltage = voltage;
}
int32_t Battery_GetVoltage(void)
{
	return g_battery_voltage;
}
uint8_t Battery_IsLow(void)
{
  if(Battery_GetVoltage() < LOW_BATTERY_VOLTAGE)return 1;
  return 0;
}
uint16_t Battery_GetAdcValue(void)
{
	return g_adc_value.Battery_Voltage;
}

/*bat state*/
volatile FunctionalState g_battery_check_state = DISABLE;
volatile Battery_t  g_battery_state = BATTERY_FULL;
void Battery_SetState(Battery_t state)
{
	g_battery_state = state;
}
Battery_t Battery_GetState(void)
{
  return g_battery_state;
}
void Battery_SetCheckState(FunctionalState state)
{
	g_battery_check_state = state;
}
FunctionalState Battery_GetCheckState(void)
{
	return g_battery_check_state;
}


/*ABS*/
int32_t ABS_Minus(int32_t A,int32_t B)
{
	if(A>B)
	{
		return A-B;
	}
	return B-A;
}

/*Set Power*/
void Power_EnableAll(void)
{
	CTRL_ALL_POWER_ON;	
	Set_3v3(Power_On);
	Set_5v(Power_On);	
	WIFI_POWER_ON;	
}
void Power_DisableAll(void)
{
	Set_3v3(Power_Off);
	Set_5v(Power_Off);
	WIFI_POWER_OFF;		
}
void Set_3v3(uint8_t data)
{
	if(data==Power_On)
	{
		CTRL_SW_3V_ON;
	}
	else
	{
		CTRL_SW_3V_OFF;
	}
}
void Set_5v(uint8_t data)
{
	if(data==Power_On)
	{		
		CTRL_SW_5V_ON;
	}
	else
	{
		CTRL_SW_5V_OFF;
	}
}

/*mode functions*/
uint8_t g_loop_ture=0;
void Set_Loop_Ture(uint8_t ture)
{
	g_loop_ture = ture;
}
uint8_t Get_Loop_Ture(void)
{
	return g_loop_ture;
}
volatile CleanMode_t g_mode_log[3] = {MODE_USERINTERFACE};
CleanMode_t g_mode_selection = MODE_NAVIGATION;
void Mode_SetMode(CleanMode_t mode)//store a new mode to the clean mode list(3 element)
{
	Mode_UpdateLog(mode);
	#ifdef WIFI_TY
	//暂停未实现
	if(!Wifi_Get_Mode_Flag())
	{
		if(mode == MODE_USERINTERFACE)
		{
			SysConfig.Ty_CurrentState = IDLE_STATE;
		}
		else if(mode == MODE_NAVIGATION)
		{
			SysConfig.Ty_CurrentState = AUTO_STATE;	
		}
		else if(mode == MODE_MOP)
		{
			SysConfig.Ty_CurrentState = MOP_STATE;	
		}	
		else if(mode == MODE_WALL)
		{
			SysConfig.Ty_CurrentState = WALL_STATE;	
		}		
		else if(mode == MODE_HOME)
		{
			SysConfig.Ty_CurrentState = HOME_STATE;	
		}	
		else if(mode == MODE_SPOT)
		{
			SysConfig.Ty_CurrentState = SPOT_STATE;	
		}
		else if(mode == MODE_SINGLE_ROOM)
		{
			SysConfig.Ty_CurrentState = SINGLE_ROOM_STATE;	
		}	
	}
	
	#endif
}
CleanMode_t Mode_GetMode(void)//get current cleaning mode
{
	return Mode_GetLog(0);
}
void Mode_UpdateLog(CleanMode_t mode)//store a new mode to the clean mode list(3 element)
{
	g_mode_log[2] = g_mode_log[1];
	g_mode_log[1] = g_mode_log[0];
	g_mode_log[0] = mode;
}
CleanMode_t Mode_GetLog(uint8_t idx)//get a clean mode list content by index: In
{
	return g_mode_log[idx];
}

void Mode_SetModeSelection(CleanMode_t mode)
{
	 g_mode_selection = mode;
}
CleanMode_t Mode_GetModeSelection(void)
{
	return g_mode_selection;
}
void Mode_SetModeAndSelection(CleanMode_t mode)
{
	Mode_SetModeSelection(mode);
  Mode_SetMode(mode);
}

#ifdef WIFI_TY
CleanMode_t g_wifi_selection = MODE_NAVIGATION;
uint8_t g_wifi_set_mode_flag=0;
void Wifi_Mode_SetMode(CleanMode_t mode)
{
	Wifi_Set_Mode_Flag(1);
	g_wifi_selection = mode;
}
CleanMode_t Wifi_Mode_GetMode(void)
{
	return g_wifi_selection;
}
void Wifi_Set_Mode_Flag(uint8_t flag)
{
	g_wifi_set_mode_flag = flag;
}
uint8_t Wifi_Get_Mode_Flag(void)
{
	return g_wifi_set_mode_flag;
}
#endif

/*Error Code & Stuck*/
volatile Error_t g_error_code = ERROR_NONE;
volatile uint8_t g_stuck_cnt = 0;
Error_t Error_GetCode(void)
{
  return g_error_code;
}
void Error_SetCode(Error_t Code)
{
  g_error_code = Code;
}
void Error_SetCleanCode(Error_t Code)
{
  g_error_code &= ~Code;
}
void Error_ResetCode(void)
{
	Error_SetCode(ERROR_NONE);
	g_stuck_cnt = 0;
}
uint8_t Stuck_IsStucked(void)
{
	g_stuck_cnt++;
	if(g_stuck_cnt > 3)return 1;
	return 0;
}

/*Bumper Error*/
volatile uint8_t g_bumper_error = 0;
void Bumper_ResetErrorCnt(void)
{
  g_bumper_error=0;
}
uint8_t Bumper_IsFailed(void)
{
	if(Bumper_GetTrigStatus())
	{
		g_bumper_error++;
	}
	else
	{
		g_bumper_error = 0;
	}
	if(g_bumper_error > 30)
	{
		g_bumper_error = 0;
		return 1;
	}	
	else 
  {		
		return 0;
	}
}
/*Bumper_GetTrigStatus*/
uint8_t Bumper_GetTrigStatus(void)
{
  uint8_t Temp_Status=0;
//  if(Is_FLeftBumper_Trig())
//  {
//		if(Is_FLeftBumper_Trig())
//		Temp_Status |= FLeftBumperTrig;
//  }
	if(Is_LeftBumper_Trig())
	{
		if(Is_LeftBumper_Trig())
		Temp_Status |= LLeftBumperTrig;	
	}
		
//  if(Is_FRightBumper_Trig())
//  {
//		if(Is_FRightBumper_Trig())
//		Temp_Status |= FRightBumperTrig;
//  }
	if(Is_RightBumper_Trig())
	{
		if(Is_RightBumper_Trig())
		Temp_Status |= RRightBumperTrig;	
	}
  return Temp_Status;
}

/*bumper check*/
volatile FunctionalState g_bumper_check_state = DISABLE;
void Bumper_CheckProcess(FunctionalState state)
{
	static uint8_t l_bumper_error = 0,r_bumper_error = 0;

	if(state == DISABLE)return;
	
	if(Bumper_GetTrigStatus()&LeftBumperTrig)
	{
		l_bumper_error++;		
	}
	else
	{
		l_bumper_error = 0;
	}

	if(Bumper_GetTrigStatus()&RightBumperTrig)
	{
		r_bumper_error++;
	}
	else
	{
		r_bumper_error = 0;
	}	
	if((l_bumper_error > 15) ||(r_bumper_error > 15))
	{
		g_check_motor_status |=  CHECK_BUMPER;
		l_bumper_error = 0;
		r_bumper_error = 0;
	}
}
void Bumper_SetCheckState(FunctionalState state)
{
  g_bumper_check_state = state;
}
FunctionalState Bumper_GetCheckState(void)
{
  return  g_bumper_check_state; 
}



/*motor power*/
volatile int32_t g_temp_brush_power = 0, g_temp_main_brush_power = 0;
volatile int32_t g_brush_power = 0,g_main_brush_power = 0;
volatile FunctionalState g_motor_state = DISABLE;
void Motor_SetState(FunctionalState state)
{
	g_motor_state = state;
	if(g_motor_state == DISABLE)
	{
		Brush_Main_SetPWM(0);
		Brush_Side_SetPWM(0);
		g_temp_brush_power = 0;
		g_temp_main_brush_power = 0;
	}
}
FunctionalState Motor_GetState(void)
{
	return g_motor_state;
}
void Motor_SetPower(uint32_t vac_speed, int32_t brush_power, int32_t main_brush_power)
{
	g_main_brush_power=main_brush_power;
	g_brush_power = brush_power;
	Vacuum_SetSpeed(vac_speed);
	if((g_brush_power != 0))Motor_SetState(ENABLE);
}
void Motor_TunePower(void)
{
	int32_t temp_battery_voltage = 0, temp_brush_target_power = 0, temp_main_brush_target_power = 0;
	
	//battery  voltage
	temp_battery_voltage =	Battery_GetVoltage();
	
	//brush
	temp_brush_target_power = (uint8_t)(g_brush_power/temp_battery_voltage);	 
	if(temp_brush_target_power > 100)temp_brush_target_power = 100;
	if(g_temp_brush_power < temp_brush_target_power)g_temp_brush_power++;
	else g_temp_brush_power--;	
	
	//main brush
	temp_main_brush_target_power = (uint8_t)(g_main_brush_power/temp_battery_voltage);	 
	if(temp_main_brush_target_power > 100)temp_main_brush_target_power = 100;		
	if(g_temp_main_brush_power < temp_main_brush_target_power)g_temp_main_brush_power++;
	else g_temp_main_brush_power--;	
	
  //set brush pwm	
	if(temp_brush_target_power == 0)g_temp_brush_power = 0;
  if(g_temp_brush_power < 0)g_temp_brush_power=0;
	Brush_Side_SetPWM(g_temp_brush_power);

  //set main brush pwm	
	if(temp_main_brush_target_power == 0)g_temp_main_brush_power = 0;
  if(g_temp_main_brush_power < 0)g_temp_main_brush_power=0;
	Brush_Main_SetPWM(g_temp_main_brush_power);
}


/*------------------------------------Disable all motors---------------------*/
void Motor_DisableAll(void)
{
	Wheel_Stop();
	WHEEL_DISABLE();
	Motor_SetPower(0, 0, 0);
	Vacuum_TurnOff();
	ActList_Clear();
	Motor_SetCheckState(DISABLE);
	Cliff_SetDetectState(DISABLE);	
}
void Motor_WorkConfigure(void)
{
	if(Mode_GetMode() == MODE_USERINTERFACE)return;
	Power_EnableAll();
	Wheel_ResetSlowCnt();
  Bumper_ResetErrorCnt();
	Motor_SetPower(VAC_SPEED_NORMAL,CLEAN_SIDE_BRUSH_POWER,CLEAN_MAIN_BRUSH_POWER);
	Cliff_SetDetectState(ENABLE);
}
void Motor_HomeConfigure(void)
{
	if(Mode_GetMode() == MODE_USERINTERFACE)return;
	Power_EnableAll();
	Wheel_ResetSlowCnt();
  Bumper_ResetErrorCnt();
	Motor_SetPower(VAC_SPEED_ECO,HOME_SIDE_BRUSH_POWER,HOME_MAIN_BRUSH_POWER);
	Cliff_SetDetectState(ENABLE);
}
void Motor_SpotConfigure(void)
{
	if(Mode_GetMode() == MODE_USERINTERFACE)return;
	Power_EnableAll();
	Wheel_ResetSlowCnt();
  Bumper_ResetErrorCnt();
	Motor_SetPower(VAC_SPEED_MAX,CLEAN_SIDE_BRUSH_POWER,CLEAN_MAIN_BRUSH_POWER);
	Cliff_SetDetectState(ENABLE);
}





/*Random Factor*/
volatile uint8_t g_random_counter=0;
uint8_t System_GetRandomValue(void)
{
	uint8_t temp=0;	
  if(g_random_counter>100)g_random_counter=100;
  temp = g_random_counter;
	g_random_counter = 0;
	return temp;
}


/*direction functions*/
volatile uint8_t g_direction_flag = 0;
void Direction_SetLastDir(uint8_t Dir)
{
	g_direction_flag = Dir;
}
uint8_t Direction_GetLastDir(void)
{
	return g_direction_flag;
}
uint8_t Direction_IsLastDirRight(void)
{
  if(Direction_GetLastDir() == DIRECTION_FLAG_RIGHT)return 1;
	return 0;
}
uint8_t Direction_IsLastDirLeft(void)
{
  if(Direction_GetLastDir() == DIRECTION_FLAG_LEFT)return 1;
	return 0;
}

//电池电量计算
uint8_t const battery_level[47] = {   
				 0 ,1 ,1 ,1 ,1 ,2 ,2 ,2 ,2 ,2,  //12.0----12.9
				 2 ,2 , 3, 4, 4, 5, 6,10,13,15, //13.0----13.9
				 18,21,26,33,41,48,52,55,58,61, //14.0----14.9
				 64,67,69,72,74,76,79,81,83,86, //15.0----15.9
				 88,92,95,98,100,100,100,       //16.0----16.9
				};
BatteryCapacity_t g_batterycapacity;
void Battery_AddCapacity_Current(uint16_t sys_current_adc)
{
	g_batterycapacity.temporary += sys_current_adc;
	
	if(sys_current_adc < g_batterycapacity.min)g_batterycapacity.min = sys_current_adc;
	
	if(sys_current_adc > g_batterycapacity.max)g_batterycapacity.max = sys_current_adc;
	
	g_batterycapacity.cnt++;
}	

void Battery_Current_Filter(void)
{
	uint64_t temp_cost=0;
	int32_t temp_current = 0; 
	uint8_t current_cnt = 0;
	static uint8_t bat_cnt=100;
		
	if(g_batterycapacity.cnt <= 20)return ;
	
	current_cnt = g_batterycapacity.cnt;
	temp_current = 10 + ((g_batterycapacity.temporary - (g_batterycapacity.min + g_batterycapacity.max))/(current_cnt - 2));	
	
	if(Is_ChargerOn())
	{
		temp_current = (System_GetCurrentBaselineAdc() - temp_current)*330*10/4096;
		if(temp_current < 0)temp_current = 0;
		
//		Usprintf("%s(%d):temporary:%d,cnt:%d,min:%d,max:%d,%d,%d\n",__FUNCTION__,__LINE__,g_batterycapacity.temporary,current_cnt,g_batterycapacity.min,g_batterycapacity.max,data,g_system.charge_current);
		g_batterycapacity.cost -= temp_current*current_cnt*2/1000;
		if(g_batterycapacity.cost < 0)g_batterycapacity.cost = 0;
	}
	else
	{
		temp_current = (temp_current - System_GetCurrentBaselineAdc())*330*10/4096;
		if(temp_current < 0)temp_current = 0;
		
		g_batterycapacity.cost += temp_current*current_cnt*2/1000;
		  
		if(g_batterycapacity.cost > BATTERY_CAPACITY_MAX)g_batterycapacity.cost = BATTERY_CAPACITY_MAX;	
	}
	
	if(Battery_GetVoltage()>=1645)
	{
		g_batterycapacity.level = 100;
		bat_cnt = 100;
	}
	else
	{
		if(bat_cnt)bat_cnt--;
		if(!bat_cnt)
		{
			temp_cost = BATTERY_CAPACITY_MAX - g_batterycapacity.cost;
			temp_cost *= 100;
			temp_cost /= BATTERY_CAPACITY_MAX;
			g_batterycapacity.level = temp_cost;			
		}
	}
	
	g_batterycapacity.min = 4096;
	g_batterycapacity.max = 0;
	g_batterycapacity.temporary = 0;
	g_batterycapacity.cnt = 0;
}

uint8_t Battery_Get_CapacityLevel(void)
{
	return g_batterycapacity.level;
}

void Battery_Capacity_Reset(void)
{
	int16_t voltage_level = 0;
	voltage_level = (Battery_GetVoltage() - 1200)/10;
	if(voltage_level < 0)voltage_level = 0;
	if(voltage_level > 46)voltage_level = 46;
	
	
	g_batterycapacity.level = battery_level[voltage_level];
	
	g_batterycapacity.cost = (100 - g_batterycapacity.level)*BATTERY_CAPACITY_MAX/100;
	
	printf2("%s(%d):battery voltage:%d,level:%d,cost:%d\n",__FUNCTION__,__LINE__,Battery_GetVoltage(),g_batterycapacity.level,g_batterycapacity.cost);
}


