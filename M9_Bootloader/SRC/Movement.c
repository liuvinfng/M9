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
#include "Movement.h"
#include "Speaker.h"
#include "Rcon.h"
#include "Charge.h"
#include "USART.h"
#include "Debug.h"



volatile uint8_t g_ad_finish_flag=0;

volatile uint8_t	Main_BrushStall_Flag = 0,Left_BrushStall_Flag = 0,Right_BrushStall_Flag = 0;

volatile int16_t MainBrush_Short_Base=4090,RightBrush_Short_Base=4090,Vacuum_Short_Base=4090;
volatile uint16_t MainBrush_Short_Circuit_Counter=0,RightSideBrush_Short_Circuit_Counter=0,Vacuum_Short_Circuit_Counter=0;

volatile int32_t MainBrush_Stall_Base=4096,RightBrush_Stall_Base=4096;




/*battery voltage*/
volatile int32_t g_battery_voltage=0;
int32_t Get_Temp_BatVoltage(void)
{
	return g_battery_voltage;
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
void Enable_PPower(void)
{
	CTRL_ALL_POWER_ON;	
	Set_3v3(Power_On);
	Set_5v(Power_On);	
	WIFI_POWER_ON;	
}
void Disable_PPower(void)
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
volatile CleanMode_t g_mode_log[3] = {MODE_USERINTERFACE};
CleanMode_t g_mode_selection = MODE_NAVIGATION;
void Mode_SetMode(CleanMode_t mode)//store a new mode to the clean mode list(3 element)
{
	Mode_UpdateLog(mode);
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

/*Bumper Error*/
volatile uint8_t g_bumper_error = 0;
void Reset_Bumper_Error(void)
{
  g_bumper_error=0;
}
uint8_t Is_Bumper_Fail(void)
{
  g_bumper_error++;
	if(g_bumper_error>3)return 1;
	else return 0;
}

/*Error Code*/
volatile uint8_t g_error_code =0;
uint8_t Get_Error_Code(void)
{
  return g_error_code;
}
void Set_Error_Code(uint8_t Code)
{
  g_error_code = Code;
}
void Reset_Error_Code(void)
{
	if(Get_Error_Code())
	{
		Set_Error_Code(0);
	}
}



/*Get_Bumper_Status*/
uint8_t Get_Bumper_Status(void)
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


/*-------------- Move Back -----------------------------*/
void Move_Back(void)
{
  uint16_t Counter_Watcher=0;
	uint8_t motor_check=0;
  Set_Dir_Backward();
	Set_Wheel_Speed(RUN_SPEED_9,RUN_SPEED_9);
  Left_Wheel_Step=0;Right_Wheel_Step=0;//reset step counter
	Counter_Watcher=0;
  while((Left_Wheel_Step<4*DISTANCE_1CM)||(Right_Wheel_Step<4*DISTANCE_1CM))
	{
	  delay(10);
		Counter_Watcher++;
	  if(Counter_Watcher>3000)
		{
			if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return;
		}
		if(Touch_Detect())
		{
			return;
		}
		if(Get_Cliff_Trig() & Status_Cliff_Back)
		{
			return;		
		}		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
  Reset_TempPWM();
}	

/*-------------------------------------Back --------------------*/
void Quick_Back(uint8_t Speed,uint16_t Distance)
{
  uint16_t Counter_Watcher=0;
	uint8_t motor_check=0;
  Set_Dir_Backward();
	Set_Wheel_Speed(Speed,Speed);
  Left_Wheel_Step = 0;
  Right_Wheel_Step = 0;
	Counter_Watcher=0;
  while((Left_Wheel_Step<Distance)||(Right_Wheel_Step<Distance))
	{
	  delay(10);
		Counter_Watcher++;
	  if(Counter_Watcher>3000)
		{
		  if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return;
		}
		if(Touch_Detect())
		{
			return;
		}
		if(Get_Cliff_Trig() & Status_Cliff_Back)
		{
			return;		
		}		
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
}


/*-------- Turn Left ------------------------*/
void Turn_Left(uint16_t speed,uint16_t angle)
{
		
  uint16_t Counter_Watcher=0;
	uint8_t motor_check=0;
  Set_Dir_Left();
  Left_Wheel_Step=0; 
  Reset_TempPWM();   
	Set_Wheel_Speed(speed,speed);
	Counter_Watcher=0;
  Rcon_ResetRemoteCode();
	angle = angle*ANGLE_MUL;
  while(Left_Wheel_Step<angle)
	{
				
	  delay(10);
		Counter_Watcher++;
	  if(Counter_Watcher>4000)
		{
			if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return;
		}
    if(Is_Turn_Remote())return;
		if(Get_Turn_Check_Cliff())
		{
			if(Get_Cliff_Trig())return;
		}
		if(Touch_Detect())
		{
			return;
		}
//		if(Get_Bumper_Status())return;
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
  Stop_Brifly();	
}

/*-------- Tur_Right ------------------------*/
void Turn_Right(uint16_t speed,uint16_t angle)
{
  uint16_t Counter_Watcher=0;
	uint8_t motor_check=0;
  Set_Dir_Right();
  Right_Wheel_Step=0;
  Reset_TempPWM();
  Set_Wheel_Speed(speed,speed);
	Counter_Watcher=0;
  Rcon_ResetRemoteCode();
	angle = angle*ANGLE_MUL;
  while(Right_Wheel_Step<angle)
	{
			
	  delay(10);
		Counter_Watcher++;
	  if(Counter_Watcher>4000)
		{
		  if(Is_Encoder_Fail())
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
			}
			return;
		}
    if(Is_Turn_Remote())return;
//		if(Get_Bumper_Status())return;		
		if(Get_Turn_Check_Cliff())
		{
			if(Get_Cliff_Trig())return;
		}
		if(Touch_Detect())
		{
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
		
	}

	Stop_Brifly();
}


/*------------------------------------Disable all motors---------------------*/
void Disable_Motors(void)
{
  Set_MainBrush_PWM(0);
	Set_Dir_Forward();
	Set_Wheel_Speed(0,0);
	WHEEL_DISABLE();
	Set_SideBrush_PWM(0,0);
	Turn_BLDC_Off();
}
/*----------------------------------------Initialize Motors ----------------------*/
void Initialize_Motor(void)
{
	Enable_PPower();
	Clear_BLDC_Fail();
	BLDC_OFF;
  delay(2000);
	Set_BLDC_TPWM(60);
	Set_Vac_Speed();
  Set_MainBrush_PWM(50);
  Set_SideBrush_PWM(60,60);
	BLDC_ON;
	delay(3000);
	Move_Forward(0,0);
  Reset_TempPWM();
	Left_Wheel_Slow=0;
	Right_Wheel_Slow=0;
  Reset_Bumper_Error();
}

uint8_t Check_LeftWheel_Current(void)//Left Wheel  Current
{
	static uint8_t Static_LeftWheel_OC=0;
	if(g_adc_value.Left_Wheel_Current<Wheel_Stall_LimitL)
	{
	}
  else if(g_adc_value.Left_Wheel_Current>g_leftwheel_baseline+Wheel_Stall_LimitH)
  {
	  Static_LeftWheel_OC+=2;
  }
  else
  {
	  Static_LeftWheel_OC=0;
  }
	if(Static_LeftWheel_OC>250)
	{		
		Static_LeftWheel_OC=0;
		return 1;
	}
	return 0;
}
uint8_t SelfCheck_LeftWheel(void)
{
	uint32_t Time_Out=0;
	uint32_t SelfCheck_Current_Summary=0;
	Left_Wheel_Slow=0;
	
	Set_Dir_Backward();

	Set_Wheel_Speed(RUN_SPEED_10,0);
	delay(5000);
	Time_Out=10;
	SelfCheck_Current_Summary=0;
	while(Time_Out--)
	{
		SelfCheck_Current_Summary += g_adc_value.Left_Wheel_Current;
		delay(1000);
	}
	SelfCheck_Current_Summary/=10;
	if((SelfCheck_Current_Summary>g_leftwheel_baseline+Wheel_Stall_LimitH)||(SelfCheck_Current_Summary<Wheel_Stall_LimitL))
	{
		Disable_Motors();
		Set_Error_Code(Error_Code_LeftWheel);
		return 1;
	}
	Stop_Brifly();
	Turn_Left(Turn_Speed,1800);
	return 0;
}

uint8_t Check_RightWheel_Current(void)
{
	static uint16_t Static_RightWheel_OC=0;
	if(g_adc_value.Right_Wheel_Current<Wheel_Stall_LimitL)
	{
	}
  else if(g_adc_value.Right_Wheel_Current>g_rightwheel_baseline+Wheel_Stall_LimitH)
  {
	  Static_RightWheel_OC+=2;
  }
  else
  {
	  Static_RightWheel_OC=0;
  }
	if(Static_RightWheel_OC>250)
	{		
		Static_RightWheel_OC=0;
		return 1;
	}
	return 0;
}

uint8_t SelfCheck_RightWheel(void)
{
	uint32_t Time_Out=0;
	uint32_t SelfCheck_Current_Summary=0;
	Right_Wheel_Slow=0;

	Set_Dir_Backward();

	Set_Wheel_Speed(0,RUN_SPEED_10);
	delay(5000);
	Time_Out=10;
	SelfCheck_Current_Summary=0;
	while(Time_Out--)
	{
		SelfCheck_Current_Summary += g_adc_value.Right_Wheel_Current;
		delay(500);
	}
	SelfCheck_Current_Summary/=10;
	if((SelfCheck_Current_Summary>g_rightwheel_baseline+Wheel_Stall_LimitH)||(SelfCheck_Current_Summary<Wheel_Stall_LimitL))
	{
		Disable_Motors();
		Set_Error_Code(Error_Code_RightWheel);
		return 1;		
	}
	Stop_Brifly();
	Turn_Right(Turn_Speed,1800);
	return 0;
}

/*Adjust Motor Stall Limit By Battery Volage and Motor PWM */
void adjust_Short_Limit(uint32_t Bat_V)
{

	Vacuum_Short_Base=4090;
	MainBrush_Short_Base=(Bat_V*Main_Brush_PWM*MainBrush_Short_Circuit/160000) +g_mainbrush_baseline;
	if(MainBrush_Short_Base<3000)MainBrush_Short_Base=3000;
	if(MainBrush_Short_Base>4090)MainBrush_Short_Base=4090;
		
	RightBrush_Short_Base=(Bat_V*Right_Brush_PWM*SideBrush_Short_Circuit/160000)+g_rightbrush_baseline;
	if(RightBrush_Short_Base<2500)RightBrush_Short_Base=2500;
	if(RightBrush_Short_Base>4090)RightBrush_Short_Base=4000;
}
void Adjust_MainBrush_StallLimit(int32_t Bat_V)
{
	int32_t Temp_M_PWM=0;
  if(Bat_V<1200)Bat_V=1200;
	
	Temp_M_PWM = Main_Brush_PWM;
	Temp_M_PWM-=15;
	if(Temp_M_PWM<0)Temp_M_PWM=1;
	Temp_M_PWM=Temp_M_PWM*100/80;
	MainBrush_Stall_Base=(Bat_V*Temp_M_PWM*MainBrush_Stall_BaseLimit/160000) +g_mainbrush_baseline;
	if(MainBrush_Stall_Base<2000)MainBrush_Stall_Base=2000;
	if(MainBrush_Stall_Base>3500)MainBrush_Stall_Base=3500;
}
void Adjust_RightBrush_StallLimit(int32_t Bat_V)
{
	int32_t Temp_M_PWM=0;
	
	if(Is_RightBrush_Enable())
	{
		Temp_M_PWM = Right_Brush_PWM;
		Temp_M_PWM-=15;
		if(Temp_M_PWM<0)Temp_M_PWM=1;
		Temp_M_PWM=Temp_M_PWM*100/85;
		RightBrush_Stall_Base=(Bat_V*Temp_M_PWM*SideBrush_Stall_BaseLimit/160000) +g_rightbrush_baseline;
		if(RightBrush_Stall_Base<2000)RightBrush_Stall_Base=2000;
		if(RightBrush_Stall_Base>3000)RightBrush_Stall_Base=3000;
	}
}



uint8_t Is_RightBrush_Stall(void)
{
	static uint8_t Static_RightBrush_Stall=0;
	
	if(!Is_RightBrush_Enable())return 0;
	Adjust_RightBrush_StallLimit(Get_Temp_BatVoltage());
	if(g_adc_value.Right_Brush_Current<(g_rightbrush_baseline+SideBrush_Stall_LimitL))
	{

	}
	else if(g_adc_value.Right_Brush_Current>RightBrush_Stall_Base)
	{
		Static_RightBrush_Stall+=5;
	}
	else
	{
		Static_RightBrush_Stall=0;
	}
	if(Static_RightBrush_Stall>100)
	{
		Static_RightBrush_Stall=0;
		Set_Right_Brush(DISABLE);
		Set_RightBrush_Stall(1);
	}
	return 0;
}

//stall flag
void Set_LeftBrush_Stall(uint8_t L)
{
	Left_BrushStall_Flag = L;
}
uint8_t Get_LeftBrush_Stall(void)
{
	return Left_BrushStall_Flag;
}
void Set_RightBrush_Stall(uint8_t R)
{
	Right_BrushStall_Flag = R;
}

uint8_t Get_RightBrush_Stall(void)
{
	return Right_BrushStall_Flag;
}

void Set_MainBrush_Stall(uint8_t M)
{
	Main_BrushStall_Flag = M;
}
uint8_t Get_MainBrush_Stall(void)
{
	return Main_BrushStall_Flag;
}

/*-----------------------------Vacuum---------------*/
uint8_t Is_Vacuum_Stall(void)
{
	return 0;
}
uint8_t SelfCheck_Vacuum(void)
{
	uint32_t Time_Out=0;
	uint32_t SelfCheck_Current_Summary=0;
	Set_Wheel_Speed(0,0);
	#ifdef BLDC_INSTALL
	BLDC_OFF;
	delay(1000);
	Set_BLDC_TPWM(30);
	Set_Vac_Speed();
	BLDC_ON;
	delay(5000);
	Time_Out=10;
	SelfCheck_Current_Summary=0;
	while(Time_Out--)
	{
		SelfCheck_Current_Summary += g_adc_value.Vacuum_Current;
		delay(1000);
	}
	SelfCheck_Current_Summary/=10;
	if ((SelfCheck_Current_Summary > (g_vac_baseline + Vacuum_Stall_LimitH)) || (SelfCheck_Current_Summary < (g_vac_baseline + Vacuum_Stall_LimitL)))
	{
		Disable_Motors();
		Set_Error_Code(Error_Code_Fan_H);
		return 1;
	}
	#endif
	return 0;
}

/*----------------------------------------------------------Main Brush OC ---------------------------------------*/
uint8_t Is_MainBrush_Stall(void)
{
	static uint8_t Static_MainBrush_Stall=0;
	if(Main_Brush_PWM<10)return 0;
	Adjust_MainBrush_StallLimit(Get_Temp_BatVoltage());
	if(g_adc_value.Main_Brush_Current<(g_mainbrush_baseline+MainBrush_Stall_LimitL))
	{
		if(Main_Brush_PWM>10)Static_MainBrush_Stall++;
	}
	else if(g_adc_value.Main_Brush_Current>MainBrush_Stall_Base)
	{
		Static_MainBrush_Stall+=4;
	}
	else
	{
		Static_MainBrush_Stall=0;
	}
	
	if(Static_MainBrush_Stall>200)
	{
		Static_MainBrush_Stall=0;
		return 1;
	}
	return 0;
}

uint8_t SelfCheck_MainBrush(void)
{
	uint32_t Time_Out=0;
	uint32_t SelfCheck_Current_Summary=0;
  Move_Back();
	Turn_Right(Turn_Speed,1800);
	Set_Wheel_Speed(0,0);
	Set_MainBrush_PWM(60);
	adjust_Short_Limit(Get_Temp_BatVoltage());
	delay(5000);
	Adjust_MainBrush_StallLimit(Get_Temp_BatVoltage());
	Time_Out=10;
	SelfCheck_Current_Summary=0;
	while(Time_Out--)
	{
		SelfCheck_Current_Summary += g_adc_value.Main_Brush_Current;
		delay(1000);
	}
	SelfCheck_Current_Summary/=10;
	if((SelfCheck_Current_Summary>MainBrush_Stall_Base)||(SelfCheck_Current_Summary<(g_mainbrush_baseline+MainBrush_Stall_LimitL)))
	{
		Disable_Motors();
		Set_Error_Code(Error_Code_MainBrush);
		return 1;
	}
	Set_MainBrush_Stall(0);
	return 0;
}

/*----------------------------------------------------------Check_Motor_Current-----------------*/
uint8_t Check_Motor_Current(void)
{
	static uint8_t BLDC_Broken=80;

	if(Get_Error_Code())
	{
		return Error_Short_Circuit;
	}
	if(g_ad_finish_flag==1)
	{
		g_ad_finish_flag = 0;
	  /*------------------------------------------------Check_Left_Wheel-----------------*/
	 if(Check_LeftWheel_Current())return Check_Left_Wheel;
	  /*------------------------------------------------Check_Right_Wheel-----------------*/
	 if(Check_RightWheel_Current())return Check_Right_Wheel; 
		/*------------------------------------------------Check_Right_Brush-----------------*/	 
    if(Is_RightBrush_Stall())return Check_Right_Brush;
		/*------------------------------------------------Check_Left_Wheel Check_Right_Wheel-----------------*/
    if(Right_Wheel_Slow>100)
		{
		  return Check_Right_Wheel;
		}
		if(Left_Wheel_Slow>100)
		{
		  return Check_Left_Wheel;
		}
		#ifdef BLDC_INSTALL
		if(Is_BLDC_Fail())
		{
			BLDC_Broken++;
			if(BLDC_Broken>100)
			{
				BLDC_Broken=0;
				BLDC_OFF;
				Set_BLDC_TPWM(30);
				delay(100);
		    Set_Vac_Speed();
				BLDC_ON;
			}
			Clear_BLDC_Fail();
		}
		#endif
		/*----------------------------------------------------------Check_Main_Brush-----------------*/	 
    if(Is_MainBrush_Stall())return Check_Main_Brush;
    /*----------------------------------------------------------Check_Main Brush_Current-----------------*/
	}
	return 0;
}

/*turn check cliff*/
volatile uint8_t g_turn_check_cliff=0;
void Set_Turn_Check_Cliff(uint8_t data)
{
	g_turn_check_cliff = data;
}

uint8_t Get_Turn_Check_Cliff(void)
{
	return g_turn_check_cliff;
}



uint8_t Self_Check(uint8_t Check_Code)
{
	Left_Wheel_Slow=0;
	Right_Wheel_Slow=0;
	
	USPRINTF("%s %d:Check_code: %d\n", __FUNCTION__, __LINE__, Check_Code);

	if(Check_Code!=Check_Right_Wheel && Check_Code!=Check_Left_Wheel)
	{
		Move_Back();	
		if(Get_Random_Factor()%2)
		{
			Turn_Left(Turn_Speed,1200+4*Get_Random_Factor());
		}
		else
		{
			Turn_Right(Turn_Speed,1200+4*Get_Random_Factor());
		}	
	}

	if(Check_Code==Check_Main_Brush)
	{
		Set_MainBrush_PWM(0);
	}
  
	Set_Dir_Forward();
	Set_Wheel_Speed(0,0);
	WHEEL_DISABLE();
	if(Check_Code==Check_Vacuum)
	{
		Turn_BLDC_Off();		
	}
	
	delay(10000);

	if(Get_Error_Code())
	{
		return 1;
	}
	Set_Turn_Check_Cliff(1);
	/*-----------------------------------------------------------Self Check Right Wheel-------------------*/
	if(Check_Code==Check_Right_Wheel)
	{	
		USPRINTF("right wheel:0x%x\n",Check_Code);
		if(SelfCheck_RightWheel())
		{
			Set_Turn_Check_Cliff(0);
			return 1;
		}
	}
	/*-----------------------------------------------------------Self Check Left Wheel-------------------*/
	else if(Check_Code==Check_Left_Wheel)
	{
		USPRINTF("left wheel:0x%x\n",Check_Code);
	  if(SelfCheck_LeftWheel())
		{
			Set_Turn_Check_Cliff(0);
			return 1;
		}
	}
	/*-----------------------------------------------------------Self Check Main Brush-------------------*/
	else if(Check_Code==Check_Main_Brush)
	{
		USPRINTF("main brush:0x%x\n",Check_Code);
    if(SelfCheck_MainBrush())
		{
			Set_Turn_Check_Cliff(0);
			return 1;
		}
	}
	/*-----------------------------------------------------------Self Check Vacuum-------------------*/
  else if(Check_Code==Check_Vacuum)
  {
		USPRINTF("vacuum:0x%x\n",Check_Code);
		if(SelfCheck_Vacuum())
		{
			Set_Turn_Check_Cliff(0);
			return 1;
		}
  }
	/*---------------------------Reset Wheel Slow Counter-------------------*/
	Left_Wheel_Slow=0;   
	Right_Wheel_Slow=0;
	Set_Turn_Check_Cliff(0);
	return 0;
}

/*--------------------------------------------------------- Set Work Motor set--------------------------*/
void Work_Motor_Configure(void)
{
	Reset_Short_Circuit_Counter();
	Set_MainBrush_PWM(50);
  Set_SideBrush_PWM(60,60);
	Set_Right_Brush(ENABLE);
	BLDC_OFF;
	Set_Vac_Speed();
}


/*----------------------------------------------------------Set side Brush ----------------*/
void Set_SideBrush_PWM(uint16_t L,uint16_t R)
{ 
	L=L;
	Right_Brush_PWM=R;
}

void Set_Right_Brush(uint8_t R)
{
	if(R==ENABLE)
	{
		TIMER_CHCTL2(TIMER7) |= TIM_CCER_CC2E;
	}
	else
	{
		TIMER_CHCTL2(TIMER7) &=~ TIM_CCER_CC2E;
	}
}
uint8_t Is_RightBrush_Enable(void)
{
	if(TIMER_CHCTL2(TIMER7) & TIM_CCER_CC2E)return 1;
	return 0;
}

/*----------------------------------------------------------Set main Brush ----------------*/
void Set_MainBrush_PWM(uint8_t Power)
{
  Main_Brush_PWM=Power;
}
void Set_Main_Brush(uint8_t M)
{
	if(M==ENABLE)
	{
		TIMER_CHCTL2(TIMER3) |= TIM_CCER_CC3E;
	}
	else
	{
		TIMER_CHCTL2(TIMER3) &=~ TIM_CCER_CC3E;
	}
}
uint8_t Is_MainBrush_Enable(void)
{
	if(TIMER_CHCTL2(TIMER3) & TIM_CCER_CC3E)return 1;
	return 0;
}

/*Random Factor*/
volatile uint8_t g_random_counter=0;
uint8_t Get_Random_Factor(void)
{
	uint8_t temp=0;	
  if(g_random_counter>100)g_random_counter=100;
  temp = g_random_counter;
	g_random_counter = 0;
	return temp;
}

/* ----------------------------- Check_Battery_and Vacuum adjust---------------------*/
uint8_t Check_Bat_SetMotors(uint32_t Vacuum_Voltage,uint32_t Side_Brush,uint32_t Main_Brush)
{
  static uint8_t Low_Acc=0;

	uint32_t Temp_Rightbrush_Power=0;
	uint32_t Temp_Vacuum_Power=0;
  uint32_t Temp_Main=0;

	static uint8_t RightBrush_Stall_Conter=0;
	
	g_battery_voltage =	GetBatteryVoltage();

	if(g_battery_voltage <Low_Battery_Limit)
  {
    Low_Acc++;
    if(Low_Acc>50)return 1;
		return 0;
 	}
  else
	{
    Low_Acc=0;
		Temp_Vacuum_Power = (uint8_t)(Vacuum_Voltage/g_battery_voltage);
		Temp_Rightbrush_Power = (uint8_t)(Side_Brush/g_battery_voltage);
    Temp_Main = (uint8_t)(Main_Brush/g_battery_voltage);
		 
		if(Temp_Vacuum_Power>100)Temp_Vacuum_Power=100;

    if(Temp_Main>100)Temp_Main=100;

		if(Get_RightBrush_Stall())
		{
			Set_Right_Brush(ENABLE);
			Temp_Rightbrush_Power=100;
			RightBrush_Stall_Conter++;			
			if(RightBrush_Stall_Conter>200)
			{
				RightBrush_Stall_Conter=0;
				Set_RightBrush_Stall(0);
			}
		}
		else
		{
			RightBrush_Stall_Conter=0;
		}
		Set_SideBrush_PWM(0,Temp_Rightbrush_Power);

    Set_MainBrush_PWM(Temp_Main);
		
		adjust_Short_Limit(g_battery_voltage);
		
		return 0;
  }
}

/* ----------------------------- Check if low Battery ---------------------*/
uint8_t Check_Battery(void)
{
  if(GetBatteryVoltage()<Low_Battery_Limit)return 1;
  return 0;
}
 
/* ------------------------------GetBatteryVoltage ---------------------*/
uint16_t GetBatteryVoltage(void)
{
  uint32_t temp=0;
  uint8_t count=0;
  for(count=0;count<10;count++)
  {
    temp+=g_adc_value.Battery_Voltage;
    delay(10);
  }
	temp/=10;
	
	temp = (temp*ReferenceVoltage*1000)/4096/180;
	
  return (uint16_t)temp;
}


/*----------------------------------------------Bumper jam---------------------------------*/
uint8_t Is_Bumper_Jamed(void)
{
	#ifdef BUMPER_JAMED
  if(Get_Bumper_Status())
	{
		if(Get_Bumper_Status())
		{
			Move_Back();			
			if(Get_Bumper_Status())
			{
				Quick_Back(100,200);
				if(Get_Bumper_Status())
				{
					if(Get_Bumper_Status()&LeftBumperTrig)
					{
						Turn_Right(Turn_Speed,2200);
					}
					else if(Get_Bumper_Status()&RightBumperTrig)
					{
						Turn_Left(Turn_Speed,2200);
					}
					if(Get_Bumper_Status())
					{
						if(Is_Bumper_Fail())
						{
							Mode_SetMode(MODE_USERINTERFACE);
							Set_Error_Code(Error_Code_Bumper);
							return 1;
						}
					}
				}
			}		
		}
	}
	#endif
  return 0;
}

void Reset_Short_Circuit_Counter(void)
{
	RightSideBrush_Short_Circuit_Counter=0;
	MainBrush_Short_Circuit_Counter=0;
	Vacuum_Short_Circuit_Counter=0;
}

void Check_Short_Circuit(void)
{
	if(g_adc_value.Right_Brush_Current>RightBrush_Short_Base)
	{
		RightSideBrush_Short_Circuit_Counter++;
		if(RightSideBrush_Short_Circuit_Counter>10)
		{
			Set_Right_Brush(DISABLE);
			Set_Error_Code(Error_Code_SideBrush);
		}
	}
	else
	{
		RightSideBrush_Short_Circuit_Counter=0;
	}
	
	if(g_adc_value.Main_Brush_Current>MainBrush_Short_Base)
	{
		MainBrush_Short_Circuit_Counter++;
		if(MainBrush_Short_Circuit_Counter>5)
		{
			Set_Main_Brush(DISABLE);
			Set_Error_Code(Error_Code_MainBrush);
		}
	}
	else
	{
		MainBrush_Short_Circuit_Counter=0;
	}
	
	if(g_adc_value.Vacuum_Current>Vacuum_Short_Base)
	{
		Vacuum_Short_Circuit_Counter++;
		if(Vacuum_Short_Circuit_Counter>5)
		{
			BLDC_OFF;
			Set_Error_Code(Error_Code_Fan_H);
		}
	}
	else
	{
		Vacuum_Short_Circuit_Counter=0;
	}
}





