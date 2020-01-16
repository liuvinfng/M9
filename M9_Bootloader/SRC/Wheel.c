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
#include "Wheel.h"


uint8_t R_F=0, L_F=0;
volatile uint8_t Signal_Counter_Flag=0;
volatile uint16_t Left_Wheel_Speed=0, Right_Wheel_Speed=0;
volatile uint16_t Left_Wheel_Counter=0, Right_Wheel_Counter=0;
volatile uint16_t Right_Wheel_Slow=0, Left_Wheel_Slow=0;
volatile int16_t Temp_Right_Wheel_PWM=0, Temp_Left_Wheel_PWM=0;
volatile uint32_t Left_Wheel_Step=0, Right_Wheel_Step=0;
volatile int32_t WallFollowRightWheelStep=0, WallFollowLeftWheelStep=0;
volatile int16_t WheelCount_Left=0, WheelCount_Right=0;

void Speed_Loop(void)
{
	static volatile int32_t Right_Proportion=0,Left_Proportion=0;	
	 /*-------------------------------------------------------------------------Left Wheel Speed Adjust --------------------*/
	if(Left_Wheel_Speed>20)
	{
	 if(Left_Wheel_Counter<20)
	 {
		 if(Left_Wheel_Counter<(Left_Wheel_Speed*7/10))Left_Wheel_Slow++;
		 else Left_Wheel_Slow=0;
		 if(Left_Wheel_Slow>200)Left_Wheel_Slow=200;
	 }
	 else
	 {
		 Left_Wheel_Slow=0;
	 }
	}
	else
	{
	 Left_Wheel_Slow=0;
	}
	if(Right_Wheel_Speed>20)
	{
	 if(Right_Wheel_Counter<20)
	 {
		 if(Right_Wheel_Counter<(Right_Wheel_Speed*7/10))Right_Wheel_Slow++;
		 else Right_Wheel_Slow=0;
		 if(Right_Wheel_Slow>200)Right_Wheel_Slow=200;
	 }
	 else
	 {
		 Right_Wheel_Slow=0;
	 }
	}
	else
	{
	 Right_Wheel_Slow=0;
	}


	if(Left_Wheel_Speed>1)
	{
	if(Left_Wheel_Counter>70)Left_Wheel_Counter=70;	
	Left_Proportion= Left_Wheel_Speed - Left_Wheel_Counter;
	if(Left_Proportion>10)Left_Proportion=10;
	 if(Left_Proportion<-10)Left_Proportion=-10;
	Temp_Left_Wheel_PWM += Left_Proportion;

	if(Temp_Left_Wheel_PWM<0)Temp_Left_Wheel_PWM=0;
	else if(Temp_Left_Wheel_PWM>99)Temp_Left_Wheel_PWM=100;
	}
	else
	{
	Temp_Left_Wheel_PWM=0;
	}
	
	/*-------------------------------------------------------------------------Right Wheel Speed Adjust --------------------*/
	if(Right_Wheel_Speed>1)
	{
	if(Right_Wheel_Counter>70)Right_Wheel_Counter=70;	
	Right_Proportion =  Right_Wheel_Speed -Right_Wheel_Counter	;
	if(Right_Proportion>10)Right_Proportion=10;
	if(Right_Proportion<-10)Right_Proportion=-10;
	Temp_Right_Wheel_PWM += Right_Proportion;

	if(Temp_Right_Wheel_PWM<0)Temp_Right_Wheel_PWM=0;
	else if(Temp_Right_Wheel_PWM>99)Temp_Right_Wheel_PWM=100;
	}
	else
	{
	Temp_Right_Wheel_PWM=0;
	}
	Left_Wheel_Counter=0;
	Right_Wheel_Counter=0;
	if(L_F==1)
	{
	TIMER_CH0CV(TIMER0) = 0;
	TIMER_CH1CV(TIMER0) = (uint16_t)Temp_Left_Wheel_PWM;
	}
	else if(L_F==2)
	{
	TIMER_CH1CV(TIMER0) = 0;
	TIMER_CH0CV(TIMER0) = (uint16_t)Temp_Left_Wheel_PWM;
	}
	else
	{
	TIMER_CH0CV(TIMER0) = 0;
	TIMER_CH1CV(TIMER0) = 0;
	}

	if(R_F==1)
	{
	TIMER_CH2CV(TIMER0) = 0;
	TIMER_CH3CV(TIMER0) = (uint16_t)Temp_Right_Wheel_PWM;
	}
	else if(R_F==2)
	{
	TIMER_CH3CV(TIMER0) = 0;
	TIMER_CH2CV(TIMER0) = (uint16_t)Temp_Right_Wheel_PWM;
	}	
	else
	{
	TIMER_CH2CV(TIMER0) = 0;
	TIMER_CH3CV(TIMER0) = 0;
	}
}

void Reset_WheelSLow(void)
{
	Right_Wheel_Slow=0;
	Left_Wheel_Slow=0;
}

/*------------------------------Temp PWM------------------------------------*/
void Set_LeftTPWM(uint8_t P)
{
	Temp_Left_Wheel_PWM = P;
}
uint8_t Get_LeftTPWM(void)
{
	return Temp_Left_Wheel_PWM;
}
void Set_RightTPWM(uint8_t P)
{
	Temp_Right_Wheel_PWM = P;
}
uint8_t Get_RightTPWM(void)
{
	return Temp_Right_Wheel_PWM;
}
void Set_TempPWM(uint8_t Left,uint8_t Right)
{
  Temp_Right_Wheel_PWM = Right;
  Temp_Left_Wheel_PWM = Left;
}
void Reset_TempPWM(void)
{
  Temp_Right_Wheel_PWM = 0;
  Temp_Left_Wheel_PWM = 0;
}

/*------------------------------Wheel Direction------------------------------------*/
void Set_Dir_Forward(void)
{
	RW_DIR_FORWARD();
	LW_DIR_FORWARD();
}
void Set_Dir_Backward(void)
{
	RW_DIR_BACKWARD();
	LW_DIR_BACKWARD();
}
void Set_Dir_Right(void)
{
	RW_DIR_BACKWARD();
	LW_DIR_FORWARD();
}
void Set_Dir_Left(void)
{
	RW_DIR_FORWARD();
	LW_DIR_BACKWARD();
}

/*------------------------------Move Distance ----------------------*/
uint8_t Is_LeftWheel_Reach(int32_t Step)
{
	if (Left_Wheel_Step > (uint32_t) Step) {
		return 1;
	}
	return 0;
}
uint8_t Is_RightWheel_Reach(int32_t Step)
{
	if (Right_Wheel_Step > (uint32_t) Step) {
		return 1;
	}
	return 0;
}

/*--------------------------------Wheel Speed -------------------------*/
int8_t Get_LeftWheel_Speed(void)
{
  return Left_Wheel_Speed;
}
void Set_LeftWheel_Speed(uint8_t Speed)
{
  if(Speed>100)Speed=100;
	Left_Wheel_Speed = Speed;
}
int8_t Get_RightWheel_Speed(void)
{
  return Right_Wheel_Speed; 
}
void Set_RightWheel_Speed(uint8_t Speed)
{
  if(Speed>100)Speed=100;
	Right_Wheel_Speed = Speed;
}
void Set_Wheel_Speed(uint8_t Left,uint8_t Right)
{
	Set_LeftWheel_Speed(Left);
	Set_RightWheel_Speed(Right);
}

/*---------------------------------Encoder Signal Watcher-------------------------------*/
void Reset_Encoder_Error(void)
{
  Signal_Counter_Flag=0;
}
uint8_t Is_Encoder_Fail(void)
{
  Signal_Counter_Flag++;
  if(Signal_Counter_Flag>3)return 1;
	else return 0;
}

/*------------------------------------Wheel Steps --------------------------------------*/
void Set_Wheel_Step(uint32_t Left,uint32_t Right)
{
  Left_Wheel_Step = Left,
	Right_Wheel_Step = Right;
}
void Reset_Wheel_Step(void)
{
  Left_Wheel_Step=0;
	Right_Wheel_Step=0;
}

void Set_RightWheel_Step(uint32_t Step)
{
  Right_Wheel_Step = Step;
}
void Reset_RightWheel_Step(void)
{
  Right_Wheel_Step = 0;
}
uint32_t Get_RightWheel_Step(void)
{
  return Right_Wheel_Step;
}

void Set_LeftWheel_Step(uint32_t Step)
{
  Left_Wheel_Step = Step;
}
void Reset_LeftWheel_Step(void)
{
  Left_Wheel_Step = 0;
}
uint32_t Get_LeftWheel_Step(void)
{
  return Left_Wheel_Step;
}

void Stop_Brifly(void)
{
	Wheel_Stop();	
}
void Wheel_Stop(void)
{
	Set_Wheel_Speed(0,0);
  Reset_TempPWM();
}

void WHEEL_DISABLE(void)
{
	L_F=0;R_F=0;
	TIMER_CH0CV(TIMER0) = 0;
	TIMER_CH1CV(TIMER0) = 0;
	TIMER_CH2CV(TIMER0) = 0;
	TIMER_CH3CV(TIMER0) = 0;
	TIMER_CHCTL2(TIMER0) &=~(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E);
}

void LW_DIR_FORWARD(void)
{
	L_F=1;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC1E|TIM_CCER_CC2E);
}

void LW_DIR_BACKWARD(void)
{
	L_F=2;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC3E|TIM_CCER_CC4E);
}

void RW_DIR_FORWARD(void)
{
	R_F=1;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC3E|TIM_CCER_CC4E);
}

void RW_DIR_BACKWARD(void)
{
	R_F=2;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC1E|TIM_CCER_CC2E);
}

/*--------- Move Forward -----------------*/
void Move_Forward(uint8_t Left_Speed,uint8_t Right_Speed)
{
  Set_Dir_Forward();
	Set_Wheel_Speed(Left_Speed,Right_Speed);
}




