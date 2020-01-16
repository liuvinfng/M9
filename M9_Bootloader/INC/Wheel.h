/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   Movement
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __WHEEL_H
#define __WHEEL_H

#include "SysInitialize.h"
#include "config.h"


extern volatile uint16_t Left_Wheel_Speed, Right_Wheel_Speed;
extern volatile uint16_t Left_Wheel_Counter, Right_Wheel_Counter;
extern volatile uint16_t Right_Wheel_Slow, Left_Wheel_Slow;
extern volatile int16_t Temp_Right_Wheel_PWM, Temp_Left_Wheel_PWM;
extern volatile uint32_t Left_Wheel_Step, Right_Wheel_Step;
extern volatile int32_t WallFollowRightWheelStep, WallFollowLeftWheelStep;
extern volatile int16_t WheelCount_Left, WheelCount_Right;



void Speed_Loop(void);
void Reset_WheelSLow(void);
void Set_TempPWM(uint8_t Left,uint8_t Right);
void Reset_TempPWM(void);
void Set_LeftTPWM(uint8_t P);
void Set_RightTPWM(uint8_t P);
uint8_t Get_LeftTPWM(void);
uint8_t Get_RightTPWM(void);
void Set_Dir_Forward(void);
void Set_Dir_Backward(void);
void Set_Dir_Right(void);
void Set_Dir_Left(void);
uint8_t Is_LeftWheel_Reach(int32_t Step);
uint8_t Is_RightWheel_Reach(int32_t Step);

void Set_Wheel_Speed(uint8_t Left,uint8_t Right);
void Set_LeftWheel_Speed(uint8_t Speed);
void Set_RightWheel_Speed(uint8_t Speed);
int8_t Get_LeftWheel_Speed(void);
int8_t Get_RightWheel_Speed(void);
void Reset_Encoder_Error(void);
uint8_t Is_Encoder_Fail(void);
void Set_Wheel_Step(uint32_t Left,uint32_t Right);
void Reset_Wheel_Step(void);
uint32_t Get_LeftWheel_Step(void);
uint32_t Get_RightWheel_Step(void);
void Set_RightWheel_Step(uint32_t Step);
void Set_LeftWheel_Step(uint32_t Step);
void Reset_RightWheel_Step(void);
void Reset_LeftWheel_Step(void);
void Stop_Brifly(void);
void Wheel_Stop(void);
void WHEEL_DISABLE(void);
void LW_DIR_FORWARD(void);
void LW_DIR_BACKWARD(void);
void RW_DIR_FORWARD(void);
void RW_DIR_BACKWARD(void);
void Move_Forward(uint8_t Left_Speed,uint8_t Right_Speed);


#endif /* __WHEEL_H */






