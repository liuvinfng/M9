 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Random Path Cleaning Function
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/

#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Spot.h"
#include "Rcon.h"
#include "Home.h"
#include "VirtualWall.h"

/* --------------------------------------------------Random Runnincg mode----------------------*/
void Spot_Mode(void)
{
	uint8_t Motor_OC_Counter=0;
  uint16_t Radius=0;
	uint8_t Move_Style=1;
	uint8_t Spot_Flag=0;
	uint8_t OBS_Counter=0;
	uint8_t Stuck=0;
	uint16_t Counter_Watcher = 0;


	Display_Clean_Status(Display_Spot); 

  Move_Style	= Spiral_Right_Out;

	Enable_PPower();
	Reset_Touch();


	Set_SideBrush_PWM(60,60);
  Set_MainBrush_PWM(90);
  Set_Vacuum_PWM(90);
	#ifdef BLDC_INSTALL
	BLDC_OFF;
	delay(100);
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
	Set_BLDC_TPWM(60);
	#endif
  delay(1000);
 	Set_Dir_Right();
	Set_Wheel_Speed(RUN_SPEED_10,RUN_SPEED_3);
	Reset_Wheel_Step();
	Counter_Watcher=0;
  Reset_Rcon_Remote();
	Motor_OC_Counter=0;
	while(Get_LeftWheel_Step()<6900)
	{
	  delay(4);
		Counter_Watcher++;
		if(Counter_Watcher>50000)break;
	  Set_Dir_Right();
		Set_Wheel_Speed(RUN_SPEED_10,RUN_SPEED_4);

		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		if(Check_Motor_Current())
		{
			Motor_OC_Counter++;
			if(Motor_OC_Counter>100)
			{
				Motor_OC_Counter=0;
				Set_Clean_Mode(Clean_Mode_Userinterface);
			  return;
			}
		}
		else
		{
			Motor_OC_Counter=0;
		}
		
    if(Get_Rcon_Remote())
    {
#if 0
  		if(Remote_Key(Remote_Wallfollow))
  		{
  		  Set_MoveWithRemote();
  		  Set_Clean_Mode(Clean_Mode_WallFollow);
  			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  			return;
  		}
#endif
			if(Remote_Key(Remote_Home))
  		{
  		  Set_MoveWithRemote();
  		  Set_Clean_Mode(Clean_Mode_GoHome);
  			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				SetHomeRemote();
  			return;
  		}
      Reset_Rcon_Remote();
    }
		if(Get_OBS_Status()||Get_Cliff_Trig())
		{
		  Move_Back();
      Stop_Brifly();
			Turn_Left(Turn_Speed,2500);
			Move_Style	= Spiral_Left_Out;
		  break;
		}
	}
	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
	Set_Wheel_Step(0,0);
	Reset_Wall_Step();
	Set_MainBrush_PWM(90);
	Set_SideBrush_PWM(60,60);
  Set_Vacuum_PWM(90);
  Display_Content(LED_Spot,100,100,0,7);
	Motor_OC_Counter=0;
  while(1)
	{
		
		/*------------------------------------------------------Check Battery-----------------------*/
		if(Check_Bat_SetMotors(135000,80000,100000))//Low Battery Event
    {
      Display_Battery_Status(Display_Low);//display low
      delay(3000);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
 		}
    Set_MainBrush_PWM(80);
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
	  if(Check_Motor_Current())
		{
			Motor_OC_Counter++;
			if(Motor_OC_Counter>100)
			{
				Motor_OC_Counter=0;
				Set_Clean_Mode(Clean_Mode_Userinterface);
			  return;
			}
		}
		else
		{
			Motor_OC_Counter=0;
		}
    if(Get_Rcon_Remote())
    {
#if 0
  		if(Remote_Key(Remote_Wallfollow))
  		{
  		  Set_MoveWithRemote();
  		  Set_Clean_Mode(Clean_Mode_WallFollow);
  			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  			return;
  		}
#endif
			if(Remote_Key(Remote_Home))
  		{
  		  Set_MoveWithRemote();
  		  Set_Clean_Mode(Clean_Mode_GoHome);
  			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				SetHomeRemote();
  			return;
  		}
      Reset_Rcon_Remote();
    }
		/*------------------------------------------------------Runing Path-----------------------*/
    Set_Dir_Forward();
	  switch(Move_Style)
		{
			case Spiral_Right_Out:	if(Get_LeftWheel_Step()>(Radius*3))
													    {
													      Reset_LeftWheel_Step();
																if(Radius<100)
																{
																  Radius+=1;
																}
																else
																{
													        Radius+=3;
																}
																if(Radius>250)
																{
																  Move_Style=Spiral_Right_In;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Stop_Brifly();
															  Turn_Left(Turn_Speed,2500);
																Move_Style = Spiral_Left_Out;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
							                Set_LeftWheel_Speed(40);
						                  Set_RightWheel_Speed((40*Radius)/(Radius+230));
										
							                break;
			case Spiral_Right_In :	if(Get_LeftWheel_Step()>(Radius*3))
													    {
													      Reset_LeftWheel_Step();
																if(Radius<3)
																{
																  Spot_Flag=1;
																}
																if(Radius<100)
																{
																  Radius-=1;
																}
																else
																{
													        Radius-=3;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Stop_Brifly();
															  Turn_Left(Turn_Speed,2500);
																Move_Style = Spiral_Left_In;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
							                Set_LeftWheel_Speed(40);
					                    Set_RightWheel_Speed((40*Radius)/(Radius+230));
											
							                break;
			case Spiral_Left_Out :	if(Get_RightWheel_Step()>(Radius*3))
													    {
													      Reset_RightWheel_Step();
													      if(Radius<100)
																{
																  Radius+=1;
																}
																else
																{
													        Radius+=3;
																}
																if(Radius>250)
																{
																  Move_Style=Spiral_Left_In;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Stop_Brifly();
															  Turn_Right(Turn_Speed,2000);
																Move_Style = Spiral_Right_Out;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
							                Set_RightWheel_Speed(40);
					                    Set_LeftWheel_Speed((40*Radius)/(Radius+230));
										
														  break;
			case Spiral_Left_In  : if(Get_RightWheel_Step()>(Radius*2))
													    {
													      Reset_RightWheel_Step();
																if(Radius<3)
																{
																  Spot_Flag=1;
																}
													      if(Radius<100)
																{
																  Radius-=1;
																}
																else
																{
													        Radius-=3;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Stop_Brifly();
															  Turn_Right(Turn_Speed,2000);
																Move_Style = Spiral_Right_In;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
							                Set_RightWheel_Speed(40);
					                    Set_LeftWheel_Speed((40*Radius)/(Radius+230));
												
							                break;
			default:break;
		  
		}
		if((OBS_Counter>15)||(Stuck>3))
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(Spot_Flag)
		{
		  Spot_Flag=0;
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
	}
}
/*----------------------------------------------------------------Random Dirt Event---------------------------------*/
uint8_t Random_Dirt_Event(void)
{
  uint16_t Radius=0;
	uint8_t Move_Style=1;
	uint8_t Spot_Flag=0;
	uint8_t OBS_Counter=0;
	uint8_t Stuck=0;
	uint8_t Flash_Counter=0;
	uint16_t Watch_Counter=0;
	uint8_t Flash_Flag=0;
  uint8_t Motor_OC_Counter=0;

	Display_Clean_Status(Display_Spot); 

  Move_Style	= First_Round;

	Enable_PPower();
	Reset_Touch();
	
	Check_Bat_SetMotors(135000,100000,100000);
	#ifdef BLDC_INSTALL
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
	Set_BLDC_TPWM(60);
	#endif
	Move_Forward(0,0);
	Set_Wheel_Step(0,0);
	Reset_Wall_Step();
	delay(100);
 	Display_Content(0,100,100,0,7);
	delay(100);
  Display_Content(LED_Spot,100,100,0,7);
	delay(100);
	Display_Content(0,100,100,0,7);
	delay(100);
	Display_Content(LED_Spot,100,100,0,7);

  Reset_Rcon_Remote();
	
	Motor_OC_Counter=0;
  while(1)
	{
	  delay(10);
		Flash_Counter++;
		if(Flash_Counter>20)
		{
		  Watch_Counter++;
			if(Watch_Counter>1000)
			{
			  Set_Error_Code(Error_Code_Encoder);
			  Set_Touch();
				return 1;
			}
		  Flash_Counter=0;
		  Flash_Flag=1-Flash_Flag;
			if(Flash_Flag)Display_Content(LED_Spot,100,100,0,7);
			else Display_Content(0,100,100,0,7);
		}

		if(Get_Rcon_Remote()!=0)
		{
//		  Main_Brush_PWM=MainBrush_Power;
			Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
			return 0;
		}
		/*------------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL

		if (Is_VirtualWall()) {
			Stop_Brifly();
			return 0;
		}

#endif

    /*
		if(Move_Style !=	First_Round)
		{
				Move_Back();
        Check_Bat_SetMotors(135000,80000,100000);
	      Turn_Left(Turn_Speed,1250);
				Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
				return 0;
		}*/
		/*------------------------------------------------------Check Battery-----------------------*/

		if(Check_Bat_SetMotors(135000,100000,120000))//Low Battery Event
    {
			Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
			return 0;
 		}

		if(Check_Motor_Current())
		{
			Motor_OC_Counter++;
			if(Motor_OC_Counter>50)
			{
				Motor_OC_Counter=0;
//				Main_Brush_PWM=MainBrush_Power;
				Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
				return 0;
			}
		}
		else
		{
			Motor_OC_Counter=0;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
			return 1;
		}
		/*------------------------------------------------------Runing Path-----------------------*/
		
	  switch(Move_Style)
		{
		  case First_Round     :	if(Get_LeftWheel_Step()>6000)
													    {
															  Move_Forward(0,0);
													      Reset_LeftWheel_Step();
																Move_Style=Spiral_Right_Out;
															}
                              Set_Dir_Right();
															Set_Wheel_Speed(RUN_SPEED_10,RUN_SPEED_4);
							                
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Display_Content(LED_Spot,100,100,0,7);
                                Stop_Brifly();
															  Turn_Left(Turn_Speed,2500);
																Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
																Move_Style = Spiral_Left_Out;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}							
															break;
			case Spiral_Right_Out:	if(Get_LeftWheel_Step()>(Radius*3))
													    {
													      Reset_LeftWheel_Step();
																if(Radius<100)
																{
																  Radius+=2;
																}
																else
																{
													        Radius+=6;
																}
																if(Radius>140)
																{
																  Move_Style=Spiral_Right_In;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Display_Content(LED_Spot,100,100,0,7);
                                Stop_Brifly();
															  Turn_Left(Turn_Speed,2500);
																Move_Style = Spiral_Left_Out;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
                              Set_Dir_Forward();
							                Set_LeftWheel_Speed(40);
						                  Set_RightWheel_Speed((40*Radius)/(Radius+230));
							                break;
			case Spiral_Right_In :	if(Get_LeftWheel_Step()>(Radius*3))
													    {
													      Reset_LeftWheel_Step();
																if(Radius<3)
																{
																  Spot_Flag=1;
																}
																if(Radius<100)
																{
																  Radius-=1;
																}
																else
																{
													        Radius-=6;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
  															Display_Content(LED_Spot,100,100,0,7);
                                Stop_Brifly();
															  Turn_Left(Turn_Speed,2500);
																Move_Style = Spiral_Left_In;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
                              Set_Dir_Forward();
							                Set_LeftWheel_Speed(40);
					                    Set_RightWheel_Speed((40*Radius)/(Radius+230));
							                break;
			case Spiral_Left_Out :	if(Get_RightWheel_Step()>(Radius*3))
													    {
													      Reset_RightWheel_Step();
													      if(Radius<100)
																{
																  Radius+=2;
																}
																else
																{
													        Radius+=6;
																}
																if(Radius>140)
																{
																  Move_Style=Spiral_Left_In;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Display_Content(LED_Spot,100,100,0,7);
                                Stop_Brifly();
															  Turn_Right(Turn_Speed,2000);
																Move_Style = Spiral_Right_Out;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
                              Set_Dir_Forward();
							                Set_RightWheel_Speed(40);
					                    Set_LeftWheel_Speed((40*Radius)/(Radius+230));
														  break;
			case Spiral_Left_In  : if(Get_RightWheel_Step()>(Radius*2))
													    {
													      Reset_RightWheel_Step();
																if(Radius<3)
																{
																  Spot_Flag=1;
																}
													      if(Radius<100)
																{
																  Radius-=1;
																}
																else
																{
													        Radius-=6;
																}
															}
															if(Get_Bumper_Status()||Get_Cliff_Trig()||Spot_OBS_Status())
															{
															  if(Get_LeftWall_Step()<3000)Stuck++;
															  if(Get_Bumper_Status())Random_Back();
																else if(Get_Cliff_Trig())Move_Back();
                                Stop_Brifly();
																//Display_Content(LED_Spot,100,100,0,7);
															  Turn_Right(Turn_Speed,2000);
																Move_Style = Spiral_Right_In;
																Reset_Wheel_Step();
																Reset_Wall_Step();
																OBS_Counter++;
															}
                              Set_Dir_Forward();
							                Set_RightWheel_Speed(40);
					                    Set_LeftWheel_Speed((40*Radius)/(Radius+230));
							                break;
			default:break;
		  
		}
		if((OBS_Counter>5)||(Stuck>2))
		{
//		  Main_Brush_PWM=MainBrush_Power;
			Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
			return 0;
		}
		if(Spot_Flag)
		{
		  Spot_Flag=0;
//		  Main_Brush_PWM=MainBrush_Power;
			Move_Forward(RUN_SPEED_12,RUN_SPEED_12);
			return 2;
		}
	}
//	return 2;

}


