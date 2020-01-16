/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    18-May-2016
  * @brief   WIFI EMW 3081 
  ******************************************************************************

  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "USART.h"
#include "SysInitialize.h"
#include "Rcon.h"
#include "Movement.h"
#include "TouchPad.h"
#include "WIFI_EMW_3081.h"
#include "Speaker.h"
#include "Display.h"

#ifdef WIFI_EMW3081

Struct_App_Data App_Command;
uint8_t WIFI_Received_Flag=0;
char WIFI_Status[16]={0};
volatile uint8_t EM3081_Status=0;

volatile uint8_t WIFI_RxBuffer[22]={0};
volatile uint8_t WIFI_RxIndex = 0;
volatile uint8_t WIFI_MsgStart = 0;
volatile uint8_t F_WIFI = 0;
volatile uint8_t WIFI_X1=0;
volatile uint8_t WIFI_X2=0;
volatile uint8_t WIFI_MAC[18]={0};


volatile uint8_t WIFI_Turn_Mode=0;
volatile uint8_t Display_WIFI_LED_Status=0;

uint8_t Get_WIFI_Turn_Mode(void)
{
	return WIFI_Turn_Mode;
}

void Set_WIFI_Turn_Mode(uint8_t code)
{
	WIFI_Turn_Mode = code;
}

void Receive_WIFI_EMW3081(char data)
{
	static uint8_t Data_Buffer[5]={0};
	Data_Buffer[0]=Data_Buffer[1];
	Data_Buffer[1]=Data_Buffer[2];
	Data_Buffer[2]=Data_Buffer[3];
	Data_Buffer[3]=Data_Buffer[4];
	Data_Buffer[4]=data;
	if((Data_Buffer[0]==0xa5)&&(Data_Buffer[4]==0xfa))//get command
	{
		if(Data_Buffer[3]==(Data_Buffer[1] + Data_Buffer[2]))
		{
			App_Command.APP_DATA = Data_Buffer[2];
			App_Command.APP_TRD = Data_Buffer[1];
			WIFI_Received_Flag=1;	WIFI_MsgStart = 0;
			WIFI_RxIndex = 0;		
		}
	}
	else if((Data_Buffer[2]==0xa5)&&(Data_Buffer[4]==0xfa))
	{
		Set_EM3081_Status(Data_Buffer[3]);WIFI_MsgStart = 0;
			WIFI_RxIndex = 0;
		//EM3081_Status=;
	}
	
	if(((uint8_t)data==0xa5)&&(WIFI_MsgStart==0))
	{
		WIFI_MsgStart = 1;
		WIFI_RxIndex = 0;
	}
	else if(WIFI_MsgStart==1)
	{
		WIFI_RxBuffer[WIFI_RxIndex] = data;
		WIFI_RxIndex++;
		if(WIFI_RxIndex==22)
		{
			if(WIFI_RxBuffer[WIFI_RxIndex-1]==0xfa)
			{
				for(WIFI_RxIndex=3;WIFI_RxIndex<21;WIFI_RxIndex++)
				{
					WIFI_MAC[WIFI_RxIndex-3]=WIFI_RxBuffer[WIFI_RxIndex];
				}
				WIFI_X1 = WIFI_RxBuffer[0];				
				WIFI_X2 = WIFI_RxBuffer[2];
				F_WIFI = 1;
			}
			WIFI_MsgStart = 0;
			WIFI_RxIndex = 0;
		}
	}
}

uint8_t Get_EM3081_Status(void)
{
	return EM3081_Status;
}
void Clear_EM3081_Status(void)
{
	EM3081_Status=0;
}
void Set_EM3081_Status(uint8_t S)
{
	EM3081_Status = S;
//	USART_Print("\n\rDisplay_WIFI_Status",Dev_USART3);
//	USART3_DMA_Numbers(EM3081_Status);
}

void Check_APP_Command(void)
{
	if(WIFI_Received_Flag)
	{
		WIFI_Received_Flag=0;
		switch(App_Command.APP_TRD)
		{
			case App_WorkMode:                  
				   switch(App_Command.APP_DATA)
					 {
						 case App_WorkMode_Pause: 
						 if(Get_WIFI_WorkMode()!=App_WorkMode_Pause)
						 {
							 Set_WIFI_Turn_Mode(1);
						 }
						 WIFI_Set_WorkMode(App_WorkMode_Pause);
						 WIFI_Set_StopCleaning(1);
						 USART3_Print("\nApp_WorkMode_Pause WIFI_Remote_Pause");
						 Set_Rcon_Remote(WIFI_Remote_Pause);
						 break;
						 case App_WorkMode_Spot: 
						 if(Get_WIFI_WorkMode()!=App_WorkMode_Spot)
						 {
							 Set_WIFI_Turn_Mode(1);
						 }
						 WIFI_Set_WorkMode(App_WorkMode_Spot);
						 WIFI_Set_StopCleaning(0);
						 USART3_Print("\nApp_WorkMode_Spot Remote_Spot");
						 Set_Rcon_Remote(Remote_Spot);						 
						 break;
						 case App_WorkMode_Auto:	
						 if(Get_WIFI_WorkMode()!=App_WorkMode_Auto)
						 {
							 Set_WIFI_Turn_Mode(1);
						 }
						 WIFI_Set_WorkMode(App_WorkMode_Auto);
						 WIFI_Set_StopCleaning(0);
						 USART3_Print("\nApp_WorkMode_Auto WIFI_Remote_Auto");
						 Set_Rcon_Remote(WIFI_Remote_Auto);						 
						 break;
						 case App_WorkMode_Home:  
						 if(Get_WIFI_WorkMode()!=App_WorkMode_Home)
						 {
							 Set_WIFI_Turn_Mode(1);
						 }
						 WIFI_Set_WorkMode(App_WorkMode_Home);
						 WIFI_Set_StopCleaning(0);
						 USART3_Print("\nApp_WorkMode_Home Remote_Home");
						 Set_Rcon_Remote(Remote_Home);
						 break;
						 case App_WorkMode_Wallfollow:
						 if(Get_WIFI_WorkMode()!=App_WorkMode_Wallfollow)
						 {
							 Set_WIFI_Turn_Mode(1);
						 }
						 WIFI_Set_WorkMode(App_WorkMode_Wallfollow);
						 WIFI_Set_StopCleaning(0);
						 USART3_Print("\nApp_WorkMode_Wallfollow WIFI_Remote_Wallfollow");
						 Set_Rcon_Remote(WIFI_Remote_Wallfollow);				
						 break;
						 case App_WorkMode_Navigation:
						 if(Get_WIFI_WorkMode()!=App_WorkMode_Navigation)
						 {
							 Set_WIFI_Turn_Mode(1);
						 }
						 WIFI_Set_WorkMode(App_WorkMode_Navigation);
						 WIFI_Set_StopCleaning(0);
						 USART3_Print("\nApp_WorkMode_SingleRoom WIFI_Remote_SingleRoom");
						 Set_Rcon_Remote(WIFI_Remote_SingleRoom);						 
						 break;//Set_Rcon_Remote(Remote_Spot);
						 default:break;
					 }
					 break;
			case App_OnOff_Direction_Forward: if((App_Command.APP_DATA)&&(Get_WIFI_WorkMode()==App_WorkMode_Pause)){
																					//Set_WIFI_Turn_Mode(1);
				                                  Set_Rcon_Remote(WIFI_Remote_Forward);
																					WIFI_Set_StopCleaning(0);
																					WIFI_Set_Forward(1);
																					WIFI_Set_Backward(0);
																					WIFI_Set_Left(0);
																					WIFI_Set_Right(0);
				                                  USART3_Print("\nApp_OnOff_Direction_Forward On");
			                                  } 
                                        else {
																					if(Get_WIFI_WorkMode()==App_WorkMode_Pause)
																					{
																						Set_Rcon_Remote(WIFI_Remote_DirPause);//
																						WIFI_Set_StopCleaning(1);
																						WIFI_Set_Forward(0);
																						
																						USART3_Print("\nApp_OnOff_Direction_Forward Off");
																					}	
																					else
																					{
																						Set_WIFI_Turn_Mode(1);
																						Set_Rcon_Remote(WIFI_Remote_Forward);
																						WIFI_Set_StopCleaning(0);
																						WIFI_Set_Forward(1);
																						WIFI_Set_Backward(0);
																						WIFI_Set_Left(0);
																						WIFI_Set_Right(0);WIFI_Set_WorkMode(App_WorkMode_Pause);
																						USART3_Print("\nApp_OnOff_Direction_Forward On");
																					}
//																					Set_Rcon_Remote(WIFI_Remote_Pause);
																					
																				}
			                                  break;
			case App_OnOff_Direction_Backward:if((App_Command.APP_DATA)&&(Get_WIFI_WorkMode()==App_WorkMode_Pause)){
				                                 // Set_WIFI_Turn_Mode(1);
																					Set_Rcon_Remote(WIFI_Remote_Backward);
																					WIFI_Set_StopCleaning(0);
																					WIFI_Set_Forward(0);
																					WIFI_Set_Backward(1);
																					WIFI_Set_Left(0);
																					WIFI_Set_Right(0);
				                                  USART3_Print("\nApp_OnOff_Direction_Backward On");
			                                  } 
                                        else {
																					if(Get_WIFI_WorkMode()==App_WorkMode_Pause)
																					{
																						Set_Rcon_Remote(WIFI_Remote_DirPause);//
																						WIFI_Set_StopCleaning(1);
																						WIFI_Set_Backward(0);
	//																					Set_Rcon_Remote(WIFI_Remote_Pause);
																						
																						USART3_Print("\nApp_OnOff_Direction_Backward Off");
																					}
																					else
																					{
																						Set_WIFI_Turn_Mode(1);
																						Set_Rcon_Remote(WIFI_Remote_Backward);
																						WIFI_Set_StopCleaning(0);
																						WIFI_Set_Forward(0);
																						WIFI_Set_Backward(1);
																						WIFI_Set_Left(0);
																						WIFI_Set_Right(0);WIFI_Set_WorkMode(App_WorkMode_Pause);
																						USART3_Print("\nApp_OnOff_Direction_Backward On");
																					}
																				}
			                                  break; 
			case App_OnOff_Direction_Left:    if((App_Command.APP_DATA)&&(Get_WIFI_WorkMode()==App_WorkMode_Pause)){
																					//Set_WIFI_Turn_Mode(1);
				                                  Set_Rcon_Remote(WIFI_Remote_Left);
																					WIFI_Set_StopCleaning(0);
																					WIFI_Set_Forward(0);
																					WIFI_Set_Backward(0);
																					WIFI_Set_Left(1);
																					WIFI_Set_Right(0);
				                                  USART3_Print("\nApp_OnOff_Direction_Left On");
			                                  } 
                                        else {
																					if(Get_WIFI_WorkMode()==App_WorkMode_Pause)
																					{
																						Set_Rcon_Remote(WIFI_Remote_DirPause);//
																						WIFI_Set_StopCleaning(1);
																						WIFI_Set_Left(0);
	//																					Set_Rcon_Remote(WIFI_Remote_Pause);
																						
																						USART3_Print("\nApp_OnOff_Direction_Left Off");
																					}
																					else
																					{
																						Set_WIFI_Turn_Mode(1);
																						 Set_Rcon_Remote(WIFI_Remote_Left);
																						WIFI_Set_StopCleaning(0);
																						WIFI_Set_Forward(0);
																						WIFI_Set_Backward(0);
																						WIFI_Set_Left(1);
																						WIFI_Set_Right(0);WIFI_Set_WorkMode(App_WorkMode_Pause);
																						USART3_Print("\nApp_OnOff_Direction_Left On");
																					}
																				}
			                                  break;
			case App_OnOff_Direction_Right:   if((App_Command.APP_DATA)&&(Get_WIFI_WorkMode()==App_WorkMode_Pause)){
																					//Set_WIFI_Turn_Mode(1);
				                                  Set_Rcon_Remote(WIFI_Remote_Right);
																					WIFI_Set_StopCleaning(0);
																					WIFI_Set_Forward(0);
																					WIFI_Set_Backward(0);
																					WIFI_Set_Left(0);
																					WIFI_Set_Right(1);
				                                  USART3_Print("\nApp_OnOff_Direction_Right On");
			                                  } 
                                        else {
																					if(Get_WIFI_WorkMode()==App_WorkMode_Pause)
																					{
																					Set_Rcon_Remote(WIFI_Remote_DirPause);//
																					WIFI_Set_StopCleaning(1);
																					WIFI_Set_Right(0);
//																					Set_Rcon_Remote(WIFI_Remote_Pause);
																					
																					USART3_Print("\nApp_OnOff_Direction_Right Stop");
																					}
																					else
																					{
																						Set_WIFI_Turn_Mode(1);
																						Set_Rcon_Remote(WIFI_Remote_Right);
																						WIFI_Set_StopCleaning(0);
																						WIFI_Set_Forward(0);
																						WIFI_Set_Backward(0);
																						WIFI_Set_Left(0);
																						WIFI_Set_Right(1);WIFI_Set_WorkMode(App_WorkMode_Pause);
																						USART3_Print("\nApp_OnOff_Direction_Right On");
																					}
																				}
			                                  break; 
			case App_DownAdjust:              if(App_Command.APP_DATA){
				                                  USART3_Print("\nApp_DownAdjust Near");
			                                  } 
                                        else {
																					USART3_Print("\nApp_DownAdjust Far");
																				}
			                                  break;
			case App_SideAdjust:              if(App_Command.APP_DATA){
				                                  USART3_Print("\nApp_SideAdjust Near");
			                                  } 
                                        else {
																					USART3_Print("\nApp_SideAdjust Far");
																				}
			                                  break; 
			case App_CleaningSpeed:           if(App_Command.APP_DATA){
				                                  Set_Rcon_Remote(WIFI_Remote_Max);
																					WIFI_Set_CleaningSpeed(1);
				                                  USART3_Print("\nApp_CleaningSpeed Vacuum Max");
																				}
																				else{
																		
																					Set_Rcon_Remote(WIFI_Remote_Normal);
																					WIFI_Set_CleaningSpeed(0);
																					USART3_Print("\nApp_CleaningSpeed Vacuum Normal");
																				}
			                                  break;
			case App_RoomMode:                USART3_Print("\nApp_RoomMode ");
                                        USART3_DMA_Numbers(App_Command.APP_DATA);
			                                  break;
			case App_Stop_Cleaning:           if(App_Command.APP_DATA){
//																					if(Get_WIFI_WorkMode()!=App_WorkMode_Pause)
//																					 {
//																						 Set_WIFI_Turn_Mode(1);
//																					 }
				                                  Set_Rcon_Remote(WIFI_Remote_Pause);
																					WIFI_Set_StopCleaning(1);
																					WIFI_Set_WorkMode(App_WorkMode_Pause);
				                                  USART3_Print("\nApp_Stop_Cleaning Stop");
			                                  } 
                                        else {
//																					Set_WIFI_Turn_Mode(1);
																					Set_Rcon_Remote(WIFI_Remote_Auto);
																					WIFI_Set_WorkMode(App_WorkMode_Auto);
																					WIFI_Set_StopCleaning(0);
																					USART3_Print("\nApp_Stop_Cleaning Mode Clean");
																				}
			                                  break;
			default:break;
		}
		WIFI_Send_Status();
		
	}
}

void Period_WIFI_Report(uint8_t code)
{
	if(Get_WIFI_Turn_Mode())
	{
		Clear_WIFIReport_Flag();
	}
	if(Is_WIFIReport_Flag())
	{
		Clear_WIFIReport_Flag();
		WIFI_Clear_Status(code);
		WIFI_Set_BatteryCapacity(WIFI_GetBattery());
		WIFI_Send_Status();
	}
}

void Bat_WIFI_Report(void)
{
	if(Get_WIFI_Turn_Mode())
	{
		Clear_WIFIReport_Flag();
	}
	if(Is_WIFIReport_Flag())
	{
		Clear_WIFIReport_Flag();		
		WIFI_Set_BatteryCapacity(WIFI_GetBattery());
		WIFI_Send_Status();
	}
}

void Set_WIFI_Command(Struct_App_Data app)
{
	App_Command=app;
}

Struct_App_Data Get_APP_Command(void)
{
	return App_Command;
}
void Clear_APP_Command(void)
{
	App_Command.APP_DATA=0;
	App_Command.APP_TRD=0;
}

void WIFI_Send_Setting(uint8_t Com)
{
	USART1_Transmit_Byte(0xA5);
	USART1_Transmit_Byte(Com);
	USART1_Transmit_Byte(0xFA);
}
void WIFI_Send_Status(void)
{
	WIFI_Status[14]=WIFI_Status[1]+
	                WIFI_Status[2]+
	                WIFI_Status[3]+
	                WIFI_Status[4]+
	                WIFI_Status[5]+
	                WIFI_Status[6]+
	                WIFI_Status[7]+
	                WIFI_Status[8]+
	                WIFI_Status[9]+
	                WIFI_Status[10]+
	                WIFI_Status[11]+
	                WIFI_Status[12]+
	                WIFI_Status[13];
	WIFI_Status[0]=0xA5;
	WIFI_Status[15]=0xFA;
	USART1_DMA_String(16,&WIFI_Status[0]);
}
uint8_t Get_WIFI_WorkMode(void)
{
	return WIFI_Status[1];
}
void WIFI_Set_WorkMode(char data)
{
	WIFI_Status[1]=data;
}
void WIFI_Set_Forward(char data)
{
	WIFI_Status[2]=data;
}
void WIFI_Set_Backward(char data)
{
	WIFI_Status[3]=data;
}
void WIFI_Set_Left(char data)
{
	WIFI_Status[4]=data;
}
void WIFI_Set_Right(char data)
{
	WIFI_Status[5]=data;
}
void WIFI_Set_DownAdjust(char data)
{
	WIFI_Status[6]=data;
}
void WIFI_Set_SideAdjust(char data)
{
	WIFI_Status[7]=data;
}
void WIFI_Set_CleaningSpeed(char data)
{
	WIFI_Status[8]=data;
}
void WIFI_Set_RoomMode(char data)
{
	WIFI_Status[9]=data;
}
void WIFI_Set_BatteryCapacity(char data)
{
	WIFI_Status[10]=data;
}
void WIFI_Set_ChargerStatus(char data)
{
	WIFI_Status[11]=data;
}
void WIFI_Set_ErrorCode(char data)
{
	WIFI_Status[12]=data;
}
void WIFI_Set_StopCleaning(char data)
{
	WIFI_Status[13]=data;
}
void WIFI_Clear_Status(uint8_t code)
{
	if(code)
	{
		WIFI_Set_WorkMode(App_WorkMode_Home);
		WIFI_Set_Forward(  0 );
		WIFI_Set_Backward( 0  );
		WIFI_Set_Left(  0 );
		WIFI_Set_Right(  0 );
		WIFI_Set_DownAdjust(  0 );
		WIFI_Set_SideAdjust( 0  );
		WIFI_Set_CleaningSpeed(  0 );
		WIFI_Set_RoomMode( 0  );
		WIFI_Set_BatteryCapacity( WIFI_GetBattery() );
		WIFI_Set_ChargerStatus( 1  );
		WIFI_Set_ErrorCode( Get_Error_Code()  );
		WIFI_Set_StopCleaning(  1 );
	}
	else
	{
		WIFI_Set_WorkMode(  0 );
		WIFI_Set_Forward(  0 );
		WIFI_Set_Backward( 0  );
		WIFI_Set_Left(  0 );
		WIFI_Set_Right(  0 );
		WIFI_Set_DownAdjust(  0 );
		WIFI_Set_SideAdjust( 0  );
		WIFI_Set_CleaningSpeed(  0 );
		WIFI_Set_RoomMode( 0  );
		WIFI_Set_BatteryCapacity( WIFI_GetBattery() );
		WIFI_Set_ChargerStatus( 0  );
		WIFI_Set_ErrorCode( Get_Error_Code()  );
		WIFI_Set_StopCleaning(  1 );
	}
  
}
uint8_t WIFI_GetBattery(void)
{
	static volatile int32_t temp[8]={0};
	static volatile uint8_t i=0;
	uint8_t k=0;
	int32_t tempB=0;
	
	tempB = GetBatteryVoltage();
	tempB -= 1250;
	if(tempB<0)tempB=0;
	if(tempB>400)tempB=400;	
	tempB/=4;
	
	temp[i]=tempB;
	i++;
	if(i>7)i=0;
	
	for(k=0;k<8;k++)
	{
		if(temp[k]==0)break;
		tempB += temp[k];	
	}
	tempB = tempB/(k+1);
	
	return tempB;
}

volatile uint8_t WIFI_Report_Flag=0;

void Set_WIFIReport_Flag(void)
{
	WIFI_Report_Flag=1;
}
void Clear_WIFIReport_Flag(void)
{
	WIFI_Report_Flag=0;
}
uint8_t Is_WIFIReport_Flag(void)
{
	return WIFI_Report_Flag;
}

/*-------- Turn Left ------------------------*/
void WIFI_Turn(uint16_t speed,uint16_t Dir)
{
	uint8_t Low_Power_Counter=0;
	volatile uint8_t Motor_Check_Code = 0;
//  uint16_t Counter_Watcher=0;
	uint8_t motor_check=0;
	
	if(Dir)Set_Dir_Left();
	else Set_Dir_Right();
  Reset_TempPWM();   
	Set_Wheel_Speed(speed,speed);
  while(1)
	{
	  delay(10);
    if(Is_Turn_Remote())break;
    if(Get_Cliff_Trig())break;
		if(Touch_Detect())
		{
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			break;
		}
		if(Get_Rcon_Remote())break;
		
		/*------------------------------------------------------Check Battery-----------------------*/
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
		  Low_Power_Counter++;
			if(Low_Power_Counter>10)
			{
			  Display_Battery_Status(Display_Low);//display low
//				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
 		}
		else
		{
		  Low_Power_Counter=0;
		}
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
//		  if(Self_Check(Motor_Check_Code))
			{
//			  Set_Clean_Mode(Clean_Mode_Userinterface);
//				Display_Content(LED_Exclamation,100,100,0,7);
				break;
			}
//			Initialize_Motor();	  
		}
	}
	Set_Wheel_Speed(0,0);
	Reset_Wheel_Step();
}

/*------wifi led */
void Display_WIFI_LED(void)
{
	static uint8_t display_switch=0;
	
	if(Get_Display_WIFI_LED_Status())
	{
		if((Get_EM3081_Status()==0xd5))//||(Get_EM3081_Status()==0))
		{
			Set_Wifi_LED(1);
			return ;
		}
		display_switch=1-display_switch;
		if(display_switch)
		{
			Set_Wifi_LED(1);
		}
		else
		{
			Set_Wifi_LED(0);
		}
	}
//	USART_Print("\n\rDisplay_WIFI_Status",Dev_USART3);
//	USART3_DMA_Numbers(Get_EM3081_Status());
}

void Set_Display_WIFI_LED_Status(uint8_t code)
{
	Display_WIFI_LED_Status = code;
}

uint8_t Get_Display_WIFI_LED_Status(void)
{
	return Display_WIFI_LED_Status;
}

void EMW3081_Vacuum_Power(void)
{
	if(Remote_Key(WIFI_Remote_Normal))
	{		
		Set_VacMode(Vac_Normal);
		Set_BLDC_TPWM(30);
		Set_Vac_Speed();
	}
	if(Remote_Key(WIFI_Remote_Max))
	{		
		Set_VacMode(Vac_Max);
		Set_BLDC_TPWM(80);
		Set_Vac_Speed();
	}
}

#endif

