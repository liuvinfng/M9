/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    24-May-2016
  * @brief   WIFI ESP8266 
  ******************************************************************************

  ******************************************************************************
  */  
   
/* Includes ------------------------------------------------------------------*/
  
#include "SysInitialize.h"
#include "Rcon.h"
#include "Movement.h"
#include "TouchPad.h"
#include "WIFI_ESP8266.h"
#include "Speaker.h" 
#include "CRC8.h"
#include "Display.h"
#include "Flash.h"
#include "USART.h" 
#include "UserInterface.h"
//#include "RTC.h"  


#ifdef WIFI_ESP8266
extern volatile uint8_t test_wifi_flag;

#define ANSWER_OK   (uint8_t)1
#define ANSWER_FAIL (uint8_t)0
/*FD*/

const char ESP8266_FD_Sleep[]="`FDrobot\0<sf td=\"pub\" k=\"DeviceAlert\" v=\"Sleep\"/></ctl>\0 \n";

/*FA*/
const char EPS8266_FA_BatteryInfo[] = "`FA\0\0<ctl td=\"BatteryInfo\"><battery power=\"\"/></ctl>\0 \n";
const char ESP8266_FA_ChargeState[] = "`FA\0\0<ctl td=\"ChargeState\"><charge type=\"\"/></ctl>\0 \n";
const char ESP8266_FA_CleanState[] = "`FA\0\0<ctl td=\"CleanReport\"><clean type=\"\" speed=\"\"/></ctl>\0 \n";
const char ESP8266_FA_Answer[] = "`FA\0\0<ctl ret=''/></ctl>\0 \n";
const char ESP8266_FA_CEN55Answer[] = "`FA\0\0<ctl ret='' NewSched='CEN55'/></ctl>\0 \n";
const char ESP8266_FA_Sched[] = "`FA\0\0<ctl td=\"Sched2\"></ctl>\0 \n";
const char ESP8266_Remote_SchedName[7][16] = {"14666666666660\n","14666666666661\n","14666666666662\n","14666666666663\n","14666666666664\n","14666666666665\n","14666666666666\n"};
const char ESP8266_FA_GetLog[] = "`FA\0\0<ctl td=\"Log\" time=\"\" evt=\"\"/></ctl>\0 \n"; //time 2014-6-24 11:11
const char ESP8266_FA_Version[] = "`FA\0\0<ctl td=\"version\"> <ver name=\"FW\"></ver></ctl>\0 \n";
const char ESP8266_FA_LifeSpan[] = "`FA\0\0<ctl td=\"LifeSpan\" type=\"\" val=\"\" total=\"365\"></ctl>\0 \n";
/*FC*/
//0f92836cf7
//20acb9003c
//5c43b4d633
//65b5c52bed
//b8bcdd00d4
//2c63d535df
//E1000872001497580001

//Ed1873ba0aaa6ff7abe1//出科沃斯1
//E3ad81a6008e4cf7650c//出科沃斯4
//Eec2affba08bcd929563
//E28c7f3ea7dd39d7b4d3//出科沃斯2
//Ed152b1966846094aad6//出科沃斯3

//E205a11f402be400feff
//Ea7b27a9f0e48439258e
//E915e0dd65820a1a8c0c
//E62a3af8122ce3e4aa44

const char EPS_Test[] = "`FC<ctl td='WIFIStat' st='c'/>\0 \n";
const char ESP8266_FC_Module_Setting[]="`FC<ctl td='DeviceCap'><MID>E205a11f402be400feff@112.ecorobot.net/atom</MID><class>112</class><acs v='setting,userman,clean'/><acms><acm ac='userman' td='GetUserInfo,AddUser,DelUser,SetAC,GetAccessControl,SetAccessControl'/><acm ac='clean' td='Move,GetChargeState,ChargeState,Clean,GetCleanState,CleanReport,GetLog,Log'/><acm ac='setting' td='SetTime,AddSched,ModSched,DelSched,Sched2,EmptyLog'/><acm ac='' td='BatteryInfo,LifeSpan,error'/></acms></ctl>\0 \n";
const char ESP8266_FC_ResetDefault[]="`FC<ctl td=\"FactoryRest\" id=\"req_resp\"/>\0 \n";
//const char ESP8266_FC_ChangeWIFIStatus[]="`FC<ctl id=\"req_resp\" td='SetWIFICmd' c='c'/>\0 \n";//
const char ESP8266_FC_GetWIFIStatus[]="`FC<ctl id=\"req_resp\" td='GetWIFIStat'/>\0 \n";

const char WString_Sched2[] ="Sched2\n";
/*FB*/
const char ESP8266_FB_BatteryInfo[]="`FB\0\0<ctl td=\"BatteryInfo\"><battery power=\"\"/></ctl>\0 \n";
const char ESP8266_FB_ChargeState[]="`FB\0\0<ctl td=\"ChargeState\"><charge type=\"\"/></ctl>\0 \n";
const char ESP8266_FB_LifeSpan[]="`FB\0\0<ctl td=\"LifeSpan\" type=\"\" val=\"\" total=\"\">\0 \n";
const char ESP8266_FB_CleanReport[]="`FB\0\0<ctl td=\"CleanReport\"><clean type=\"\" speed=\"\"/></ctl>\0 \n";
const char ESP8266_FB_Sched2[]= "`FB\0\0<ctl td=\"Sched2\"><s n=\"\" o=\"0\" t=\"\" r=''><ctl td=\"clean\" type=\"auto\"/></s></ctl>\0 \n";
const char ESP8266_FB_Sched[] = "`FB\0\0<ctl td=\"\"></ctl>\0 \n";
const char ESP8266_FB_error[]="`FB\0\0<ctl td=\"error\" errno=\"\" error=\"\"/>\0 \n";
/*FB charge state*/
const char WString_Going[]="Going\n";
const char WString_SlotCharging[]="SlotCharging\n";
const char WString_WireCharging[]="WireCharging\n";
const char WString_Idle[]="Idle\n";
/*FB Clean report*/
const char WString_SpotArea[]="SpotArea\n";
/*FA anser ok fail*/
const char WString_Ok[] = "ok\n";
const char WString_Fail[] = "fail\n"; 
/*Schedule*/
const char WString_Schedule[]="<s n=\"\" o=\"\" t=\"\" r=''><ctl td=\"clean\" type=\"auto\"/></s>\n";
/*G*/
const char WString_GetDeviceCap[] = "GetDeviceCap\n";
const char WString_GetBatteryInfo[] = "GetBatteryInfo\n";
const char WString_GetChargeState[] = "GetChargeState\n";
const char WString_GetVersion[] = "GetVersion\n";
const char WString_GetLifeSpan[] = "GetLifeSpan\n";
const char WString_GetLog[] = "GetLog\n";
const char WString_GetSched[] = "GetSched\n";
const char WString_GetCleanState[] = "GetCleanState\n";
/*A*/
const char WString_AddSched[] = "AddSched\n";
/*S*/
const char WString_SetTime[] = "SetTime\n";
const char WString_Standard[] = "standard\n";
const char WString_Strong[] = "strong\n";
const char WString_SetLifeSpan[] = "SetLifeSpan\n";
/*D*/
const char WString_DelSched[] = "DelSched\n";
const char WString_DCStatus[] = "DCStatus\n";
/*M*/
const char WString_ModSched[] = "ModSched\n";
const char WString_Move[] = "Move\n";
const char WString_forward[] = "forward\n";
const char WString_backward[] = "backward\n";
const char WString_stop[] = "stop\n";
const char WString_brake[] = "brake\n";
const char WString_SpinLeft[] = "SpinLeft\n";
const char WString_SpinRight[] = "SpinRight\n";
const char WString_go[] = "go\n";
const char WString_stopGo[] = "stopGo\n";
const char WString_auto[] = "auto\n";
const char WString_spot[] = "spot\n";
const char WString_border[] = "border\n";
const char WString_idle[] = "idle\n";
const char WString_singleRoom[] = "singleRoom\n";
/*n*/
const char WString_name[]="name\n";
/*c*/
const char WString_Clean[] = "Clean\n";
const char WString_Charge[] = "Charge\n";
/*w*/
const char WString_WIFIStat[] = "WIFIStat\n";
const char WString_WIFIStat_i[] = "i'\n";
const char WString_WIFIStat_s[] = "s'\n";
const char WString_WIFIStat_c[] = "c'\n";
const char WString_WIFIStat_o[] = "o'\n";
const char WString_WIFIStat_2[] = "2'\n";

const char WString_SideBrush[] = "SideBrush\n";
const char WString_DustCaseHeap[] = "DustCaseHeap\n";
const char WString_Brush[] = "Brush\n";

const char WString_timestamp[] = "timestamp\n";

const char test_ID[] = "103224@ecouser.net/androide063e5af176a\n";

volatile uint8_t ESP8266_Schedule[7][3][20]={0};//sched order , 

ESP8266_Sched Schedule_Data[14];//14

char Sched_String_Buffer[120]={0};
char String_Buffer[1700]={0};//2000
char Send_String_Buffer[1700]={0};
char ESP8266_ID[100]={0};
char ESP8266_Rec_ID[16]={0};
Log_Type Log_1[8];

//char day_search[7]={0};

char ESP8266_Receive_Buffer[RECEIVE_SET_SIZE][RECEIVE_BUFFER_SIZE]={0};//[2][500]
char temp_T[2][8]={0};
char temp_tt[8]={0};

/*char ESP8266_USER_ID[33]="abcdefghijklmnopqrstuvwxyz012345";*/

volatile char *Receive_Buffer_Pointer = ESP8266_Receive_Buffer[0];
volatile uint32_t Buffer_Pointer_Position=0;
//volatile uint32_t ESP8266_Received_Length=0;
volatile uint32_t ESP8266_Received_Flag=0;
volatile uint32_t ESP8266_Check_Postion=0;
volatile uint8_t ESP8266_Buffer_Number=0;
volatile WIFI_Rec Charge_Status=W_Charge_Idle;
volatile uint8_t Clean_Status=0;
volatile uint8_t Feed_Dog_Flag=0;
volatile uint8_t ESP8266_WIFI_S=0;
volatile uint8_t Sched_Order=0;
volatile uint8_t Log_Order=0;
volatile uint8_t Log_Count=0;
uint8_t Test_Module_Flag=0;
uint8_t Answer_Flag=0;

extern volatile uint8_t Receive_Barcode;
extern volatile char Barcode_Content[33]; 
extern volatile uint8_t Barcode_Receive_End;


void ESP8266_Event(void)
{
	ESP8266_Check_Data();
	Report_Battery_Onchange();
	Report_CleanState_Onchange();
	Report_Error_Onchange();
	Check_And_FeedDog();
}
void Report_Error_Onchange(void)
{
	static uint8_t Code_Buffer;
	if(Get_Error_Code()!=Code_Buffer)
	{
		ESP8266_Report_Error(Get_Error_Code());
	}
	Code_Buffer=Get_Error_Code();
}
void Report_CleanState_Onchange(void)
{
	static uint8_t Vac_Buffer=0;
	uint8_t Change_Flag=0;
	if(Get_VacMode()!=Vac_Buffer)Change_Flag=1;
	Vac_Buffer=Get_VacMode();
	if(Change_Flag)ESP8266_Report_CleanState(Get_Clean_State(),ESP8266_FB);
}
	
void Report_Battery_Onchange(void)
{
	static uint16_t BAT_Buffer=0;
	uint16_t Cur_Bat=0;
	static int32_t Temp_Time=0;
	int32_t Temp_C_Time=Get_WorkTime();
	if(((Temp_C_Time-Temp_Time)>80)||(Temp_C_Time<Temp_Time))//40S
	{
		Temp_Time=Temp_C_Time;
		Cur_Bat = Get_WIFI_Battery();
		if(BAT_Buffer!=Cur_Bat)
		{
			ESP8266_Report_Battery(Cur_Bat,ESP8266_FB);
		}
		BAT_Buffer=Cur_Bat;
	}
}

/*Data Receive Process -----------------------------------------------------*/
void Receive_ESP8266(char data)
{
//	DT TT;
	static uint8_t Receive_Data_Order=0;
	if(data==0x60)
	{
		Buffer_Pointer_Position=0;
	}
	ESP8266_Receive_Buffer[Receive_Data_Order][Buffer_Pointer_Position]=data;
	if(data==0x0a)
	{
		Set_ESP8266_Received();

//		TT = Get_DateTime();
//		temp_T[Receive_Data_Order][0]='T';
//		temp_T[Receive_Data_Order][1]='0'+TT.MNT;
//		temp_T[Receive_Data_Order][2]='0'+TT.MNU;
//		temp_T[Receive_Data_Order][3]=':';
//		temp_T[Receive_Data_Order][4]='0'+TT.ST;
//		temp_T[Receive_Data_Order][5]='0'+TT.SU;
//		temp_T[Receive_Data_Order][6]=' ';
//		temp_T[Receive_Data_Order][7]=' ';

		Receive_Data_Order++;
		if(Receive_Data_Order>(RECEIVE_SET_SIZE-1))Receive_Data_Order=0;
	}
	Buffer_Pointer_Position++;
	if(Buffer_Pointer_Position>(RECEIVE_BUFFER_SIZE-1))Buffer_Pointer_Position=0;
	//USART3_Transmit_Byte(data);
}
/*ESP8266 USART data flag*/
uint8_t Is_ESP8266_Received(void)
{
	return ESP8266_Buffer_Number;
}
void Set_ESP8266_Received(void)
{
  ESP8266_Buffer_Number++;
	//if(ESP8266_Buffer_Number>3)ESP8266_Buffer_Number=3;
}
void Clear_ESP8266_Receive(void)
{
  if(ESP8266_Buffer_Number>0)ESP8266_Buffer_Number--;
//	USART3_DMA_String(17,(char *)"\nReceive_Buffer: ");
//	delay(500);
//	USART3_DMA_Numbers(ESP8266_Buffer_Number);
}


/*Data Check and respond function -----------------------------------------------------*/
void ESP8266_Check_Data(void)
{
//	uint8_t NM=0;
	uint32_t Extract_Result=0;
	uint8_t Data_Get=0;
	uint32_t Check_Start = 0,Check_End = 0 , i=0;
	uint32_t Temp_Length=0;
	uint8_t Temp_CRC=0;
	static uint8_t ESP8266_CheckData_Order=0;
	
	char *Data_Buffer = &ESP8266_Receive_Buffer[ESP8266_CheckData_Order][0];
	
	if(Is_ESP8266_Received()==0)return;
	
//  USART3_Print(Data_Buffer);
//	USART_Print(Data_Buffer);
	
	Check_Start = ESP8266_Check_Postion;
	Data_Get=0;
	for(i=0;i<(RECEIVE_BUFFER_SIZE-1);i++)//check data header == 60 and end == 0a
	{
		 if(ESP8266_Receive_Buffer[ESP8266_CheckData_Order][i]==0x60)
		 {
			 Data_Buffer = &ESP8266_Receive_Buffer[ESP8266_CheckData_Order][i];
			 Check_Start=i;
			 Data_Get|=0x01;
		 }
		 if(ESP8266_Receive_Buffer[ESP8266_CheckData_Order][i]==0x0a)
		 {
			 Check_End=i;
			 Data_Get|=0x02;
			 break;
		 }
		 if(i>=(RECEIVE_BUFFER_SIZE-1))break;
	}
	if(Data_Get==0x03)
	{
		Temp_Length=Check_End-Check_Start;
		
		Data_Buffer+=3;
		Temp_CRC = calcBufCrc8(Data_Buffer,Temp_Length-4);//get crc
		
		if(ESP8266_Receive_Buffer[ESP8266_CheckData_Order][Check_End-1]==Temp_CRC)//data veryfy ok! crc compare
		{
			/*do some data extraction*/
			ESP8266_Get_ID(Data_Buffer);
//--------------------------		
			Data_Buffer+=Get_String_Length(ESP8266_ID);
//			USART3_Print("\nFrom Module: \n");
//			delay(20);
//			USART3_Transmit_String(8,&temp_T[ESP8266_CheckData_Order][0]);
//			delay(20);
//      USART3_Print(Data_Buffer);
//      USART3_Print((char *)"\nExtracting data\n");
			Extract_Result = ESP8266_Data_Extract(Data_Buffer,Temp_Length);
			switch(Extract_Result)
			{
				case W_GetDeviceCap:		 //ESP8266_Initialize_Module();
				                       Send_ESP8266_Setting((char *)ESP8266_FC_Module_Setting);
																///USART3_Print((char *)"\nESP8266_Initialize\n");
																break;
				case W_GetBatteryInfo:	//Syn_FB_Bat();
				                        ESP8266_Report_Battery(Get_WIFI_Battery(),ESP8266_FA);
																break;
				case W_GetChargeState:	ESP8266_Report_ChargeState(Get_Charge_State(),ESP8266_FA);
																break;
				case W_GetCleanState:		ESP8266_Report_CleanState(Get_Clean_State(),ESP8266_FA);
																break;
				case W_Move_Forward:		Set_Rcon_Remote(ESP8266_Remote_Forward);
																//USART3_Print((char *)"\nESP8266_Move_Forward\n");
																break;
				case W_Move_SpinRight:	Set_Rcon_Remote(ESP8266_Remote_Right);
																//USART3_Print((char *)"\nESP8266_Move_right\n");
																break;
				case W_Move_SpinLeft:		Set_Rcon_Remote(ESP8266_Remote_Left);
																//USART3_Print((char *)"\nESP8266_Move_left\n");
																break;
				case W_Move_Backward:		Set_Rcon_Remote(ESP8266_Remote_Backward);
																//USART3_Print((char *)"\nESP8266_Move_backward\n");
																break;
				case W_Move_Stop:				Set_Rcon_Remote(ESP8266_Remote_MoveStop);
																//USART3_Print((char *)"\nESP8266_Move_stop\n");
																Set_Touch();//Set_Touch_Flag(1);//
																//USART3_Print((char *)"\nSet Touch\n");
																break;
				case W_Charge_Go:				Set_Rcon_Remote(ESP8266_Remote_Home);
																//USART3_Print((char *)"\nESP8266_Home\n");
																break;
				case W_Charge_StopGo:   Set_Rcon_Remote(ESP8266_Remote_ChargeStopGo);
																//USART3_Print((char *)"\nESP8266_Charge Stop Go\n");
																Set_Touch();//Set_Touch_Flag(1);//
																//USART3_Print((char *)"\nSet Touch\n");
																break;
				case W_Clean_Auto:      Set_Rcon_Remote(ESP8266_Remote_Auto);
																//USART3_Print((char *)"\nESP8266_Clean_auto\n");
																break;
				case W_Clean_Spot:			//if(Get_Clean_Mode()!=Clean_Mode_RandomMode)//edit by vin
																{Set_Rcon_Remote(ESP8266_Remote_Spot);}
																//USART3_Print((char *)"\nESP8266_Clean_spot\n");
																break;
				case W_Clean_Border:		//if(Get_Clean_Mode()!=Clean_Mode_RandomMode)//edit by vin
																{Set_Rcon_Remote(ESP8266_Remote_Border);}
																//USART3_Print((char *)"\nESP8266_Clean Border\n");
																break;
				case W_Clean_Stop:			Set_Rcon_Remote(ESP8266_Remote_CleanStop);
																//USART3_Print((char *)"\nESP8266_Clean Stop\n");
																Set_Touch();//Set_Touch_Flag(1);//
																//USART3_Print((char *)"\nSet Touch\n");
																break;
				case W_Clean_SingleRoom:Set_Rcon_Remote(ESP8266_Remote_SingleRoom);
																//USART3_Print((char *)"\nESP8266_Move_single room\n");
																break;//pending
				case W_WIFIState_Connected:	Set_WIFI_Status(ESP8266_WIFIStatus_Connected);
																		//USART3_Print((char *)"\nESP8266_WIFI_Connected\n");
																		break;
				case W_WIFIState_Connecting:	Set_WIFI_Status(ESP8266_WIFIStatus_Connecting);
																			//USART3_Print((char *)"\nESP8266_WIFI_Connecting\n");
																		break;
				case W_WIFIState_Smart:	Set_WIFI_Status(ESP8266_WIFIStatus_Connecting);
																//USART3_Print((char *)"\nESP8266_WIFI_Smart\n");
																						break;
				case W_WIFIState_Idle:	Set_WIFI_Status(ESP8266_WIFIStatus_Idle);
																//USART3_Print((char *)"\nESP8266_WIFI_Idle\n");
																						break;		
				case W_SetTime: //if(ESP8266_Set_Time(Data_Buffer))ESP8266_Answer_OF(ANSWER_FAIL);//answer fail
												//else ESP8266_Answer_OF(ANSWER_OK);//answer ok
				                ESP8266_Set_Time(Data_Buffer);
				                Reset_UI_OutCounter(); 
//				                USART3_Transmit_String(17,(char *)"\nPrint RTC time: ");
//												delay(10);
//				                Print_RTC_Time(Get_DateTime());
												//USART3_Print((char *)"ESP8266_WIFI_Set_Time\n");
												break;				
				case W_GetSched:ESP8266_Report_Schedule(ESP8266_FA);
				                Reset_UI_OutCounter();
												//USART3_Print((char *)"\nFA Report Schedule!\n");
												break;
				case W_AddSched:if(ESP8266_Add_Schedule(Data_Buffer))ESP8266_Answer_OF(ANSWER_FAIL);//answer fail
												else ESP8266_Answer_OF(ANSWER_OK);//answer ok
												Sort_Schedule_Data();
												Check_and_SetAlarm();
				                Reset_UI_OutCounter();
												//Print_RTC_Time(Get_DateTime());
												//Print_RTC_Alarm();
												//Print_Scheduled_Time();
												//USART3_Print((char *)"\nESP8266_WIFI_Add Schedule\n");
												break;
				case W_ModSched:if(ESP8266_Mod_Schedule(Data_Buffer))ESP8266_Answer_OF(ANSWER_FAIL);//answer fail
												else ESP8266_Answer_OF(ANSWER_OK);//answer ok
												Sort_Schedule_Data();
												Check_and_SetAlarm();
												Reset_UI_OutCounter();
//												Print_RTC_Time(Get_DateTime());
//												Print_RTC_Alarm();
//												Print_Scheduled_Time();
												Beep(2);
									      //USART3_Print((char *)"\nESP8266_WIFI_Mod Schedule\n");
												break;
				case W_DelSched:if(ESP8266_Del_Schedule(Data_Buffer))ESP8266_Answer_OF(ANSWER_FAIL);//answer fail
												else ESP8266_Answer_OF(ANSWER_OK);//answer ok
												//USART3_Print((char *)"\nESP8266_WIFI_Delete Schedule\n");
												Sort_Schedule_Data();
												Check_and_SetAlarm();
												Reset_UI_OutCounter();
//												Print_RTC_Time(Get_DateTime());
//												Print_RTC_Alarm();
//												Print_Scheduled_Time();
												Beep(2);
												break;
				case W_GetLog:  ESP8266_Report_Log();
					              break;
				case W_Answer_Ok:break;				//USART3_Print((char *)"\nESP8266_WIFI_Answer_OK\n");break;
				case W_Answer_Fail:break;			//USART3_Print((char *)"\nESP8266_WIFI_Answer_Fail\n");break;
				case W_GetVersion:			ESP8266_Report_Version();break;
				case W_GetLifeSpan_SideBrush:			ESP8266_Report_LifeSpan(Life_SideBrush);break;
				case W_GetLifeSpan_Brush:					ESP8266_Report_LifeSpan(Life_Brush);break;
				case W_GetLifeSpan_DustCaseHeap:	ESP8266_Report_LifeSpan(Life_DustCaseHeap);break;
				case W_SetSideBrushLife:					ESP8266_Reset_LifeSpan(W_SetSideBrushLife);break;
				case W_SetBrushLife:							ESP8266_Reset_LifeSpan(W_SetBrushLife);break;
				case W_SetDustCaseHeapLife:				ESP8266_Reset_LifeSpan(W_SetDustCaseHeapLife);break;
				default:break;
			}
//			if(NM)
//			{
//				USART3_DMA_String(16,(char *)"\nExtrac_Result: ");
//				USART3_DMA_Numbers(Extract_Result);
//				USART3_Print((char *)" \n");
//			}
		}
	}
	for(i=Check_Start;i<(Check_Start+10);i++)//clear received data
	{
		if(i<RECEIVE_BUFFER_SIZE)
		{
			ESP8266_Receive_Buffer[ESP8266_CheckData_Order][i]=0;
		}
	}
  ESP8266_CheckData_Order=1-ESP8266_CheckData_Order;

//	ESP8266_CheckData_Order++;
//	if(ESP8266_CheckData_Order>(RECEIVE_SET_SIZE-1))ESP8266_CheckData_Order=0;
	Clear_ESP8266_Receive();
}
/*Data Extract -----------------------------------------------------*/
WIFI_Rec ESP8266_Data_Extract(char *data,uint32_t Length)
{
//	uint32_t Equal_Position=0;
	char String_Result=1;
	WIFI_Rec Temp_Clean=W_Null;
	char *buffer = data;
  
	Get_Rec_ID(data);//get rec id
	
	buffer = String_P(buffer,(char *)"td\n",&String_Result);//go to td = 
	if(String_Result)//no td
	{
		buffer = String_P(buffer,(char *)"ret\n",&String_Result);//got ret =
		if(String_Result)return W_Null;//no ret
	}
	switch(*buffer)
	{
		case 'o': if(Check_String_Match(buffer,(char *)WString_Ok))
							{
								Answer_Flag=1;
								return W_Answer_Ok;
							}break;
		case 'f': if(Check_String_Match(buffer,(char *)WString_Fail))return W_Answer_Fail;
							break;
		case 'G': if(Check_String_Match(buffer,(char *)WString_GetSched))return W_GetSched;		
							if(Check_String_Match(buffer,(char *)WString_GetDeviceCap))return W_GetDeviceCap;
							if(Check_String_Match(buffer,(char *)WString_GetBatteryInfo))return W_GetBatteryInfo;
							if(Check_String_Match(buffer,(char *)WString_GetChargeState))return W_GetChargeState;
							if(Check_String_Match(buffer,(char *)WString_GetVersion))return W_GetVersion;
							if(Check_String_Match(buffer,(char *)WString_GetCleanState))return W_GetCleanState;  
		          if(Check_String_Match(buffer,(char *)WString_GetLog))return W_GetLog;
							if(Check_String_Match(buffer,(char *)WString_GetLifeSpan))
							{
								buffer = String_P(buffer,(char *)"type\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_SideBrush))return W_GetLifeSpan_SideBrush;
								if(Check_String_Match(buffer,(char *)WString_Brush))return W_GetLifeSpan_Brush;
								if(Check_String_Match(buffer,(char *)WString_DustCaseHeap))return W_GetLifeSpan_DustCaseHeap;
							}
							break;
		case 'A': if(Check_String_Match(buffer,(char *)WString_AddSched))return W_AddSched;		
							break;
		case 'D': if(Check_String_Match(buffer,(char *)WString_DelSched))return W_DelSched;		
							if(Check_String_Match(buffer,(char *)WString_DCStatus))
							{
								buffer = String_P(buffer,(char *)"conn\n",&String_Result);
								if(Check_String_Match(buffer,(char *)WString_WIFIStat_2))
								{
									return W_WIFIState_Connected;
								}
							}
							break;
		case 'S': if(Check_String_Match(buffer,(char *)WString_SetTime))return W_SetTime;		
							if(Check_String_Match(buffer,(char *)WString_SetLifeSpan))
							{
								buffer = String_P(buffer,(char *)"type\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_SideBrush))return W_SetSideBrushLife;
								if(Check_String_Match(buffer,(char *)WString_Brush))return W_SetBrushLife;
								if(Check_String_Match(buffer,(char *)WString_DustCaseHeap))return W_SetDustCaseHeapLife;
							}
							break;
		case 'M': if(Check_String_Match(buffer,(char *)WString_Move))
							{
								buffer = String_P(buffer,(char *)"action\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_forward))return W_Move_Forward;
								if(Check_String_Match(buffer,(char *)WString_backward))return W_Move_Backward;
								if(Check_String_Match(buffer,(char *)WString_SpinRight))return W_Move_SpinRight;
								if(Check_String_Match(buffer,(char *)WString_SpinLeft))return W_Move_SpinLeft;
								if(Check_String_Match(buffer,(char *)WString_brake))return W_Move_Brake;
								if(Check_String_Match(buffer,(char *)WString_stop))return W_Move_Stop;
							}
							if(Check_String_Match(buffer,(char *)WString_ModSched))return W_ModSched;
							break;
		case 'C': if(Check_String_Match(buffer,(char *)WString_Clean))
							{
								buffer = String_P(buffer,(char *)"type\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_auto))Temp_Clean =  W_Clean_Auto;
								else if(Check_String_Match(buffer,(char *)WString_spot)){if(Get_Clean_Mode()==Clean_Mode_Navigation)break;Temp_Clean = W_Clean_Spot;}//edit by vin
								else if(Check_String_Match(buffer,(char *)WString_border)){if(Get_Clean_Mode()==Clean_Mode_Navigation)break;Temp_Clean = W_Clean_Border;}//edit by vin
								else if(Check_String_Match(buffer,(char *)WString_stop))return W_Clean_Stop;
								else if(Check_String_Match(buffer,(char *)WString_singleRoom))Temp_Clean = W_Clean_SingleRoom;
								else if(Check_String_Match(buffer,(char *)WString_go))Temp_Clean = W_Charge_Go;
								else if(Check_String_Match(buffer,(char *)WString_stopGo))Temp_Clean = W_Charge_StopGo;
								buffer = String_P(buffer,(char *)"speed\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_Standard))
								{	
									Set_VacMode(Vac_Normal);//Set_BLDC_Run_Flag(0);//
									Set_Vac_Buffer(Vac_Normal);//old_bldc_run_flag=0;//
									if((Get_Clean_Mode()==Clean_Mode_WallFollow)||(Get_Clean_Mode()==Clean_Mode_RandomMode)||(Get_Clean_Mode()==Clean_Mode_Navigation)||(Get_Clean_Mode()==Clean_Mode_Remote))//||(Get_Clean_Mode()==Clean_Mode_Spot)
									{
										Set_Vac_Speed();//Set_Vacuum(Vacuum_Power);//
									}
									//USART3_Print((char *)"\nSet_Vac_MIN\n");
								}
								else if(Check_String_Match(buffer,(char *)WString_Strong))
								{
									Set_VacMode(Vac_Max);//Set_BLDC_Run_Flag(1);//
									Set_Vac_Buffer(Vac_Max);//old_bldc_run_flag=1;//
									if((Get_Clean_Mode()==Clean_Mode_WallFollow)||(Get_Clean_Mode()==Clean_Mode_RandomMode)||(Get_Clean_Mode()==Clean_Mode_Spot)||(Get_Clean_Mode()==Clean_Mode_Navigation)||(Get_Clean_Mode()==Clean_Mode_Remote))
									{
										Set_Vac_Speed();//Set_Vacuum(Vacuum_Power);//
									}
									//USART3_Print((char *)"\nSet_Vac_MAX\n");
								}
								if(Temp_Clean)return Temp_Clean;
							}
							if(Check_String_Match(buffer,(char *)WString_Charge))
							{
								buffer = String_P(buffer,(char *)"type\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_go))return W_Charge_Go;
								if(Check_String_Match(buffer,(char *)WString_stopGo))return W_Charge_StopGo;
							}
							break;
		case 'W': if(Check_String_Match(buffer,(char *)WString_WIFIStat))
							{
								buffer = String_P(buffer,(char *)"st\n",&String_Result);
								if(String_Result)break;
								if(Check_String_Match(buffer,(char *)WString_WIFIStat_i))return W_WIFIState_Idle;
								if(Check_String_Match(buffer,(char *)WString_WIFIStat_s))return W_WIFIState_Smart;
								if(Check_String_Match(buffer,(char *)WString_WIFIStat_c))return W_WIFIState_Connecting;
								if(Check_String_Match(buffer,(char *)WString_WIFIStat_o))
								{
									Test_Module_Flag=1;
//									if((Get_WIFI_Status()==ESP8266_WIFIStatus_Idle)&&(Get_Clean_Mode()!=Clean_Mode_Sleep))
//									{
//										USART3_Print("\nConnecting!\n");
//										return W_WIFIState_Connecting;
//									}
								}
							}
							break;
		default:break;
	}
	return W_Null;
}

/*Initilize WIFI Module*/
uint8_t Check_WIFI_ID(void)
{
	Get_WIFI_ID((char *)Barcode_Content);
	Barcode_Content[32]='\n';
	if(Barcode_Content[0]=='E')return 1;
	return 0;
}
void ESP8266_Initialize_Module(void)
{
//	uint8_t i=0;
	Get_WIFI_ID((char *)Barcode_Content);
	Barcode_Content[32]='\n';
//	USART3_Print(Barcode_Content);
//	delay(1000);
	Add_String((char *)ESP8266_FC_Module_Setting,(char *)Barcode_Content,(char *)"><MI\n",Send_String_Buffer);
	Send_ESP8266_Setting(Send_String_Buffer);
//	USART3_Print(Send_String_Buffer);
}
/*check if is time to feed the wifi watch dog*/ 
void Check_And_FeedDog(void)
{
	if(Need_Feed_Dog())
	{
		Clear_Feed_Dog();
		ESP8266_FeedDog();
	}
}
/*report error*/
void ESP8266_Report_Error(uint8_t E)
{
	char Temp_Error_String[4];
	
	if(E==0)E=100;
	if(E>99)Temp_Error_String[0]='1';
	else Temp_Error_String[0]='0';
	E=E%100;
	Temp_Error_String[1]='0'+E/10;
	Temp_Error_String[2]='0'+E%10;
	Temp_Error_String[3]='\n';
	
	Add_String((char *)ESP8266_FB_error,(char *)Temp_Error_String,(char *)"errno\n",Send_String_Buffer);
	Send_ESP8266_Setting(Send_String_Buffer);

}
void ESP8266_Clear_Error(void)
{
	ESP8266_Report_Error(100);
}
/*FD sleep */
void ESP8266_Report_Sleep(void)
{
	//Add_String((char *)ESP8266_FD_Sleep,(char *)WString_timestamp,(char *)"ts\n",Send_String_Buffer);
	Send_ESP8266_Setting((char *)ESP8266_FD_Sleep);
	//USART3_Print(Send_String_Buffer);
}
/*report clean status */
void ESP8266_Report_CleanState(WIFI_Rec S,uint8_t FAB)
{
	if(FAB==ESP8266_FA)
	{
		Add_UserID_String((char *)ESP8266_FA_CleanState,ESP8266_ID);
		switch(S)
		{
			case W_Clean_Auto:				Add_String(Send_String_Buffer,(char *)WString_auto,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFA report clean Auto!\n");
																break;
			case W_Clean_Spot:				Add_String(Send_String_Buffer,(char *)WString_spot,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFA report clean Spot!\n");
																break;
			case W_Clean_SpotArea:		Add_String(Send_String_Buffer,(char *)WString_SpotArea,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFA report clean SportArea!\n");
																break;
			case W_Clean_Stop:				Add_String(Send_String_Buffer,(char *)WString_stop,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFA report clean Stop!\n");
																break;
			case W_Clean_Border:      Add_String(Send_String_Buffer,(char *)WString_border,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFA report clean Border!\n");
																break;
			case W_Clean_Idle:        Add_String(Send_String_Buffer,(char *)WString_idle,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFA report clean Idle!\n");
																break;
			default://USART3_Print((char *)"Report Clean FA No match!\n");
							return;
		}
	}
	else
	{
		switch(S)
		{
			case W_Clean_Auto:				Add_String((char *)ESP8266_FB_CleanReport,(char *)WString_auto,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFB report clean Auto!\n");
																break;
			case W_Clean_Spot:				Add_String((char *)ESP8266_FB_CleanReport,(char *)WString_spot,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFB report clean Spot!\n");
																break;
			case W_Clean_SpotArea:		Add_String((char *)ESP8266_FB_CleanReport,(char *)WString_SpotArea,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFB report clean SpotArea!\n");
																break;
			case W_Clean_Stop:				Add_String((char *)ESP8266_FB_CleanReport,(char *)WString_stop,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFB report clean Stop!\n");
																break;
			case W_Clean_Border:      Add_String((char *)ESP8266_FB_CleanReport,(char *)WString_border,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFB report clean Border!\n");
																break;
			case W_Clean_Idle:        Add_String((char *)ESP8266_FB_CleanReport,(char *)WString_idle,(char *)"type\n",Send_String_Buffer);
																//USART3_Print((char *)"\nFB report clean Idle!\n");
																break;
			default://USART3_Print((char *)"Report Clean FB No match!\n");
							return;
		}
	}
	if(Get_VacMode()==Vac_Normal)////!Get_BLDC_Run_Flag()
	{
		Add_String(Send_String_Buffer,(char *)WString_Standard,(char *)"speed\n",Send_String_Buffer);
	}
	else
	{
		Add_String(Send_String_Buffer,(char *)WString_Strong,(char *)"speed\n",Send_String_Buffer);
	}
	Send_ESP8266_Setting(Send_String_Buffer);
	////USART3_Print(Send_String_Buffer);
}
/*Report charget state  FA or FB */
void ESP8266_Report_ChargeState(WIFI_Rec S,uint8_t FAB)//report charge state
{
	if(FAB==ESP8266_FA)
	{
		Add_UserID_String((char *)ESP8266_FA_ChargeState,ESP8266_ID);
		switch(S)
		{
			case W_Charge_Going:				Add_String(Send_String_Buffer,(char *)WString_Going,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFA report Charge Going!\n");
																	break;
			case W_Charge_SlotCharging:	Add_String(Send_String_Buffer,(char *)WString_SlotCharging,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFA report Charge SlotCharging!\n");
																	break;
			case W_Charge_WireCharging:	Add_String(Send_String_Buffer,(char *)WString_WireCharging,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFA report Charge WireChargeing!\n");
																	break;
			case W_Charge_Idle:					Add_String(Send_String_Buffer,(char *)WString_Idle,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFA report Charge Idle!\n");
																	break;
			default:return;//USART3_Print((char *)"Report Charge State FA No match!\n");return;
		}
	}
	else
	{
		switch(S)
		{
			case W_Charge_Going:				Add_String((char *)ESP8266_FB_ChargeState,(char *)WString_Going,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFB report Charge Going!\n");
																	break;
			case W_Charge_SlotCharging:	Add_String((char *)ESP8266_FB_ChargeState,(char *)WString_SlotCharging,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFB report Charge SlotCharging!\n");
																	break;
			case W_Charge_WireCharging:	Add_String((char *)ESP8266_FB_ChargeState,(char *)WString_WireCharging,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFB report Charge WireCharging!\n");
																	break;
			case W_Charge_Idle:					Add_String((char *)ESP8266_FB_ChargeState,(char *)WString_Idle,(char *)"type\n",Send_String_Buffer);
																	//USART3_Print((char *)"\nFB report Charge Idle!\n");
																	break;
			default:return;//USART3_Print((char *)"Report Charge State FB No match!\n");return;
		}
	}
	Send_ESP8266_Setting(Send_String_Buffer);
	////USART3_Print(Send_String_Buffer);
}
/*report batttery :v percentage of battery power , FAB = ESP8266_FA,ESP8266_FB,ESP8266_FC channel*/
void ESP8266_Report_Battery(uint32_t V,uint8_t FAB)//Report Battery Voltage
{
	char Temp_Battery_String[4];

	if(V>99)Temp_Battery_String[0]='1';
	else Temp_Battery_String[0]='0';
	V=V%100;
	Temp_Battery_String[1]='0'+V/10;
	Temp_Battery_String[2]='0'+V%10;
	Temp_Battery_String[3]='\n';
//if(V>99)V=99;
//Temp_Battery_String[0]='0'+V/10;
//Temp_Battery_String[1]='0'+V%10;
//Temp_Battery_String[2]='\n';
	
	if(FAB==ESP8266_FA)
	{
		Add_UserID_String((char *)EPS8266_FA_BatteryInfo,ESP8266_ID);
		Add_String(Send_String_Buffer,(char *)Temp_Battery_String,(char *)"power\n",Send_String_Buffer);
		//USART3_Print((char *)"\nFA Report Battery Info\n");
	}
	else
	{
		Add_String((char *)ESP8266_FB_BatteryInfo,(char *)Temp_Battery_String,(char *)"power\n",Send_String_Buffer);
		//USART3_Print((char *)"\nFB Report Battery Info\n");
	}
	Send_ESP8266_Setting(Send_String_Buffer);
}

/* WIFI feed dog*/
void ESP8266_FeedDog(void)
{
	Send_ESP8266_Setting((char *)ESP8266_FC_GetWIFIStatus);
	//USART3_Print((char *)"\nFeed_ESP8266\n");
}
/* Reset WIFI module*/
void Reset_ESP8266(void)
{
	Send_ESP8266_Setting((char *)ESP8266_FC_ResetDefault);
//	Send_ESP8266_Setting((char *)ESP8266_FC_ChangeWIFIStatus);
}
/* Set Time */
uint8_t ESP8266_Set_Time(char *data)
{
	DT Temp_DT;
	char String_Result;
//	int8_t TimeZone=0;
	char *buffer=data;

	buffer = String_P(buffer,(char *)"year\n",&String_Result);//year='
	if(String_Result)return 1;
	  buffer+=2;
		Temp_DT.YT = *(buffer++)-'0';
	  Temp_DT.YU = *buffer-'0';
	buffer = String_P(buffer,(char *)"month\n",&String_Result);//month='
	if(String_Result)return 1;
	  buffer++;
	  if(*buffer=='"')//less then 10
		{
			buffer--;
			Temp_DT.MT = 0;
			Temp_DT.MU = *buffer-'0';
		}
		else
		{
			buffer--;
			Temp_DT.MT = *(buffer++)-'0';
			Temp_DT.MU = *buffer-'0';
		}
	buffer = String_P(buffer,(char *)"date\n",&String_Result);//date='
	if(String_Result)return 1;
		buffer++;
	  if(*buffer=='"')//less then 10
		{
			buffer--;
			Temp_DT.DT = 0;
			Temp_DT.DU = *buffer-'0';
		}
		else
		{
			buffer--;
			Temp_DT.DT = *(buffer++)-'0';
			Temp_DT.DU = *buffer-'0';
		}
	buffer = String_P(buffer,(char *)"hour\n",&String_Result);//hour='
	if(String_Result)return 1;
		buffer++;
	  if(*buffer=='"')//less then 10
		{
			buffer--;
			Temp_DT.HT = 0;
			Temp_DT.HU = *buffer-'0';
		}
		else
		{
			buffer--;
			Temp_DT.HT = *(buffer++)-'0';
			Temp_DT.HU = *buffer-'0';
		}
	buffer = String_P(buffer,(char *)"minute\n",&String_Result);//minute='
	if(String_Result)return 1;
		buffer++;
	  if(*buffer=='"')//less then 10
		{
			buffer--;
			Temp_DT.MNT = 0;
			Temp_DT.MNU = *buffer-'0';
		}
		else
		{
			buffer--;
			Temp_DT.MNT = *(buffer++)-'0';
			Temp_DT.MNU = *buffer-'0';
		}
//	buffer = String_P(buffer,(char *)"tz\n");//tz=' no use now
//    if(*(buffer++)=='+')
//		{
//			TimeZone=*buffer-'0';
//		}
//		else
//		{
//			TimeZone=-(*buffer-'0');
//		}
		
	Temp_DT.WK = weekday(Temp_DT.YT*10+Temp_DT.YU+2000,Temp_DT.MT*10+Temp_DT.MU,Temp_DT.DT*10+Temp_DT.DU);
		
	Set_RTC_DateTime(Temp_DT);
	if(!Check_RTC_Set(Temp_DT))
	{
//		ESP8266_CEN55_Answer(ANSWER_FAIL);
		ESP8266_Answer_OF(ANSWER_FAIL);
		Reset_RTC();
		RTC_Configuration();
//		Beep(5);
//		Beep(1);
//		Beep(5);
//		Beep(5);
//		USART3_Print("\nRe Configuration RTC!\n");
//		delay(100);
	}
	else
	{
//		ESP8266_CEN55_Answer(ANSWER_OK);
		ESP8266_Answer_OF(ANSWER_OK);
//		USART3_Print("\nRTC Set Date Time OK!\n");
//		delay(100);
	}
//	ESP8266_Answer_OF(1);//answer ok
//	USART3_Transmit_String(17,(char *)"\nPrint Set time: ");
//	Print_RTC_Time(Temp_DT);	
		return 0;		
}

/* Add Schedule */
uint8_t ESP8266_Add_Schedule(char *data)
{
	uint8_t i=0;
	char String_Result;
	char *buffer=data;
//	for(j=0;j<7;j++)
	{
			buffer = String_P(buffer,(char *)"name\n",&String_Result);//td='
			if(String_Result)return 1;
			Store_SchedName(Sched_Order,buffer);
			buffer = String_P(buffer,(char *)"on\n",&String_Result);//on='
			if(String_Result)return 1;
			Schedule_Data[Sched_Order].OF = *(buffer++)-'0';
			buffer = String_P(buffer,(char *)"time\n",&String_Result);//time='
			if(String_Result)return 1;
			buffer++;
			if(*buffer==':')
			{
				buffer--;
				Schedule_Data[Sched_Order].HT = 0;
				Schedule_Data[Sched_Order].HU = *(buffer++)-'0';
			}
			else
			{
				buffer--;
				Schedule_Data[Sched_Order].HT = *(buffer++)-'0';
				Schedule_Data[Sched_Order].HU = *(buffer++)-'0';
			}
			buffer+=2;
			if((*buffer==0x27)||(*buffer==0x22))//0x27 == ' ,0x22 == "
			{
				buffer--;
				Schedule_Data[Sched_Order].MNT = 0;
				Schedule_Data[Sched_Order].MNU = *(buffer++)-'0';
			}
			else
			{
				buffer--;
				Schedule_Data[Sched_Order].MNT = *(buffer++)-'0';
				Schedule_Data[Sched_Order].MNU = *(buffer++)-'0';
			}
			buffer = String_P(buffer,(char *)"repeat\n",&String_Result);//repeat='
			if(String_Result)return 1;
			for(i=0;i<7;i++)
			{
				if(*buffer=='1')
				{
					Schedule_Data[Sched_Order].Day=i;//i
					break;//you
				}
				buffer++;		
			}
			
			if(Sched_Order<7)//limit schedule to 7 sets//14
			{
				Sched_Order++;
				Beep(2);
				return 0;
			}
			else 
			{
				return 1;
			}
	}
}
/*//uint8_t ESP8266_Add_Schedule(char *data)
//{
//	uint8_t i=0,j=0,k=0;
//	char String_Result;
//	char *buffer=data;
//	
//	buffer = String_P(buffer,(char *)"repeat\n",&String_Result);//repeat='
//	if(String_Result)return 1;
//	for(j=0;j<7;j++)
//	{
//		day_search[j]=*(buffer++)-'0';
//	}
//	
//	for(k=0;k<7;k++)
//	{
//		if(day_search[i]==1)
//		{
//		  buffer=data;
//			buffer = String_P(buffer,(char *)"name\n",&String_Result);//td='
//			if(String_Result)return 1;
//			Store_SchedName(Sched_Order,buffer);
//			buffer = String_P(buffer,(char *)"on\n",&String_Result);//on='
//			if(String_Result)return 1;
//			Schedule_Data[Sched_Order].OF = *(buffer++)-'0';
//			buffer = String_P(buffer,(char *)"time\n",&String_Result);//time='
//			if(String_Result)return 1;
//			buffer++;
//			if(*buffer==':')
//			{
//				buffer--;
//				Schedule_Data[Sched_Order].HT = 0;
//				Schedule_Data[Sched_Order].HU = *(buffer++)-'0';
//			}
//			else
//			{
//				buffer--;
//				Schedule_Data[Sched_Order].HT = *(buffer++)-'0';
//				Schedule_Data[Sched_Order].HU = *(buffer++)-'0';
//			}
//			buffer+=2;
//			if((*buffer==0x27)||(*buffer==0x22))//0x27 == ' ,0x22 == "
//			{
//				buffer--;
//				Schedule_Data[Sched_Order].MNT = 0;
//				Schedule_Data[Sched_Order].MNU = *(buffer++)-'0';
//			}
//			else
//			{
//				buffer--;
//				Schedule_Data[Sched_Order].MNT = *(buffer++)-'0';
//				Schedule_Data[Sched_Order].MNU = *(buffer++)-'0';
//			}
//			
//			Schedule_Data[Sched_Order].Day=i;//i
//	
//			if(Sched_Order<10)//limit schedule to 7 sets//14
//			{
//				Sched_Order++;
//			}
//			else 
//			{
//				return 1;
//			}
//		}
//		i++;
//	}
//	Beep(2);
//	return 0;
//}*/
/*Add Schedule By remote*/
void Remote_Add_Schedule(uint16_t Sched)
{
	uint8_t i=0,j=0,sch_flag=0;
	uint16_t Temp_T,Temp_M;
	Sort_Schedule_Data();
	Temp_T = Sched/60;
	Temp_M = Sched%60;

	for(j=0;j<7;j++)
	{
		for(i=0;i<Sched_Order;i++)//search for day match
		{
			if(Schedule_Data[i].Day==j)
			{
				if(Check_String_Match((char*)ESP8266_Remote_SchedName[j],Schedule_Data[i].name))
				{
					Schedule_Data[i].OF=1;
					Schedule_Data[i].Day=j;
					Schedule_Data[i].HT=Temp_T/10;
					Schedule_Data[i].HU=Temp_T%10;
					Schedule_Data[i].MNT=Temp_M/10;
					Schedule_Data[i].MNU=Temp_M%10;
					sch_flag=1;
				}
			}
		}
		if(!sch_flag)
		{
			for(i=0;i<16;i++)
			{
				Schedule_Data[Sched_Order].name[i]=ESP8266_Remote_SchedName[j][i];
			}
			Schedule_Data[Sched_Order].OF=1;
			Schedule_Data[Sched_Order].Day=j;
			Schedule_Data[Sched_Order].HT=Temp_T/10;
			Schedule_Data[Sched_Order].HU=Temp_T%10;
			Schedule_Data[Sched_Order].MNT=Temp_M/10;
			Schedule_Data[Sched_Order].MNU=Temp_M%10;
			
			if(Sched_Order<14)Sched_Order++;
		}
		sch_flag=0;
	}
}
/*Delete Schedule By remote*/
void Remote_Del_Schedule(void)
{
	uint8_t i=0,j=0;
	for(i=0;i<Sched_Order;i++)
	{
		for(j=0;j<20;j++){
			Schedule_Data[i].name[j]=0;
		}
		Schedule_Data[i].OF=0;
	}
	Sched_Order=0;
}

/* Mod Schedule */
uint8_t ESP8266_Mod_Schedule(char *data)//pending
{
	uint8_t Order=0,i=0,Name_Flag=0;
	char String_Result;
  char *buffer=data;
	buffer = String_P(buffer,(char *)"name\n",&String_Result);//td='
	if(String_Result)return 1;
	for(Order=0;Order<Sched_Order;Order++)//search for schedule name
	{
		if(Check_String_Match(buffer,Schedule_Data[Order].name))
		{
			Name_Flag=1;
			break;
		}
	}
	if(Name_Flag==0)
	{
		ESP8266_Answer_OF(0);//answer fail
		return 1;
	}
	buffer = String_P(buffer,(char *)"on\n",&String_Result);//on='
	if(String_Result)return 1;
	Schedule_Data[Order].OF = *(buffer++)-'0';
	buffer = String_P(buffer,(char *)"time\n",&String_Result);//time='
	if(String_Result)return 1;
	buffer++;
	if(*buffer==':')
	{
		buffer--;
		Schedule_Data[Order].HT = 0;
		Schedule_Data[Order].HU = *(buffer++)-'0';
	}
	else
	{
		buffer--;
		Schedule_Data[Order].HT = *(buffer++)-'0';
		Schedule_Data[Order].HU = *(buffer++)-'0';
	}
	buffer+=2;
	if((*buffer==0x27)||(*buffer==0x22))//0x27 == ' ,0x22 == "
	{
		buffer--;
		Schedule_Data[Order].MNT = 0;
		Schedule_Data[Order].MNU = *(buffer++)-'0';
	}
	else
	{
		buffer--;
		Schedule_Data[Order].MNT = *(buffer++)-'0';
		Schedule_Data[Order].MNU = *(buffer++)-'0';
	}
	buffer = String_P(buffer,(char *)"repeat\n",&String_Result);//repeat='
	if(String_Result)return 1;
	for(i=0;i<7;i++)
	{
		if(*buffer=='1')
		{
			Schedule_Data[Order].Day=i;
			break;
		}
		buffer++;
	}
	return 0;
	//ESP8266_Answer_OF(1);//answer ok
}
/* Delete Schedule */
uint8_t ESP8266_Del_Schedule(char *data)
{
	uint8_t Order=0,Name_Flag=0,i=0;
  char *buffer=data;
	char String_Result;
	buffer = String_P(buffer,(char *)"name\n",&String_Result);//name='
	if(String_Result)return 1;
	for(Order=0;Order<Sched_Order;Order++)//search for schedule name
	{
		if(Check_String_Match(buffer,Schedule_Data[Order].name))
		{
			Name_Flag=1;
			break;
		}
	}
	if(Name_Flag==0)
	{
		//ESP8266_Answer_OF(0);//answer fail
		return 1;
	}
	for(i=0;i<20;i++)
	{
		Schedule_Data[Order].name[i]=0;
	}
	Schedule_Data[Order].Day=0;
	Schedule_Data[Order].HT=0;
	Schedule_Data[Order].HU=0;
	Schedule_Data[Order].MNT=0;
	Schedule_Data[Order].MNU=0;
	Schedule_Data[Order].OF=0;
	for(i=Order;i<(Sched_Order-1);i++)//shift all schedules to left
	{
		Schedule_Data[i]=Schedule_Data[i+1];
	}
	if(Sched_Order>0)Sched_Order--;//deleted a schedule
	return 0;
}
/* report schedule */
void ESP8266_Report_Schedule(uint8_t FAB)
{
	uint8_t Sched_Count=0;
	uint8_t test=0;
  if(FAB==ESP8266_FA)
	{
		Add_UserID_String((char *)ESP8266_FA_Sched,ESP8266_ID);
		for(Sched_Count=0;Sched_Count<Sched_Order;Sched_Count++)//Sched_Order
		{
			Add_Sched_String((char *)WString_Schedule);//add WString_Schedule into Sched_String_Buffer 
			Add_Sched_Content(Schedule_Data[Sched_Count]);//add schedule content into Sched_String_Buffer
			Add_String(Send_String_Buffer,(char *)Sched_String_Buffer,(char *)"Sched2\n",Send_String_Buffer);//add Sched_String_Buffer into send_string_buffer
		}
		if(Sched_Order<1)
		{
			
			//USART3_Print((char *)"\n No Schedule in Ram!\n");
		}
	}
	else
	{
		Add_String((char *)ESP8266_FB_Sched,(char *)WString_Sched2,(char *)"td\n",Send_String_Buffer);
		
		for(Sched_Count=0;Sched_Count<Sched_Order;Sched_Count++)
		{
			Add_Sched_String((char *)WString_Schedule);//add WString_Schedule into Sched_String_Buffer 
			Add_Sched_Content(Schedule_Data[Sched_Count]);//add schedule content into Sched_String_Buffer
			Add_String(Send_String_Buffer,(char *)Sched_String_Buffer,(char *)"Sched2\n",Send_String_Buffer);//add Sched_String_Buffer into send_string_buffer
			test++;
		}
		if(Sched_Order<1)
		{
			//USART3_Print((char *)"\n No Schedule in Ram!\n");
		}
	}
	Send_ESP8266_Setting(Send_String_Buffer);
//	//USART3_Print(Send_String_Buffer);
}
/*sort schedule data from low to high*/
void Sort_Schedule_Data(void)
{
	uint16_t Total_Time1=0,Total_Time2=0;
	ESP8266_Sched Temp;
	uint8_t i=0,j=0;
	for(i=0;i<(Sched_Order-1);i++)
	{
		for(j=0;j<(Sched_Order-1);j++)
		{
			if(Schedule_Data[j].Day>Schedule_Data[j+1].Day)
			{
				Temp=Schedule_Data[j];
				Schedule_Data[j]=Schedule_Data[j+1];
				Schedule_Data[j+1]=Temp;
			}
			else if(Schedule_Data[j].Day==Schedule_Data[j+1].Day)
			{
				Total_Time1 = (Schedule_Data[j].HT*10+Schedule_Data[j].HU)*60 + Schedule_Data[j].MNT*10+Schedule_Data[j].MNU;
				Total_Time2 = (Schedule_Data[j+1].HT*10+Schedule_Data[j+1].HU)*60 + Schedule_Data[j+1].MNT*10+Schedule_Data[j+1].MNU;
				if(Total_Time1>Total_Time2)
				{
					Temp=Schedule_Data[j];
					Schedule_Data[j]=Schedule_Data[j+1];
					Schedule_Data[j+1]=Temp;
				}
			}
		}
	}
}
/* add schedule content : schedule name , on , hour , minute , repeat */
void Add_Sched_Content(ESP8266_Sched data)
{
	char temp[8]={0},i=0;
//	char *buffer;
	
	Add_String(Sched_String_Buffer,(char *)data.name,(char *)" n\n",Sched_String_Buffer);//add sched name
	temp[0]=data.OF+'0';
	temp[1]='\n';
	Add_String(Sched_String_Buffer,temp,(char *)" o\n",Sched_String_Buffer);//add sched on/off
	temp[0]=data.HT+'0';
	temp[1]=data.HU+'0';
	temp[2]=':';
	temp[3]=data.MNT+'0';
	temp[4]=data.MNU+'0';
	temp[5]='\n';
	Add_String(Sched_String_Buffer,temp,(char *)" t\n",Sched_String_Buffer);//add sched time
	for(i=0;i<7;i++)
	{
		temp[i]='0';
	}
	temp[data.Day]='1';
	temp[i]='\n';
	Add_String(Sched_String_Buffer,temp,(char *)" r\n",Sched_String_Buffer);//add sched day
}
/* add WString Schedule to Sched_String_Buffer[],
	 WString_Schedule is const value ,but this need to modify the time content and add to Send_String_Buffer[]*/
void Add_Sched_String(char *data)
{
	uint16_t length=0,i=0;
	length = Get_String_Length(data);
	if(length>118)length=118;
	for(i=0;i<(length+1);i++)
	{
		Sched_String_Buffer[i]=*data;
		data++;
	}
	for(i=i;i<120;i++)//clear the rest
	{
		Sched_String_Buffer[i]=0;
	}
}
/* store sched name to Schedule_Data[]*/
void Store_SchedName(uint8_t Order,char *schdata)
{
	uint8_t i=0;
	for(i=0;i<20;i++)//clear arry
	{
		Schedule_Data[Order].name[i]=0;
	}
	for(i=0;i<20;i++)
	{
		Schedule_Data[Order].name[i]=*(schdata++);
		if(*schdata==0x27)break;//'
    if(*schdata==0x22)break;//"
	}
	i++;
	Schedule_Data[Order].name[i]='\n';
}
/*Get Rec id*/
void Get_Rec_ID(char *data)
{
	uint8_t i=0;
	char result=1;
	char *id_buffer=data;
	
	ESP8266_Rec_ID[15]=0;
	
	id_buffer = String_P(data,(char *)" id\n",&result);//got ret =
	if(result)return;
	ESP8266_Rec_ID[0]='i';
	ESP8266_Rec_ID[1]='d';
	ESP8266_Rec_ID[2]='=';
	ESP8266_Rec_ID[3]=0x27;//'
	for(i=4;i<14;i++)//12
	{
		if((*id_buffer==0x27)||(*id_buffer==0x22))//0x27 == ' ,0x22 == "
		{
			ESP8266_Rec_ID[i]=0x27;
			i++;
			ESP8266_Rec_ID[i]=' ';
			i++;
			ESP8266_Rec_ID[i]='\n';
			ESP8266_Rec_ID[15]=1;
			return;
		}
		ESP8266_Rec_ID[i]=*id_buffer;
		id_buffer++;
	}
}
/* Answer ok or fail after received command from wifi module*/
void ESP8266_Answer_OF(uint8_t S)// OK FAIL
{
	Add_UserID_String((char *)ESP8266_FA_Answer,ESP8266_ID);
	if(ESP8266_Rec_ID[15])
	{
		Add_String(Send_String_Buffer,(char *)ESP8266_Rec_ID,(char *)"<ct\n",Send_String_Buffer);
	}
	if(S)
	{
		Add_String(Send_String_Buffer,(char *)WString_Ok,(char *)"ret\n",Send_String_Buffer);
		//USART3_Print((char *)"\nESP8266_Answer Ok!\n");
	}
	else
	{
		Add_String(Send_String_Buffer,(char *)WString_Fail,(char *)"ret\n",Send_String_Buffer);
		//USART3_Print((char *)"\nESP8266_Answer Fail!\n");
	}
	Send_ESP8266_Setting(Send_String_Buffer);
}
/*Special Answer command for cen55*/
void ESP8266_CEN55_Answer(uint8_t S)
{
//	Add_UserID_String((char *)ESP8266_FA_CEN55Answer,ESP8266_ID);
	Add_UserID_String((char *)ESP8266_FA_CEN55Answer,ESP8266_ID);
	//------------------------------   

	//----------------------------------
	if(ESP8266_Rec_ID[15])//////////////////////////
	{
//		Beep(5);
//    Beep(5);Beep(5);Beep(5);
		Add_String(Send_String_Buffer,(char *)ESP8266_Rec_ID,(char *)"<ct\n",Send_String_Buffer);
//		Add_String(Send_String_Buffer,(char *)99,(char *)"<ct\n",Send_String_Buffer);
	}
	if(S)
	{
		Add_String(Send_String_Buffer,(char *)WString_Ok,(char *)"ret\n",Send_String_Buffer);
		//USART3_Print((char *)"\nESP8266_Answer Ok!\n");
	}
	else
	{
		Add_String(Send_String_Buffer,(char *)WString_Fail,(char *)"ret\n",Send_String_Buffer);
		//USART3_Print((char *)"\nESP8266_Answer Fail!\n");
	}
	Send_ESP8266_Setting(Send_String_Buffer);
}
/* Add CRC calculated value into *data and send *data through USART1*/
void Send_ESP8266_Setting(char *data)
{
//	DT kk;
	uint32_t Length=0;
	char *buffer;
	buffer = data;
	while(1)
	{
		String_Buffer[Length]=*data;
		if(*data=='\n')break;
		data++;
		Length++;
		if(Length>1699)return;
	}
	data=buffer;
	buffer+=3;
	String_Buffer[Length-1]=calcBufCrc8(buffer,Length-4);
	USART_DMA_String(Length+1,String_Buffer);
//	USART3_Print("\nRespond : \n");
//	delay(20);
//	kk = Get_DateTime();
//	temp_tt[0]='T';
//	temp_tt[1]='0'+kk.MNT;
//	temp_tt[2]='0'+kk.MNU;
//	temp_tt[3]=':';
//	temp_tt[4]='0'+kk.ST;
//	temp_tt[5]='0'+kk.SU;
//	temp_tt[6]=' ';
//	temp_tt[7]=' ';
//	USART3_Transmit_String(8,temp_tt);
//	delay(10);
//	USART3_DMA_String(Length+1,String_Buffer);
}
/*Get USER ID from wifi module and store at cahr ESP8266_ID[]*/
void ESP8266_Get_ID(char *input)
{
//	uint8_t j=0;
	uint8_t i=0;
	for(i=0;i<100;i++)
	{
		ESP8266_ID[i]=0;
	}
	for(i=0;i<100;i++)
	{
		if(*input==0)break;
		input++;
	}
	input++;
	for(i=0;i<100;i++)
	{
		ESP8266_ID[i]=*input;
		input++;
		if(*input==0)break;
	}
	i++;
	ESP8266_ID[i]='\n';
}
/*Add USER ID to Send_String_Buffer[] from ESP8266_ID[]*/
void Add_UserID_String(char *Orig,char *UD)//
{
	uint32_t Orig_Length=0,UD_Length=0;
	uint32_t i=0,P=0,i_L=0;
	for(i=0;i<1700;i++)//clear string buffer
	{
		String_Buffer[i]=0;
	}
	Orig_Length = Get_String_Length(Orig)+1;
	UD_Length = Get_String_Length(UD);
	
	if(Orig_Length>999)Orig_Length=999;
	if(UD_Length>100)UD_Length=100;
	
	for(i=0;i<Orig_Length;i++)//get fa
	{
		String_Buffer[P]=*Orig;
		if(*Orig=='A')
		{
			Orig++;
			P++;
			break;
		}
		Orig++;
		P++;
		if(P>999)break;
	}
	i_L = P;
	for(i=0;i<UD_Length;i++)//get UID
	{
		String_Buffer[P]=*UD;
		UD++;
		P++;
		if(P>999)break;
	}
	for(i=i_L;i<Orig_Length+1;i++)//get rest of the string
	{
		String_Buffer[P]=*Orig;
		Orig++;
		P++;
		if(P>999)break;
	}
	for(i=0;i<999;i++)
	{
		Send_String_Buffer[i]=String_Buffer[i];
	}
}
/*add string from *st into *orig at *name ,and store to *output*/
void Add_String(char *Orig,char *St,char *name,char *output)//orig = FA/FB/FC Full string ,St = command ,name = postion to add st 
{
	uint32_t i=0,P=0,i_L=0;
	uint32_t Orig_Length=0,St_Length=0,name_length=0;
  for(i=0;i<1700;i++)
	{
		String_Buffer[i]=0;
	}
	Orig_Length = Get_String_Length(Orig)+1;
	St_Length = Get_String_Length(St);
	name_length	= Get_String_Length(name);
	if(Orig_Length>1700)Orig_Length=1700;
	if(St_Length>500)St_Length=500;
	if(name_length>100)name_length=100;
	
	for(i=0;i<Orig_Length;i++)//add first segment
	{
		String_Buffer[P]=*Orig;
		if(*Orig==*name)
		{
			if(Check_String_Match(Orig,name))break;
		}
		P++;
		Orig++;
		if(P>1699)break;
	}
	for(i=0;i<(name_length+2);i++)//add name
	{
		String_Buffer[P]=*Orig;
		P++;
		Orig++;
		if(P>1699)break;
	}
	i_L = P;
	for(i=0;i<St_Length;i++)//add segment
	{
		String_Buffer[P]=*St;
		P++;
		St++;
		if(P>1699)break;
	}
	for(i=i_L;i<Orig_Length;i++)
	{
		String_Buffer[P]=*Orig;
		Orig++;
		P++;
		if(P>1699)break;
	}
	if(P>1699)P=1699;
	for(i=0;i<P;i++)
	{
		*output=String_Buffer[i];
		output++;
	}
}

/* Charge Status function*/
void Set_Charge_State(WIFI_Rec S)
{
	Charge_Status=S;
}
WIFI_Rec Get_Charge_State(void)
{
	return Charge_Status;
}
/* Clean Status function*/
void Set_Clean_State(uint8_t S)
{
	Clean_Status=S;
}
uint8_t Get_Clean_State(void)
{
	return Clean_Status;
}
/* wifi feed dog function*/
uint8_t Need_Feed_Dog(void)
{
	return Feed_Dog_Flag;
}
void Set_Feed_Dog(void)
{
	Feed_Dog_Flag=1;
}
void Clear_Feed_Dog(void)
{
	Feed_Dog_Flag=0;
}
/*wifi led function*/
void Set_WIFI_Status(uint8_t S)
{
	ESP8266_WIFI_S=S;
}
uint8_t Get_WIFI_Status(void)
{
	return ESP8266_WIFI_S;
}
/*Get Scheduled Time data from  Schedule_Data[]*/
ESP8266_Sched Get_Scheduled_Data(uint8_t d)
{
	if(d>6)d=6;
	return Schedule_Data[d];
}
/*print scheduled time from Schedule_Data[] for debug*/
void Print_Scheduled_Time(void)
{
	#ifdef PRINT_RTC
	uint8_t i=0;
//	char temp[10];
	for(i=0;i<Sched_Order;i++)
	{
		USART3_Transmit_String(22,(char *)"\nSchedule Time: name =");
		//USART3_Print(Schedule_Data[i].name);
		USART3_Transmit_String(4,(char *)" on=");
		USART3_Transmit_Byte(Schedule_Data[i].OF+'0');
		USART3_Transmit_String(6,(char *)" time=");
		USART3_Transmit_Byte(Schedule_Data[i].HT+'0');
		USART3_Transmit_Byte(Schedule_Data[i].HU+'0');
		USART3_Transmit_String(1,(char *)":");
		USART3_Transmit_Byte(Schedule_Data[i].MNT+'0');
		USART3_Transmit_Byte(Schedule_Data[i].MNU+'0');
		USART3_Transmit_String(8,(char *)" repeat=");
		USART3_Transmit_Byte(Schedule_Data[i].Day+'0');
		USART3_Transmit_String(2,(char *)" \n");
		delay(100);
	}
	delay(500);
	#endif
}
/*Check and set Schedule to alarm */
void Check_and_SetAlarm(void)
{
	DT temp_DT;
	uint32_t Temp_Time_Count=0,Temp_Alarm_Count=0;
	uint8_t i=0,Next_Day=0,Get_Alarm_Flag=0;
	
	
	temp_DT = Get_DateTime();
	
	for(i=0;i<Sched_Order;i++)
	{
		if(temp_DT.WK<=Schedule_Data[i].Day)
		{
			if(Schedule_Data[i].OF==1)
			{
				Next_Day=i;
				Get_Alarm_Flag=1;
				break;
			}
		}
	}
	if(!Get_Alarm_Flag)
	{
		Disable_Alarm();
		return;
	}
	
	if(temp_DT.WK==Schedule_Data[Next_Day].Day)//same day
	{
		Temp_Time_Count = temp_DT.HT*600+temp_DT.HU*60+temp_DT.MNT*10+temp_DT.MNU;//covert time to minute
		Temp_Alarm_Count = Schedule_Data[Next_Day].HT*600+Schedule_Data[Next_Day].HU*60+Schedule_Data[Next_Day].MNT*10+Schedule_Data[Next_Day].MNU;//covert time to minute
		if(Temp_Alarm_Count<=Temp_Time_Count)//set
		{
			Next_Day++;
			if(Next_Day>=Sched_Order)Next_Day=0;
		}
	}
	Set_ESP8266_RTCAlarm(Schedule_Data[Next_Day]);
}
/*Get Log*/
void ESP8266_Report_Log(void)
{
	uint8_t i=0;
	char Temp_Time_String[17]={0};//2014-6-24 11:11 
  if(Log_Count<1)Log_Count=1;	
	for(i=7;i>0;i--)
	{
		if(Log_1[i].Evt!=0)
		{
			Temp_Time_String[0]=2+'0';
			Temp_Time_String[1]='0';
			Temp_Time_String[2]=Log_1[i].Time.YT+'0';
			Temp_Time_String[3]=Log_1[i].Time.YU+'0';
			Temp_Time_String[4]='-';
			Temp_Time_String[5]=Log_1[i].Time.MT+'0';
			Temp_Time_String[6]=Log_1[i].Time.MU+'0';
			Temp_Time_String[7]='-';
			Temp_Time_String[8]=Log_1[i].Time.DT+'0';
			Temp_Time_String[9]=Log_1[i].Time.DU+'0';
			Temp_Time_String[10]=' ';
			Temp_Time_String[11]=Log_1[i].Time.HT+'0';
			Temp_Time_String[12]=Log_1[i].Time.HU+'0';
			Temp_Time_String[13]=':';
			Temp_Time_String[14]=Log_1[i].Time.MNT+'0';
			Temp_Time_String[15]=Log_1[i].Time.MNU+'0';
			Temp_Time_String[16]='\n';

			Add_UserID_String((char *)ESP8266_FA_GetLog,ESP8266_ID);
			Add_String(Send_String_Buffer,(char *)Temp_Time_String,(char *)"time\n",Send_String_Buffer);
			Temp_Time_String[0]=Log_1[i].Evt+'0';
			Temp_Time_String[1]='\n';
			Add_String(Send_String_Buffer,(char *)Temp_Time_String,(char *)"evt\n",Send_String_Buffer);
			Send_ESP8266_Setting(Send_String_Buffer);
			delay(200);
		}
	}
	//USART3_Print((char *)"\nFA Report Log\n");
}
void Log_Robot_Status(uint8_t St)
{
	uint8_t i=0;
	for(i=0;i<7;i++)
	{
		if(i<7)Log_1[i]=Log_1[i+1];
	}

	Log_1[7].Time = Get_DateTime();
	
	switch(St)
	{
		case Error_Code_PickUp:Log_1[7].Evt = Log_Robot_Hang;break;
		case Error_Code_Cliff:Log_1[7].Evt = Log_Robot_Cliff;break;
		case Error_Code_LeftWheel:Log_1[7].Evt = Log_Robot_Wheel;break;
		case Error_Code_Stuck:Log_1[7].Evt = Log_Robot_Stuck;break;
		case Log_Robot_StartClean:Log_1[7].Evt = Log_Robot_StartClean;break;
		case Log_Robot_EndClean:Log_1[7].Evt = Log_Robot_EndClean;break;
		default:Log_1[7].Evt = 0;break;
	}
}

void Reset_Robot_Log(void)
{
	uint8_t i=0;
	for(i=0;i<8;i++)
	{
		Log_1[i].Evt=0;
	}
	Log_Order=0;
}

Log_Type Get_Log_Content(uint8_t O)
{
	if(O>9)O=9;
	return Log_1[O];
}
/*report version*/
void ESP8266_Report_Version(void)
{
	char v[5]={0};
	uint32_t Temp_V=Test_Version;
	v[0]=Temp_V/1000+'0';
	
	Temp_V=Temp_V%1000;
	v[1]=Temp_V/100+'0';
	
	Temp_V=Temp_V%100;
	v[2]=Temp_V/10+'0';
	
	Temp_V=Temp_V%10;
	v[3]=Temp_V+'0';
	v[4]='\n';
	
	Add_UserID_String((char *)ESP8266_FA_Version,ESP8266_ID);
	Add_String(Send_String_Buffer,(char *)v,(char *)"FW\n",Send_String_Buffer);
	Send_ESP8266_Setting(Send_String_Buffer);
//	//USART3_Print(Send_String_Buffer);
	//USART3_Print((char *)"\nFA Report Version!\n");
}
/* life span */
void ESP8266_Report_LifeSpan(Life_Span_Type Type)
{
	char l[4];
	uint32_t Temp_Life;

	Add_UserID_String((char *)ESP8266_FA_LifeSpan,ESP8266_ID);
	
	switch(Type)
	{
		case Life_SideBrush:
		
			Add_String(Send_String_Buffer,(char *)WString_SideBrush,(char *)"type\n",Send_String_Buffer);
		  Temp_Life = Get_LifeSpan(Life_SideBrush);
			if(Temp_Life>87600)
			{
				Temp_Life=0;
				Clear_LifeSpan(W_SetSideBrushLife);
			}
		  Temp_Life = (87600-Temp_Life)*100/87600;
			if(Temp_Life>100)Temp_Life=100;
			l[0]=Temp_Life/100+'0';
			l[1]=((Temp_Life%100)/10)+'0';
			l[2]=(Temp_Life%10)+'0';
			l[3]='\n';
			Add_String(Send_String_Buffer,(char *)l,(char *)"val\n",Send_String_Buffer);
			Send_ESP8266_Setting(Send_String_Buffer);
//			//USART3_Print(Send_String_Buffer);
			//USART3_Print((char *)"\nFA Report Life_SideBrush!\n");
		  break;
		
		case Life_Brush:
			Add_String(Send_String_Buffer,(char *)WString_Brush,(char *)"type\n",Send_String_Buffer);
			Temp_Life = Get_LifeSpan(Life_Brush);
			if(Temp_Life>87600)
			{
				Temp_Life=0;
				Clear_LifeSpan(W_SetBrushLife);
			}
		  Temp_Life = (87600-Temp_Life)*100/87600;
			if(Temp_Life>100)Temp_Life=100;
			l[0]=Temp_Life/100+'0';
			l[1]=((Temp_Life%100)/10)+'0';
			l[2]=(Temp_Life%10)+'0';
			l[3]='\n';
			Add_String(Send_String_Buffer,(char *)l,(char *)"val\n",Send_String_Buffer);
			Send_ESP8266_Setting(Send_String_Buffer);
//			//USART3_Print(Send_String_Buffer);
			//USART3_Print((char *)"\nFA Report Life_Brush!\n");
			break;
		
		case Life_DustCaseHeap:
		
			Add_String(Send_String_Buffer,(char *)WString_DustCaseHeap,(char *)"type\n",Send_String_Buffer);
			Temp_Life = Get_LifeSpan(Life_DustCaseHeap);
		  if(Temp_Life>87600)
			{
				Temp_Life=0;
				Clear_LifeSpan(W_SetDustCaseHeapLife);
			}
		  Temp_Life = (87600-Temp_Life)*100/87600;
			if(Temp_Life>100)Temp_Life=100;
			l[0]=Temp_Life/100+'0';
			l[1]=((Temp_Life%100)/10)+'0';
			l[2]=(Temp_Life%10)+'0';
			l[3]='\n';
			Add_String(Send_String_Buffer,(char *)l,(char *)"val\n",Send_String_Buffer);
			Send_ESP8266_Setting(Send_String_Buffer);
			////USART3_Print(Send_String_Buffer);
			//USART3_Print((char *)"\nFA Report Life_DustCaseHeap!\n");
		  break;
		default:break;
	}
}
void ESP8266_Reset_LifeSpan(WIFI_Rec T)
{
	Clear_LifeSpan(T);
	ESP8266_Answer_OF(ANSWER_OK);
	
}
void Clear_LifeSpan(WIFI_Rec T)//address:254k
{
	uint32_t Life_Time,Life_SideBrush,Life_Brush,Life_DustHepa;
	Life_Time = Read_Flash(0x0803f800);
	Life_SideBrush = Read_Flash(0x0803f804);
	Life_Brush = Read_Flash(0x0803f808);
	Life_DustHepa = Read_Flash(0x0803f80c);

	if(T==W_SetSideBrushLife){
		Life_SideBrush=0;
		//USART3_Print((char *)"\nFA Reset Side brush Life!\n");
	}
	if(T==W_SetBrushLife){
		Life_Brush=0;
		//USART3_Print((char *)"\nFA Reset brush Life!\n");
	}
	if(T==W_SetDustCaseHeapLife){
		Life_DustHepa=0;
		//USART3_Print((char *)"\nFA Reset DustHepa Life!\n");
	}
	Unlock_Flash();
	Flash_Erase_Page(0x0803f800);
  Write_Flash_Word(0x0803f800,Life_Time);
	Write_Flash_Word(0x0803f804,Life_SideBrush);
	Write_Flash_Word(0x0803f808,Life_Brush);
	Write_Flash_Word(0x0803f80c,Life_DustHepa);
	Lock_Flash();
}
uint32_t Get_LifeSpan(Life_Span_Type t)
{
	switch(t)
	{
		case Life_DustCaseHeap:return Read_Flash(0x0803f80c);
		case Life_Brush:return Read_Flash(0x0803f808);
		case Life_SideBrush:return Read_Flash(0x0803f804);
    default:return 0;
	}
}
/*test wifi module*/
/*void ESP8266_Send_TestCommand(void)
{
	Test_Module_Flag=0;
	Add_String((char *)"`FC<ctl id='ranomdid' td='SetWIFICfg'><cfg s='' p='123456789'/></ctl>",(char *)"ROBOTWIFI\n",(char *)" s\n",Send_String_Buffer);
//	Add_String((char *)"`FC<ctl id='ranomdid' td='SetWIFICfg'><cfg s='' p='ilife2016'/></ctl>",(char *)"ilife3\n",(char *)" s\n",Send_String_Buffer);
	Send_ESP8266_Setting(Send_String_Buffer);
	//USART3_Print((char *)"\nSend FC SetWIFICfg !\n");
}*/
void Test_ESP8266_Module(void)
{
//	uint32_t i=0;
	uint32_t t=0;
//	Test_Module_Flag=0;
	Answer_Flag=0;
	test_wifi_flag=0;
	Set_LED(100,100,0);//Set_LED_Status(10,MCU_RED_LED|MCU_GREEN_LED);//
		/*for(i=0;i<3;i++)
	{
	ESP8266_Send_TestCommand();
		t=0;
		while(Answer_Flag==0)
		{
			t++;
			if(t>1000)break;
			ESP8266_Check_Data();
			delay(10);
		}
		if(Answer_Flag)break;
	}*/
	
	while(1)
	{
		t++;
		if(t>3000)
		{
			Set_LED(0,100,0);//Set_LED_Status(10,MCU_RED_LED);
			Beep(5);
			Beep(5);
			Beep(5);
			Beep(5);
			Beep(5);
			break;
		}
		if(Test_Module_Flag)
		{
			Set_WIFI_Status(ESP8266_WIFIStatus_Connected);
			Set_LED(100,0,0);//Set_LED_Status(10,MCU_GREEN_LED);
			Beep(1);
			break;
		}
		ESP8266_Check_Data();
		delay(100);
	}
	while(1)
	{
		if(Get_Key_Press()==KEY_CLEAN)////Get_Key_Press(Switch_Clean)
		{
			delay(1000);
			while(Get_Key_Press()==KEY_CLEAN);////Get_Key_Press(Switch_Clean)
			delay(1000);
			break;
		}
	}
}
/*user id*/

/*WIFI ID*/

void Set_WIFI_ID(char *data)//address:252k
{
	uint32_t buffer[8]={0};
	uint8_t i=0;
	for(i=0;i<8;i++)
	{
			buffer[i]|=*data;
			buffer[i]<<=8;
			data++;
			
			buffer[i]|=*data;
			buffer[i]<<=8;
			data++;
			
			buffer[i]|=*data;
			buffer[i]<<=8;
			data++;
			
			buffer[i]|=*data;
			data++;
	}
	Unlock_Flash();
	Flash_Erase_Page(0x0803f000); 
  Write_Flash_Word(0x0803f000,buffer[0]);
	Write_Flash_Word(0x0803f004,buffer[1]);
	Write_Flash_Word(0x0803f008,buffer[2]);
	Write_Flash_Word(0x0803f00c,buffer[3]);
	Write_Flash_Word(0x0803f010,buffer[4]);
	Write_Flash_Word(0x0803f014,buffer[5]);
	Write_Flash_Word(0x0803f018,buffer[6]);
	Write_Flash_Word(0x0803f01c,buffer[7]);
	Lock_Flash();
}

void Get_WIFI_ID(char *data)
{
	uint8_t i=0;
	uint32_t buffer[8]={0};
	uint32_t temp=0;
	char d_3=0,d_2=0,d_1=0,d_0=0;
	buffer[0]=Read_Flash(0x0803f000);
	buffer[1]=Read_Flash(0x0803f004);
	buffer[2]=Read_Flash(0x0803f008);
	buffer[3]=Read_Flash(0x0803f00c);
	buffer[4]=Read_Flash(0x0803f010);
	buffer[5]=Read_Flash(0x0803f014);
	buffer[6]=Read_Flash(0x0803f018);
	buffer[7]=Read_Flash(0x0803f01c);
	for(i=0;i<8;i++)
	{
		temp = buffer[i];
		d_0 = temp&0x000000ff;
		temp>>=8;
		d_1 = temp&0x000000ff;
		temp>>=8;
		d_2 = temp&0x000000ff;
		temp>>=8;
		d_3 = temp&0x000000ff;
		*data=d_3;
		data++;
		*data=d_2;
		data++;
		*data=d_1;
		data++;
		*data=d_0;
		data++;
	}
}

void Receiving_WIFI_ID(void)
{
	Receive_Barcode=1;
	Barcode_Receive_End=0;
	TIM3->CCER =0;//关闭OBS、Cliff
	Set_LED(100,100,0);//Set_LED_Status(10,MCU_RED_LED|MCU_GREEN_LED);//
	while(1)
	{
		if(Barcode_Receive_End)
		{
			Barcode_Receive_End=0;
			//Receive_Barcode=0;
			if(Barcode_Content[0])
			{
				if(calcBufCrc8((char *)Barcode_Content,32)==Barcode_Content[32])
				{
					if(Barcode_Content[0]=='E')
					{
						Beep(2);
	//					USART3_DMA_String(12,(char *)"\nReceived: \n");
	//					USART3_DMA_String(33,(char *)Barcode_Content);
	//					USART3_DMA_String(12,(char *)"\nFC: \n");
	//					delay(10);
	//	    		USART3_Print("\n\rCRC R\n");
	//				  USART3_DMA_Numbers(Barcode_Content[32]);
	//				  USART3_Print("\n\rCRC B\n");
	//				  USART3_DMA_Numbers(calcBufCrc8((char *)Barcode_Content,32));
						Set_WIFI_ID((char *)Barcode_Content);
						Set_LED(100,0,0);//Set_LED_Status(10,MCU_GREEN_LED);//
						ESP8266_Initialize_Module();
					}
				}
			}
		}
	}
}

#endif/*esp8266*/


