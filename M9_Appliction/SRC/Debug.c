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

char outString[256];

//更新精简导航数据
void UpgradeNavData_simple(int32_t x, int32_t y, int32_t angle, uint8_t type,char *NavDataTemp)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	
	//帧头
	NavDataTemp[0] = 0xfa;
	NavDataTemp[1] = 0xaf;
	//功能字
	NavDataTemp[2] = 0x03;
	//数据长度
	NavDataTemp[3] = 14;
	//X
	NavDataTemp[4] = ((x & 0xff000000) >> 24);
	NavDataTemp[5] = ((x & 0xff0000) >> 16);
	NavDataTemp[6] = ((x & 0xff00) >> 8);
	NavDataTemp[7] = x & 0xff;
	//Y
	NavDataTemp[8] = ((y & 0xff000000) >> 24);
	NavDataTemp[9] = ((y & 0xff0000) >> 16);
	NavDataTemp[10] = ((y & 0xff00) >> 8);
	NavDataTemp[11] = y & 0xff;
	//角度
	NavDataTemp[12] = ((angle & 0xff000000) >> 24);
	NavDataTemp[13] = ((angle & 0xff0000) >> 16);
	NavDataTemp[14] = ((angle & 0xff00) >> 8);
	NavDataTemp[15] = angle & 0xff;
	//类型
	NavDataTemp[16] = type;
	
	for(i = 4;i < 17;i++)
	{
		sum += NavDataTemp[i];
	}
	
	NavDataTemp[17] = sum & 0xff;
	//NavDataTemp[18] = '\n';
}
void mUpgradeNavData_simple(int32_t x, int32_t y, int32_t angle, uint8_t type,char *NavDataTemp)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	
	//帧头
	NavDataTemp[0] = 0xfa;
	NavDataTemp[1] = 0xaf;
	//功能字
	NavDataTemp[2] = 0x04;
	//数据长度
	NavDataTemp[3] = 14;
	//X
	NavDataTemp[4] = ((x & 0xff000000) >> 24);
	NavDataTemp[5] = ((x & 0xff0000) >> 16);
	NavDataTemp[6] = ((x & 0xff00) >> 8);
	NavDataTemp[7] = x & 0xff;
	//Y
	NavDataTemp[8] = ((y & 0xff000000) >> 24);
	NavDataTemp[9] = ((y & 0xff0000) >> 16);
	NavDataTemp[10] = ((y & 0xff00) >> 8);
	NavDataTemp[11] = y & 0xff;
	//角度
	NavDataTemp[12] = ((angle & 0xff000000) >> 24);
	NavDataTemp[13] = ((angle & 0xff0000) >> 16);
	NavDataTemp[14] = ((angle & 0xff00) >> 8);
	NavDataTemp[15] = angle & 0xff;
	//类型
	NavDataTemp[16] = type;
	
	for(i = 4;i < 17;i++)
	{
		sum += NavDataTemp[i];
	}
	
	NavDataTemp[17] = sum & 0xff;
	//NavDataTemp[18] = '\n';
}
//发送参数数据
//void PC_InfoDebug(void)
//{
//	uint8_t i = 0;
//	UpgradeInfoData();
//	for (i = 0; i < 95; i++)
//	{
//		USART3->TDR = InfoDataTemp[i];
//		while(!(USART3->ISR&USART_ISR_TXE));
//	}
//}

//发送导航数据
void PC_NavDebug(int32_t x, int32_t y, int32_t angle, uint8_t type)
{
	#ifdef ENABLE_DEBUG
	char data_temp[18]={0};
	UpgradeNavData_simple(x, y, angle, type,data_temp);
	USART_SendNavData(USART3_PrintStringByLength,data_temp,18);
	#endif
}
 
void mPC_NavDebug(int32_t x, int32_t y, int32_t angle, uint8_t type)
{
	#ifdef DEBUG_MAP
	char data_temp[18]={0};
	UpgradeNavData_simple(x, y, angle, type,data_temp);
	USART_SendNavData(USART3_PrintStringByLength,data_temp,18);
	#endif
}

void PC_NavDebug_Line(Line_t L,uint8_t type)
{
	#ifdef ENABLE_DEBUG
	int16_t i=0;
  CleanMode_t mode_buffer = Mode_GetMode();
	for(i = L.x_start; i <= (L.x_start+L.x_offset); i++)
	{
		PC_NavDebug(i,L.y,0,type);
		if(Mode_GetMode() !=  mode_buffer)return;
		//vTaskDelay(100/portTICK_RATE_MS);
	}
	#endif
}
void PC_NavDebug_AllMap(void)
{
	#ifdef ENABLE_DEBUG_TARGET
	int16_t mx,my;
	CleanMode_t mode_buffer = Mode_GetMode();
	Usprintf("%s(%d)\n", __FUNCTION__, __LINE__);
	for(mx = (Map_GetXMin() - 1);mx <= Map_GetXMax();mx++)
	{
		for(my = (Map_GetYMin() - 1);my <= Map_GetYMax();my++)
		{
			PC_NavDebug(mx,my,0,Map_GetCell(mx,my));
			if(Mode_GetMode() !=  mode_buffer)return;
		}
	}
	#endif
}

void USART_SendNavData(void (*usartsendfun)(char *,uint8_t),char *data,uint8_t lenght)
{
	(* usartsendfun)(data,lenght);
}

//////////////////////////////////////////////////////////////////////////////////
//在Map.c中的MAP_Set_Cell函数中添加
//PC_NavDebug(x, y, 0, value);



