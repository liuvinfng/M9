#include "Debug.h"
#include "Zone.h"
#include "PathPlanning.h"
#include "SysInitialize.h"

char outString[256];

#ifdef DEBUG_PC_MAP
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
}

void PC_NavDebug(int32_t x, int32_t y, int32_t angle, uint8_t type)
{
	char i=0,data_temp[18]={0};

	UpgradeNavData_simple(x, y, angle, type,data_temp);
	for(i=0;i<18;i++)
	{
		Debug_Write_Byte(data_temp[i]);
	}
}
#endif







