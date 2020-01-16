#ifndef __DEBUG_H__
#define __DEBUG_H__

//#include "SysInitialize.h"
//#include "ShortestPath.h"
#include "stdint.h"
#include "config.h"

extern char outString[];

extern int usprintf(char *out, const char *format, ...);

#ifdef MY_D
#define printf2(...)	{						\
				usprintf(outString, ##__VA_ARGS__);	\
				ups3(outString);			\
				ups3("\r");				\
			}
#else
#define printf2(fmt, ...)	{do {;} while (0);}				
#endif

#if ENABLE_DEBUG

#define USPRINTF(...)	{						\
				usprintf(outString, ##__VA_ARGS__);	\
				ups3(outString);			\
				ups3("\r");				\
			}
#define printf(...)	{						\
				usprintf(outString, ##__VA_ARGS__);	\
				ups3(outString);			\
				ups3("\r");				\
			}
#define Usprintf(...)	{						\
				usprintf(outString, ##__VA_ARGS__);	\
				ups3(outString);			\
				ups3("\r");				\
			}
#else
#define USPRINTF(fmt, ...)	{do {;} while (0);}
#define printf(fmt, ...)	{do {;} while (0);}	
#define Usprintf(fmt, ...)	{do {;} while (0);}			
#endif

void UpgradeNavData_simple(int32_t x, int32_t y, int32_t angle, uint8_t type,char *NavDataTemp);
void mUpgradeNavData_simple(int32_t x, int32_t y, int32_t angle, uint8_t type,char *NavDataTemp);
void PC_NavDebug(int32_t x, int32_t y, int32_t angle, uint8_t type);
void mPC_NavDebug(int32_t x, int32_t y, int32_t angle, uint8_t type);
//void PC_NavDebug_Line(Line_t L,uint8_t type);
void USART_SendNavData(void (*usartsendfun)(char *,uint8_t),char *data,uint8_t lenght);
void PC_NavDebug_AllMap(void);
				
#endif
			
			
			
