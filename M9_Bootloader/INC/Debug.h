#ifndef __DEBUG_H__
#define __DEBUG_H__

//#include "config.h"
//#include "Map.h"
#include "USART.h"
#include "config.h"

extern char outString[];

extern int usprintf(char *out, const char *format, ...);

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
#endif

void UpgradeNavData_simple(int32_t x, int32_t y, int32_t angle, uint8_t type,char *NavDataTemp);
void PC_NavDebug(int32_t x, int32_t y, int32_t angle, uint8_t type);
void Next_Target_Debug(int32_t x, int32_t y);

/*			
Debug_Print("@Test Start!",Dev_USART0);			
USPRINTF("#BaseLineFail:%d\n",Temp_Base_Line);	
*/					
#endif
			
			
			
