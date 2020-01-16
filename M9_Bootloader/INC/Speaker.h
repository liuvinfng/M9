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

#ifndef __SPEAKER_H
#define __SPEAKER_H

#include "SysInitialize.h"
#include "Movement.h"


#define addr_1      0x00000
#define addr_2      0x14ec0
#define addr_3      0x281c0
#define addr_4      0x3aaf0   
#define addr_5      0x497c0
#define addr_6      0x53970
#define addr_7      0x5d640
#define addr_8      0x67310
#define addr_9      0x70e30
#define addr_10     0x7a7b0
#define addr_11     0x83910
#define addr_12     0x8c8d0
#define addr_13     0x956f0
#define addr_14     0x9d660
#define addr_15     0xa5280
#define addr_16     0xac9c0
#define addr_17     0xb3a70
#define addr_18     0xba980
#define addr_19     0xc1890
#define addr_20     0xc87a0
#define addr_21     0xcf6b0
#define addr_22     0xd6420
#define addr_23     0xdce50
#define addr_24     0xe31f0
#define addr_25     0xe90b0
#define addr_26     0xeedd0
#define addr_27     0xf3f80
#define addr_28     0xf8df0
#define addr_29     0xfd910



#define num_1        85682
#define num_2        78577
#define num_3        76069
#define num_4        60605
#define num_5        41378
#define num_6        40125
#define num_7        40125
#define num_8        39707
#define num_9        39289
#define num_10       37199
#define num_11       36781
#define num_12       36363
#define num_13       32601
#define num_14       31765
#define num_15       30512
#define num_16       28840
#define num_17       28422
#define num_18       28422
#define num_19       28422
#define num_20       28422
#define num_21       28004
#define num_22       27168
#define num_23       25496
#define num_24       24242
#define num_25       23824
#define num_26       20898
#define num_27       20063
#define num_28       19227
#define num_29       17555



typedef enum {
	SPK_SYS_MUSIC							= 1,	
	SPK_CHECK_BUMPER					= 2,
	SPK_CHECK_CLIFF						= 3,
	SPK_LOW_POWER_WILL_OFF		= 4,
	SPK_ROBOT_TO_START_POINT	= 5,
	SPK_CHECK_DIRTY_TANK			= 6,	
	SPK_WIFI_CONNECT_FAIL			= 7,
	SPK_AREA_CLEAN_START			= 8,	
	SPK_CLEAN_START			  		= 9,
	SPK_LOW_POWER_PLE_CHARG		= 10,
	SPK_PLE_START_FLOOR		    = 11,	
	SPK_MOP_MODE_START				= 12,
	SPK_CHECK_MAIN_BRUSH			= 13,	
	SPK_WALLFOLLOW_MODE_START	= 14,	
	SPK_CHECK_SIDE_BRUSH			= 15,
	SPK_CHECK_LEFT_WHEEL			= 16,
	SPK_ENTER_RECHARGE_MODE		= 17,	
	SPK_CHECK_RIGHT_WHEEL			= 18,	
	SPK_EXIT_RECHARGE_MODE		= 19,
	SPK_CHECK_FAN			        = 20,	
	SPK_CLEAN_FINISH		  		= 21,
	SPK_CLEAN_STOP			  		= 22,	
	SPK_WIFI_CONNECTED				= 23,	
	SPK_CHARGING_START				= 24,
	SPK_WIFI_CONNECT_START		= 25,	
	SPK_GO_ON									= 26,	
	SPK_KEY_INVALID 					= 27,	
	SPK_DON										= 28,
	SPK_SUSPEND								= 29,
}SPK_t;


void Speaker(uint8_t data);
void Beep(uint8_t Sound);
//void SPK_Report_Error(Error_t error_val);
void Audio(void);  






#endif /* __Speaker_H */



