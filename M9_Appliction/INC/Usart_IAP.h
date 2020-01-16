/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   UserInterface Fuction
	           Display Button lights and waiting for user to select cleaning mode
						 Plan setting , set hours and minutes
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */ 
#ifndef __USART_IAP_H
#define __USART_IAP_H

#include "SysInitialize.h"


#define DOWN_LOAD_ADDRESS       (uint32_t)0x0801D000 
#define APPLICATION_ADDRESS     (uint32_t)0x08004000  //定义新程序所占flash空间的起始地址
#define Bootloader_ADDRESS     	(uint32_t)0x08000000  //定义bootloader所占flash空间的起始地址
#define Upgrade_Flag_Address		(uint32_t)0x0803FC00	//定义升级标志位处理的地址



typedef enum
{
	AC_OTA_START           = 0x21,
	AC_OTA_TRAN            = 0x22,	
	AC_OTA_CHECK           = 0x23,	
	AC_OTA_FINISH          = 0x24,	
	
	AC_SYS_REGISTER        = 0x02,
	AC_SYS_REGISTER_ACK    = 0x07,
	AC_SYS_SMART_LINK      = 0x08,
	AC_SYS_AP_LINK         = 0x0C,
	AC_SYS_WIFI_SLEEP      = 0x09,
	AC_SYS_WIFI_UNBUNDLED	 = 0x24,
	AC_SYS_WIFI_WAKEUP		 = 0x0A,
	AC_SYS_WIFI_REBOT      = 0x0B,
	AC_SYS_WIFI_TEST       = 0x01,
//AC_SYS_WIFI_CONNECT		 = 0x02,
	AC_SYS_WIFI_DISCONNECT = 0x03,
	AC_SYS_AC_CONNECT      = 0x04,
	AC_SYS_AC_DISCONNECT   = 0x05,
	AC_SYS_NTP             = 0x0D,
	AC_SYS_WIFI_VERSION    = 0xfe,
	AC_SYS_WIFI_MAC        = 0xff,
	
	AC_CHECK_STATUS        = 0x41,
	AC_CHECK_SCHEDULE      = 0x42,
	AC_CHECK_HISTORY       = 0x43,
	AC_CHECK_LIFE          = 0x44,
	AC_CHECK_FIRMWARE      = 0x45,

	AC_CMD_WORK_MODE       = 0x46,
	AC_CMD_CARPET_MODE     = 0x47,
	AC_CMD_CLEAN_LEVEL     = 0x48,
	AC_CMD_DIR_MODE        = 0x49,
	AC_CMD_SCHEDULE        = 0x4A,
	AC_CMD_LIFE_REST       = 0x4B,
	AC_CMD_TIME            = 0x4C,
	AC_CMD_UPLOAD          = 0x4D,
	AC_CMD_QUIET           = 0x4E,
	AC_CMD_FACTORY_REST    = 0x4F,
	AC_CMD_WATERBUMP_LEVEL = 0x50,	
	AC_CMD_FINDING_ROBOT   = 0x51,
	
	AC_UPLOAD_STATUS       = 0xC8, //200
	AC_UPLOAD_HISTORY      = 0xC9, //201
	AC_UPLOAD_MAP          = 0xCA, //202
	AC_UPLOAD_WORKTIME     = 0xCB, //203
	AC_UPLOAD_FACTORY_REST = 0xCC, //204
	AC_UPLOAD_CLEAR_MAP    = 0xCD, //205
	AC_UPLOAD_APPMAP_CLEAR = 0xA1, //206
	
	AC_GET_FIRMWARE_INFO   = 0xFB,
	AC_GET_FIRMWARE_PACK   = 0xFC,	
}AC_MsgCode;

typedef struct
{
	uint16_t length;
	uint8_t packnum;
	AC_MsgCode msgcode;
	volatile uint8_t *rxbuf;
}Ac_RxPackStruct_t;

typedef struct 
{
	uint8_t packnum;
	AC_MsgCode msgcode;
	uint8_t checksum;
	uint16_t data_length;
	uint8_t databuf[TxPACKBUF_MAX + 1];
}AC_TxPackStruct_t;


extern uint8_t Rec_Data[RXBUF_MAX];//接收的数据
extern volatile uint8_t Rec_Data_Flag;//数据接收状态：0-等待接收 1-正在接收 2-接收并校验通过 3-接收失败
extern volatile uint16_t Package_Num;


typedef  void (*pFunction)(void);
void BootLoader_Jump(uint32_t addr);
void IAP_Set(uint32_t addr);

uint16_t Get_Upgrade_Flag(void);
void Set_Upgrade_Flag(uint16_t flag);

void FLASH_EraseALL(unsigned long datasize);
uint16_t FLASH_ReadU16(uint32_t addr);
void Flash_WriteU16(uint32_t addr, uint16_t data);

void Clear_Rec_Flags(void);

void Check_for_IAP(void);
	
void Flash_WriteU32(uint32_t addr, uint32_t data);
uint32_t FLASH_ReadU32(uint32_t addr);


uint8_t Calc_PackageSum(char *data_buf,uint16_t length);	
void AC_DecodeRx_Buf(Ac_RxPackStruct_t *rx_pack,volatile uint8_t *rec_data);
void AC_Decode_Command(volatile uint8_t *Order_Data, uint16_t Length);
void AC_SendData2Cloud(void);
void Save_Rec_Data(uint8_t data);
void WriteData(uint16_t pack_num,volatile uint8_t *rec_data);

#endif

