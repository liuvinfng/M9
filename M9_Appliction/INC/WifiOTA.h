#ifndef __WIFIOTA_H__
#define __WIFIOTA_H__

#include "SysInitialize.h"

/*
#ifdef UPDATE_FLASH
BL:  0~BFFF      0~47.99K
1K               48~48.99K
APP1:C400~45BFF  49~278.99K
1K               279~279.99K
APP2:46000~7F7FF 280~509.99K 
1K               510~510.99K
FLAG:7FC00~7FFFF 511~511.99K
*/

#define CODE_SIZE              230//k
#define CODE_SIZE_BIT          235520//字节
#define SIZE_WORD              4//word
#define APPLICATION_OFFSET     0xC000//48k处
#define BOOTLOADER_OFFSET      0

#define W25Q16_VOICE_ADDRESS   0
#define W25Q16_CODE_ADDRESS    1572864//1536K/1.5M处
#define W25Q16_FLAG_ADDRESS	   2096128//2047K/2M-1K处

#define APPLICATION_ADDRESS    (uint32_t)0x0800C000 //48k处
#define FLASH_CODE_ADDRESS     (uint32_t)0x08046000 //280k处
#define FLASH_FLAG_ADDRESS		 (uint32_t)0x0807FC00	//511K处
#define BOOTLOADER_ADDRESS     (uint32_t)0x08000000 //定义bootloader


typedef  void (*pFunction)(void);


uint8_t Get_Upgrade_Flag(void);
void Set_Upgrade_Flag(uint8_t flag);

void Check_for_IAP(void);

uint16_t FLASH_ReadU16(uint32_t addr);
void Flash_WriteU16(uint32_t addr, uint16_t data);

void Flash_WriteU32(uint32_t addr, uint32_t data);
uint32_t FLASH_ReadU32(uint32_t addr);

void BootLoader_Jump(uint32_t addr);

void Flash_Erase(unsigned long datasize);
void Flash_Write(const unsigned char value[],u32 WriteAddr,u16 NumByteToWrite);

void Set_Wifi_Enable(uint8_t flag);
uint8_t Get_Wifi_Enable(void);

#endif







