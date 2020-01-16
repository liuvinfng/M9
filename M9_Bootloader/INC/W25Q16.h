#ifndef __W25Q16_H__
#define __W25Q16_H__

#include "SysInitialize.h"




//W25Xϵ��/Qϵ��оƬ�б�	   
#define W25Q80 	0XC813 	
#define W25Q16 	0XC814
#define W25Q32 	0XC815
#define W25Q64 	0XC816
#define W25Q128	0XC817


//ָ���
#define W25X_WriteStatusReg		0x01 
#define W25X_PageProgram			0x02
#define W25X_ReadData			  	0x03 
#define W25X_WriteDisable			0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteEnable			0x06
#define W25X_FastReadData			0x0B 
#define W25X_FastReadDual			0x3B 
#define W25X_BlockErase				0xD8 
#define W25X_SectorErase			0x20 
#define W25X_ChipErase				0xC7 
#define W25X_PowerDown				0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			    0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F
  


#define W25Q16_CS_ON			(GPIO_BC(GPIOA)  |= MCU_SPI2_CS)
#define W25Q16_CS_OFF		  (GPIO_BOP(GPIOA) |= MCU_SPI2_CS)




u8 W25QXX_ReadSR(void);  
void W25QXX_Write_SR(u8 sr);

void W25QXX_Write_Enable(void);
void W25QXX_Write_Disable(void);

u16 W25QXX_ReadID(void);


void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u32 NumByteToWrite);
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //��ȡflash
uint8_t W25QXX_Read_One(u32 ReadAddr);
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//д��flash
void W25QXX_Erase_Chip(void);    	  	//��Ƭ����
void W25QXX_Erase_Sector(u32 Dst_Addr);	//��������
void W25QXX_Wait_Busy(void);           	//�ȴ�����
void W25QXX_PowerDown(void);        	//�������ģʽ
void W25QXX_WAKEUP(void);				//����











#endif






