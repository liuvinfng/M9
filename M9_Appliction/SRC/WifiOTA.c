/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Wifi peripherals
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
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
#include "wifiOTA.h"
#include "string.h"


///////////////////////////////1616161616616161616166161616////////////////////////////////
//��flash��ָ����ַд16λ����
void Flash_WriteU16(uint32_t addr, uint16_t data)
{
	fmc_unlock();
	fmc_halfword_program(addr, data);
	fmc_lock();
}
//��ȡaddr��ַ�е�16λ����
uint16_t FLASH_ReadU16(uint32_t addr)
{
	return *(uint16_t *)addr;
}

///////////////////////////////3232323232323232323323232323////////////////////////////////
//��flash��ָ����ַд16λ����
void Flash_WriteU32(uint32_t addr, uint32_t data)
{
	fmc_unlock();
	fmc_word_program(addr, data);
	fmc_lock();
}

uint32_t FLASH_ReadU32(uint32_t addr)
{
	return *(uint32_t *)addr;
}

void Flash_Erase(unsigned long datasize)//���뵥λ���ֽ�
{
	#ifdef UPDATE_FLASH
	uint8_t i;
	uint8_t page = datasize/1024 + 1;
	if(page>CODE_SIZE)page=CODE_SIZE;
	fmc_unlock();
	for(i=0; i<page; i++)
	{
		fmc_page_erase(FLASH_CODE_ADDRESS+(i*1024));
	}
	fmc_lock();
	#endif
}
void Flash_Write(const unsigned char value[],u32 WriteAddr,u16 NumByteToWrite)
{
	uint8_t i=0;
	uint16_t t = 0;
	uint32_t data_temp = 0,addr = WriteAddr;
	fmc_unlock();
	for(i=0; i<(NumByteToWrite/SIZE_WORD); i++)
	{
		data_temp = value[t] + (value[1 + t] << 8) + (value[2 + t] << 16) + (value[3 + t] << 24);
		fmc_word_program(addr, data_temp);
		addr += SIZE_WORD; 
		t += SIZE_WORD;
	}
	fmc_lock();
}

//��ȡFLASH�е�����״̬��־λ
uint8_t Get_Upgrade_Flag(void)
{
	#ifdef UPDATE_FLASH
	return FLASH_ReadU16(FLASH_FLAG_ADDRESS);
	#else
	uint8_t temp[1]={0};
	W25QXX_Read(temp,W25Q16_FLAG_ADDRESS,1);
	return temp[0];	
	#endif
}
//����FLASH�е�����״̬��־λΪ����ģʽ
void Set_Upgrade_Flag(uint8_t flag)
{
	#ifdef UPDATE_FLASH
	fmc_unlock();
	fmc_page_erase(FLASH_FLAG_ADDRESS);
	Flash_WriteU16(FLASH_FLAG_ADDRESS, flag);
	fmc_lock();
	#else
	uint8_t temp_flag[1];
	temp_flag[0]=flag;
	W25QXX_Write(temp_flag,W25Q16_FLAG_ADDRESS,1);
	#endif
}

//updat
/*void Check_for_IAP(void)
{
	uint8_t i,tempbuf[4]={0,0,0,0};
	uint32_t addr=0,buf=0,lednum=0;
	USPRINTF("Upgrade_Flag:%d\n",Get_Upgrade_Flag());
	if(Get_Upgrade_Flag() == 0x55)
	{		
		//��48K��ʼ����flash250KB
		fmc_unlock();
		for(i=0; i<250; i++)
		{
			fmc_page_erase(APPLICATION_ADDRESS+(i*1024));
		}
		fmc_lock();							

		for(addr=0;addr<(250*1024);addr+=4)//copy flash 250KB
		{
			W25QXX_Read(tempbuf,W25Q16_CODE_ADDRESS+addr,4); 
			buf=((tempbuf[0]<<24) + (tempbuf[1]<<16) + (tempbuf[2]<<8) + tempbuf[3]);
			Flash_WriteU32(APPLICATION_ADDRESS+addr,buf);		
		}
		USPRINTF("BootLoader_Jump111\n");
		delay(5000);
		BootLoader_Jump(APPLICATION_ADDRESS);
		while(1)
		{
			//��תʧ��
			USPRINTF("BootLoader_Jump111_falie\n");
			Set_Upgrade_Flag(0xAA);//����Ϊ����ģʽ
			BootLoader_Jump(BOOTLOADER_ADDRESS);
		}		
	}
	else if(Get_Upgrade_Flag() == 0xAA)
	{
		Set_LED_On_Blink(1,1,1,1,1,1);
		Set_LED_On_Switch(0,0,0,0,0,0);		
    Speaker(SPK_WIFI_CONNECT_START);	
		delay(10000);
		while(1)
		{
			lednum++;
			if(lednum>100000)lednum=0;
			if(Is_ChargerOn())
			{
				Charge_SetSwitch(ENABLE);
				Set_LED_On_Blink(0,0,0,0,0,0);
				Set_LED_On_Switch(0,0,0,1,1,0);	
				Set_LED_Table(0,0,0,lednum/1000,lednum/1000,0);
				Charge_Process();
			}
			else				
			{
				Charge_SetSwitch(DISABLE);
				Set_LED_On_Blink(1,1,1,1,1,1);
				Set_LED_On_Switch(0,0,0,0,0,0);					
			}			
			#ifdef WIFI_TY
			if(Is_Wifi_Enable())
			{
				wifi_uart_service();
			}	
			if(Get_Key_Time(KEY_HOME)>100)
			{
				Speaker(SPK_WIFI_CONNECT_START);	
				mcu_set_wifi_mode(SMART_CONFIG);	
			}
			#endif
		}
	}
	else//��������״̬����ת���û�����ִ��
	{
		USPRINTF("BootLoader_Jump222\n");
		BootLoader_Jump(APPLICATION_ADDRESS);
		while(1)
		{
			//��תʧ��
			USPRINTF("BootLoader_Jump222_falie\n");
			Set_Upgrade_Flag(0xaa);//����Ϊ����ģʽ
			BootLoader_Jump(BOOTLOADER_ADDRESS);
		}
	}
}	

//��ת����ָ���ĵ�ַ����
pFunction Jump_To_Application;
uint32_t JumpAddress;
void BootLoader_Jump(uint32_t addr)
{
	if (((*(__IO uint32_t*)addr) & 0x2FFE0000 ) == 0x20000000)
	{ 
		// Jump to user application //
		JumpAddress = *(__IO uint32_t*) (addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		
		// Initialize user application's Stack Pointer //
		__set_MSP(*(__IO uint32_t*) addr);
		
		// Jump to application //
		Jump_To_Application();
	}
}
*/

uint8_t g_wifi_enable=0;
void Set_Wifi_Enable(uint8_t flag)
{
	g_wifi_enable = flag;
}
uint8_t Get_Wifi_Enable(void)
{
	return g_wifi_enable;
}







