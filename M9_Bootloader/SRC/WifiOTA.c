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
#include "WifiOTA.h"
#include "W25Q16.h"
#include "String.h"


///////////////////////////////1616161616616161616166161616////////////////////////////////
//往flash中指定地址写16位数据
void Flash_WriteU16(uint32_t addr, uint16_t data)
{
	fmc_unlock();
	fmc_halfword_program(addr, data);
	fmc_lock();
}
//读取addr地址中的16位数据
uint16_t FLASH_ReadU16(uint32_t addr)
{
	return *(uint16_t *)addr;
}

//读取addr的连续num个16位数据
void FLASH_ReadBuffer(uint32_t addr, uint16_t *data, uint16_t num)
{
	uint16_t i;
	for(i=0; i<num; i++)
	{
		*(data+i) = FLASH_ReadU16(addr + (i*2));
	}
}
///////////////////////////////3232323232323232323323232323////////////////////////////////
//往flash中指定地址写16位数据
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

void Flash_Erase(unsigned long datasize)//输入单位是字节
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

//获取FLASH中的升级状态标志位
uint8_t Get_Upgrade_Flag(void)
{
	#ifdef UPDATE_FLASH
	return FLASH_ReadU16(FLASH_FLAG_ADDRESS);
	#else
	return W25QXX_Read_One(W25Q16_FLAG_ADDRESS);
	#endif
}
//设置FLASH中的升级状态标志位为升级模式
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
void Check_for_IAP(void)
{
	uint8_t i;
	uint32_t addr=0,buf=0,lednum=0;
	USPRINTF("Upgrade_Flag:%x\n",Get_Upgrade_Flag());
	g_copy_led=0;
	if(Get_Upgrade_Flag() == 0x55)
	{		
		AUDIO_DISABLE;
		g_copy_led=1;
		Set_LED_On_Blink(0,0,0,0,0,0);
		Set_LED_On_Switch(0,0,0,0,0,0);
		//从48K开始擦除flash CODE_SIZE KB
		fmc_unlock();
		for(i=0; i<CODE_SIZE; i++)
		{ 
			fmc_page_erase(APPLICATION_ADDRESS+(i*1024));
		}
		fmc_lock();							

		for(addr=0;addr<(CODE_SIZE*1024);addr+=4)//copy flash CODE_SIZE KB
		{
			#ifdef UPDATE_FLASH
			buf=FLASH_ReadU32(FLASH_CODE_ADDRESS+addr);
			#else
			uint8_t tempbuf[4]={0,0,0,0};
			W25QXX_Read(tempbuf,W25Q16_CODE_ADDRESS+addr,4); 
			buf=((tempbuf[3]<<24) + (tempbuf[2]<<16) + (tempbuf[1]<<8) + tempbuf[0]);
			#endif			
			Flash_WriteU32(APPLICATION_ADDRESS+addr,buf);		
		}
		USPRINTF("BootLoader_Jump_0x55\n");
		delay(100);
		BootLoader_Jump(APPLICATION_ADDRESS);
		while(1)
		{
			//跳转失败
			USPRINTF("BootLoader_Jump_0x55_falie\n");
			Set_Upgrade_Flag(0xAA);//设置为升级模式
			delay(100);				
			BootLoader_Jump(BOOTLOADER_ADDRESS);
		}		
	}
	else if(Get_Upgrade_Flag() == 0xAA)
	{
		Set_LED_On_Blink(1,1,1,1,1,0);
		Set_LED_On_Switch(0,0,0,0,0,0);		
		Speaker(SPK_WIFI_CONNECT_START);	
		delay(1000);
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
			if(Get_Wifi_Enable())
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
	else//不是升级状态，跳转至用户命令执行
	{
		USPRINTF("BootLoader_Jump_app\n");
		delay(1000);
		BootLoader_Jump(APPLICATION_ADDRESS);
		while(1)
		{
			//跳转失败
			USPRINTF("BootLoader_Jump_app_falie\n");
			Set_Upgrade_Flag(0xAA);//设置为升级模式
			delay(1000);		
			BootLoader_Jump(BOOTLOADER_ADDRESS);
		}
	}
}	

//跳转程序到指定的地址运行

uint32_t JumpAddress = 0;
pFunction Jump_To_Application = 0;

void BootLoader_Jump(uint32_t addr)
{
	if(((*(__IO uint32_t*)(addr+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
	{
		if (((*(__IO uint32_t*)addr) & 0x2FFE0000 ) == 0x20000000)//判断Flash数据是否合法
		{					
			__disable_irq() ;
			__disable_fault_irq();
			nvic_vector_table_set(NVIC_VECTTAB_FLASH, addr-NVIC_VECTTAB_FLASH); //地址映射
			/* Jump to user application */
			JumpAddress = *(__IO uint32_t*) (addr + 4);
			Jump_To_Application = (pFunction) JumpAddress;		
			/* Initialize user application's Stack Pointer */
			__set_MSP(*(__IO uint32_t*) addr);		
			/* Jump to application */
			Jump_To_Application();
		}	
	}

}
//void BootLoader_Jump(uint32_t addr)
//{
//	uint32_t base;
//	uint32_t offset;
//	
//	uint16_t Buf[4];
//	uint32_t msp;
//	uint32_t reset;	
//	
//	AUDIO_DISABLE;
//	/*直接从FLASH中读取*/
//	FLASH_ReadBuffer(addr, Buf, 4);
//	msp = Buf[0] + (Buf[1]<<16);	//新程序的堆栈栈顶指针
//	reset = Buf[2] + (Buf[3]<<16);	//获得复位向量
//	
//	
//	
//	base = (addr > NVIC_VectTab_FLASH) ? NVIC_VectTab_FLASH : NVIC_VectTab_RAM;
//	offset = addr - base;
//	
//	nvic_vector_table_set(base, offset);	//设置中断向量表

//	__set_MSP(msp);				//设置主堆栈指针
//	
//	__disable_irq();			//关闭所有中断
//	
//	( (void (*)())(reset))();	//跳到指定flash地址
//}

uint8_t g_wifi_enable=0;
void Set_Wifi_Enable(uint8_t flag)
{
	g_wifi_enable = flag;
}
uint8_t Get_Wifi_Enable(void)
{
	return g_wifi_enable;
}


