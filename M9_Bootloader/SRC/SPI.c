/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Initialize Usart function and prosessing the characters transmitting
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "SPI.h"
#include "gd32f30x_spi.h"
#include "gd32f30x.h"

 
#define SPI1_MAX_RETRY			1000
bool gSPI1_TimerOutErrFlag = false;
u32 gSPI1_RetryTimeCnt = 0;


u8 SPI1_ReadWriteByte(u8 txData)
{
	gSPI1_RetryTimeCnt = 0;
	gSPI1_TimerOutErrFlag = false; 
	while((SPI_STAT(SPI1) & SPI_I2S_FLAG_TXE) == RESET)
	{
		if(gSPI1_RetryTimeCnt++>SPI1_MAX_RETRY)	 
		{
			gSPI1_TimerOutErrFlag = true;
			return 0;
		}
	}
	SPI_DATA(SPI1) = txData;
	gSPI1_RetryTimeCnt= 0;
	while((SPI_STAT(SPI1) & SPI_I2S_FLAG_RXNE) == RESET)
	{
		if(gSPI1_RetryTimeCnt++>SPI1_MAX_RETRY)	 
		{
			gSPI1_TimerOutErrFlag = true;
			return 0;
		}
	}
	return (SPI_DATA(SPI1));	
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 txData)
{		
	u8 retry=0;				 	
	while((SPI_STAT(SPI2) & SPI_I2S_FLAG_TXE) == RESET)//检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_DATA(SPI2) = txData;//通过外设SPIx发送一个数据
	retry=0;

	while((SPI_STAT(SPI2) & SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return (SPI_DATA(SPI2));//返回通过SPIx最近接收的数据					    
}



