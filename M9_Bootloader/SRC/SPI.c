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

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 txData)
{		
	u8 retry=0;				 	
	while((SPI_STAT(SPI2) & SPI_I2S_FLAG_TXE) == RESET)//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_DATA(SPI2) = txData;//ͨ������SPIx����һ������
	retry=0;

	while((SPI_STAT(SPI2) & SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return (SPI_DATA(SPI2));//����ͨ��SPIx������յ�����					    
}



