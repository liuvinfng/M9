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
#include "gd32f30x_spi.h"
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


u8 SPI1_ReadWriteByte(u8 txData)
{
	u16 retry=0;				 	
	while((SPI_STAT(SPI1) & SPI_I2S_FLAG_TXE) == RESET)//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>1000)return 0;
	}			  
	SPI_DATA(SPI1) = txData;//ͨ������SPIx����һ������
	retry=0;

	while((SPI_STAT(SPI1) & SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>1000)return 0;
	}	  						    
	return (SPI_DATA(SPI1));//����ͨ��SPIx������յ�����	
}

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 txData)
{		
	u16 retry=0;				 	
	while((SPI_STAT(SPI2) & SPI_I2S_FLAG_TXE) == RESET)//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>1000)return 0;
	}			  
	SPI_DATA(SPI2) = txData;//ͨ������SPIx����һ������
	retry=0;

	while((SPI_STAT(SPI2) & SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>1000)return 0;
	}	  						    
	return (SPI_DATA(SPI2));//����ͨ��SPIx������յ�����					    
}



