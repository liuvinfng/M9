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
#include "stdarg.h"
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

char Usart_Data[10]={0,48,48,48,48,48,48,48,48,48};


/*-------------------------------------USART Config------------------------------*/
void USART0_Configuration(void)//120M_DEBUG
{ 
  USART_BAUD(USART0) = USART_120M_115200;
	USART_CTL2(USART0) |= USART_CR3_DMAT|USART_CR3_EIE ;
  USART_CTL0(USART0) |= USART_CR1_UE|USART_CR1_TE;
//	USART_CTL0(USART0) |= USART_CR1_RE|USART_CR1_RXNEIE;
}

void USART1_Configuration(void)//60M_WIFI
{ 
	#ifdef M_30
  USART_BAUD(USART1) = USART_30M_115200;
	#else
  USART_BAUD(USART1) = USART_60M_115200;
	#endif		
	USART_CTL2(USART1) |= USART_CR3_DMAT|USART_CR3_EIE;
	USART_CTL0(USART1) |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
}
void USART2_Configuration(void)//60M_gyro
{ 
	#ifdef M_30
  USART_BAUD(USART2) = USART_30M_115200;
	#else
  USART_BAUD(USART2) = USART_60M_115200;
	#endif
	USART_CTL2(USART2) |= USART_CR3_DMAT|USART_CR3_EIE;
	USART_CTL0(USART2) |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
}

/*-------------------------------------USART DMA------------------------------*/
void USART0_DMA_String(uint16_t Length,char *data)//DMA_CH3
{
//	#ifdef USART_PRINT_ENABLE
	while(DMA_CHCNT(DMA0, DMA_CH3));
	DMA_CHCTL(DMA0, DMA_CH3) &=~ DMA_CCR1_EN;
	DMA_CHPADDR(DMA0, DMA_CH3) = (uint32_t)&(USART_DATA(USART0));
	DMA_CHMADDR(DMA0, DMA_CH3) = (uint32_t)data;
	DMA_CHCNT(DMA0, DMA_CH3) = Length;
	DMA_CHCTL(DMA0, DMA_CH3) |= DMA_CCR1_MINC|DMA_CCR1_DIR;
	DMA_CHCTL(DMA0, DMA_CH3) |= DMA_CCR1_EN|DMA_CCR1_TCIE;
//	#endif
}
void USART1_DMA_String(uint16_t Length,char *data)//DMA_CH6
{
	while(DMA_CHCNT(DMA0, DMA_CH6));
	DMA_CHCTL(DMA0, DMA_CH6) &=~ DMA_CCR1_EN;
	DMA_CHPADDR(DMA0, DMA_CH6) = (uint32_t)&(USART_DATA(USART1));
	DMA_CHMADDR(DMA0, DMA_CH6) = (uint32_t)data;
	DMA_CHCNT(DMA0, DMA_CH6) = Length;
	DMA_CHCTL(DMA0, DMA_CH6) |= DMA_CCR1_MINC|DMA_CCR1_DIR;
	DMA_CHCTL(DMA0, DMA_CH6) |= DMA_CCR1_EN|DMA_CCR1_TCIE;
} 
void USART2_DMA_String(uint16_t Length,char *data)//DMA_CH1
{	
	while(DMA_CHCNT(DMA0, DMA_CH1));
	DMA_CHCTL(DMA0, DMA_CH1) &=~ DMA_CCR1_EN;
	DMA_CHPADDR(DMA0, DMA_CH1) = (uint32_t)&(USART_DATA(USART2));
	DMA_CHMADDR(DMA0, DMA_CH1) = (uint32_t)data;
	DMA_CHCNT(DMA0, DMA_CH1) = Length;
	DMA_CHCTL(DMA0, DMA_CH1) |= DMA_CCR1_MINC|DMA_CCR1_DIR;
	DMA_CHCTL(DMA0, DMA_CH1) |= DMA_CCR1_EN;
}

/*------------------------------------USART Transmit String------------------*/
void USART0_Transmit_String(uint16_t length,char *Data)
{
  while(length)
  {
    length--;
    USART0_Transmit_Byte(*Data);
    Data++;
  }
}
void USART1_Transmit_String(uint16_t length,char *Data)
{
  while(length)
  {
    length--;
    USART1_Transmit_Byte(*Data);
    Data++;
  }
}
void USART2_Transmit_String(uint16_t length,char *Data)
{
  while(length)
  {
    length--;
    USART2_Transmit_Byte(*Data);
    Data++;
  }
}
/*------------------------------------USART Transmit Byte------------------*/
void USART0_Transmit_Byte(char Data)
{
	#ifdef USART_PRINT_ENABLE	
  USART_DATA(USART0) = Data;
  while(!(USART_STAT0(USART0) & USART_SR_TXE));
	#endif	
}
void USARTMY_Transmit_Byte(char Data)
{
	#ifdef MY_D	
  USART_DATA(USART0) = Data;
  while(!(USART_STAT0(USART0) & USART_SR_TXE));
	#endif	
}
void USART1_Transmit_Byte(unsigned char Data)
{
  USART_DATA(USART1) = Data;
  while(!(USART_STAT0(USART1) & USART_SR_TXE));
}
void USART2_Transmit_Byte(char Data)
{
	USART_DATA(USART2) = Data;
	while(!(USART_STAT0(USART2) & USART_SR_TXE));
}

void USARTX_DMA_Numbers(uint8_t x,int32_t numbers)
{
  Usart_Data[1]=48;
	Usart_Data[2]=48;
	Usart_Data[3]=48;
	Usart_Data[4]=48;
	Usart_Data[5]=48;
	Usart_Data[6]=48;
  if(numbers<0)
  {
    numbers=-numbers;
    Usart_Data[0] = '-';//USART1_Transmit_String(1,"-");
  }
  else
  {
    Usart_Data[0] = '+';//USART1_Transmit_String(1,"+");
  }

  while(numbers>9)
  {
    if(numbers>99999)
    {
      numbers-=100000;
      Usart_Data[1]++;
    }
    else if(numbers>9999)
    {
      numbers-=10000;
      Usart_Data[2]++;
    }
		else if(numbers>999)
    {
      numbers-=1000;
      Usart_Data[3]++;
    }
		else if(numbers>99)
    {
      numbers-=100;
      Usart_Data[4]++;
    }
		else
		{
			numbers-=10;
      Usart_Data[5]++;
		}
	}
	Usart_Data[6]+=numbers;
	if(x==0)
	{
		USART0_DMA_String(7,Usart_Data);
	}
	else if(x==1)
	{
		USART1_DMA_String(7,Usart_Data);	
	}
	else if(x==2)
	{
		USART2_DMA_String(7,Usart_Data);	
	}	
}

//--------debug--------//

void Debug_DMA_Numbers(int32_t numbers)
{
	USARTX_DMA_Numbers(0,numbers);
}
void Debug_Print(char *data,uint8_t Dev)
{
	#ifdef USART_PRINT_ENABLE
	uint32_t Length=0;
	char *buffer;
	buffer = data;
	while(*data++)
	{
		Length++;
	}
	data=buffer;
	switch(Dev)
	{
		case Dev_USART0:Debug_DMA_Write_String(Length,data);
			              return;		
		case Dev_USART1:Debug_DMA_Write_String(Length,data);
			              return;
		case Dev_USART2:Debug_DMA_Write_String(Length,data);
			              return;
		default:return;
	}
	#endif
}
void Debug_DMA_Write_String(uint16_t length,char *Data)
{
	USART0_DMA_String(length,Data);
}
void Debug_Write_String(uint16_t length,char *Data)
{
	USART0_Transmit_String(length,Data);
}
void Debug_Write_Byte(char ch)
{
	#ifdef MY_D
	USARTMY_Transmit_Byte(ch);
	#else
	USART0_Transmit_Byte(ch);
	#endif
}
//--------外设通信------//
void WIFI_DMA_Write_String(uint16_t length,char *Data)
{
//	USART1_DMA_String(length,Data);	
	USART0_DMA_String(length,Data);	
}
void Gyro_DMA_Write_String(uint16_t length,char *Data)
{
	#ifndef GYRO1
	USART2_DMA_String(length,Data);
	#endif
}


/*rtos*/
#define UART3_IDX_LENGTH 30
#define UART3_TEXT_LENGTH 100
xQueueHandle g_usart_queue;
TaskHandle_t g_usart3_print_task_handler;
xSemaphoreHandle g_binary_semaphore_UsartTXF;
char g_usart3_data[UART3_IDX_LENGTH+1][UART3_TEXT_LENGTH+1]={0};
void USART3_CreateTask(void)
{
	portBASE_TYPE xTaskStatus;
	vSemaphoreCreateBinary(g_binary_semaphore_UsartTXF);
	g_usart_queue = xQueueCreate(50,sizeof(US_Queue_t));
	xTaskStatus = xTaskCreate(USART3_PrintTask,"US3PrintTask",USART_TASK_STK_SIZE,NULL,osPriorityAboveNormal,&g_usart3_print_task_handler);
	if(xTaskStatus==pdPASS)
	{
		printf("US3Print Task Created!!\n");
	}
}
void USART3_PrintTask(void *p_agument)
{
	US_Queue_t Queue_Data;
	portBASE_TYPE xStatus;
	for(;;)
	{
		xStatus = xQueueReceive(g_usart_queue,&Queue_Data,portMAX_DELAY);
		if(xStatus == pdPASS)
		{
			Debug_DMA_Write_String(Queue_Data.length,g_usart3_data[Queue_Data.idx]);
			xSemaphoreTake(g_binary_semaphore_UsartTXF,10/portTICK_RATE_MS); 
		}
	}
}

volatile uint8_t g_usart_data_idx=0;
void USART3_PrintStringByLength(char *p_data,uint8_t length)
{
	US_Queue_t queue_temp_data;
	uint8_t text_temp_idx = 0;
	for(text_temp_idx= 0 ;text_temp_idx < length; text_temp_idx++)
	{
		g_usart3_data[g_usart_data_idx][text_temp_idx] = *p_data;
		p_data++;
		if(text_temp_idx >= UART3_TEXT_LENGTH)break;
	}
	queue_temp_data.idx = g_usart_data_idx;
	queue_temp_data.length = length;
	xQueueSend(g_usart_queue, &queue_temp_data, 1/portTICK_RATE_MS);
	g_usart_data_idx++;
	if(g_usart_data_idx >= (UART3_IDX_LENGTH))g_usart_data_idx = 0;
	if(uxQueueMessagesWaiting(g_usart_queue) >= (UART3_IDX_LENGTH))//queue buffer full,wait for usart send
	{
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}


void ups3(char *Data) {
        char    *data_ptr = Data;
        while (*data_ptr) {
                Debug_Write_Byte(*data_ptr);
                data_ptr++;
        }
}

void ups(char *Data) {
        char    *data_ptr = Data;
        uint16_t        length = 0;

        while (*data_ptr) {
                length++;
                data_ptr++;
        }

        Debug_DMA_Write_String(length, Data);
}

void upn(int32_t data) {
        uint8_t c = 1, d, e;
        int32_t l;
        char    f;

        if(data < 0) {
                f = '-';
                Debug_DMA_Write_String(1, &f);
                data *= -1;
        }

        l = data;
        while(l >= 10) {
                l /= 10;
                ++c;
        }

        for(d = c; d > 0; --d) {
                l = data;
                for(e = d - 1; e > 0; --e) {
                        l /= 10;
                }
                f = (l % 10) + 48;
                Debug_DMA_Write_String(1, &f);
        }
}

