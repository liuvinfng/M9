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
#include "SysInitialize.h"
#include "USART.h"
#include "Rcon.h"
#include "Movement.h"
#include "TouchPad.h"
#include "Gyro.h"
#include "stdarg.h"
#include "string.h"

char Usart_Data[10]={0,48,48,48,48,48,48,48,48,48};


/*-------------------------------------USART Config------------------------------*/
void USART0_Configuration(void)//120M_DEBUG
{ 
  USART_BAUD(USART0) = USART_120M_115200;
	USART_CTL2(USART0) |= USART_CR3_DMAT|USART_CR3_EIE; 
  USART_CTL0(USART0) |= USART_CR1_UE|USART_CR1_TE;
}

void USART1_Configuration(void)//60M_WIFI
{ 
  USART_BAUD(USART1) = USART_60M_115200;
	USART_CTL2(USART1) |= USART_CR3_DMAT|USART_CR3_EIE;
	USART_CTL0(USART1) |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
}
void USART2_Configuration(void)//60M_gyro
{ 
  USART_BAUD(USART2) = USART_60M_115200;//set baudrate 115200
	USART_CTL2(USART2) |= USART_CR3_DMAT|USART_CR3_EIE;
	USART_CTL0(USART2) |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
}

/*-------------------------------------USART DMA------------------------------*/
void USART0_DMA_String(uint16_t Length,char *data)//DMA_CH3
{
	#ifdef USART_PRINT_ENABLE
	while(DMA_CHCNT(DMA0, DMA_CH3));
	DMA_CHCTL(DMA0, DMA_CH3) &=~ DMA_CCR1_EN;
	DMA_CHPADDR(DMA0, DMA_CH3) = (uint32_t)&(USART_DATA(USART0));
	DMA_CHMADDR(DMA0, DMA_CH3) = (uint32_t)data;
	DMA_CHCNT(DMA0, DMA_CH3) = Length;
	DMA_CHCTL(DMA0, DMA_CH3) |= DMA_CCR1_MINC|DMA_CCR1_DIR;
	DMA_CHCTL(DMA0, DMA_CH3) |= DMA_CCR1_EN;
	#endif
}
void USART1_DMA_String(uint16_t Length,char *data)//DMA_CH6
{
	while(DMA_CHCNT(DMA0, DMA_CH6));
	DMA_CHCTL(DMA0, DMA_CH6) &=~ DMA_CCR1_EN;
	DMA_CHPADDR(DMA0, DMA_CH6) = (uint32_t)&(USART_DATA(USART1));
	DMA_CHMADDR(DMA0, DMA_CH6) = (uint32_t)data;
	DMA_CHCNT(DMA0, DMA_CH6) = Length;
	DMA_CHCTL(DMA0, DMA_CH6) |= DMA_CCR1_MINC|DMA_CCR1_DIR;
	DMA_CHCTL(DMA0, DMA_CH6) |= DMA_CCR1_EN;
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
//    USART1_Transmit_Byte(*Data);
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
    Usart_Data[0] = '-';
  }
  else
  {
    Usart_Data[0] = '+';
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
	USART0_Transmit_Byte(ch);
}
//--------外设通信------//
void WIFI_DMA_Write_String(uint16_t length,char *Data)
{
	USART0_DMA_String(length,Data);	
}
void Gyro_DMA_Write_String(uint16_t length,char *Data)
{
	USART2_DMA_String(length,Data);
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

//USART_DATA_BUFF gUSART_DataBuff;
//void USART0_PrintfDebugInfo(char *pStr,...)
//{
//	u8 slen;
//	
//	va_list pList;
//	 
//	va_start(pList,pStr);
//	vsprintf(gUSART_DataBuff.TX_ARR,pStr,pList);
//	va_end(pList);

//	gUSART_DataBuff.TX_LEN = strlen(gUSART_DataBuff.TX_ARR);
//	
//	for (slen = 0; slen < gUSART_DataBuff.TX_LEN; slen ++)
//	{
//		USART0_Transmit_Byte(gUSART_DataBuff.TX_ARR[slen]);	
//	}	
//}



