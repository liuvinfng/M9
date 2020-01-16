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
#include "I2C.h"
//#include "SysInitialize.h"



void I2C_Start(void)
{
  I2C_SDA_OUT;
	I2C_SDA_H;
	I2C_SCL_H;
	delay(1);
	I2C_SDA_L;
	I2C_SCL_H;
	delay(1);
	I2C_SCL_L;
}
void I2C_Stop(void)
{
  I2C_SDA_OUT;
	I2C_SDA_L;
	I2C_SCL_H;
	delay(1);
	I2C_SDA_H;
	I2C_SCL_H;
	delay(1);
	I2C_SCL_L;
}
void I2C_Master_ACK(void)
{
  I2C_SDA_OUT;
  I2C_SDA_L;
	I2C_SCL_L;
	delay(1);
	I2C_SCL_H;
	delay(1);
	I2C_SCL_L;
}
uint8_t Check_Acknowledge(void)
{
  uint8_t Temp_Flag=0;
	//GPIOE->ODR|= MCU_RTC_SDA;
  I2C_SDA_IN;
	I2C_SCL_L;
	delay(1);
	I2C_SCL_H;
	//if(GPIOE->IDR & MCU_RTC_SDA)Temp_Flag=0;
	//else Temp_Flag=1;
	I2C_SCL_L;
	return Temp_Flag;
}
void I2C_Transfer_Byte(uint8_t Data)
{
  uint8_t Byte_Order=0;
  I2C_SDA_OUT;
  I2C_SDA_L;
	I2C_SCL_L;
	for(Byte_Order=0;Byte_Order<8;Byte_Order++)
	{
		if (Data & 0x80) {
			I2C_SDA_H;
		} else {
			I2C_SDA_L;
		}
		Data<<=1;
		delay(1);
		I2C_SCL_H;
		delay(1);
		I2C_SCL_L;
	}
	I2C_SDA_IN;
	//delay(1);
}
uint8_t I2C_Receive_Byte(void)
{
  uint8_t Byte_Order=0;
	uint8_t Temp_Data=0;
  I2C_SDA_IN;
  I2C_SDA_L;
	I2C_SCL_L;
	for(Byte_Order=0;Byte_Order<8;Byte_Order++)
	{
		delay(1);
		I2C_SCL_H;
//		if(GPIOE->IDR & MCU_RTC_SDA)Temp_Data|=0x01;
		if(Byte_Order<7)Temp_Data<<=1;
		delay(1);
		I2C_SCL_L;
	}
	return Temp_Data;
}
uint8_t I2C_Write_Data(uint8_t Address,uint8_t Data)
{
  I2C_Start();
	I2C_Transfer_Byte(DS1307_Address_Write);
  if(!Check_Acknowledge())return 1;//
	I2C_Transfer_Byte(Address);
  if(!Check_Acknowledge())return 1;//	
	I2C_Transfer_Byte(Data);
	if(!Check_Acknowledge())return 1;//
	I2C_Stop();

	return 0;
}
uint8_t I2C_Read_Data(uint8_t Address,uint8_t *Data)
{
  uint8_t Temp_Data=0;
  I2C_Start();
	I2C_Transfer_Byte(DS1307_Address_Write);
  if(!Check_Acknowledge())return 1;//
	I2C_Transfer_Byte(Address);
  if(!Check_Acknowledge())return 1;//
	
	I2C_Start();
	I2C_Transfer_Byte(DS1307_Address_Read);
  if(!Check_Acknowledge())return 1;//
	Temp_Data = I2C_Receive_Byte();
	Check_Acknowledge();
	I2C_Stop();

	*Data = Temp_Data;

	return 0;
}

uint32_t I2C_Read_32bit(uint8_t Address)
{
  uint32_t Temp_Data=0;

  I2C_Start();
	I2C_Transfer_Byte(DS1307_Address_Write);
	if (!Check_Acknowledge()) {
		;
	}

	I2C_Transfer_Byte(Address);	// Set Address
	if (!Check_Acknowledge()) {
		;
	}
	
	I2C_Start(); // Start to read data
	I2C_Transfer_Byte(DS1307_Address_Read);
	if (!Check_Acknowledge()) {
		;
	}

	Temp_Data |= I2C_Receive_Byte();
	I2C_Master_ACK();

	Temp_Data<<=8;
	Temp_Data |= I2C_Receive_Byte();
	I2C_Master_ACK();

	Temp_Data<<=8;
	Temp_Data |= I2C_Receive_Byte();
	I2C_Master_ACK();

	Temp_Data<<=8;
	Temp_Data |= I2C_Receive_Byte();
	Check_Acknowledge();

	I2C_Stop();
	return Temp_Data;
}

uint8_t I2C_Write_16Bit(uint8_t Address,uint16_t Data)
{
  uint16_t Temp_Data=0;
	Temp_Data =	Data;
	Temp_Data>>=8;

  I2C_Start();
	I2C_Transfer_Byte(DS1307_Address_Write);
  if(!Check_Acknowledge())return 1;//
	I2C_Transfer_Byte(Address);
  if(!Check_Acknowledge())return 1;//	

	I2C_Transfer_Byte(Temp_Data&0x00ff);
	if(!Check_Acknowledge())return 1;//

	I2C_Transfer_Byte(Data&0x00ff);
	if(!Check_Acknowledge())return 1;//
	I2C_Stop();

	return 0;
}

uint8_t I2C_Read_16bit(uint8_t Address,volatile uint32_t *Data)
{
  uint16_t Temp_Data=0;

  I2C_Start();
	I2C_Transfer_Byte(DS1307_Address_Write);
  if(!Check_Acknowledge())return 1;//
	I2C_Transfer_Byte(Address);	// Set Address
  if(!Check_Acknowledge())return 1;//
	
	I2C_Start(); // Start to read data
	I2C_Transfer_Byte(DS1307_Address_Read);
  if(!Check_Acknowledge())return 1;//

	Temp_Data=0;

	Temp_Data |= I2C_Receive_Byte();
	I2C_Master_ACK();

	Temp_Data<<=8;

	Temp_Data |= I2C_Receive_Byte();
	I2C_Master_ACK();

	I2C_Stop();
	*Data = Temp_Data;
	return 0;
}
