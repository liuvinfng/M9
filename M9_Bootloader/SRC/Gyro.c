/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   The touch pad and remote functions which related to the user input
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Gyro.h"
#include "Charge.h"
#include "Speaker.h"
#include "Display.h"
#include "USART.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Movement.h"
#include "UserInterface.h"
#include "Rcon.h"
#include "RTC.h"
#include "SysInitialize.h"
#include "display.h"
#include "debug.h"
#include "usart.h"
#include "limits.h"
#include "config.h"
#include "SPI.h"
#include "String.h"
#include "math.h"

uint8_t g_gyro_update_flag = 0;
uint8_t Gyro_RxBuffer[13];

uint8_t Gyro_RxIndex = 0;
uint8_t Gyro_MsgStart = 0;

uint8_t gyro_idx;          //[2]     = Index
uint16_t gyro_angle[2];    //[3:3]   = Angle
uint16_t gyroAngle;
int16_t gyro_rate[2];      //[5:6]   = Rate
int16_t gyro_xacc,xaccbuf; //[7:8]   = X Acceleration
int16_t gyro_yacc,yaccbuf; //[9:10]  = Y Acceleration
int16_t gyro_zacc,zaccbuf; //[11:12] = Z Acceleration

int16_t gyro_xacc_offset;
int16_t gyro_yacc_offset;
int16_t gyro_zacc_offset;

uint16_t gyro_offset = 0;
int32_t gyro_odometer = 0;
uint16_t gyro_time = 0;

int16_t	gyro_raw = 0;

volatile uint8_t gyro_calibration = 0;
volatile uint8_t Gyro_Calibration_Flag=0;

volatile uint8_t Send_Data_Buf[15] = {0xaa, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00};

void Gyro_Reset_Cmd(void)
{
        Send_Data_Buf[2] = 0xff;
        Send_Data_Buf[3] = 0xff;
        Send_Data_Buf[4] = 0xff;

        Send_Data_Buf[14] = 0XFD;

        Gyro_DMA_Write_String(15, (char *) Send_Data_Buf);
				delay(100);
				Gyro_Calibration_EnableCmd();
}

void Gyro_On_Cmd(void)
{
        Send_Data_Buf[2] = 0xee;
        Send_Data_Buf[3] = 0xee;
        Send_Data_Buf[4] = 0xee;

        Send_Data_Buf[14] = 0xca;

        Gyro_DMA_Write_String(15, (char *) Send_Data_Buf);
}

void Gyro_Sleep_Cmd(void)
{
			Send_Data_Buf[2] = 0xDD;
			Send_Data_Buf[3] = 0xDD;
			Send_Data_Buf[4] = 0xDD;

			Send_Data_Buf[14] = 0x97;

			Gyro_DMA_Write_String(15, (char *) Send_Data_Buf);
}

void Gyro_Debug_Cmd(void)
{
        Send_Data_Buf[2] = 0x66;
        Send_Data_Buf[3] = 0x66;
        Send_Data_Buf[4] = 0x66;

        Send_Data_Buf[14] = 0x32;

        Gyro_DMA_Write_String(15, (char *) Send_Data_Buf);
}

void Gyro_Calibration_Cmd(EventStatus NewState)
{
	if(Is_Gyro_Calibration())
	{
		if (NewState == ENABLE) {
						Send_Data_Buf[2] = 0x55;
						Send_Data_Buf[3] = 0x55;
						Send_Data_Buf[4] = 0x55;

						Send_Data_Buf[14] = 0xff;
		} else {
						Send_Data_Buf[2] = 0x44;
						Send_Data_Buf[3] = 0x44;
						Send_Data_Buf[4] = 0x44;

						Send_Data_Buf[14] = 0xcc;
		}	
		Gyro_DMA_Write_String(15, (char *) Send_Data_Buf);
	}
}

void Gyro_Calibration_EnableCmd(void)
{
	Send_Data_Buf[2] = 0x55;
	Send_Data_Buf[3] = 0x55;
	Send_Data_Buf[4] = 0x55;
	Send_Data_Buf[14] = 0xff;
	Gyro_DMA_Write_String(15, (char *) Send_Data_Buf);	
}

void Gyro_Cmd(EventStatus NewState) {
	if (NewState != DISABLE) {
		#ifdef GYRO1
    GYRORST_L;
    delay(500);
    GYRORST_H;
		delay(3000);
		#endif
		Gyro_On_Cmd();
		Gyro_Reset();
		gyro_xacc_offset = gyro_yacc_offset = gyro_zacc_offset = SHRT_MAX;
	} else {
		Gyro_Reset_Cmd();

		gyro_xacc_offset = gyro_yacc_offset = gyro_zacc_offset = SHRT_MAX;
	}
}


int16_t Gyro_GetXAcc(void) {
	return gyro_xacc;
}

int16_t Gyro_GetXAccDiff(void) {
	return (gyro_xacc - gyro_xacc_offset);
}

int16_t Gyro_GetYAcc(void) {
	return gyro_yacc;
}

int16_t Gyro_GetYAccDiff(void) {
	return (gyro_yacc - gyro_yacc_offset);
}

int16_t Gyro_GetZAcc(void) {
	return gyro_zacc;
}

int16_t Gyro_GetZAccDiff(void) {
	return (gyro_zacc - gyro_zacc_offset);
}



int16_t Gyro_GetRaw(void) {
	return gyro_raw;
}

uint16_t Gyro_GetAngleRaw(uint8_t id) {
	return (gyro_angle[id] + 3600) % 3600;
}

int16_t Gyro_GetRate(uint8_t id) {
	return gyro_rate[id];
}



uint8_t Gyro_IsUpdated(void) {
	if (g_gyro_update_flag) {
		g_gyro_update_flag = 0;
		return 1;
	}
	return 0;
}
uint8_t Gryo_GetUpdateFlag(void)
{
	return g_gyro_update_flag;
}
void Gyro_ResetUpdateFlag(uint8_t flag)
{
	g_gyro_update_flag = flag;
}


void Gyro_SetOffset(int16_t offset) {
	gyro_offset = (3600 + offset + gyro_offset) % 3600;
	USPRINTF("Gyro Offset: %d\n", gyro_offset);
}

void Gyro_SetAngle(int16_t theta) {
	gyro_offset = (gyro_angle[0] + 3600 - theta) % 3600;
}

void Gyro_Reset() {
	gyro_calibration = 0;

	gyro_offset = gyro_angle[0];
	gyro_odometer = 0;
	gyro_time = 0;

	gyro_xacc_offset = gyro_xacc;
	gyro_yacc_offset = gyro_yacc;
	gyro_zacc_offset = gyro_zacc;
}

void Gyro_Reset_With_Offset(int16_t offset) {
	gyro_calibration = 0;

	gyro_offset = offset;
	gyro_odometer = 0;
	gyro_time = 0;

	gyro_xacc_offset = gyro_xacc;
	gyro_yacc_offset = gyro_yacc;
	gyro_zacc_offset = gyro_zacc;
}


uint8_t Gyro_GetCalibration(void) {
	return gyro_calibration;
}

#ifdef GYRO1
void Gyro_ReceiveCharacter(uint8_t c)
{
	if (Gyro_MsgStart == 0 && c == 0xFF)
	{
			Gyro_MsgStart = 1;
	}
	else if (Gyro_MsgStart == 1)
	{
		if ( c == 0xFF)
		{
				Gyro_MsgStart = 2;
		}
		else
		{
				Gyro_RxIndex = 0;
				Gyro_MsgStart = 0;
		}
	}
	else if (Gyro_MsgStart == 2)
	{
		Gyro_RxBuffer[Gyro_RxIndex] = c;
		Gyro_RxIndex++;
		if (Gyro_RxIndex == 11)//6
		{ 
			Gyro_RxIndex = 0;
			Gyro_MsgStart = 0;
			if (Gyro_ParseMsg(Gyro_RxBuffer))
			{
					g_gyro_update_flag = 1;
			}
		}
	}
}

uint8_t Gyro_ParseMsg(uint8_t * msg)
{
//    uint8_t c;
    int16_t i,j,k,m, checksum = 0;
    int32_t l;
    ++gyro_time;
    j=(*(msg + 1) << 8);
    j+=*(msg + 0);
    k=(*(msg + 3) << 8);
    k+=*(msg + 2);
    m=(*(msg + 5) << 8);
    m+=*(msg + 4);
    checksum=j+k+0xffff;
    if (checksum != m)
    {
        return 0;
    }
    else
    {
        gyro_idx = *msg;
        gyro_rate[1] = gyro_rate[0];
        gyro_angle[1] = gyro_angle[0];
        gyro_rate[0] = (int16_t)((*(msg + 0) & 0xFF) | ((*(msg + 1) << 8) & 0xFF00)) * -1;
        i = (*(msg + 2) & 0xFF) | ((*(msg + 3) << 8) & 0xFF00);  //[3:4]   = Angle
        gyro_raw = i;
        l = (int32_t)i * -1;
        l = (l + 36000) % 36000;// + 5
        l /= 10;
        gyroAngle = (uint16_t)l;
        gyro_angle[0] = (uint16_t)l;
        gyro_odometer += gyro_angle[0] - gyro_angle[1];
        gyro_xacc_offset = SHRT_MAX;
        gyro_yacc_offset = SHRT_MAX;
        gyro_zacc_offset = SHRT_MAX;
        return 1;
    }
}
#else
void Gyro_ReceiveCharacter(uint8_t c) {
	if (Gyro_MsgStart == 0 && c == 0xAA) {
		Gyro_MsgStart = 1;
	} else if (Gyro_MsgStart == 1) {
		if ( c == 0x00) {
			Gyro_MsgStart = 2;
		} else {
			Gyro_RxIndex = 0;
			Gyro_MsgStart = 0;
		}
	} else if (Gyro_MsgStart == 2) {
		Gyro_RxBuffer[Gyro_RxIndex] = c;
		Gyro_RxIndex++;
		if (Gyro_RxIndex == 13) {
			Gyro_RxIndex = 0;
			Gyro_MsgStart = 0;
			if (Gyro_ParseMsg(Gyro_RxBuffer)) {
				g_gyro_update_flag = 1;
			}
		}
	}
}
uint8_t Gyro_ParseMsg(uint8_t * msg)
{
	uint8_t c, checksum = 0;
	int16_t i;
	int32_t l;

	++gyro_time;

	for (c = 0; c < 12; ++c) {
		checksum += *(msg + c);
	}

	if (checksum != *(msg + 12)) {
		return 0;
	} else {
		gyro_idx = *msg;
		gyro_rate[1] = gyro_rate[0];
		gyro_angle[1] = gyro_angle[0];
		gyro_rate[0] = (int16_t)((*(msg + 3) & 0xFF) | ((*(msg + 4) << 8) & 0xFF00)) * -1;

		i = (*(msg + 1) & 0xFF) | ((*(msg + 2) << 8) & 0xFF00);  //[3:4]   = Angle
		gyro_raw = i;
		l = (int32_t)i * -1;
		l = (l + 36000 + 5) % 36000;
		l /= 10;
		gyroAngle = (uint16_t)l;
		gyro_angle[0] = (uint16_t)l;
		gyro_odometer += gyro_angle[0] - gyro_angle[1];

		gyro_yacc = (*(msg + 5) & 0xFF) | ((*(msg + 6) << 8) & 0xFF00);  //[7:8]   = X Acceleration
		gyro_xacc = (*(msg + 7) & 0xFF) | ((*(msg + 8) << 8) & 0xFF00);  //[9:10]  = Y Acceleration
		gyro_xacc *= -1;
		gyro_zacc = (*(msg + 9) & 0xFF) | ((*(msg + 10) << 8) & 0xFF00);  //[11:12] = Z Acceleration
		gyro_zacc *= -1;

		gyro_calibration = (*(msg + 11) & 0xFF);

		if (gyro_xacc_offset == SHRT_MAX) {
			gyro_xacc_offset = gyro_xacc;
		}
		if (gyro_yacc_offset == SHRT_MAX) {
			gyro_yacc_offset = gyro_yacc;
		}
		if (gyro_zacc_offset == SHRT_MAX) {
			gyro_zacc_offset = gyro_zacc;
		}
		return 1;
	}
}
#endif

uint8_t Is_Gyro_Calibration(void)
{
	return Gyro_Calibration_Flag;	
}

void Set_Gyro_Calibration(uint8_t code)
{
	Gyro_Calibration_Flag = code;
}


uint16_t Gyro_GetAngle(uint8_t id) {
	return (gyro_angle[id] + 3600) % 3600;
}

