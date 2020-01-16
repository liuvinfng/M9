 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    5-Dec-2011
  * @brief   Charge Function 
	           initial charge current 50 mA then increasing by time to 500 mA 
						 untill Fully charged ,which by detecting the rising rate of the 
						 temperature ( equote to falling rate of NTC >15 per minute).or 
						 battery voltage reach 1850 mV or NTC temperature reach 45 degree
						 or battery voltage overcharge flag came .
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Icm42688.h"
#include "Icm426xxTransport.h"
#include "Icm426xxDefs.h"
#include "Icm426xxDriver_HL.h"
#include "Icm426xxExtFunc.h"
#include "invn_algo_robovac.h"
#include "system-interface.h"
#include "example-robovac.h"
#include "debug.h"


#define SERIF_TYPE ICM426XX_UI_SPI4
#define PROCESSING_FREQUENCY 100 /*Hz*/

#define DUMP_LEN 23
uint8_t regs_dump[DUMP_LEN] = {0x11, 0x14, 0x16, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x68, 0x75};
uint8_t ch[DUMP_LEN], reg;

static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif);


int rc = 0;
void Icm42688_Init(void)
{
//	uint8_t state=0;
	Set_Icm42688_Loop(0);
	struct inv_icm426xx_serif icm426xx_serif;	
	/* Initialize MCU hardware */
	SetupMCUHardware(&icm426xx_serif);	
	/* Initialize Icm426xx */
	rc = SetupInvDevice(&icm426xx_serif);
	if(rc!=0)
	{
//		printf("error while setting up INV device");
	}
	
	rc = InitInvRobovacAlgo(PROCESSING_FREQUENCY);
	if(rc!=0)
	{
//		printf("error while initializing Robovac algorithm");
	}	
	
	rc = ConfigureInvDevice(PROCESSING_FREQUENCY);
	if(rc!=0)
	{
//		printf("error while configuring INV device");
	}
#if 0
	int i = 0;
  for (; i < DUMP_LEN; i++) {
	  ICM42688_ReadBuff(regs_dump[i], 1, &reg);
	  ch[i] = reg;
  }

	Set_Input_State(0);	
	rc=0;
	state=0;
	while(1)
	{
		
		if(Get_Icm42688_Interr())
		{
			GetDataFromInvDevice();
			Set_Icm42688_Interr(0);
		}
		if(Get_Output_State())
		{
			state = 1;			
		}
		else
		{	
			if(state ==1)
			state = 2;
		}
		rc++;
		if(rc >60000)
		{
			if(state == 2)break;
		}		
	}
	Set_Input_State(3);
  Set_Icm42688_Loop(1);
//	while(1)
//	{
//		if(Get_Output_State())
//		{
//			Set_Input_State(1);
//		}
//		else
//		{
//			Set_Input_State(3);
//		}		
//		if(Get_Icm42688_Interr())
//		{
//			GetDataFromInvDevice();
//			Set_Icm42688_Interr(0);
//		}		
//	}
#endif
}

void Icm42688_Loop(void)
{
	#ifdef GYRO42688
	if(Get_Icm42688_Loop())
	{
		if(Get_Icm42688_Interr())
		{
			Gyro_ResetUpdateFlag(1);
			GetDataFromInvDevice();
			Set_Icm42688_Interr(0);
		}		
	}
	#endif
}

static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif)
{
	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read  = 1024*32*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024*32*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
//	inv_io_hal_init(icm_serif);
}


/*
 * Icm426xx driver needs to get time in us. Let's give its implementation here.
 */
uint64_t g_time_us_icm42688=0;
uint64_t inv_icm426xx_get_time_us(void)
{
	return g_time_us_icm42688;
}

void inv_icm42688_time_us_increase(void)
{
	g_time_us_icm42688++;
}

uint32_t g_delay_us=0;
void inv_icm426xx_sleep_us(uint32_t us)
{
  g_delay_us=0;
  while(g_delay_us<us);
}
void inv_icm426xx_sleep_us_increase(void)
{
	g_delay_us++;
}

/*
 * Clock calibration module needs to disable IRQ. Thus inv_helper_disable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_disable_irq(void)
{
//	disable_irq();
}

/*
 * Clock calibration module needs to enable IRQ. Thus inv_helper_enable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_enable_irq(void)
{
//	enable_irq();
}


volatile uint8_t g_icm42688_interr=0;
void Set_Icm42688_Interr(uint8_t dat)
{
	g_icm42688_interr=dat;
}
uint8_t Get_Icm42688_Interr(void)
{
	return g_icm42688_interr;
}












