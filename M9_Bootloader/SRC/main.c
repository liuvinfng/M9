/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version Ver 0918
  * @date    17-Nov-2018
  * @brief   Main program body
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "SysInitialize.h"
#include "TestMode.h"
#include "W25Q16.h"

void VectorTabs_setup(void)
{
	 __disable_irq() ;
	 __disable_fault_irq();
	 nvic_vector_table_set(NVIC_VECTTAB_FLASH, BOOTLOADER_OFFSET);
	 __enable_irq();
	 __enable_fault_irq();
}

int main(void)
{  		

	SystemInitialize();
	VectorTabs_setup();	
	CTRL_ALL_POWER_ON;
	Disable_PPower();
	Set_5v(Power_On);	
	delay(2000);
	g_baselineadc=GetBaseLineADCV();
	Enable_PPower();
	delay(1000);
	BLDC_OFF;
	#ifdef WIFI_TY
	Wifi_All_Data_Init();
	wifi_protocol_init();
	Set_Wifi_Enable(0);
	#endif	

#if 0	
	uint8_t n=0,m=0;
	while(1)
	{
		if(Key_GetStatus()==(KEY_CLEAN))
		{
			while(Key_GetStatus()==(KEY_CLEAN));
			n++;
			if(n>4)n=1;
			m=n;
		}
		if(m==1)
		{
			m=0;
      Set_BLDC_Speed(500);
			BLDC_ON;
			Set_BLDC_TPWM(30);
			delay(30000);			
		}
		if(m==2)
		{
			m=0;			
      Set_BLDC_Speed(1000);
			BLDC_ON;
			Set_BLDC_TPWM(30);
			delay(30000);			
		}		
		if(m==3)
		{
			m=0;			
      Set_BLDC_Speed(1500);
			BLDC_ON;
			Set_BLDC_TPWM(30);
			delay(30000);			
		}	
		if(m==4)
		{
			m=0;			
      Set_BLDC_Speed(2000);
			BLDC_ON;
			Set_BLDC_TPWM(30);
			delay(30000);			
		}			
	}
#endif
	
	if(Key_GetStatus()==(KEY_CLEAN))
	{
		Mode_SetMode(MODE_TEST);
	}
	else
	{
		Mode_SetMode(MODE_USERINTERFACE);
	}

	/*-------------------------------------------Enter Normal Mode------------------------*/
	System_StoreMotorBaseline();
	delay(1000);

	#ifdef WIFI_TY
	Set_Wifi_Enable(1);
	#endif
	
	#if 0
	while(1)
	{
		if(Is_Key_Press(KEY_CLEAN))
		{
			while(Is_Key_Press(KEY_CLEAN));
			Turn_Left(30,900);
		}
	}
	#endif
//  Set_Upgrade_Flag(0xAA);
  Mode_SetMode(MODE_TEST);
	while(1)
	{
		if(Mode_GetMode()==MODE_TEST)
		{
			Test_Mode();
		}
		else
		{
			Check_for_IAP();
		}
  }
}


/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 ILife CO.LTD *****END OF FILE****/
