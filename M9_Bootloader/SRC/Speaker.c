/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Set up the speaker function to control the voice speaking peripheral IC
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Speaker.h"
#include "SPI.h"
#include "W25Q16.h"


void Beep(uint8_t Sound)
{

}

uint8_t gmusic_value=0,g_step=0;
uint32_t gsound_code_start=0,gsound_code_end=0;
void Audio(void)   
{ 
	static uint32_t num=0;
	if(Is_Audio_Enable())
	{
		if(g_step==0)
		{
			W25Q16_CS_ON;                            	//使能器件   
			SPI2_ReadWriteByte(W25X_ReadData);         	//发送读取命令   
			SPI2_ReadWriteByte((u8)((gsound_code_start)>>16));  	//发送24bit地址    
			SPI2_ReadWriteByte((u8)((gsound_code_start)>>8));   
			SPI2_ReadWriteByte((u8)gsound_code_start);
			num=0;
			g_step=1;
		}
		else if(g_step==1)
		{
			gmusic_value=SPI2_ReadWriteByte(0XFF);
			dac_data_set(DAC0, DAC_ALIGN_8B_R, gmusic_value);
			num++;
			if(num>gsound_code_end)
			{				
				W25Q16_CS_OFF; 
				AUDIO_DISABLE;
				g_step=2;
			}			
		}
	}   				    	      
} 

void Speaker(uint8_t data)
{
	if(Is_Audio_Enable())return;
	gsound_code_start=0;
	gsound_code_end=0;
	gmusic_value=0;
	g_step=0; 
	switch(data)
	{
		case SPK_SYS_MUSIC:    					gsound_code_start=addr_1;
																		gsound_code_end=num_1;
																    break;
		case SPK_CHECK_BUMPER:    			gsound_code_start=addr_2;
																		gsound_code_end=num_2;
																    break;
		case SPK_CHECK_CLIFF:    				gsound_code_start=addr_3;
																		gsound_code_end=num_3;
																    break;		
		case SPK_LOW_POWER_WILL_OFF:    gsound_code_start=addr_4;
																		gsound_code_end=num_4;
																    break;
		case SPK_ROBOT_TO_START_POINT:  gsound_code_start=addr_5;
																		gsound_code_end=num_5;
																    break;
		case SPK_CHECK_DIRTY_TANK:    	gsound_code_start=addr_6;
																		gsound_code_end=num_6;
																    break;
		case SPK_WIFI_CONNECT_FAIL:    	gsound_code_start=addr_7;
																		gsound_code_end=num_7;
																    break;
		case SPK_AREA_CLEAN_START:    	gsound_code_start=addr_8;
																		gsound_code_end=num_8;
																    break;
		case SPK_CLEAN_START:    				gsound_code_start=addr_9;
																		gsound_code_end=num_9;
																    break;
		case SPK_LOW_POWER_PLE_CHARG:   gsound_code_start=addr_10;
																		gsound_code_end=num_10;
																    break;
		case SPK_PLE_START_FLOOR:    		gsound_code_start=addr_11;
																		gsound_code_end=num_11;
																    break;
		case SPK_MOP_MODE_START:    		gsound_code_start=addr_12;
																		gsound_code_end=num_12;
																    break;
		case SPK_CHECK_MAIN_BRUSH:    	gsound_code_start=addr_13;
																		gsound_code_end=num_13;
																    break;
		case SPK_WALLFOLLOW_MODE_START: gsound_code_start=addr_14;
																		gsound_code_end=num_14;
																    break;
		case SPK_CHECK_SIDE_BRUSH:    	gsound_code_start=addr_15;
																		gsound_code_end=num_15;
																    break;
		case SPK_CHECK_LEFT_WHEEL:    	gsound_code_start=addr_16;
																		gsound_code_end=num_16;
																    break;
		case SPK_ENTER_RECHARGE_MODE:   gsound_code_start=addr_17;
																		gsound_code_end=num_17;
																    break;
		case SPK_CHECK_RIGHT_WHEEL:    	gsound_code_start=addr_18;
																		gsound_code_end=num_18;
																    break;
		case SPK_EXIT_RECHARGE_MODE:    gsound_code_start=addr_19;
																		gsound_code_end=num_19;
																    break;
		case SPK_CHECK_FAN:    					gsound_code_start=addr_20;
																		gsound_code_end=num_20;
																    break;
		case SPK_CLEAN_FINISH:    			gsound_code_start=addr_21;
																		gsound_code_end=num_21;
																    break;
		case SPK_CLEAN_STOP:    				gsound_code_start=addr_22;
																		gsound_code_end=num_22;
																    break;
		case SPK_WIFI_CONNECTED:    		gsound_code_start=addr_23;
																		gsound_code_end=num_23;
																    break;
		case SPK_CHARGING_START:    		gsound_code_start=addr_24;
																		gsound_code_end=num_24;
																    break;
		case SPK_WIFI_CONNECT_START:    gsound_code_start=addr_25;
																		gsound_code_end=num_25;
																    break;
		case SPK_GO_ON:    							gsound_code_start=addr_26;
																		gsound_code_end=num_26;
																    break;
		case SPK_KEY_INVALID:    				gsound_code_start=addr_27;
																		gsound_code_end=num_27;
																    break;
		case SPK_DON:    								gsound_code_start=addr_28;
																		gsound_code_end=num_28;
																    break;
		case SPK_SUSPEND:    						gsound_code_start=addr_29;
																		gsound_code_end=num_29;
																    break;	
		default:break;	
	}
	AUDIO_ENABLE;	
}

//void SPK_Report_Error(Error_t error_val)
//{
//	
//}




