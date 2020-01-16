/*!
    \file  gd32f30x_it.c
    \brief interrupt service routines
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-06-23, V1.0.0, demo for GD32F30x
*/
#include "gd32f30x_it.h"
#include "SysInitialize.h"
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
#include "wifiota.h"
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


volatile uint16_t l_wheel_speed_counter=0,r_wheel_speed_counter=0;

volatile uint8_t rece_usart=0;





/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void SVC_Handler(void)
//{
//}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void PendSV_Handler(void)
//{
//}

void RTC_IRQHandler(void)
{
	if(exti_flag_get(EXTI_17) != RESET)
	{
		exti_flag_clear(EXTI_17);
	  if((Mode_GetMode()==MODE_USERINTERFACE)||(Mode_GetMode()==MODE_SLEEP))
		{
			g_alarm_flag=1;
		}		
	}	
}

extern xSemaphoreHandle g_binary_semaphore_UsartTXF;
void DMA0_Channel3_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	if((DMA_INTF(DMA0)&DMA_ISR_TCIF4)==DMA_ISR_TCIF4)
	{
		DMA_INTC(DMA0) |= DMA_IFCR_CTCIF4;
		xSemaphoreGiveFromISR(g_binary_semaphore_UsartTXF,&xHigherPriorityTaskWoken);
	}	
}
void DMA0_Channel6_IRQHandler(void)
{
	if((DMA_INTF(DMA0)&DMA_ISR_TCIF7)==DMA_ISR_TCIF7)
	{
		DMA_INTC(DMA0) |= DMA_IFCR_CTCIF7;
	}	
}

/*----------------------------USART0 Interrupt-----------------------*/
void USART0_IRQHandler(void)
{
	if(usart_interrupt_flag_get(USART0,USART_INT_RBNEIE) != RESET)
  {
		usart_flag_clear(USART0,USART_FLAG_RBNE);
		rece_usart = USART_DATA(USART0);
	}	
	if(usart_flag_get(USART0,USART_FLAG_ORERR) != RESET)
	{		
		rece_usart = USART_STAT0(USART0);
		rece_usart = USART_DATA(USART0);
	}			
}

/*----------------------------USART1 Interrupt-----------------------*/
void USART1_IRQHandler(void)
{
	if(usart_interrupt_flag_get(USART1,USART_INT_RBNEIE) != RESET)
  {
		usart_flag_clear(USART1,USART_FLAG_RBNE);
		#ifdef WIFI_TY   
		if(Get_Wifi_Enable())
		{
			uart_receive_input(USART_DATA(USART1));
		}		
		else
		rece_usart = USART_DATA(USART1);	
		#else
		rece_usart = USART_DATA(USART1);
		#endif
	}		
	if(usart_flag_get(USART1,USART_FLAG_ORERR) != RESET)
	{		
		rece_usart = USART_STAT0(USART1);
		rece_usart = USART_DATA(USART1);
	}		
}

/*----------------------------USART2 Interrupt-----------------------*/
void USART2_IRQHandler(void)
{ 
	if(usart_interrupt_flag_get(USART2,USART_INT_RBNEIE) != RESET)
  {
		usart_flag_clear(USART2,USART_FLAG_RBNE);
		Gyro_ReceiveCharacter(USART_DATA(USART2));		
	}	
	if(usart_flag_get(USART2,USART_FLAG_ORERR) != RESET)
	{		
		rece_usart = USART_STAT0(USART2);
		rece_usart = USART_DATA(USART2);
	}			
}

void EXTI0_IRQHandler(void)
{
	if(exti_flag_get(EXTI_0) != RESET)
	{
		exti_flag_clear(EXTI_0);
		//Пе
	}
}

void EXTI1_IRQHandler(void)
{
	if(exti_flag_get(EXTI_1) != RESET)
	{
		exti_flag_clear(EXTI_1);
		//clean
	}
}

void EXTI2_IRQHandler(void)
{
	if(exti_flag_get(EXTI_2) != RESET)//left wheel
	{
		exti_flag_clear(EXTI_2);
	  if(l_wheel_speed_counter>1)
	  {
			l_wheel_speed_counter=0;
			g_left_speed_cnt++;
			g_left_wheel_step++;
		
			if(L_F==1)
			{
			  g_wallfollow_left_wheel_step++;
			  g_left_move_step++;
			  g_left_wheel_cnt++;
			}
			else if(L_F==2)
			{
			  g_wallfollow_left_wheel_step--;
			  g_left_move_step--;	
			  g_left_wheel_cnt--;	
			}	
		}			
	}
}

void EXTI3_IRQHandler(void)
{
	if(exti_flag_get(EXTI_3) != RESET)//IR_FL
	{
		exti_flag_clear(EXTI_3);
		Rcon_FL();
	}
}

void EXTI4_IRQHandler(void)
{
	if(exti_flag_get(EXTI_4) != RESET)//IR_L
	{
		exti_flag_clear(EXTI_4);
		Rcon_L();
	}
}

void EXTI5_9_IRQHandler(void)
{
	if(exti_flag_get(EXTI_5) != RESET)//IR_R
	{
		exti_flag_clear(EXTI_5);
		Rcon_R();
	}
	if(exti_flag_get(EXTI_6) != RESET)
	{
		exti_flag_clear(EXTI_6);
		//Пе
	}	
	if(exti_flag_get(EXTI_7) != RESET)
	{
		exti_flag_clear(EXTI_7); 
		Rcon_FR();	
	}
	if(exti_flag_get(EXTI_8) != RESET)//IR_B
	{
		exti_flag_clear(EXTI_8);
		Rcon_BR();
	}		
	if(exti_flag_get(EXTI_9) != RESET)//right wheel
	{
		exti_flag_clear(EXTI_9);
		if(r_wheel_speed_counter>1)
		{
			r_wheel_speed_counter=0;		
			g_right_speed_cnt++;
			g_right_wheel_step++; 
			g_wallfollow_acccelerate++;
			if(R_F==1)
			{
				g_wallfollow_right_wheel_step++;
				g_right_move_step++;
				g_right_wheel_cnt++;
			}
			else if(R_F==2)
			{
				g_wallfollow_right_wheel_step--;
				g_right_move_step--;
				g_right_wheel_cnt--;
			}
		}		
	}			
}

void EXTI10_15_IRQHandler(void)
{
	if(exti_flag_get(EXTI_10) != RESET)
	{
		exti_flag_clear(EXTI_10);
		//Пе
	}
	if(exti_flag_get(EXTI_11) != RESET)
	{
		exti_flag_clear(EXTI_11);
		//Пе
	}	
	if(exti_flag_get(EXTI_12) != RESET)
	{
		exti_flag_clear(EXTI_12);
		//Пе
	}
	if(exti_flag_get(EXTI_13) != RESET)
	{
		exti_flag_clear(EXTI_13);
		//charge det
	}		
	if(exti_flag_get(EXTI_14) != RESET)//bldc fg
	{
		exti_flag_clear(EXTI_14);
		g_vac_pulse_cnt++;
	}	
	if(exti_flag_get(EXTI_15) != RESET)//IR_FR
	{
		exti_flag_clear(EXTI_15);

	}		
}



/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
	osSystickHandler();
}

void TIMER5_IRQHandler(void)//0.1ms
{
	static uint8_t ms_10=0;
	if(timer_interrupt_flag_get(TIMER5,TIMER_INT_UP) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER5,TIMER_INT_UP);
		l_wheel_speed_counter++;
		r_wheel_speed_counter++;
		
		Rcon_Timer();
		
		/*-----------------------------------------------------Delay Counter----------------*/
		g_delay_counter++;
    if(g_delay_counter>0xfffffffd)g_delay_counter=0;
    
		g_random_counter++;
		if(g_random_counter>100)g_random_counter=0;
   
    LED_Display();		
		ms_10++;
		if(ms_10>99)//99
		{
			ms_10=0;
		}
	}
}

void TIMER6_IRQHandler(void)//1us
{
	static uint16_t num=0;
	if(timer_interrupt_flag_get(TIMER6,TIMER_INT_UP) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER6,TIMER_INT_UP);
		num++;
		if(num>52)//52
		{
			num=0;
			Audio();
		}
	}
}

void TIMER1_IRQHandler(void)
{	
	volatile static uint8_t OBS_Counter=0,OBS_Run_Flag=0,OBS_CHX=0;//,Near_Num=0
	volatile static OBS_ADC OBS_VAL1,OBS_VAL2,OBS_VAL3,OBS_VAL4,OBS_VAL5;
	if(timer_interrupt_flag_get(TIMER1,TIMER_INT_UP) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_UP);
	}	
	
	if(timer_interrupt_flag_get(TIMER1,TIMER_INT_CH0) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_CH0);				
		CLIFF_CTRL_ON;
		if(OBS_Run_Flag)
		{
			OBS_Run_Flag=0;					
			g_obs_adc.Front_OBS = g_adc_value.OBS_CH2 - g_obs_sunlight.Front_OBS;			
			if(OBS_CHX)
			{
				g_obs_adc.Xp_Wall   = g_adc_value.Xp_Wall - g_obs_sunlight.Xp_Wall - g_xpwall_baseline;
				g_obs_adc.Left_Wall = g_adc_value.OBS_CH1 - g_obs_sunlight.Left_Wall - g_leftwall_baseline;				
				g_obs_adc.Right_OBS = g_adc_value.OBS_CH3 - g_obs_sunlight.Right_OBS;
				OBS_VAL5.Right_OBS=OBS_VAL4.Right_OBS;				
				OBS_VAL4.Right_OBS=OBS_VAL3.Right_OBS;				
				OBS_VAL3.Right_OBS=OBS_VAL2.Right_OBS;			
				OBS_VAL2.Right_OBS=OBS_VAL1.Right_OBS;
				OBS_VAL1.Right_OBS=g_obs_adc.Right_OBS;	
			}
			else
			{
				g_obs_adc.Left_OBS   = g_adc_value.OBS_CH1 - g_obs_sunlight.Left_OBS;
				g_obs_adc.Right_Wall = g_adc_value.OBS_CH3 - g_obs_sunlight.Right_Wall - g_rightwall_baseline;	
				OBS_VAL5.Left_OBS=OBS_VAL4.Left_OBS;				
				OBS_VAL4.Left_OBS=OBS_VAL3.Left_OBS;				
				OBS_VAL3.Left_OBS=OBS_VAL2.Left_OBS;			
				OBS_VAL2.Left_OBS=OBS_VAL1.Left_OBS;
				OBS_VAL1.Left_OBS=g_obs_adc.Left_OBS;				
			}	

			if(((g_obs_adc.Front_OBS>OBS_VAL1.Front_OBS)&&(OBS_VAL1.Front_OBS>OBS_VAL2.Front_OBS)&&(OBS_VAL2.Front_OBS>OBS_VAL3.Front_OBS)
				 &&(OBS_VAL3.Front_OBS>OBS_VAL4.Front_OBS))//&&(OBS_VAL4.Front_OBS>OBS_VAL5.Front_OBS)
				 ||((g_obs_adc.Left_OBS>OBS_VAL1.Left_OBS)&&(OBS_VAL1.Left_OBS>OBS_VAL2.Left_OBS)&&(OBS_VAL2.Left_OBS>OBS_VAL3.Left_OBS)
				 &&(OBS_VAL3.Left_OBS>OBS_VAL4.Left_OBS))//&&(OBS_VAL4.Left_OBS>OBS_VAL5.Left_OBS)
				 ||((g_obs_adc.Right_OBS>OBS_VAL1.Right_OBS)&&(OBS_VAL1.Right_OBS>OBS_VAL2.Right_OBS)&&(OBS_VAL2.Right_OBS>OBS_VAL3.Right_OBS)
				 &&(OBS_VAL3.Right_OBS>OBS_VAL4.Right_OBS))//&&(OBS_VAL4.Right_OBS>OBS_VAL5.Right_OBS)				
				)	
				{
					Set_near_Flag(1);
				}
				else
				{
					if((g_obs_adc.Left_OBS<700) && (g_obs_adc.Front_OBS<700) && (g_obs_adc.Right_OBS<700))
					{
						Set_near_Flag(0);
					}
//					Near_Num++;
//					if(Near_Num>100)
//					{
//						Near_Num=0;
//						Set_near_Flag(0);
//					}
				}
				OBS_VAL5.Front_OBS=OBS_VAL4.Front_OBS;				
				OBS_VAL4.Front_OBS=OBS_VAL3.Front_OBS;				
				OBS_VAL3.Front_OBS=OBS_VAL2.Front_OBS;			
				OBS_VAL2.Front_OBS=OBS_VAL1.Front_OBS;
				OBS_VAL1.Front_OBS=g_obs_adc.Front_OBS;			
		}
				
		g_cliff_adc.Front_Left_Cliff   =3000;// g_adc_value.Front_Left_Cliff  - g_cliff_sunlight.Front_Left_Cliff;
		g_cliff_adc.Front_Right_Cliff  =3000;// g_adc_value.Front_Right_Cliff - g_cliff_sunlight.Front_Right_Cliff;
		g_cliff_adc.Back_Left_Cliff    =3000;// g_adc_value.Back_Left_Cliff   - g_cliff_sunlight.Back_Left_Cliff;		
		g_cliff_adc.Back_Right_Cliff   =3000;// g_adc_value.Back_Right_Cliff  - g_cliff_sunlight.Back_Right_Cliff;
	
		OBS_Counter++;
		if(OBS_Counter>6)
		{
			OBS_Counter = 0;
			OBS_Run_Flag = 1;
			OBS_CHX = 1 - OBS_CHX;
			if(OBS_CHX)
			{
				WALL_CTRL_ON;	
				OBS_CH1_ON;
				OBS_CH2_ON;	
				OBS_CH3_OFF;				
				OBS_CH4_OFF;					
			}
			else
			{				
				OBS_CH3_ON;				
				OBS_CH4_ON;	
				OBS_CH1_OFF;				
				OBS_CH2_OFF;					
			}		
		}			
	}
		
	if(timer_interrupt_flag_get(TIMER1,TIMER_INT_CH1) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_CH1);	
		ADC_CTL1(ADC0) |= ADC_CR2_SWSTART;	
	}	
	
	if(timer_interrupt_flag_get(TIMER1,TIMER_INT_CH2) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_CH2);		

		OBS_CH1_OFF;
		OBS_CH2_OFF;
		OBS_CH3_OFF;			
		OBS_CH4_OFF;
		CLIFF_CTRL_OFF;
		WALL_CTRL_OFF;
		if(OBS_Run_Flag)
		{		
			g_obs_sunlight.Front_OBS = g_adc_value.OBS_CH2;			
			if(OBS_CHX)
			{
				g_obs_sunlight.Xp_Wall   = g_adc_value.Xp_Wall;
				g_obs_sunlight.Left_Wall = g_adc_value.OBS_CH1;				
				g_obs_sunlight.Right_OBS = g_adc_value.OBS_CH3;
			}
			else
			{
				g_obs_sunlight.Left_OBS   = g_adc_value.OBS_CH1;
				g_obs_sunlight.Right_Wall = g_adc_value.OBS_CH3;					
			}						
		}
				
		g_cliff_sunlight.Front_Left_Cliff  = g_adc_value.Front_Left_Cliff;
		g_cliff_sunlight.Front_Right_Cliff = g_adc_value.Front_Right_Cliff;
		g_cliff_sunlight.Back_Left_Cliff   = g_adc_value.Back_Left_Cliff;
		g_cliff_sunlight.Back_Right_Cliff  = g_adc_value.Back_Right_Cliff;
		Battery_AddCapacity_Current(g_adc_value.System_Current);		
	}	
	
	if(timer_interrupt_flag_get(TIMER1,TIMER_INT_CH3) !=RESET)
	{
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_CH3);	
		ADC_CTL1(ADC0) |= ADC_CR2_SWSTART;	
	}		
}










