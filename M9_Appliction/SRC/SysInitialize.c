/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Initialize the system peripherals
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "sysinitialize.h"
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

ParameterStruct SysConfig;
volatile uint16_t g_clean_area=0,g_clean_time=0;
volatile int16_t g_leftwheel_baseline=0,g_rightwheel_baseline=0,g_leftbrush_baseline=0,g_rightbrush_baseline=0,g_mainbrush_baseline=0,g_vac_baseline=0;
volatile ADC_Value_Struct g_adc_value;
volatile uint32_t g_delay_counter=0,Time_25ms = 0,Time_2ms = 0;
uint16_t g_minute_counter=0;
volatile uint16_t g_baseline_adc=0;
volatile uint16_t g_map_id=0,g_map_offset=0;
volatile uint8_t g_loop=1;	
volatile Cliff_ADC g_cliff_adc,g_cliff_sunlight;
volatile OBS_ADC g_obs_adc,g_obs_sunlight;
uint8_t g_green_time[7];

/* All System Initialize -----------------------------------------------------*/
void SystemInitialize(void)
{
  RCC_Configuration();
	GPIO_Configuration();
	Systick_Configuration();
	Timer0_Configuration();
	Timer1_Configuration();
	Timer3_Configuration();
	Timer5_Configuration();
	Timer6_Configuration();	
	Timer7_Configuration();	 
	USART0_Configuration();	
	USART1_Configuration();
	USART2_Configuration();
	EXTI_Configuration();	
	ADC_Configuration();
	DAC_Configuration();
  SPI2_Configuration();
	RTC_Configuration();	
	NVIC_Configuration();	
}

/*System Tick Configuration ----------------------------------------*/
void Systick_Configuration(void)
{
	STK_VAL  = 0;
	STK_LOAD = 120000;//1 MS INTERRUPT
	STK_CTRL = 0X07;// processor clock, interrupt , enable
}

/* Setup System Clocks -------------------------------------------------------*/
void RCC_Configuration(void)
{
	uint32_t TO=0;
	
	/* select HXTAL/2 as clock source */
	RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
	RCU_CFG0 |= (RCU_CFG0_PLLSEL| RCU_CFG0_PREDV0);// 

	/* CK_PLL = (CK_HXTAL/2) * 30 = 120 MHz */
	RCU_CFG1 &= ~RCU_CFG1_PLLPRESEL;
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
	RCU_CFG0 |= RCU_PLL_MUL30;
	/* enable PLL */
	RCU_CTL |= RCU_CTL_PLLEN;

	/* wait until PLL is stable */
	while(0U == (RCU_CTL & RCU_CTL_PLLSTB))
	{
		TO++;
		if(TO>0xf4240)break;			
	}

	/* enable the high-drive to extend the clock frequency to 120 MHz */
	PMU_CTL |= PMU_CTL_HDEN;
	while(0U == (PMU_CS & PMU_CS_HDRF)){
	}

	/* select the high-drive mode */
	PMU_CTL |= PMU_CTL_HDS;
	while(0U == (PMU_CS & PMU_CS_HDSRF)){
	}

	/* select PLL as system clock */
	RCU_CFG0 &= ~RCU_CFG0_SCS;
	RCU_CFG0 |= RCU_CKSYSSRC_PLL;


	
	/* wait until PLL is selected as system clock */
	TO=0;
	while(0U == (RCU_CFG0 & RCU_SCSS_PLL))
	{
		TO++;
		if(TO>0xf4240)break;				
	}	
	
	/* APB1 =AHB/2 */
	#ifdef M_30
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV4;
	#else
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;
	#endif

	
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);// Set ADC precaler div by 4	
	//120M
  RCU_AHBEN	 |= RCU_AHBEN_DMA0EN;
	//120M
	RCU_APB2EN |= RCU_APB2EN_AFEN|
								RCU_APB2EN_PAEN|
								RCU_APB2EN_PBEN|
								RCU_APB2EN_PCEN|	
								RCU_APB2EN_PDEN|	
								RCU_APB2EN_PEEN|
								RCU_APB2EN_ADC0EN|
								RCU_APB2EN_TIMER7EN|
								RCU_APB2EN_TIMER0EN|
								RCU_APB2EN_USART0EN;	
	//外设60M-Time120M
	RCU_APB1EN |= RCU_APB1EN_TIMER1EN|
								RCU_APB1EN_TIMER3EN|
								RCU_APB1EN_TIMER5EN|
								RCU_APB1EN_TIMER6EN|
								RCU_APB1EN_USART1EN|
								RCU_APB1EN_USART2EN|
								RCU_APB1EN_SPI2EN|								
								RCU_APB1EN_DACEN;	

}

/* ----------------------------- Timer0 Initialize ---------------------*/
void Timer0_Configuration(void)//120M_10khz left wheel ch0/ch1  right wheel ch2/ch3
{
  TIMER_CTL0(TIMER0) &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR|TIM_CR1_CMS)));//set counter mode	
	TIMER_CAR(TIMER0) 	= 99;//99
	TIMER_PSC(TIMER0) 	= 119;//199
  TIMER_SWEVG(TIMER0) = (uint16_t)TIM_EGR_UG;
	
 /* SET PWM register  --------------*/
  TIMER_CCHP(TIMER0)  = (uint16_t)(TIM_BDTR_MOE);
	
	TIMER_CHCTL2(TIMER0) = (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E
													|TIM_CCER_CC1P|TIM_CCER_CC2P|TIM_CCER_CC3P|TIM_CCER_CC4P);
	
  TIMER_CHCTL0(TIMER0) = (uint16_t)(TIM_CCMR1_OC1PE|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|
																		TIM_CCMR1_OC2PE|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2);
	TIMER_CHCTL1(TIMER0) = (uint16_t)(TIM_CCMR2_OC4PE|TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2|
																		TIM_CCMR2_OC3PE|TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);

	/* Enable the Timer 1 -----------*/
  TIMER_CTL0(TIMER0) |= TIM_CR1_CEN|TIM_CR1_ARPE;
}

/* ----------------------------- Timer1 Initialize ---------------------*/
void Timer1_Configuration(void)//120M_500hz obs cliff 
{	
  TIMER_CTL0(TIMER1) &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR|TIM_CR1_CMS)));//set counter mode
	TIMER_CAR(TIMER1) 	= 299;	

	#ifdef M_30
	TIMER_PSC(TIMER1) 	= 399;
	#else
	TIMER_PSC(TIMER1) 	= 799;
	#endif
	
  TIMER_SWEVG(TIMER1) = (uint16_t)TIM_EGR_UG;
	
  TIMER_CHCTL2(TIMER1) = 0;  
  TIMER_CHCTL0(TIMER1) = 0;
  TIMER_CHCTL1(TIMER1) = 0;
	
  TIMER_CH0CV(TIMER1) =1;
  TIMER_CH1CV(TIMER1) =30;
	TIMER_CH2CV(TIMER1) =50;//40
  TIMER_CH3CV(TIMER1) =248;	

  TIMER_DMAINTEN(TIMER1) = TIM_DIER_CC1IE|TIM_DIER_CC2IE|TIM_DIER_CC3IE|TIM_DIER_CC4IE;	
  TIMER_CTL0(TIMER1) |= TIM_CR1_CEN|TIM_CR1_ARPE;
}

/* ----------------------------- Timer3 Initialize ---------------------*/
void Timer3_Configuration(void)//120M_10khz ch2:mainbrush
{
  TIMER_CTL0(TIMER3) &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR|TIM_CR1_CMS)));//set counter mode
	TIMER_CAR(TIMER3) 	= 99;	

	#ifdef M_30
	TIMER_PSC(TIMER3) 	= 59;
	#else
	TIMER_PSC(TIMER3) 	= 119;
	#endif
	
  TIMER_SWEVG(TIMER3) = (uint16_t)TIM_EGR_UG;
	
 /* SET PWM register  --------------*/
  TIMER_CCHP(TIMER3)  = (uint16_t)(TIM_BDTR_MOE);
	
	TIMER_CHCTL2(TIMER3) = (TIM_CCER_CC3E);
	
  TIMER_CHCTL0(TIMER3) = (uint16_t)0;
	TIMER_CHCTL1(TIMER3) = (uint16_t)(TIM_CCMR2_OC3PE|TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);

	/* Enable the Timer 3 -----------*/
  TIMER_CTL0(TIMER3) |= TIM_CR1_CEN|TIM_CR1_ARPE;
	
	AFIO_PCF0 &= ~AFIO_PCF0_TIMER3_REMAP;
}

void Charge_Configuration(void)//120M_120khz ch0:carge
{
  TIMER_CTL0(TIMER3) &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR|TIM_CR1_CMS)));//set counter mode
	TIMER_CAR(TIMER3) 	= 999;
	TIMER_PSC(TIMER3) 	= 0;
	
  TIMER_SWEVG(TIMER3) = (uint16_t)TIM_EGR_UG;
	
 /* SET PWM register  --------------*/
  TIMER_CCHP(TIMER3)  = (uint16_t)(TIM_BDTR_MOE);
	
	TIMER_CHCTL2(TIMER3) = (TIM_CCER_CC1E);
	
  TIMER_CHCTL0(TIMER3) = (uint16_t)(TIM_CCMR1_OC1PE|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2);
	TIMER_CHCTL1(TIMER3) = (uint16_t)0;

	/* Enable the Timer 1 -----------*/
  TIMER_CTL0(TIMER3) |= TIM_CR1_CEN|TIM_CR1_ARPE;
	
	AFIO_PCF0 |= AFIO_PCF0_TIMER3_REMAP;
}

/* Setup and Initialize the Timer5 -----------------------------------*/ 
void Timer5_Configuration(void)//120M_10k_0.1ms delay and rcon time base
{
	TIMER_CAR(TIMER5) = 99;	

	#ifdef M_30
	TIMER_PSC(TIMER5) = 59;
	#else
	TIMER_PSC(TIMER5) = 119;
	#endif	
	
	TIMER_SWEVG(TIMER5) = TIM_EGR_UG;
	TIMER_DMAINTEN(TIMER5) = TIM_DIER_UIE;	
	TIMER_CTL0(TIMER5) = TIM_CR1_CEN|TIM_CR1_ARPE;	
}
void Timer6_Configuration(void)//120M_10k_0.001ms 
{
	TIMER_CAR(TIMER6) = 9;

	#ifdef M_30
	TIMER_PSC(TIMER6) = 5;	
	#else
	TIMER_PSC(TIMER6) = 11;	
	#endif	
	
	TIMER_SWEVG(TIMER6) = TIM_EGR_UG;
	TIMER_DMAINTEN(TIMER6) = TIM_DIER_UIE;	
	TIMER_CTL0(TIMER6) = TIM_CR1_CEN|TIM_CR1_ARPE;	
}

void Timer7_Configuration(void)//120M_10khz ch1:sidebrush ch2:vacuum
{		
  TIMER_CTL0(TIMER7) &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR|TIM_CR1_CMS)));//set counter mode
	TIMER_CAR(TIMER7) 	= 99;
	TIMER_PSC(TIMER7) 	= 119;
	
  TIMER_SWEVG(TIMER7) = (uint16_t)TIM_EGR_UG;
	
 /* SET PWM register  --------------*/
  TIMER_CCHP(TIMER7)  = (uint16_t)(TIM_BDTR_MOE);
	
	TIMER_CHCTL2(TIMER7) = (TIM_CCER_CC2E|TIM_CCER_CC3E);
	
  TIMER_CHCTL0(TIMER7) = (uint16_t)(TIM_CCMR1_OC2PE|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2);
	TIMER_CHCTL1(TIMER7) = (uint16_t)(TIM_CCMR2_OC3PE|TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);

	/* Enable the Timer 1 -----------*/
  TIMER_CTL0(TIMER7) |= TIM_CR1_CEN|TIM_CR1_ARPE;
}

/* Setup and Initialize the Exit Interrupt -----------------------------------*/ 
/** 
  * @摘要 外部中断初始化
	* 
	* @参数 x000:PA[x]pin  x001:PB[x]pin  x010:PC[x]pin  x011:PD[x]pin  
	*				x100:PE[x]pin  x101:PF[x]pin  x110:PG[x] pin     
	*  			EXTICR[0](GPIO[3-0])     
	*				EXTICR[1](GPIO[7-4])     
	*				EXTICR[1](GPIO[11-8])      
	*				EXTICR[1](GPIO[15-12])     
	*				A=0;B=1;C=2;D=3;E=4;F=5;G=6
	* @返回值 无 
  */  
void EXTI_Configuration(void)
{ 
	AFIO_EXTISS0 = 0x1130;//B3/B2/D1/空
	AFIO_EXTISS1 = 0x3011;//D7/空/B5/B4
	AFIO_EXTISS2 = 0x0034;//空/空/D9/E8
	AFIO_EXTISS3 = 0x0330;//空/D14/D13/空
	
	EXTI_INTEN = 0x000263BE;
	EXTI_EVEN  = 0x000263BE;
	EXTI_RTEN  = 0x000263BE;
	EXTI_FTEN  = 0x000263BE;
}



/* Setup and Initialize the Analog to Digital Converter ----------------------*/ 
void ADC_Configuration(void)
{
	/*------------------------------ADC config----------------------------------*/	
	ADC_CTL0(ADC0) = 0x00000000;
	ADC_CTL1(ADC0) = 0x00000000;
	ADC_CTL1(ADC0) |= ADC_CR2_ADON;
  ADC_CTL1(ADC0) |= ADC_CR2_RSTCAL;
  while(ADC_CTL1(ADC0)&ADC_CR2_RSTCAL);//waiting for reset calibration
  ADC_CTL1(ADC0) |= ADC_CR2_CAL;
  while(ADC_CTL1(ADC0)&ADC_CR2_CAL);//waiting for calibration
	
	
	ADC_CTL0(ADC0) &= ~(ADC_CR1_AWDEN|ADC_CR1_JAWDEN) ;//Disable watchdog

	ADC_SAMPT0(ADC0) |= (ADC_SMPR1_SMP10_2|ADC_SMPR1_SMP11_2|ADC_SMPR1_SMP12_2|
											 ADC_SMPR1_SMP13_2|ADC_SMPR1_SMP14_2|ADC_SMPR1_SMP15_2); 
	ADC_SAMPT1(ADC0) |= (ADC_SMPR2_SMP0_2|ADC_SMPR2_SMP1_2|ADC_SMPR2_SMP2_2|ADC_SMPR2_SMP3_2|ADC_SMPR2_SMP5_2|
											 ADC_SMPR2_SMP6_2|ADC_SMPR2_SMP7_2|ADC_SMPR2_SMP8_2|ADC_SMPR2_SMP9_2); 
	ADC_RSQ2(ADC0) 	|= (10<<0)|(11<<5)|(12<<10)|(13<<15)|(0<<20)|(1<<25);//0-5 Select channel 10 ,11, 12, 13, 0, 1
	ADC_RSQ1(ADC0) 	|= (2<<0)|(3<<5)|(5<<10)|(6<<15)|(7<<20)|(14<<25);//6-11 Select channel 2,3,5,6,7,14
	ADC_RSQ0(ADC0) 	|= (15<<0)|(8<<5)|(9<<10);//12-15 Select channel 15,8,9
	ADC_RSQ0(ADC0) 	|= ((ADC_SQR1_L_3|ADC_SQR1_L_2|ADC_SQR1_L_1));//1110: Total  15 channel

	ADC_CTL0(ADC0) |= ADC_CR1_SCAN;
	ADC_CTL1(ADC0) |= ADC_CR2_EXTTRIG;
	ADC_CTL1(ADC0) |= ADC_CR2_EXTSEL|ADC_CR2_DMA ; //Software trigger and Enable Dma 
	ADC_CTL1(ADC0) |= ADC_CR2_ADON;			
	ADC_CTL1(ADC0) |= ADC_CR2_SWSTART;	

	/*------------------------------DMA config----------------------------------*/
	DMA_CHCTL(DMA0, DMA_CH0) = 0;
	DMA_CHCTL(DMA0, DMA_CH0) = DMA_CCR1_MSIZE_0|
														 DMA_CCR1_PSIZE_0|
												     DMA_CCR1_CIRC|
												     DMA_CCR1_MINC;	
	DMA_CHPADDR(DMA0, DMA_CH0) = (uint32_t)&(ADC_RDATA(ADC0));//Adress of ADC1_DR
	DMA_CHMADDR(DMA0, DMA_CH0) = (uint32_t)&(g_adc_value);//set memory address	
	DMA_CHCNT(DMA0, DMA_CH0) = 15;
	DMA_CHCTL(DMA0, DMA_CH0) |= DMA_CCR1_EN;	
		
}

void DAC_Configuration(void)
{
	dac_deinit();
	/* configure the DAC0 */
	dac_trigger_disable(DAC0);
	dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);
	dac_output_buffer_enable(DAC0);

	/* enable DAC0 and set data */
	dac_enable(DAC0);
}


void SPI2_Configuration(void)
{
	SPI_CTL0(SPI2) = 0;
	SPI_CTL0(SPI2) |= SPI_PSC_16;                                                                  
	SPI_CTL0(SPI2) |= SPI_CR1_CPOL;                                                          
	SPI_CTL0(SPI2) |= SPI_CR1_CPHA;                                                          
	SPI_CTL0(SPI2) &= ~SPI_CR1_RXONLY;                                                    
	SPI_CTL0(SPI2) &= ~SPI_CR1_BIDIMODE;                                                          
	SPI_CTL0(SPI2) &= ~SPI_CR1_LSBFIRST;                                               
	SPI_CTL0(SPI2) |= SPI_CR1_SSM;                                                            
	SPI_CTL0(SPI2) |= SPI_CR1_SSI;                                                                                                                      
	SPI_CTL0(SPI2) |= SPI_CR1_MSTR;                                                         
	SPI_CTL0(SPI2) |= SPI_CR1_SPE; 
}

/* Setup and Initialize the GPIO  --------------------------------------------*/
/** 
  * @摘要 GPIO初始化
	*
	* @参数 MODER  00 00:0 模拟输入
	*			         01 00:4 浮空输入
	*			         10 00:8 上拉/下拉输入
	*
	*			         00 11:3 推挽输出 	  
	*			         01 11:7 开漏输出			      
	*			         10 11:B 复用推挽输出        
	* 			       11 11:F 复用开漏输出			          			       
	* @返回值 无 
  */
void GPIO_Configuration(void)
{
	/*----------------------------------- GPIOA -----------------------------------------*/
	//0模拟输入 4浮空输入 8上拉/下拉输入 3推挽输出 7开漏输出	B复用推挽输出  F复用开漏输出	
	/*输入:10/11/12  输出:8/15  复用:9/13/14  模拟:0/1/2/3/4/5/6/7  */
	/*(7_6_5_4_3_2_1_0)*/
	GPIO_CTL0(GPIOA) = (uint32_t)0x00000000;
	/*(15_14_13_12_11_10_9_8)*/
	GPIO_CTL1(GPIOA) = (uint32_t)0x300888B7;	
	AFIO_PCF0 |= 0x02000000;
	GPIO_BOP(GPIOA) |= MCU_SPI2_CS;
	
	/*----------------------------------- GPIOB -----------------------------------------*/
	/*输入:2/3/4/5/6/7/11/14  输出:9/12/13/15  复用:8/10  模拟:0/1  */
	//0模拟输入 4浮空输入 8上拉/下拉输入 3推挽输出 7开漏输出	B复用推挽输出  F复用开漏输出
	/*(7_6_5_4_3_2_1_0)*/
	GPIO_CTL0(GPIOB) = (uint32_t)0x88888800;
	/*(15_14_13_12_11_10_9_8)*/
	GPIO_CTL1(GPIOB) = (uint32_t)0x38338B7B;		
	
	/*----------------------------------- GPIOC -----------------------------------------*/
	/*输入:9/14/15  输出:6/13  复用:7/8/10/11/12  模拟:0/1/2/3/4/5  */ 
	//0模拟输入 4浮空输入 8上拉/下拉输入 3推挽输出 7开漏输出	B复用推挽输出  F复用开漏输出
  /*(7_6_5_4_3_2_1_0)*/
	GPIO_CTL0(GPIOC) = (uint32_t)0xB3000000;
	/*(15_14_13_12_11_10_9_8)*/
	GPIO_CTL1(GPIOC) = (uint32_t)0x447BBB8B;								 
	AFIO_PCF0 |= AFIO_PCF0_SPI2_REMAP;
	
	/*----------------------------------- GPIOD -----------------------------------------*/
	/*输入:0/1/2/6/7/8/9/13/14  输出:3/4/10/11/15  复用:5/12  模拟:无  */	
	//0模拟输入 4浮空输入 8上拉/下拉输入 3推挽输出 7开漏输出	B复用推挽输出  F复用开漏输出	
	/*(7_6_5_4_3_2_1_0)*/
	GPIO_CTL0(GPIOD) = (uint32_t)0x88B33888;
	/*(15_14_13_12_11_10_9_8)*/
	GPIO_CTL1(GPIOD) = (uint32_t)0x384B3388;	
	AFIO_PCF0 |= AFIO_PCF0_USART1_REMAP;
		
	/*----------------------------------- GPIOE -----------------------------------------*/							 
	/*输入:2/3/8/10  输出:0/1/4/5/6/7/12/15  复用:9/11/13/14  模拟:无  */	
	//0模拟输入 4浮空输入 8上拉/下拉输入 3推挽输出 7开漏输出	B复用推挽输出  F复用开漏输出	
	/*(7_6_5_4_3_2_1_0)*/
	GPIO_CTL0(GPIOE) = (uint32_t)0x33378877;
	/*(15_14_13_12_11_10_9_8)*/
	GPIO_CTL1(GPIOE) = (uint32_t)0x3BB3B8B8;	
	CTRL_ALL_POWER_ON;
	AFIO_PCF0 |= AFIO_PCF0_TIMER0_REMAP;
			
}


/* Setup and Initialize the NVIC --------------------------------------------*/ 
void NVIC_Configuration(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
	
	nvic_irq_enable(TIMER1_IRQn,0,0);	
	nvic_irq_enable(TIMER5_IRQn,0,0);
	nvic_irq_enable(TIMER6_IRQn,5,0);	
	
	nvic_irq_enable(EXTI0_IRQn,2,0);	
	nvic_irq_enable(EXTI1_IRQn,2,0);	
	nvic_irq_enable(EXTI2_IRQn,2,0);	
	nvic_irq_enable(EXTI3_IRQn,2,0);	
	nvic_irq_enable(EXTI4_IRQn,2,0);		
	nvic_irq_enable(EXTI5_9_IRQn,2,0);
	nvic_irq_enable(EXTI10_15_IRQn,2,0);	
	
	nvic_irq_enable(RTC_IRQn,4,0);	
	nvic_irq_enable(USART0_IRQn,4,0);
	nvic_irq_enable(USART1_IRQn,0,0);	
	nvic_irq_enable(USART2_IRQn,0,0);	
	
	nvic_irq_enable(DMA0_Channel3_IRQn,6,0);
	nvic_irq_enable(DMA0_Channel6_IRQn,7,0);	
}

/* --------------------------------------------- Delay Function time = 100us = 0.1 ms ---------------*/
void delay(uint16_t time)
{
  g_delay_counter=0;
  while(g_delay_counter<time);
}
	
/*sys cur*/
volatile int16_t g_system_current = 0;
uint16_t GetBaseLineADCV(void)
{
  uint32_t SystemVoltageSum=0;
  uint8_t temp=0;
  I_CTRL_ON;
	delay(100);
  for(temp=0;temp<10;temp++)
  {
    delay(10);
    SystemVoltageSum+=g_adc_value.System_Current;
  }
  I_CTRL_OFF; 
	SystemVoltageSum/=10;
	return (uint16_t)SystemVoltageSum;
}

void System_SetCurrentBaselineAdc(int32_t baseline)
{
	g_baseline_adc = baseline;
}
int32_t System_GetCurrentBaselineAdc(void)
{
	return g_baseline_adc;
}

void System_SetAverageCurrent(int16_t current)
{
	g_system_current = current;
}
int16_t System_GetAverageCurrent(void)
{
	return g_system_current;
}
uint16_t System_GetAdc(void)
{
	return g_adc_value.System_Current;
}

void System_StoreMotorBaseline(void)
{
	uint8_t i=0;
	int32_t W_LW=0,W_RW=0,B_LB=0,B_RB=0,B_MB=0,B_Va=0;
	for(i=0;i<10;i++)
	{ 
		B_RB+=g_adc_value.Right_Brush_Current;
		B_MB+=g_adc_value.Main_Brush_Current;
		W_LW+=g_adc_value.Left_Wheel_Current;
		W_RW+=g_adc_value.Right_Wheel_Current;		
		B_Va+=g_adc_value.Vacuum_Current;
		vTaskDelay(10/portTICK_RATE_MS);
	}
	g_leftwheel_baseline  =W_LW/10;
	g_rightwheel_baseline =W_RW/10;	
	g_leftbrush_baseline  =B_LB/10;
	g_rightbrush_baseline =B_RB/10;
	g_mainbrush_baseline  =B_MB/10;
	g_vac_baseline        =B_Va/10;
}

#ifdef WIFI_TY

void Wifi_Update_MapReport_Init(void)
{
  unsigned char length = 0;
  
  length = set_wifi_uart_byte(length,0x13);
  length = set_wifi_uart_byte(length,0);
  length = set_wifi_uart_byte(length,0);	
  length = set_wifi_uart_byte(length,3);
  length = set_wifi_uart_byte(length,Origin_Lower_Left_Corner);	
  length = set_wifi_uart_byte(length,Update_Map_Size>>8); 
  length = set_wifi_uart_byte(length,Update_Map_Size & 0xff);	
  wifi_uart_write_frame(STATE_UPLOAD_CMD, length);
}

void Wifi_Open_Stream_trans(void)
{  
  wifi_uart_write_frame(OPEN_STREAM_TRANS_CMD, 0);
}

void Wifi_Start_Stream_trans(void)
{  
	static uint16_t old_map_id=0;
	unsigned char length = 0;
	old_map_id++;
	g_map_id = old_map_id;
	length = set_wifi_uart_byte(length,g_map_id>>8);
	length = set_wifi_uart_byte(length,g_map_id & 0xff);	
  wifi_uart_write_frame(START_STREAM_TRANS_CMD, length);
//	printf2("g_map_id_start:(%d,%d)\n",g_map_id,g_map_offset);
}


void Wifi_Stop_Stream_trans(void)
{  
	unsigned char length = 0;
	length = set_wifi_uart_byte(length,g_map_id>>8);
	length = set_wifi_uart_byte(length,g_map_id & 0xff);	
	length = set_wifi_uart_byte(length,g_map_offset>>24);	
	length = set_wifi_uart_byte(length,g_map_offset>>16);	
	length = set_wifi_uart_byte(length,g_map_offset>>8);	
	length = set_wifi_uart_byte(length,g_map_offset & 0xff);	
  wifi_uart_write_frame(STOP_STREAM_TRANS_CMD, length);
	printf2("g_map_id_stop:(%d,%d)\n",g_map_id,g_map_offset);	
}


AC_RealMap_t RealMap[Report_Point_Max + 1];
uint8_t AC_RealMap_Cnt = 0,AC_RealMap_Input = 0,AC_RealMap_Output = 0,AC_RealMap_Flag = 0;
void AC_SetFlag(uint8_t flag)
{
	AC_RealMap_Flag = flag;
}
uint8_t Get_ACFlag(void)
{
	return AC_RealMap_Flag;
}

void AC_Reset_RealMap(void)
{
	uint8_t c;
	AC_RealMap_Cnt = AC_RealMap_Input = AC_RealMap_Output = 0;
	g_map_id = g_map_offset =0;
	for(c = 0; c < Report_Point_Max; c++) 
	{
		RealMap[c].x=0;
		RealMap[c].y=0;	
		RealMap[c].state=0;			
	}	
}
uint16_t AC_RealMap_GetCnt(void)
{
	return AC_RealMap_Cnt;
}

uint8_t AC_RealMap_AddPoint(int16_t x,int16_t y,uint8_t state)
{
//	uint8_t i = 0,temp_idx = 0;
	if(AC_RealMap_Cnt > Report_Point_Max)
	{
		Usprintf("%s(%d):the realmap cnt is full\n",__FUNCTION__,__LINE__);
		return 0;
	}
	
//	temp_idx = AC_RealMap_Input;
//	for(i = 1;i <= 9;i++)
//	{
//		if(temp_idx == 0)temp_idx = Report_Point_Max;
//		else temp_idx--;
//		
//		if((RealMap[temp_idx].x == x)&&(RealMap[temp_idx].y == y))
//		{
//			return 0;
//		}
//	}		
	
	if(state == BLOCKED)state = 1;
	else state = 2;	
	RealMap[AC_RealMap_Input].x = x;
	RealMap[AC_RealMap_Input].y = y;
	RealMap[AC_RealMap_Input].state = state;
	
	if((++AC_RealMap_Input) > Report_Point_Max)AC_RealMap_Input = 0;
	AC_RealMap_Cnt++;
	return 1;
}

uint8_t AC_RealMap_ReadPoint(AC_RealMap_t *map_point)
{
	if(AC_RealMap_Cnt == 0)
	{
		Usprintf("%s(%d):No Path out\n",__FUNCTION__,__LINE__);
		return 0;
	}

	*map_point = RealMap[AC_RealMap_Output];
	if((++AC_RealMap_Output) > Report_Point_Max)AC_RealMap_Output = 0;
	if(AC_RealMap_Cnt > 0)AC_RealMap_Cnt--;
	return 1;
}

void Wifi_Report_Map(void)
{
	AC_RealMap_t temp_point;
	unsigned char temp_buf[AC_RealMap_GetCnt()*3+1],i=0;
	if(Mode_GetMode()!=MODE_NAVIGATION)
	{
		AC_SetFlag(0);
		return;
	}
	if(AC_RealMap_GetCnt())
	{
		while(AC_RealMap_GetCnt())
		{
			if(AC_RealMap_ReadPoint(&temp_point))
			{
				temp_buf[i] = temp_point.x + Update_Map_Size/2;
				temp_buf[i+1] = temp_point.y + Update_Map_Size/2;			
				temp_buf[i+2] = temp_point.state;
				i+=3;
			}
//			printf2("point(%d,%d,%d)\n",temp_point.x,temp_point.y,temp_point.state);
		}
		i-=3;
		temp_buf[i+2] = 0;	
		stop_update_flag = DISABLE; 
		if(SUCCESS == stream_trans(g_map_id, g_map_offset, temp_buf, i+3))
		{
			return;
		}	
		g_map_offset++;
	}	
}

void Wifi_All_Data_Init(void)
{
  SysConfig.Ty_PowerSwitch = Enable;
  SysConfig.Ty_ErrorCode = ERROR_NONE;
  SysConfig.Ty_CleanMode = Mode_GetMode();
  SysConfig.Ty_RemoteDir = 4;
  SysConfig.Ty_VacMode = Get_VacMode();
  SysConfig.Ty_CleanSwitch = Enable;
  SysConfig.Ty_CurrentState = IDLE_STATE;
  SysConfig.Ty_DumpEnergy = Battery_Get_CapacityLevel();
  SysConfig.Ty_CleanBuf = 0;
	SysConfig.Ty_CleanRecord = 0;
  SysConfig.Ty_CleanArea = 0;
  SysConfig.Ty_CleanTime = 0;
  SysConfig.Ty_Volume = 0;
  SysConfig.Ty_Hypa = 1;
  SysConfig.Ty_Filter = 1;
  SysConfig.Ty_Mop = 1;
  SysConfig.Ty_EdgeBrush = 1;
  SysConfig.Ty_RollBrush = 1; 
  SysConfig.Ty_SweepMop = Sweep;	
}

void Wifi_Clean_Record_Report(void)
{
	uint8_t temp_data[6];
	temp_data[0] = g_clean_time/100 + '0';
	temp_data[1] = g_clean_time/10 + '0';
	temp_data[2] = g_clean_time%10 + '0';	
	
	temp_data[3] = g_clean_area/100 + '0';
	temp_data[4] = g_clean_area/10 + '0';	
	temp_data[5] = g_clean_area%10 + '0';
//	
//	temp_data[6] = 2 + '0'
//	temp_data[7] = 0 + '0';	
//	temp_data[8] = g_green_time[0]/10 + '0';
//	temp_data[9] = g_green_time[0]%10 + '0';	
//	
//	temp_data[10] = g_green_time[1]/10 + '0';	
//	temp_data[11] = g_green_time[1]%10 + '0';
//	
//	temp_data[12] = g_green_time[2]/10 + '0';	
//	temp_data[13] = g_green_time[2]%10 + '0';
//	
//	temp_data[14] = g_green_time[3]/10 + '0';	
//	temp_data[15] = g_green_time[3]%10 + '0';
//	
//	temp_data[16] = g_green_time[4]/10 + '0';		
//	temp_data[17] = g_green_time[4]%10 + '0';		
	
	mcu_dp_string_update(DPID_CLEAN_RECORD,temp_data,6);
}

void Wifi_Even_Report(void)
{
	static uint8_t curr_state_buf=IDLE_STATE;
	if(SysConfig.Ty_DumpEnergy != Battery_Get_CapacityLevel())
	{
		SysConfig.Ty_DumpEnergy = Battery_Get_CapacityLevel();
		mcu_dp_value_update(DPID_ELECTRICITY_LEFT,SysConfig.Ty_DumpEnergy);
	}
	if(SysConfig.Ty_CleanArea != g_clean_area)
	{
		SysConfig.Ty_CleanArea = g_clean_area;
		mcu_dp_value_update(DPID_CLEAN_AREA,SysConfig.Ty_CleanArea);		
	}
	if(SysConfig.Ty_CleanTime != g_clean_time)
	{
		SysConfig.Ty_CleanTime = g_clean_time;
		mcu_dp_value_update(DPID_CLEAN_TIME,SysConfig.Ty_CleanTime);	
	}	
	
	if(!Wifi_Get_Mode_Flag())
	{
		if(SysConfig.Ty_CleanMode != Mode_GetMode())
		{
			SysConfig.Ty_CleanMode = Mode_GetMode();
			mcu_dp_enum_update(DPID_MODE,SysConfig.Ty_CleanMode);
		}	
		if(SysConfig.Ty_CurrentState != curr_state_buf)
		{
			curr_state_buf = SysConfig.Ty_CurrentState;
			mcu_dp_enum_update(DPID_EQUIPMENT_STATE,SysConfig.Ty_CurrentState);	
		}		
	}

}
#endif



