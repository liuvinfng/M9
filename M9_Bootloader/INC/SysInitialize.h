/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2018
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __SysInitialize_H
#define __SysInitialize_H

#include "gd32f30x.h"
#include "Movement.h"
#include "Obscliff.h"
#include "Wheel.h"
#include "Bldc.h"
#include "Brush.h"
#include "TouchPad.h"
#include "Display.h"
#include "Debug.h"
#include "Charge.h"
#include "Speaker.h"
#include "WifiOTA.h"
#include "wifi.h"

#define Test_Version    1
#define GYRO1           (1) 
#define BLDC_INSTALL    (1) 
#define WIFI_TY         (1)
//#define UPDATE_FLASH    (1)
	
/* PA function define -------------------------------------------------------*/
#define MCU_ADC0             	 		  (uint16_t)0x0001 //0
#define MCU_ADC1 		    			      (uint16_t)0x0002 //1
#define MCU_ADC2 					          (uint16_t)0x0004 //2
#define MCU_ADC3 									  (uint16_t)0x0008 //3
#define MCU_DAC0 					 	        (uint16_t)0x0010 //4
#define MCU_ADC5           		      (uint16_t)0x0020 //5
#define MCU_ADC6 			              (uint16_t)0x0040 //6
#define MCU_ADC7 								    (uint16_t)0x0080 //7
#define MCU_WALL_CTRL	       	 	    (uint16_t)0x0100 //8
#define MCU_USART0_TX  						  (uint16_t)0x0200 //9
#define MCU_USART0_RX    				    (uint16_t)0x0400 //10
#define MCU_L_BUMPER_SW1 			 			(uint16_t)0x0800 //11
#define MCU_L_BUMPER_SW2						(uint16_t)0x1000 //12
#define MCU_SWDIO									  (uint16_t)0x2000 //13
#define MCU_SWCLK							      (uint16_t)0x4000 //14
#define MCU_SPI2_CS			  		      (uint16_t)0x8000 //15

/* PB function define -------------------------------------------------------*/
#define MCU_AD8								      (uint16_t)0x0001 //0
#define MCU_AD9								      (uint16_t)0x0002 //1
#define MCU_L_WHEEL_SPEED						(uint16_t)0x0004 //2
#define MCU_FL_RCON  		    				(uint16_t)0x0008 //3
#define MCU_L_RCON									(uint16_t)0x0010 //4
#define MCU_R_RCON		        			(uint16_t)0x0020 //5
#define MCU_NULL1         				  (uint16_t)0x0040 //6
#define MCU_L_WHEEL_DROP  					(uint16_t)0x0080 //7
#define MCU_MAIN_BRUSH_PWM					(uint16_t)0x0100 //8
#define MCU_OBS_CH1_CTRL						(uint16_t)0x0200 //9
#define MCU_USART2_TX		            (uint16_t)0x0400 //10
#define MCU_USART2_RX			          (uint16_t)0x0800 //11
#define MCU_SW_3V3								  (uint16_t)0x1000 //12
#define MCU_SW_5V					 			    (uint16_t)0x2000 //13
#define MCU_R_WHEEL_DROP					 	(uint16_t)0x4000 //14
#define MCU_HOME_LED						    (uint16_t)0x8000 //15

/* PC function define -------------------------------------------------------*/
#define MCU_AD10 						    		(uint16_t)0x0001 //0
#define MCU_AD11              			(uint16_t)0x0002 //1
#define MCU_AD12 	   	      	  		(uint16_t)0x0004 //2
#define MCU_AD13 					      		(uint16_t)0x0008 //3
#define MCU_AD14			    	    		(uint16_t)0x0010 //4
#define MCU_AD15 						    		(uint16_t)0x0020 //5
#define MCU_VACUUM_CTRL       		  (uint16_t)0x0040 //6
#define MCU_SIDE_BRUSH_PWM       		(uint16_t)0x0080 //7
#define MCU_BLDC_PWM		 				    (uint16_t)0x0100 //8
#define MCU_DUSTBIN_DET		 			    (uint16_t)0x0200 //9
#define MCU_SPI2_SCK		    				(uint16_t)0x0400 //10
#define MCU_SPI2_MISO  							(uint16_t)0x0800 //11
#define MCU_SPI2_MOSI				        (uint16_t)0x1000 //12
#define MCU_OBS_CH4_CTRL		        (uint16_t)0x2000 //13
#define MCU_OSC32_IN		            (uint16_t)0x4000 //14
#define MCU_OSC32_OUT		            (uint16_t)0x8000 //15

/* PD function define -------------------------------------------------------*/
#define MCU_KEY_SPOT 						    (uint16_t)0x0001 //0
#define MCU_KEY_CLEAN               (uint16_t)0x0002 //1
#define MCU_KEY_HOME 	   	      		(uint16_t)0x0004 //2
#define MCU_BAT_LED_R 					  	(uint16_t)0x0008 //3
#define MCU_BAT_LED_G			    			(uint16_t)0x0010 //4
#define MCU_USART1_TX 						  (uint16_t)0x0020 //5
#define MCU_USART1_RX       				(uint16_t)0x0040 //6
#define MCU_FR_RCON       					(uint16_t)0x0080 //7
#define MCU_NULL2		 				        (uint16_t)0x0100 //8
#define MCU_R_WFEEL_SPEED		 				(uint16_t)0x0200 //9
#define MCU_CLEAN_LED		    		    (uint16_t)0x0400 //10
#define MCU_SPOT_LED  							(uint16_t)0x0800 //11
#define MCU_CHARGE_PWM							(uint16_t)0x1000 //12
#define MCU_CHARGE_DET		          (uint16_t)0x2000 //13
#define MCU_VAC_FG		              (uint16_t)0x4000 //14
#define MCU_WIFI_LED		            (uint16_t)0x8000 //15

/* PE function define -------------------------------------------------------*/
#define MCU_OBS_CH2_CTRL 						(uint16_t)0x0001 //0
#define MCU_OBS_CH3_CTRL            (uint16_t)0x0002 //1
#define MCU_R_BUMPER_SW2 	   	      (uint16_t)0x0004 //2
#define MCU_R_BUMPER_SW1 					  (uint16_t)0x0008 //3
#define MCU_CLIFF_CTRL			        (uint16_t)0x0010 //4
#define MCU_ALL_POWER_ON 						(uint16_t)0x0020 //5
#define MCU_I_CTRL       				    (uint16_t)0x0040 //6
#define MCU_GYRO_RST       			    (uint16_t)0x0080 //7
#define MCU_B_RCON		 							(uint16_t)0x0100 //8
#define MCU_L_WHEEL_CTRL		 			  (uint16_t)0x0200 //9
#define MCU_WATER_TANK_DET		    	(uint16_t)0x0400 //10
#define MCU_L_WHEEL_PWM  					  (uint16_t)0x0800 //11
#define MCU_WIFI_PWER_CTRL					(uint16_t)0x1000 //12
#define MCU_R_WHEEL_CTRL		        (uint16_t)0x2000 //13
#define MCU_R_WHEEL_PWM		          (uint16_t)0x4000 //14
#define MCU_AUDIO_ENABLE		        (uint16_t)0x8000 //15

/*----------------------------------System tick Register ---------------------------*/
#define STK_CTRL      (*((volatile unsigned long *)0xE000E010))
#define STK_LOAD      (*((volatile unsigned long *)0xE000E014))
#define STK_VAL       (*((volatile unsigned long *)0xE000E018))
#define STK_CALIB     (*((volatile unsigned long *)0xE000E01C))
	
/*---------------------------------------------------------*/

#define Is_ChargerOn()               (GPIO_ISTAT(GPIOD) & MCU_CHARGE_DET)
#define Is_AtHomeBase()              (GPIO_ISTAT(GPIOD) & MCU_CHARGE_DET)
#define Is_LeftBumper_Trig()         ((!(GPIO_ISTAT(GPIOE) & MCU_R_BUMPER_SW2)))
#define Is_FLeftBumper_Trig()        ((!(GPIO_ISTAT(GPIOE) & MCU_R_BUMPER_SW1)))
#define Is_RightBumper_Trig() 	     ((!(GPIO_ISTAT(GPIOA) & MCU_L_BUMPER_SW1)))
#define Is_FRightBumper_Trig() 	     ((!(GPIO_ISTAT(GPIOA) & MCU_L_BUMPER_SW2)))
#define Is_Right_Wheel_Drop()        (!(GPIO_ISTAT(GPIOB) & MCU_R_WHEEL_DROP))
#define Is_Left_Wheel_Drop()         (!(GPIO_ISTAT(GPIOB) & MCU_L_WHEEL_DROP)) 
#define Is_Bldc_Open()               (GPIO_ISTAT(GPIOC) & MCU_VACUUM_CTRL)
#define Is_Key_Home()								 (GPIO_ISTAT(GPIOD) & MCU_KEY_HOME)
#define Is_Key_Spot()								 (GPIO_ISTAT(GPIOD) & MCU_KEY_SPOT)
#define Is_Key_Clean()							 (GPIO_ISTAT(GPIOD) & MCU_KEY_CLEAN)
#define Is_Dustbin()                 (!(GPIO_ISTAT(GPIOC) & MCU_DUSTBIN_DET))
#define Is_Water_Tank()              (!(GPIO_ISTAT(GPIOE) & MCU_WATER_TANK_DET))
#define Is_Audio_Enable()            (!(GPIO_ISTAT(GPIOE) & MCU_AUDIO_ENABLE))

#define RCON_B_IDR                   (GPIO_ISTAT(GPIOE) & MCU_B_RCON)
#define RCON_L_IDR                   (GPIO_ISTAT(GPIOB) & MCU_L_RCON)
#define RCON_FL_IDR                  (GPIO_ISTAT(GPIOB) & MCU_FL_RCON)
#define RCON_FR_IDR                  (GPIO_ISTAT(GPIOD) & MCU_FR_RCON)
#define RCON_R_IDR                   (GPIO_ISTAT(GPIOB) & MCU_R_RCON)

#define WIFI_POWER_ON                (GPIO_BC(GPIOE)  |= MCU_WIFI_PWER_CTRL)    
#define WIFI_POWER_OFF      			   (GPIO_BOP(GPIOE) |= MCU_WIFI_PWER_CTRL)
#define BLDC_ON               			 (GPIO_BOP(GPIOC) |= MCU_VACUUM_CTRL)
#define BLDC_OFF              			 (GPIO_BC(GPIOC)  |= MCU_VACUUM_CTRL)
#define CTRL_ALL_POWER_ON         	 (GPIO_BOP(GPIOE) |= MCU_ALL_POWER_ON)
#define CTRL_ALL_POWER_OFF        	 (GPIO_BC(GPIOE)  |= MCU_ALL_POWER_ON)
#define CTRL_SW_5V_ON         			 (GPIO_BC(GPIOB)  |= MCU_SW_5V)
#define CTRL_SW_5V_OFF        			 (GPIO_BOP(GPIOB) |= MCU_SW_5V)
#define CTRL_SW_3V_ON         			 (GPIO_BOP(GPIOB) |= MCU_SW_3V3)
#define CTRL_SW_3V_OFF        			 (GPIO_BC(GPIOB)  |= MCU_SW_3V3)
#define I_CTRL_ON             			 (GPIO_BOP(GPIOE) |= MCU_I_CTRL)
#define I_CTRL_OFF            			 (GPIO_BC(GPIOE)  |= MCU_I_CTRL)
#define AUDIO_DISABLE                (GPIO_BOP(GPIOE) |= MCU_AUDIO_ENABLE)
#define AUDIO_ENABLE                 (GPIO_BC(GPIOE)  |= MCU_AUDIO_ENABLE)
#define GYRORST_H                    (GPIO_BOP(GPIOE) |= MCU_GYRO_RST)
#define GYRORST_L                    (GPIO_BC(GPIOE)  |= MCU_GYRO_RST)



#define OBS_CH1_ON               		 (GPIO_BC(GPIOB)  |= MCU_OBS_CH1_CTRL)
#define OBS_CH1_OFF              		 (GPIO_BOP(GPIOB) |= MCU_OBS_CH1_CTRL)
#define OBS_CH2_ON               		 (GPIO_BC(GPIOE)  |= MCU_OBS_CH2_CTRL)
#define OBS_CH2_OFF              		 (GPIO_BOP(GPIOE) |= MCU_OBS_CH2_CTRL)
#define OBS_CH3_ON               		 (GPIO_BC(GPIOE)  |= MCU_OBS_CH3_CTRL)
#define OBS_CH3_OFF              		 (GPIO_BOP(GPIOE) |= MCU_OBS_CH3_CTRL)
#define OBS_CH4_ON               		 (GPIO_BC(GPIOC)  |= MCU_OBS_CH4_CTRL)
#define OBS_CH4_OFF              		 (GPIO_BOP(GPIOC) |= MCU_OBS_CH4_CTRL)
#define CLIFF_CTRL_ON                (GPIO_BC(GPIOE)  |= MCU_CLIFF_CTRL)
#define CLIFF_CTRL_OFF               (GPIO_BOP(GPIOE) |= MCU_CLIFF_CTRL)
#define WALL_CTRL_ON                 (GPIO_BC(GPIOA)  |= MCU_WALL_CTRL)
#define WALL_CTRL_OFF                (GPIO_BOP(GPIOA) |= MCU_WALL_CTRL)




#define LED1_On								 			 (GPIO_BOP(GPIOD) |= MCU_SPOT_LED)
#define LED1_Off							 			 (GPIO_BC(GPIOD)  |= MCU_SPOT_LED)
#define LED2_On 							 			 (GPIO_BOP(GPIOB) |= MCU_HOME_LED)  
#define LED2_Off 							 			 (GPIO_BC(GPIOB)  |= MCU_HOME_LED)
#define LED3_On     					 			 (GPIO_BOP(GPIOD) |= MCU_CLEAN_LED) 
#define LED3_Off    					 			 (GPIO_BC(GPIOD)  |= MCU_CLEAN_LED)	
#define LED4_On     					 			 (GPIO_BOP(GPIOD) |= MCU_BAT_LED_G) 
#define LED4_Off    					 			 (GPIO_BC(GPIOD)  |= MCU_BAT_LED_G)
#define LED5_On     					 			 (GPIO_BOP(GPIOD) |= MCU_BAT_LED_R) 
#define LED5_Off    					 			 (GPIO_BC(GPIOD)  |= MCU_BAT_LED_R)
#define LED6_On     					 			 (GPIO_BOP(GPIOD) |= MCU_WIFI_LED) 
#define LED6_Off    					 			 (GPIO_BC(GPIOD)  |= MCU_WIFI_LED)


#define Status_Cliff_Left            ((uint8_t)0x01)
#define Status_Cliff_Right           ((uint8_t)0x02)
#define Status_Cliff_Back_Left       ((uint8_t)0x04)
#define Status_Cliff_Back_Right      ((uint8_t)0x08)
#define Status_Cliff_Front           ((uint8_t)0x03)
#define Status_Cliff_Back            ((uint8_t)0x0C)
#define Status_Cliff_All             ((uint8_t)0x0F)

#define ReferenceVoltage             (uint16_t)330
#define Low_Battery_Limit            (uint16_t)1200

#define VAC_SPEED_MAX                 1700//17000rpm=1000PA
#define VAC_SPEED_NORMAL              1300//13000rpm
#define VAC_SPEED_ECO                 900//9000rpm

#define Vac_Normal                    0
#define Vac_Max                       1
#define Vac_Eco                       2

#define Clean_MainBrush_Power         85000
#define Home_MainBrush_Power          85000

#define Clean_SideBrush_Power         100000
#define Home_SideBrush_Power          40000




#define Display_Clean                 0
#define Display_Wall                  1
#define Display_Zizag                 2
#define Display_Remote                3
#define Display_Spot                  4
#define Display_Home                  5
#define Display_Test                  6
#define Display_Userinterface         7


#define Disable  0
#define Enable   1

#define Sweep    0
#define Mop      1

#define Origin_Top_Left_Corner     0
#define Origin_Lower_Left_Corner   1
#define	Update_Map_Size            110
#define Report_Point_Max           80

#define FLeftBumperTrig  0x01
#define LLeftBumperTrig  0x10
#define FRightBumperTrig 0x02
#define RRightBumperTrig 0x20

#define LeftBumperTrig   0x11
#define RightBumperTrig  0x22
#define FrontBumperT     0x03
#define AllBumperT       0x03


#define Power_On      	0x01
#define Power_Off     	0x00


#define Charge_PWM            TIMER_CH0CV(TIMER3)
#define Main_Brush_PWM			  TIMER_CH2CV(TIMER3)
#define BLDC_PWM              TIMER_CH2CV(TIMER7)
#define Right_Brush_PWM       TIMER_CH1CV(TIMER7)

typedef struct{
	volatile uint16_t Battery_Voltage;        //10
	volatile uint16_t Main_Brush_Current;     //11
	volatile uint16_t System_Current;         //12
	volatile uint16_t Front_Left_Cliff;       //13	
	volatile uint16_t Back_Left_Cliff;        //0
	volatile uint16_t OBS_CH1;                //1
	volatile uint16_t OBS_CH2;                //2
	volatile uint16_t OBS_CH3;                //3	
  volatile uint16_t Xp_Wall;                //5	
	volatile uint16_t Left_Wheel_Current;     //6
	volatile uint16_t Right_Wheel_Current;    //7	
  volatile uint16_t Right_Brush_Current;    //14
	volatile uint16_t Vacuum_Current;         //15	
	volatile uint16_t Front_Right_Cliff;      //8
	volatile uint16_t Back_Right_Cliff;       //9
}ADC_Value_Struct;

typedef struct{
	volatile int16_t Left_Wall;
	volatile int16_t Left_OBS;
	volatile int16_t Front_OBS;	
	volatile int16_t Right_OBS;
	volatile int16_t Right_Wall;	
	volatile int16_t Xp_Wall;		
}OBS_ADC;

typedef struct
{
	volatile int16_t Front_Left_Cliff;
	volatile int16_t Front_Right_Cliff;
	volatile int16_t Back_Left_Cliff;	
	volatile int16_t Back_Right_Cliff;	
}Cliff_ADC;

void SystemInitialize(void);
void RCC_Configuration(void);
void EXTI_Configuration(void);
void Systick_Configuration(void);
void ADC_Configuration(void);
void DAC_Configuration(void);
void SPI1_Configuration(void);
void SPI2_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
 
void Timer0_Configuration(void);
void Timer1_Configuration(void);
void Timer3_Configuration(void);
void Charge_Configuration(void);
void Timer5_Configuration(void);
void Timer6_Configuration(void);
void Timer7_Configuration(void);


void delay(uint16_t time);
void System_StoreMotorBaseline(void);
uint16_t GetSystemVoltage(void);
void Wifi_All_Data_Init(void);

extern volatile int16_t g_leftwheel_baseline,g_rightwheel_baseline,g_leftbrush_baseline,g_rightbrush_baseline,g_mainbrush_baseline,g_vac_baseline;
extern volatile uint8_t LED_On_Counter,LED1_On_Switch,LED2_On_Switch,LED3_On_Switch,LED1_On_Blink,LED2_On_Blink,LED3_On_Blink;

extern volatile uint32_t g_delay_counter; 
extern uint8_t R_F,L_F;
extern uint16_t g_minute_counter;
extern 	uint16_t g_copy_led;
extern volatile ADC_Value_Struct g_adc_value;
extern volatile OBS_ADC g_obs_adc,g_obs_sunlight;
extern volatile Cliff_ADC g_cliff_adc,g_cliff_sunlight;
extern volatile uint16_t g_baselineadc;

#endif /* __SysInitialize_H */

