/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __Mobility_H
#define __Mobility_H

#include "gd32f30x.h"

#define ANGLE_D  108
#define ANGLE_P  100
	
#define ME_CS                                (uint16_t)1
#define ME_ROBS                              (uint16_t)2
#define ME_FOBS                              (uint16_t)3
#define ME_LOBS                              (uint16_t)4
#define ME_RBumper                           (uint16_t)5
#define	ME_LBumper                           (uint16_t)6
#define	ME_WSensor                           (uint16_t)7
#define	ME_RWSensor                          (uint16_t)8
#define	ME_MobSensor                         (uint16_t)9
#define	ME_LCliff                            (uint16_t)11
#define	ME_FCliff                            (uint16_t)12
#define	ME_RCliff                            (uint16_t)14
#define	ME_Encoder                           (uint16_t)15
#define	ME_WhSpeed                           (uint16_t)16
#define ME_RunCurrent                        (uint16_t)20
#define ME_LRcon                             (uint16_t)21
#define ME_FLRcon                            (uint16_t)22
#define ME_RRcon                             (uint16_t)23
#define ME_FRRcon                            (uint16_t)24
#define	ME_BLRcon                            (uint16_t)26
#define ME_BRRcon                            (uint16_t)27
#define ME_ALLOBS                            (uint16_t)30
#define ME_LLOBS                            (uint16_t)31
#define ME_FFOBS                            (uint16_t)32
#define ME_RROBS                            (uint16_t)33
#define	ME_LWStall                           (uint16_t)101
#define	ME_RWStall                           (uint16_t)102
#define	ME_LBStall                           (uint16_t)103
#define	ME_RBStall                           (uint16_t)104
#define	ME_MBStall                           (uint16_t)108
#define	ME_FStall                            (uint16_t)116
#define	ME_WIFI                            	 (uint16_t)117

#define Test_Status_Left_Bumper	   0x0001
#define Test_Status_Right_OBS  	   0x0002
#define Test_Status_Front_OBS 	   0x0004
#define Test_Status_Left_OBS  	   0x0008
#define Test_Status_Right_Bumper	 0x0010

#define Test_Status_FRRCON    	    0x1000
#define Test_Status_FLRCON  	      0x2000
#define Test_Status_LRCON      	    0x4000
#define Test_Status_BLRCON    	    0x8000
#define Test_Status_BRRCON      	 0x10000
#define Test_Status_RRCON      	   0x20000

#define Test_Status_Left_Cliff 	   0x0020
#define Test_Status_Front_Cliff    0x0040
#define Test_Status_Right_Cliff	   0x0100
#define Test_Status_Virtualwall		 0x0200
#define Test_Status_Charger_Station_Left  0x0400
#define Test_Status_Charger_Station_Right 0x0800

#define Test_OBS_Status_0  	   0x01
#define Test_OBS_Status_1  	   0x02
#define Test_OBS_Status_2  	   0x04
#define Test_OBS_Status_3  	   0x08
#define Test_OBS_Status_4  	   0x10
#define Test_OBS_Status_5  	   0x20
#define Test_OBS_Status_6  	   0x40
#define Test_OBS_Status_7  	   0x80

void Mobility_Mode(void);
void Run_Gyro_Mobility(uint8_t code);


#endif /*----Behaviors------*/





