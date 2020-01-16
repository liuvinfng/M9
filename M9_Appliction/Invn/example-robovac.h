/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2018-2019 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#ifndef __EXAMPLE_ROBOVAC_H__
#define __EXAMPLE_ROBOVAC_H__

#include <stdint.h>
#include "Icm426xxTransport.h"
#include "Icm426xxDefs.h"
#include "Icm426xxDriver_HL.h"
#include "Icm426xxExtFunc.h"


/* InvenSense LibExport */
#include "invn_algo_robovac.h"



/* --------------------------------------------------------------------------------------
 *  Example configuration
 * -------------------------------------------------------------------------------------- */

/*
 * Select UART port on which INV_MSG() will be logged.
 * Select UART port on which messages will be sent / received.
 * UART1: Carrier Board (FTDI, faster)
 * UART2: ST Link
 */
#define LOG_UART_ID UART1

/*
 * Printing frequency (prevent terminal overload)
 */
#define PRINTING_FREQUENCY_HZ 1

/*
 *  Print input and output of the algorithm
 */
#define PRINT_INPUTS_OUTPUTS 1

/*
 * Select between quaternion and angles when printing output
 * Warning: no effect if `PRINT_INPUTS_OUTPUTS` is set to 0
 */
#define PRINT_QUATERNIONS_INSTEAD_OF_ANGLES 0

typedef enum {
	CLEAN = 3,		// Vacuum cleaner
	MOTOR_ON = 1,	// No vacuum cleaner
	MOTOR_OFF = 0	// No movement, No vacuum cleaner
} RobotState;


/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be succesfully executed before any access to Icm426xx device.
 * 
 * \param[in] icm_serif : Serial interface object.
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 *
 * \param[in] processing_freq_hz : Processing frequency of the algorithm in Hertz. 
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice(uint32_t processing_freq_hz);

/**
 * \brief This function initialize the Robovac algorithm.
 * 
 * \param[in] processing_freq_hz : Processing frequency of the algorithm in Hertz. 
 * \return 0 on success, negative value on error.
 */
int InitInvRobovacAlgo(uint32_t processing_freq_hz);

/**
 * \brief This function extracts data from the Icm426xx FIFO.
 *
 * The function just calls Icm426xx driver function inv_icm426xx_get_data_from_fifo.
 * But note that for each packet extracted from FIFO, a user defined function is called to 
 * allow custom handling of each packet. In this example custom packet handling function
 * is HandleInvDeviceFifoPacket.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromInvDevice(void);

/**
 * \brief This function is the custom handling packet function.
 *
 * It is passed in parameter at driver init time and it is called by 
 * inv_icm426xx_get_data_from_fifo function each time a new valid packet is extracted 
 * from FIFO.
 * In this implementation, function extends packet timestamp from 16 to 64 bits and then
 * process data from packet and print them on UART.
 *
 * \param[in] event structure containing sensor data from one packet
 */
void HandleInvDeviceFifoPacket(inv_icm426xx_sensor_event_t * event);

/**
 * \defgroup Robot Commands
 * \brief Function to send commabds to the robot
 * \{
 */
/** \brief Send a "start cleaning" message in the UART. */
void robovac_rCommand_StartCleaning(void);
/** \brief Send a "start cleaning" message in the UART. */
void robovac_rCommand_StopCleaning(void);
/** \} */

/**
 * \defgroup Algorithm Commands
 * \brief Notify the gyro data provider algorithm
 * \{
 */
/** \brief Modify the algorithm state input: go to the state CLEANING (3) */
void robovac_aCommand_StartCleaning(void);
/** \brief Modify the algorithm state input: go to the state MOTOR_ON (1) from CLEANING (3) */
void robovac_aCommand_StopCleaning(void);
/** \brief Modify the algorithm state input: go to the state MOTOR_OFF (0) */
void robovac_aCommand_Stop(void);
/** \} */

void Set_Input_State(int32_t flag);
int32_t Get_Input_State(void);
void Set_Output_State(int32_t flag);
int32_t Get_Output_State(void);
uint16_t Return_Ang(void);
void Set_Icm42688_Loop(uint8_t flag);
uint8_t Get_Icm42688_Loop(void);


#endif /* !__EXAMPLE_ROBOVAC_H__ */
