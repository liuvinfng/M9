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

#include "example-robovac.h"

/* InvenSense utils */
#include "Message.h"
#include "RingBuffer.h"
#include "system-interface.h"
#include <math.h>
#include <USART.h>

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Icm426xx driver object */
static struct inv_icm426xx icm_driver;

/* Contains algorithms inputs */
static InvnAlgoRobovacInput input;

/* Contains algorithms outputs */
static InvnAlgoRobovacOutput output;

/* Contains algorithms configuration */
static InvnAlgoRobovacConfig config;

/* Counter to keep track of number of sample received */
static int iter = 0;

static int decimation_factor_print = 1;

#define STATE_INITIAL_CALIBRATION      	(1<<1)
#define STATE_CLEANING                 	(1<<2)
#define STATE_CALIBRATION              	(1<<3)



/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

#if PRINT_INPUTS_OUTPUTS
static void print_algo_inputs(void);
static void print_algo_outputs(void);
#endif

uint8_t g_icm42688_loop=0;
void Set_Icm42688_Loop(uint8_t flag)
{
	g_icm42688_loop = flag;
}
uint8_t Get_Icm42688_Loop(void)
{
	return g_icm42688_loop;
}
void Set_Input_State(int32_t flag)
{
	input.sRobotState = flag;
}
int32_t Get_Input_State(void)
{
	return input.sRobotState;
}
void Set_Output_State(int32_t flag)
{
	output.stop_request = flag;
}
int32_t Get_Output_State(void)
{
	return output.stop_request;
}


/*
 * This function set up the communication with the ICM chip.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	INV_MSG(INV_MSG_LEVEL_INFO, "#######################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   RoboVac example   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "#######################");


	/* Init device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx");

	rc = inv_icm426xx_init(&icm_driver, icm_serif, HandleInvDeviceFifoPacket);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}
	
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_INFO, "Check Icm426xx whoami value");

	rc = inv_icm426xx_get_who_am_i(&icm_driver, &who_am_i);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value.");
		return rc;
	}
	
	if(who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}
	
	return rc;
}

/*
 * This function set up the MEMS (accel, gyro)
 */
int ConfigureInvDevice(uint32_t processing_freq_hz)
{
	int rc = 0;
	uint16_t wm;
	uint8_t data;

//	rc |= inv_icm426xx_enable_clkin_rtc(&icm_driver, 1);

	rc |= inv_icm426xx_enable_high_resolution_fifo(&icm_driver);

	/* Configure FIFO watermark */
	wm = (processing_freq_hz == 0) ? 1 : (1000/processing_freq_hz);
	rc |= inv_icm426xx_configure_fifo_wm(&icm_driver, wm);

	/* Configure filter to ODR/10, 3rd order */
	rc |= inv_icm426xx_set_gyro_ln_bw(&icm_driver, ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10);
	rc |= inv_icm426xx_set_accel_ln_bw(&icm_driver, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10);
	data = 0x15; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_ACCEL_CONFIG1, 1, &data);
	data = 0x1a; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_GYRO_CONFIG1, 1, &data);

	/* Configure AAF to 99Hz */
	data = 0x02; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_REG_BANK_SEL, 1, &data);
	data = 0x12; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_ACCEL_CONFIG_STATIC2_B2, 1, &data);
	data = 0x51; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_ACCEL_CONFIG_STATIC3_B2, 1, &data);
	data = 0x90; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_ACCEL_CONFIG_STATIC4_B2, 1, &data);
	data = 0x01; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_REG_BANK_SEL, 1, &data);
	data = 0x09; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_GYRO_CONFIG_STATIC3_B1, 1, &data);
	data = 0x51; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_GYRO_CONFIG_STATIC4_B1, 1, &data);
	data = 0x90; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_GYRO_CONFIG_STATIC5_B1, 1, &data);
	data = 0x00; rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_REG_BANK_SEL, 1, &data);

	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ);
	rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, ICM426XX_GYRO_CONFIG0_ODR_1_KHZ);

	rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);
	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);
	
	return rc;
}

/*
 * This function set up the robovac algorithm
 */
int InitInvRobovacAlgo(uint32_t processing_freq_hz)
{
	int rc = 0;

	memset(&input, 0, sizeof(input));
	memset(&output, 0, sizeof(output));
	memset(&config, 0, sizeof(config));

	config.is_high_res_en = 1;
	config.acc_fsr = 16;
	config.gyr_fsr = 2000;
	
	/* 
	 * CLKIN uses 32768Hz clock
	 * For 1KHz requested ODR, the effective ODR will be 32768/32=1024Hz
	 * Configure odr-us field to 1/1024=977us
	 */
	config.acc_odr_us = 977; 
	config.gyr_odr_us = 977;
	config.min_dynamic_calib_duration = 1*1000*1000; // 1s

	config.temp_offset = 25 << 16;
	config.temp_sensitivity = (int32_t)((int64_t)((int64_t)100 << 30)/13248); // high-res

	config.decimation_factor = (processing_freq_hz == 0) ? 1 : (1000/processing_freq_hz);
		 
	/* Initialize robovac algorithms */
	rc |= invn_algo_robovac_init(&icm_driver, &config);//edit by vin

	/* Compute decimation factor to print traces at PRINTING_FREQUENCY_HZ */
	decimation_factor_print = (processing_freq_hz == 0) ? 1000/PRINTING_FREQUENCY_HZ : processing_freq_hz/PRINTING_FREQUENCY_HZ;

	return rc;
}

/*
 * Function called if the MCU reveives an IRQ
 */
int GetDataFromInvDevice(void)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. 
	 * HandleInvDeviceFifoPacket) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	return inv_icm426xx_get_data_from_fifo(&icm_driver);
}


/*
 * Callback called after fetching the accel and gyro value from the chip FIFO
 */
void HandleInvDeviceFifoPacket(inv_icm426xx_sensor_event_t * event)
{

	/* Skipping processing if FIFO doesn't contain all required data */
	if(    !(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL))
		|| !(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))
		|| !(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_TEMPERATURE)) )
		return;


	/* Create input data */
	input.mask = INVN_ALGO_ROBOVAC_INPUT_MASK_ROBOT_STATE;
	input.mask |= INVN_ALGO_ROBOVAC_INPUT_MASK_ACC; 
	input.mask |= INVN_ALGO_ROBOVAC_INPUT_MASK_GYR;
	if(icm_driver.fifo_highres_enabled) {
		input.sRacc_data[0] = (((int32_t)event->accel[0] << 4)) | event->accel_high_res[0];
		input.sRacc_data[1] = (((int32_t)event->accel[1] << 4)) | event->accel_high_res[1];
		input.sRacc_data[2] = (((int32_t)event->accel[2] << 4)) | event->accel_high_res[2];
	} else {
		input.sRacc_data[0] = ((int32_t)event->accel[0] /*<< 4*/);
		input.sRacc_data[1] = ((int32_t)event->accel[1] /*<< 4*/);
		input.sRacc_data[2] = ((int32_t)event->accel[2] /*<< 4*/);
	}
	
	input.mask |= INVN_ALGO_ROBOVAC_INPUT_MASK_GYR; 
	if(icm_driver.fifo_highres_enabled) {
		input.sRgyr_data[0] = (((int32_t)event->gyro[0] << 4)) | event->gyro_high_res[0];
		input.sRgyr_data[1] = (((int32_t)event->gyro[1] << 4)) | event->gyro_high_res[1];
		input.sRgyr_data[2] = (((int32_t)event->gyro[2] << 4)) | event->gyro_high_res[2];
	} else {
		input.sRgyr_data[0] = ((int32_t)event->gyro[0] /*<< 4*/);
		input.sRgyr_data[1] = ((int32_t)event->gyro[1] /*<< 4*/);
		input.sRgyr_data[2] = ((int32_t)event->gyro[2] /*<< 4*/);
	}
	input.sRtemp_data = event->temperature;
	
	
	/* Process the robovac algo */
	invn_algo_robovac_process(&input, &output);//edit by vin
	
	if(output.mask) {
#if PRINT_INPUTS_OUTPUTS

		iter++;
		print_algo_inputs();
		print_algo_outputs();
#endif
//		robovac_user_state_machine();//edit by vin
	
	}
}


/* --------------------------------------------------------------------------------------
 *  Utility function used for convert the quaternions to angles
 * -------------------------------------------------------------------------------------- */
void fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift, const uint8_t dim)
{
	int i;
	float scale = 1.f/(1<<fxp_shift);

	for (i=0; i<dim; i++)
		out[i] = scale * in[i];
}

void quaternions_to_angles(const float quat[4], float angles[3])
{
	const float RAD_2_DEG = (180.f/3.14159265358979f);
	float rot_matrix[9];

	{ // quaternion_to_rotation_matrix
		const float dTx  = 2.0f * quat[1];
		const float dTy  = 2.0f * quat[2];
		const float dTz  = 2.0f * quat[3];
		const float dTwx = dTx  * quat[0];
		const float dTwy = dTy  * quat[0];
		const float dTwz = dTz  * quat[0];
		const float dTxx = dTx  * quat[1];
		const float dTxy = dTy  * quat[1];
		const float dTxz = dTz  * quat[1];
		const float dTyy = dTy  * quat[2];
		const float dTyz = dTz  * quat[2];
		const float dTzz = dTz  * quat[3];

		rot_matrix[0] = 1.0f - (dTyy + dTzz);
		rot_matrix[1] = dTxy - dTwz;
		rot_matrix[2] = dTxz + dTwy;
		rot_matrix[3] = dTxy + dTwz;
		rot_matrix[4] = 1.0f - (dTxx + dTzz);
		rot_matrix[5] = dTyz - dTwx;
		rot_matrix[6] = dTxz - dTwy;
		rot_matrix[7] = dTyz + dTwx;
		rot_matrix[8] = 1.0f - (dTxx + dTyy);
	}

	angles[0] = atan2f(-rot_matrix[3], rot_matrix[0])*RAD_2_DEG;
	angles[1] = atan2f(-rot_matrix[7], rot_matrix[8])*RAD_2_DEG;
	angles[2] = asinf(-rot_matrix[6])*RAD_2_DEG;

	if(angles[0] < 0.f)
		angles[0] += 360.f;
}

/* --------------------------------------------------------------------------------------
 *  Utility function for the data logging
 * -------------------------------------------------------------------------------------- */
#if PRINT_INPUTS_OUTPUTS
float quat[4], angles_deg[3];
static void print_algo_outputs(void) {
//	if (iter % decimation_factor_print == 0) 
	{
		float acc_g[3];
		float gyr_dps[3];
		float gyr_bias[3];
		
		float temp;
		/* Convert data to float before send it to the terminal */
		fixedpoint_to_float(output.acc_cal_q16, acc_g, 16, 3);
		fixedpoint_to_float(output.gyr_cal_q16, gyr_dps, 16, 3);
		fixedpoint_to_float(output.grv_quat_q30, quat, 30, 4);
		fixedpoint_to_float(output.gyr_bias_q16, gyr_bias, 16, 3);
		fixedpoint_to_float(&output.temp_degC_q16, &temp, 16, 1);

		quaternions_to_angles(quat, angles_deg);		

//		if(Get_Icm42688_Loop())
//		USART0_PrintfDebugInfo("Angles=[%.2f, %.2f, %.2f]\n           ",
//		angles_deg[0], angles_deg[1], angles_deg[2]
//		);
//		if(Get_Icm42688_Loop())
//		USART0_PrintfDebugInfo("OUTPUT MASK=[0x%0.2x]\n StopReq=[%d]\n Acc=[%.2f, %.2f, %.2f, %d]\n Gyr=[%.2f, %.2f, %.2f, %d]\n GyrBias=[%.2f, %.2f, %.2f]\n Angles=[%.2f, %.2f, %.2f]\n Temp=%.2f C\n\n",
//		output.mask,
//		output.stop_request,
//		acc_g[0], acc_g[1], acc_g[2], (int32_t)output.acc_accuracy_flag,
//		gyr_dps[0], gyr_dps[1], gyr_dps[2], (int32_t)output.gyr_accuracy_flag,
//		gyr_bias[0], gyr_bias[1], gyr_bias[2],
//		angles_deg[0], angles_deg[1], angles_deg[2],
//		temp);
				
		/*INV_MSG(INV_MSG_LEVEL_INFO,
	#if PRINT_QUATERNIONS_INSTEAD_OF_ANGLES
   		    "OUTPUT StopReq=[%d] Acc=[%.2f, %.2f, %.2f, %d] Gyr=[%.2f, %.2f, %.2f, %d] GyrBias=[%.2f, %.2f, %.2f] Quat=[%.2f, %.2f, %.2f, %.2f] Temp=%.2f C",
	#else
		    "OUTPUT MASK=[0x%0.2x] StopReq=[%d] Acc=[%.2f, %.2f, %.2f, %d] Gyr=[%.2f, %.2f, %.2f, %d] GyrBias=[%.2f, %.2f, %.2f] Angles=[%.2f, %.2f, %.2f] Temp=%.2f C",
	#endif
		    output.mask,
				output.stop_request,
		    acc_g[0], acc_g[1], acc_g[2], (int32_t)output.acc_accuracy_flag, 
 			gyr_dps[0], gyr_dps[1], gyr_dps[2], (int32_t)output.gyr_accuracy_flag,
		    gyr_bias[0], gyr_bias[1], gyr_bias[2],
	#if PRINT_QUATERNIONS_INSTEAD_OF_ANGLES
		    quat[0], quat[1], quat[2], quat[3],
	#else
		    angles_deg[0], angles_deg[1], angles_deg[2],
	#endif
		    temp
		);*/
	}
}

uint16_t Return_Ang(void)
{
//	return angles_deg[0]*10;
	return angles_deg[0]*10;
}

static void print_algo_inputs(void){
	if (iter % decimation_factor_print == 0) {
		INV_MSG(INV_MSG_LEVEL_INFO, "INPUT Mask=[0x%0.2x] Acc=[%d %d %d] Gyr=[%d %d %d] Temp=[%d] RobotState=[%x]",
		    input.mask, 
		    input.sRacc_data[0], input.sRacc_data[1], input.sRacc_data[2], 
		    input.sRgyr_data[0], input.sRgyr_data[1], input.sRgyr_data[2],
		    (int32_t)input.sRtemp_data, 
		    input.sRobotState
		    );
	}
}
#endif


/* --------------------------------------------------------------------------------------
 * Commands to the robot, here put message on the UART
 * -------------------------------------------------------------------------------------- */
void robovac_rCommand_StartCleaning(void){
	if (iter % decimation_factor_print == 0) {
		uint64_t timestamp = inv_icm426xx_get_time_us();
		INV_MSG(INV_MSG_LEVEL_INFO, "Algo calibrated. Robot may start moving. Send 'c' when it is.");
	}
}

void robovac_rCommand_StopCleaning(void){
	if (iter % decimation_factor_print == 0) {
		INV_MSG(INV_MSG_LEVEL_INFO, "Algo need calibration. Robot required to stop moving. Send 's' when it is.");
	}
}

/* --------------------------------------------------------------------------------------
 * Robot acknowledge callbacks to modify the algorithm input
 * -------------------------------------------------------------------------------------- */
void robovac_aCommand_StartCleaning(void){
	uint64_t timestamp = inv_icm426xx_get_time_us();
	input.sRobotState = CLEAN;
	INV_MSG(INV_MSG_LEVEL_INFO, "Robot running: [motor on] [vacuum on]");
}

void robovac_aCommand_StopCleaning(void){
	uint64_t timestamp = inv_icm426xx_get_time_us();
	input.sRobotState = MOTOR_ON;
	INV_MSG(INV_MSG_LEVEL_INFO, "Robot stop moving: [motor off] [vacuum on]");
}

void robovac_aCommand_Stop(void){
	uint64_t timestamp = inv_icm426xx_get_time_us();
	input.sRobotState = MOTOR_OFF;
	INV_MSG(INV_MSG_LEVEL_INFO, "Robot fully stopped: [motor off] [vacuum off]");
}


