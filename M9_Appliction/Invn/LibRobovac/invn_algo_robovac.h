/*
$License:
	Copyright (C) 2018 InvenSense Corporation, All Rights Reserved.
$
*/
 

#ifndef _INVN_ALGO_ROBOVAC_H_
#define _INVN_ALGO_ROBOVAC_H_

#include "invn/common/invn_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup Robovac Robovac
 *  \brief Algorithm that provides robot orientation. Algorithm inputs are raw IMU data (accelerometer and gyroscope data) and robot state. Algorithm outputs calibrated sensor and sensor fusion.
 *  \warning supported sampling frequency [50 Hz, 1000 Hz]
 *  \warning supported gyroscope FSR [250 dps, 500 dps, 1000 dps, 2000 dps, 4000 dps]
 *  \warning supported accelerometer FSR [1 g, 2 g, 4 g, 8 g, 16 g]
 */

//#define INVN_ALGO_USE_MAGNETOMETER
#define INVN_ALGO_ROBOVAC_INPUT_MASK_ACC					1	///< Raw Accel update mask
#define INVN_ALGO_ROBOVAC_INPUT_MASK_GYR					2	///< Raw Gyro update mask
#define INVN_ALGO_ROBOVAC_INPUT_MASK_ROBOT_STATE			8	///< Robot state update mask

#define INVN_ALGO_ROBOVAC_OUTPUT_MASK_ACCEL_CAL			1 ///< Accel cal output update mask
#define INVN_ALGO_ROBOVAC_OUTPUT_MASK_GYRO_CAL			2 ///< gyro cal output update mask
#define INVN_ALGO_ROBOVAC_OUTPUT_MASK_QUAT				4 ///< Quaternion (Accel and Gyro Fusion) output update mask 

#ifdef INVN_ALGO_USE_MAGNETOMETER
#define INVN_ALGO_ROBOVAC_INPUT_MASK_MAG				4	///< Raw Mag update mask
#define INVN_ALGO_ROBOVAC_OUTPUT_MASK_RV				8 ///< Game Rotation Vector (Accel, Gyro and Magnetometer Fusion) output update mask 
#endif

	
/*! \struct InvnAlgoRobovacInput
 * Robovac input structure (raw data) \ingroup Robovac
 */
typedef struct 
{
	int32_t mask;					           /*!<  mask to specify updated inputs. */
	int64_t sRimu_time_us;           /*!<  timestamp \f$ [\mu s]\f$ of raw accel and gyro */
	int32_t sRacc_data[3];           /*!<  raw accelerometer: 20 valid bits in high resolution mode or 16 valid bits otherwise*/
	int32_t sRgyr_data[3];           /*!<  raw gyroscope: 20 valid bits in high resolution mode or 16 valid bits otherwise*/
	int32_t sRtemp_data;             /*!<  raw temperature: 22 valid bits in high resolution mode or 16 valid bits otherwise*/
	int32_t sRobotState;             /*!<  robot state: 0 when motor is turned off, 1 when motor is on but wheel is not moving, 3 when the robot is doing cleaning */
#ifdef INVN_ALGO_USE_MAGNETOMETER
	int64_t sRmag_time_us;           ///< time of raw mag
	int32_t sRmag_data[3];           ///< raw mag
#endif
} InvnAlgoRobovacInput; 


	
/*! \struct InvnAlgoRobovacOutput
 * Robovac output structure (calibrated sensors and fusion output)  \ingroup Robovac
 */
typedef struct 
{
	int32_t mask;					           /*!< mask to specify updated outputs */
	int32_t acc_uncal_q16[3];        /*!< uncalibrated accelerometer (1 g = 0x10000) */
	int32_t acc_cal_q16[3];          /*!< calibrated accelerometer (1 g = 0x10000) */
	int32_t acc_bias_q16[3];         /*!< accelerometer bias (1 g = 0x10000)*/
	int8_t  acc_accuracy_flag;       /*!< accelerometer accuracy from 0(non calibrated) to 3(well calibrated) */

	int32_t gyr_uncal_q16[3];        /*!< uncalibrated gyroscope (1 dps = 0x10000) */
	int32_t gyr_cal_q16[3];          /*!< calibrated gyroscope (1 dps = 0x10000) */
	int32_t gyr_bias_q16[3];         /*!< gyro bias (1 dps = 0x10000)*/
	int8_t  gyr_accuracy_flag;       /*!< gyro accuracy, from 0(non calibrated) to 3(well calibrated) */

	int32_t grv_quat_q30[4];         /*!< 6-axis (accel and gyro fusion) quaternion, 1 = 0x40000000*/
	int32_t gravity_q16[3];			     /*!< 6-axis (accel and gyro fusion) gravity, \warning not used */
	int32_t linearacc_q16[3];		     /*!< 6-axis (accel and gyro fusion) linear acceleration in sensor frame, \warning not used */

#ifdef INVN_ALGO_USE_MAGNETOMETER
	int32_t mag_uncal_q16[3];        ///< uncalibrated magnetometer (1uT = 1<<16)
	int32_t mag_cal_q16[3];          ///< calibrated magnetometer (1uT = 1<<16)
	int32_t mag_bias_q16[3];         ///< magnetometer bias (1uT = 1<<16)
	int8_t  mag_accuracy_flag;       ///< magnetometer accuracy 
	int32_t rv_quat_q30[4];          ///< 9-axis (accel, gyro and magnetometer fusion) quaternion
	int32_t rv_accuracy_q30[4];      ///< 9-axis (accel, gyro and magnetometer fusion) accuracy
#endif

	int32_t temp_degC_q16;           /*!< temperature (1 \f$ [^{\circ}C]\f$ = 0x10000)*/
	int32_t stop_request;            /*!< stop request, 1(require motor stop) or 0(do not require stop) */
	float euler_angles[3];           /*!< Euler angles (yaw, pitch, raw), \warning not used*/
} InvnAlgoRobovacOutput; 

/*! \struct InvnAlgoRobovacConfig
 * Robovac configuration structure (sensor related settings) \ingroup Robovac
 */
typedef struct 
{
	int32_t * acc_bias_q16;    /*!<  Previously stored accel bias pointer */
	int32_t * gyr_bias_q16;    /*!<  Previously stored gyro bias pointer */
	int8_t * gyr_accuracy;    /*!<  Previously stored gyroscope accuracy pointer */
	
	int32_t acc_fsr;           /*!<  accelerometer full scale range [g] */
	int32_t gyr_fsr;           /*!<  gyroscope full scale range [dps] */
	
	uint32_t acc_odr_us;       /*!<  accelerometer output data rate in \f$ [\mu s]\f$ */
	uint32_t gyr_odr_us;       /*!<  gyroscope output data rate \f$ [\mu s]\f$ */

	int32_t min_dynamic_calib_duration; /*!< minimum duration of dynamic calibration \f$ [\mu s]\f$ */
	
#ifdef INVN_ALGO_USE_MAGNETOMETER
	int32_t * mag_bias_q16;			/*!<  mag_bias_q16 Previously stored mag bias (NULL if mag is not present) */
	int32_t   mag_fsr_q16;       /*!<  magnetometer sensitivity (uT/LSB, e.g. mag_uT = (mag_fsr_q16 * raw_mag_LSB)/65536) */
	uint32_t  mag_odr_us;       /*!<  magnetometer sensitivity (uT/LSB, e.g. mag_uT = (mag_fsr_q16 * raw_mag_LSB)/65536) */
#endif
	uint8_t is_high_res_en;    /*!<  is fifo high resolution enabled */
        
    int32_t temp_sensitivity;  /*!<  temperature sensitivity in q30 (if temperature(\f$ ^{\circ}C \f$) = LSB * k + z, then temp_sensitivity = k) */
    int32_t temp_offset;       /*!<  temperature offset in q16 (if temperature(\f$ ^{\circ}C \f$) = LSB * k + z, then temp_offset = z) */
	int32_t decimation_factor; /*!<  decimation factor for sensor fusion : 20, 10, 5, 2 or 1(no decimation) */
} InvnAlgoRobovacConfig; 

/* Forward declarations */
struct inv_icm426xx;

/*!
 * \brief Get library version in the format of x.y.z-suffix
 * \param[out] major version number (x) 
 * \param[out] minor version number (y) 
 * \param[out] patch version number (z) 
 * \param[out] version suffix string
 * \return version retrieval indicator
 * \retval 0 Robot Gyro Calibration.
 * \retval 1 Fail to init.
 * \ingroup Robovac
 */
uint8_t invn_algo_robovac_version(uint8_t *major, uint8_t *minor, uint8_t *patch, char* suffix);

/*!
 * \brief Initializes algorithms with default parameters and reset states.
 * \param[in] icm_device ICM426XX device pointer.
 * \param[in] config init parameters structure.
 * \return initialization success indicator.
 * \retval 0 Success to initialize.
 * \retval 1 Fail to initialize.
 * \ingroup Robovac
 */
uint8_t invn_algo_robovac_init(struct inv_icm426xx * icm_device, const InvnAlgoRobovacConfig * config);


/*!
 * \brief Set the configuration structure of Robovac without reinitialize the states. It is highly recommended to use invn_algo_robovac_init instead.
 * \param[in] icm_device ICM426XX device pointer.
 * \param[in] config configuration structure of the algorithm.
 * \ingroup Robovac
 */
void invn_algo_robovac_set_config(struct inv_icm426xx * icm_device, const InvnAlgoRobovacConfig * config);

/*!
 * \brief Process the given input structure and update the output structure.
 * \param[in] inputs algorithm input structure containing the raw accelerometer, the raw gyroscope, the raw temperature and the robot state. the variable "mask" in InvnAlgoRobovacInput should be set with respect to the avalability of corresponding new sensor data.
 * \param[out] outputs algorithm output structure containing the processing results. The variable mask in InvnAlgoRobovacOutput reports the avalability of accelerometer, gyroscope and quaternion outputs in InvnAlgoRobovacOutput.
 * \ingroup Robovac
 */
void invn_algo_robovac_process(const InvnAlgoRobovacInput *inputs, InvnAlgoRobovacOutput *outputs);

#ifdef __cplusplus
}
#endif




#endif
