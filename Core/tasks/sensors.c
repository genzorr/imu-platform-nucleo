/*
 * sensors.c
 *
 *  Created on: Jul 6, 2019
 *      Author: michael
 */
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <stm32f3xx_hal.h>

#include "MadgwickAHRS.h"
#include "state.h"
#include "sensors.h"

#include "MPU9255.h"
#include "lsm/lsm6ds3_tools.h"
#include "lsm/lsm303c_tools.h"


lsm6ds3_ctx_t dev_ctx;


static uint8_t	get_gyro_staticShift(float* gyro_staticShift);
static uint8_t	get_accel_staticShift(float* accel_staticShift);
int 			get_staticShifts(void);

void 			IMU_Init(void);
int 			IMU_updateDataAll(void);
void 			_IMUtask_updateData(void);


/**
  * @brief	Static function used to get gyro static shift
  * @param	gyro_staticShift	Array used to store that shift
  * @retval	Device's wire error
  */
static uint8_t get_gyro_staticShift(float* gyro_staticShift)
{
	uint8_t error = 0;
	uint16_t zero_orientCnt = 1000;

	float gyro[3] = {0, 0, 0};

	//	Get static gyro shift
	for (int i = 0; i < zero_orientCnt; i++)
	{
		gyro[0] = 0;
		gyro[1] = 0;
		gyro[2] = 0;

		//	Collect data
		if (MPU9255)
		{
			int16_t accelData[3] = {0, 0, 0};
			int16_t gyroData[3] = {0, 0, 0};
			PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
			mpu9255_recalcGyro(gyroData, gyro);
		}
		else if (LSM6DS3)
		{
			PROCESS_ERROR(lsm6ds3_get_g_data_rps(gyro));
		}

		for (int m = 0; m < 3; m++)
			gyro_staticShift[m] += gyro[m];
	}

	for (int m = 0; m < 3; m++)
		gyro_staticShift[m] /= zero_orientCnt;

	printf("%f %f %f\n", gyro_staticShift[0], gyro_staticShift[1], gyro_staticShift[2]);
end:
	return error;
}


/**
  * @brief	Static function used to get accel static shift
  * @param	accel_staticShift	Array used to store accel shift
  * @retval	Device's wire error
  */
static uint8_t get_accel_staticShift(float* accel_staticShift)
{
	uint8_t error = 0;
	uint16_t zero_orientCnt = 500;

	for (int i = 0; i < zero_orientCnt; i++)
	{
		float accel[3] = {0, 0, 0};

		//	Collect data
		if (MPU9255)
		{
			int16_t accelData[3] = {0, 0, 0};
			int16_t gyroData[3] = {0, 0, 0};

			PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
			mpu9255_recalcAccel(accelData, accel);
		}
		else if (LSM6DS3)
		{
			PROCESS_ERROR(lsm6ds3_get_xl_data_g(accel));
		}

		// Set accel static shift vector as (0,0,g)
		accel_staticShift[0] = 0;
		accel_staticShift[1] = 0;
		accel_staticShift[2] += sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
	}

	//	Divide shift by counter
	for (int m = 0; m < 3; m++)
		accel_staticShift[m] /= zero_orientCnt;

end:
	return error;
}


/**
  * @brief	Used to combine getting IMU static shifts
  */
int get_staticShifts(void)
{
	int error = 0;
	float gyro_staticShift[3] = {0, 0, 0};
	float accel_staticShift[3] = {0, 0, 0};

	PROCESS_ERROR(get_gyro_staticShift(gyro_staticShift));
	PROCESS_ERROR(get_accel_staticShift(accel_staticShift));
	HAL_Delay(2000);
	PROCESS_ERROR(get_gyro_staticShift(gyro_staticShift));
	PROCESS_ERROR(get_accel_staticShift(accel_staticShift));

	for (int i = 0; i < 3; i++)
	{
		state_zero.gyro_staticShift[i] = gyro_staticShift[i];
		state_zero.accel_staticShift[i] = accel_staticShift[i];
	}
end:
	return error;
}


/**
  * @brief	Initializes I2C for IMU and IMU too
  */
void IMU_Init(void)
{
	if(MPU9255)
	{
		//	MPU9255 init
		uint8_t mpu9255_initError = mpu9255_init(&hi2c1);
		if (mpu9255_initError != HAL_OK)
		{
			HAL_Delay(100);
			mpu9255_initError = mpu9255_init(&hi2c1);
		}
		mpu9255_initError |= get_staticShifts();

		printf("mpu: %d\n", mpu9255_initError);
		state_system.MPU_state = mpu9255_initError;
	}

	else
	{
		if (LSM6DS3)
		{
			//	LSM6DS3 init
			int error = lsm6ds3_platform_init();
			error |= get_staticShifts();
			printf("lsm6ds3: %d\n", error);
			state_system.MPU_state = error;
		}
		if (LSM303C)
		{
			//	LSM303C init
			int error = lsm303c_platform_init();
			printf("lsm303c: %d\n", error);
			state_system.NRF_state = error;
		}
	}
}


/**
  * @brief	Collects data from IMU, stores it and makes quat using S.Madgwick's algo
  * @retval	R/w IMU error
  */
int IMU_updateDataAll(void)
{
	int error = 0;

	//	Arrays
	float accel[3] = {0, 0, 0};
	float gyro[3] = {0, 0, 0};
	float magn[3] = {0, 0, 0};

	////////////////////////////////////////////////////
	/////////////////	GET IMU DATA  //////////////////
	////////////////////////////////////////////////////

#if (MPU9255)
	int16_t accelData[3] = {0, 0, 0};
	int16_t gyroData[3] = {0, 0, 0};
	int16_t magnData[3] = {0, 0, 0};

	PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
	PROCESS_ERROR(mpu9255_readCompass(magnData));

	//	Recalc data to floats
	mpu9255_recalcAccel(accelData, accel);
	mpu9255_recalcGyro(gyroData, gyro);
	mpu9255_recalcMagn(magnData, magn);
#endif
#if (LSM6DS3)
	error = lsm6ds3_get_xl_data_g(accel);
	error |= lsm6ds3_get_g_data_rps(gyro);
	if (error)
	{
		state_system.MPU_state = error;
		goto end;
	}
#endif

#if (LSM303C)
	error = lsm303c_get_m_data_mG(magn);
	if (error)
	{
		state_system.NRF_state = error;
		goto end;
	}
#endif

	float _time = (float)HAL_GetTick() / 1000;
	state_system.time = _time;

	for (int k = 0; k < 3; k++)
	{
		stateIMU_rsc.accel[k] = accel[k];
		gyro[k] -= state_zero.gyro_staticShift[k];
		stateIMU_rsc.gyro[k] = gyro[k];
		stateIMU_rsc.magn[k] = magn[k];
	}

	// Quaternion update.
	float quaternion[4] = {1, 0, 0, 0};
	float dt = _time - state_system_prev.time;

	float beta = 1;
	MadgwickAHRSupdate(quaternion, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], magn[0], magn[1], magn[2], dt, beta);

	for (int k = 0; k < 4; k++)
		stateIMU_isc.quaternion[k] = quaternion[k];

	// Rotate vectors using quaternion.
	float accel_ISC[3] = {0, 0, 0};
	vect_rotate(accel, quaternion, accel_ISC);

	// Copy vectors to global structure
	for (int k = 0; k < 3; k++)
	{
		stateIMU_isc.accel[k] = accel_ISC[k] - state_zero.accel_staticShift[k];
		stateIMU_isc.magn[k] = magn[k];
	}

end:
	state_system.MPU_state = error;
	return error;
}


/**
  * @brief	Special function for updating previous values structures by current values
  */
void _IMUtask_updateData(void)
{
	memcpy(&stateIMU_isc_prev, 			&stateIMU_isc,			sizeof(stateIMU_isc));
	memcpy(&state_system_prev, 			&state_system,		 	sizeof(state_system));
}

