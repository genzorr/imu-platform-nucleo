#include <stdint.h>
#include <math.h>
#include <stm32f3xx.h>
#include <stdio.h>

#include "lsm6ds3_reg.h"
#include "state.h"
#include "vector.h"


#define LSM_TIMEOUT	1
#define MG_TO_MPS2	9.8155 / 1000
#define MDPS_TO_RAD	M_PI / 180 / 1000

//	Accelerometer bias & transform matrix
//#define X_ACCEL_OFFSET		0.014983
//#define Y_ACCEL_OFFSET		0.086828
//#define Z_ACCEL_OFFSET		0.028621
//#define XX_ACCEL_TRANSFORM_MATIX	 1.003357
//#define YY_ACCEL_TRANSFORM_MATIX	 1.009286
//#define ZZ_ACCEL_TRANSFORM_MATIX	 1.002768
//#define XY_ACCEL_TRANSFORM_MATIX	 0.000575
//#define XZ_ACCEL_TRANSFORM_MATIX	-0.002213
//#define YZ_ACCEL_TRANSFORM_MATIX	 0.001784
#define X_ACCEL_OFFSET		0.061246
#define Y_ACCEL_OFFSET		-0.035023
#define Z_ACCEL_OFFSET		0.131825
#define XX_ACCEL_TRANSFORM_MATIX	 1.007997
#define YY_ACCEL_TRANSFORM_MATIX	 1.003820
#define ZZ_ACCEL_TRANSFORM_MATIX	 1.006542
#define XY_ACCEL_TRANSFORM_MATIX	 -0.000285
#define XZ_ACCEL_TRANSFORM_MATIX	 -0.000035
#define YZ_ACCEL_TRANSFORM_MATIX	 0.002550

#define X_GYRO_OFFSET		0.065393
#define Y_GYRO_OFFSET		-0.063552
#define Z_GYRO_OFFSET		-0.044984
#define XX_GYRO_TRANSFORM_MATIX	 0.010933
#define YY_GYRO_TRANSFORM_MATIX	 0.006685
#define ZZ_GYRO_TRANSFORM_MATIX	 0.009419
#define XY_GYRO_TRANSFORM_MATIX	 -0.000241
#define XZ_GYRO_TRANSFORM_MATIX	 0.000266
#define YZ_GYRO_TRANSFORM_MATIX	 -0.000171

static uint8_t whoamI, rst;

I2C_HandleTypeDef* i2c_lsm6ds3;
lsm6ds3_ctx_t lsm6ds3_dev_ctx;

#define LSM6DS3_I2C_ADD	0b11010111

int32_t lsm6ds3_bus_init(void* handle);
int32_t lsm6ds3_platform_init(void);

uint32_t lsm6ds3_get_xl_data_g(float* accel);
uint32_t lsm6ds3_get_g_data_rps(float* gyro);



static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == i2c_lsm6ds3)
	{
		error = HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else return -1;

	return error;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == i2c_lsm6ds3)
	{
		error = HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else return -1;

	return error;
}


int32_t lsm6ds3_bus_init(void* handle)
{
	int error = 0;
	if (handle == i2c_lsm6ds3)
	{

	}
	else
		printf("invalid lsm6ds3 handle\n");

	return error;
}


int32_t lsm6ds3_platform_init(void)
{
	int error = 0;

	lsm6ds3_dev_ctx.write_reg = platform_write;
	lsm6ds3_dev_ctx.read_reg = platform_read;
	lsm6ds3_dev_ctx.handle = i2c_lsm6ds3;

	//	Set needed bus parameters
	error |= lsm6ds3_bus_init(lsm6ds3_dev_ctx.handle);

	// Reset to defaults
	error |= lsm6ds3_reset_set(&lsm6ds3_dev_ctx, PROPERTY_ENABLE);
	do {
		error = lsm6ds3_reset_get(&lsm6ds3_dev_ctx, &rst);
	} while (rst);

	// Check who_am_i
	error |= lsm6ds3_device_id_get(&lsm6ds3_dev_ctx, &whoamI);
	if (whoamI != LSM6DS3_ID)
	{
		printf("lsm6ds3 not found, %d\terror: %d\n", whoamI, error);
		return -19;
	}
	else
		printf("lsm6ds3 OK\n");

	error |= lsm6ds3_xl_power_mode_set(&lsm6ds3_dev_ctx, LSM6DS3_XL_HIGH_PERFORMANCE);
	error |= lsm6ds3_gy_power_mode_set(&lsm6ds3_dev_ctx, LSM6DS3_GY_HIGH_PERFORMANCE);

	error |= lsm6ds3_fifo_mode_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);
	error |= lsm6ds3_block_data_update_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);

	error |= lsm6ds3_xl_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3_4g);
	error |= lsm6ds3_gy_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3_1000dps);

	error |= lsm6ds3_xl_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3_XL_ODR_208Hz);
	error |= lsm6ds3_gy_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3_GY_ODR_208Hz);

	error |= lsm6ds3_xl_filter_analog_set(&lsm6ds3_dev_ctx, LSM6DS3_ANTI_ALIASING_50Hz);
	error |= lsm6ds3_xl_lp2_bandwidth_set(&lsm6ds3_dev_ctx, LSM6DS3_XL_LP_ODR_DIV_50);
	error |= lsm6ds3_gy_hp_bandwidth_set(&lsm6ds3_dev_ctx, LSM6DS3_HP_CUT_OFF_2Hz07);

	return error;
}


uint32_t lsm6ds3_get_xl_data_g(float* accel)
{
	axis3bit16_t data_raw_acceleration;
	uint8_t error;
	//	Read acceleration field data
	PROCESS_ERROR(lsm6ds3_acceleration_raw_get(&lsm6ds3_dev_ctx, data_raw_acceleration.u8bit));
	accel[0] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]) * MG_TO_MPS2;
	accel[1] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]) * MG_TO_MPS2;
	accel[2] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]) * MG_TO_MPS2;

	//	Apply accelerometer bias and transform matrix
	float offset_vector[3] = {X_ACCEL_OFFSET, Y_ACCEL_OFFSET, Z_ACCEL_OFFSET};
	float transform_matrix[3][3] =	{{XX_ACCEL_TRANSFORM_MATIX, XY_ACCEL_TRANSFORM_MATIX, XZ_ACCEL_TRANSFORM_MATIX},
									 {XY_ACCEL_TRANSFORM_MATIX, YY_ACCEL_TRANSFORM_MATIX, YZ_ACCEL_TRANSFORM_MATIX},
									 {XZ_ACCEL_TRANSFORM_MATIX, YZ_ACCEL_TRANSFORM_MATIX, ZZ_ACCEL_TRANSFORM_MATIX}};

	vmv(accel, offset_vector, accel);
	mxv(transform_matrix, accel, accel);

end:
	return error;
}


uint32_t lsm6ds3_get_g_data_rps(float* gyro)
{
	axis3bit16_t data_raw_angular_rate;
	uint8_t error;
	//	Read acceleration field data
	PROCESS_ERROR(lsm6ds3_angular_rate_raw_get(&lsm6ds3_dev_ctx, data_raw_angular_rate.u8bit));
	gyro[0] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[0]) * MDPS_TO_RAD;
	gyro[1] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[1]) * MDPS_TO_RAD;
	gyro[2] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[2]) * MDPS_TO_RAD;

	//	Apply gyroscope bias and transform matrix
	float offset_vector[3] = {X_GYRO_OFFSET, Y_GYRO_OFFSET, Z_GYRO_OFFSET};
//	float transform_matrix[3][3] =	{{XX_GYRO_TRANSFORM_MATIX, XY_GYRO_TRANSFORM_MATIX, XZ_GYRO_TRANSFORM_MATIX},
//									 {XY_GYRO_TRANSFORM_MATIX, YY_GYRO_TRANSFORM_MATIX, YZ_GYRO_TRANSFORM_MATIX},
//									 {XZ_GYRO_TRANSFORM_MATIX, YZ_GYRO_TRANSFORM_MATIX, ZZ_GYRO_TRANSFORM_MATIX}};

	vmv(gyro, offset_vector, gyro);
//	mxv(transform_matrix, gyro, gyro);

end:
	return error;
}
