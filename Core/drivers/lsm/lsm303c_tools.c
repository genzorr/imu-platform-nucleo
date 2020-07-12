#include <stdint.h>
#include <stdio.h>
#include <stm32f3xx.h>

#include "lis3mdl_reg.h"
#include "state.h"
#include "vector.h"


#define LSM_TIMEOUT	1
#define MDPS_TO_RAD	M_PI / 180 / 1000

//	Magnetometer bias & transform matrix
#define X_MAGN_OFFSET		 -189.064391
#define Y_MAGN_OFFSET		 108.525582
#define Z_MAGN_OFFSET		 136.961069
#define XX_MAGN_TRANSFORM_MATIX	 1.495283
#define YY_MAGN_TRANSFORM_MATIX	 1.472561
#define ZZ_MAGN_TRANSFORM_MATIX	 1.529185
#define XY_MAGN_TRANSFORM_MATIX	 0.041707
#define XZ_MAGN_TRANSFORM_MATIX	 0.041062
#define YZ_MAGN_TRANSFORM_MATIX	 -0.011496


static uint8_t whoamI, rst;

I2C_HandleTypeDef*	i2c_lsm303c;
lis3mdl_ctx_t 		lsm303c_dev_ctx;

int32_t lsm303c_bus_init(void* handle);
int32_t lsm303c_platform_init(void);

uint32_t lsm303c_get_m_data_mG(float* magn);



static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == i2c_lsm303c)
	{
		/* Write multiple command */
		reg |= 0x80;
		error = HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else
	{
		printf("lsm303c invalid handle\n");
		error = -19;
	}

	return error;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == i2c_lsm303c)
	{
		/* Write multiple command */
		reg |= 0x80;
		error = HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else
	{
		printf("lsm303c invalid handle\n");
		error = -19;
	}

	return error;
}


int32_t lsm303c_bus_init(void* handle)
{
	int error = 0;
	if (handle == i2c_lsm303c)
	{
		// Initialized by Cube.
	}
	else
	{
		printf("invalid lsm303c handle\n");
		error = -19;
	}

	return error;
}


int32_t lsm303c_platform_init(void)
{
	int error = 0;

	lsm303c_dev_ctx.write_reg = platform_write;
	lsm303c_dev_ctx.read_reg = platform_read;
	lsm303c_dev_ctx.handle = i2c_lsm303c;

	//	Set needed bus parameters
	error |= lsm303c_bus_init(lsm303c_dev_ctx.handle);

	// Reset to defaults
	error |= lis3mdl_reset_set(&lsm303c_dev_ctx, PROPERTY_ENABLE);
	do {
		error = lis3mdl_reset_get(&lsm303c_dev_ctx, &rst);
	} while (rst);

	// Check who_am_i
	error |= lis3mdl_device_id_get(&lsm303c_dev_ctx, &whoamI);
	if (whoamI != LIS3MDL_ID)
	{
		printf("lsm303c not found, %d\terror: %d\n", whoamI, error);
		return -19;
	}
	else
		printf("lsm303c OK\n");

	error |= lis3mdl_block_data_update_set(&lsm303c_dev_ctx, PROPERTY_DISABLE);

	error |= lis3mdl_data_rate_set(&lsm303c_dev_ctx, LIS3MDL_HP_300Hz);

	error |= lis3mdl_full_scale_set(&lsm303c_dev_ctx, LIS3MDL_16_GAUSS);

	error |= lis3mdl_operating_mode_set(&lsm303c_dev_ctx, LIS3MDL_CONTINUOUS_MODE);

	return error;
}


uint32_t lsm303c_get_m_data_mG(float* magn)
{
	axis3bit16_t data_raw_magnetic;
	uint8_t error;
	//	Read data
	PROCESS_ERROR(lis3mdl_magnetic_raw_get(&lsm303c_dev_ctx, data_raw_magnetic.u8bit));
	magn[0] = 1000 * LIS3MDL_FROM_FS_16G_TO_G(data_raw_magnetic.i16bit[0]);
	magn[1] = 1000 * LIS3MDL_FROM_FS_16G_TO_G(data_raw_magnetic.i16bit[1]);
	magn[2] = 1000 * LIS3MDL_FROM_FS_16G_TO_G(data_raw_magnetic.i16bit[2]);

	if (!IMU_CALIBRATION)
	{
		//	Magnetometer bias and transform matrix (to provide real values)
		float offset_vector[3] = {X_MAGN_OFFSET, Y_MAGN_OFFSET, Z_MAGN_OFFSET};
		float transform_matrix[3][3] =	{	{XX_MAGN_TRANSFORM_MATIX, XY_MAGN_TRANSFORM_MATIX, XZ_MAGN_TRANSFORM_MATIX},
											{XY_MAGN_TRANSFORM_MATIX, YY_MAGN_TRANSFORM_MATIX, YZ_MAGN_TRANSFORM_MATIX},
											{XZ_MAGN_TRANSFORM_MATIX, YZ_MAGN_TRANSFORM_MATIX, ZZ_MAGN_TRANSFORM_MATIX}};

		vmv(magn, offset_vector, magn);
		mxv(transform_matrix, magn, magn);

		//	Change axes to be like in accelerometer
		float tmp = magn[0];
		magn[0] = -magn[1];
		magn[1] = -tmp;
	}

end:
	return error;
}
