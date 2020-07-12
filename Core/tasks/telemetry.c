#include <stdint.h>
#include <stdio.h>

#include "state.h"
#include "telemetry.h"

uint8_t msg_state_send(UART_HandleTypeDef *huart)
{
	uint8_t error = 0;

	state_msg_t msg_state;
	msg_state.descr1 = 0x0A;
	msg_state.descr2 = 0x0A;

	msg_state.time = (float)HAL_GetTick() / 1000;

	msg_state.IMU_state = state_system.MPU_state;
	msg_state.NRF_state = state_system.NRF_state;

	for (int i = 0; i < 3; i++)
	{
		msg_state.accel_rsc[i] = stateIMU_rsc.accel[i];
		msg_state.gyro[i] = stateIMU_rsc.gyro[i];
		msg_state.magn[i] = stateIMU_rsc.magn[i];
		msg_state.accel_isc[i] = stateIMU_isc.accel[i];
	}
	for (int j = 0; j < 4; j++)
		msg_state.quaternion[j] = stateIMU_isc.quaternion[j];

	error |= HAL_UART_Transmit(huart, (uint8_t*)(&msg_state), sizeof(msg_state), 6);

	return error;
}

