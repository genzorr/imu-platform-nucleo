#ifndef SRC_TASKS_TELEMETRY_H_
#define SRC_TASKS_TELEMETRY_H_

#include <stdint.h>

uint8_t msg_state_send(UART_HandleTypeDef *huart);

typedef struct
{
	uint8_t descr1;
	uint8_t descr2;
	float time;

	uint8_t IMU_state;
	uint8_t NRF_state;

	float accel_rsc[3];
	float gyro[3];
	float magn[3];

	float accel_isc[3];
	float quaternion[4];

}__attribute__((packed, aligned(1))) state_msg_t;


#endif /* SRC_TASKS_TELEMETRY_H_ */
