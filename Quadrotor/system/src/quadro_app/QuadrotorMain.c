/*
 * QuadrotorMain.c
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */

#include "QuadroUtils.h"
int main(void) {
	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &addrOfQuadro);
	quadroInit();
	showVersion(5);
	BSP_ENABLE_INTERRUPTS();
	ADC_Enable();
	SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	while (1) {
		/* clear msg */
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
		if (receive(&addrInvalid, msg) == SUCCESS) {
			if (send(&addrOfSender, msg) != SUCCESS) {
			}
			writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
		}
		if (ADReady != 0) {
			//			cal_Sensor_Volt();
			memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);
			getADValues();
			SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
			if (send(&addrOfSender, msg) == SUCCESS) {
			}
			writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
			ADReady = 0;
			ADC_Enable();

		}
	}
}
