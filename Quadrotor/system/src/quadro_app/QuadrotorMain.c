/*
 * QuadrotorMain.c
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */

#include "QuadroUtils.h"
int main(void) {
	quadroInit();
	showVersion(5);
	BSP_ENABLE_INTERRUPTS();
	ADC_Enable();
	while (1) {
		if (ADReady != 0) {
			cal_Sensor_Volt();
			getADValues();
			writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
			ADReady = 0;
			ADC_Enable();

		}
	}
}
