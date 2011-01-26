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

	while (1) {
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);
		if (getFromUart_BSP(msg) == GETUART_SUCCESS) {
			writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
		}
	}
}
