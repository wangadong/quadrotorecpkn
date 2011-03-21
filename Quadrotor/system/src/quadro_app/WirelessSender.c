/*
 * WirelessSender.c
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */
#define WirelessSender
#include "QuadroUtils.h"

int main(void) {
	unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES] = { 0 };

	/* initial the addr */
	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &addrOfNode);

	/* Board Init */
	QuadroInit();
	showVersion(2);

	/* RX on */
	SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	turnOffLeds();
	while (1) {
		/* clear msg */
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);

		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

		if (getFromUart_BSP(msg) == GETUART_SUCCESS) {
			if (send(&addrOfQuadro, msg) != SUCCESS) {
				warningLog();
			}
		}

		/* clear msg */
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);
		if (receive(&addrInvalid, msg) == SUCCESS) {

			writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
		}
		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	}
}
