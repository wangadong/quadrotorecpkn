/*
 * QuadroUtils.c
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */
#include "QuadroUtils.h"
/**
 * Send in APP-layer.
 *
 * @param *addr pointer to address.
 * @param *msg pointer to message.
 * @return SUCCESS success to send;FAILURE fail to send.
 */
unsigned char send(addr_t * addr, unsigned char * msg) {
	register unsigned char delayCnt = WRITE_TRY_TIMES_IN_SEND;
	ioctlRawSend_t sendInfo;
	sendInfo.addr = addr;
	sendInfo.msg = msg;
	sendInfo.len = COMMAND_MAX_LENGTH_IN_BYTES;
	sendInfo.port = 0x3F;
	while (delayCnt--) {
		if (SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &sendInfo)
				== SMPL_SUCCESS) {
			return SUCCESS;
		}
		FEED_WDT;
	}
	return FAILURE;
}

/**
 * Receive in APP-layer.
 *
 * @param *addr 从哪个地址收到的包，把来源地址写到这个变量中
 * @param *msg pointer to message.
 * @return SUCCESS success to receive;FAILURE no data to receive.
 */
unsigned char receive(addr_t * addrOfSender, unsigned char * msg) {

	ioctlRawReceive_t receiveInfo;
	receiveInfo.addr = addrOfSender;
	receiveInfo.len = sizeof(msg);
	receiveInfo.port = 0x3F;
	receiveInfo.msg = msg;
	receiveInfo.hopCount = 0;

	if (SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &receiveInfo)
			== SMPL_SUCCESS) {
		return SUCCESS;
	}
	return FAILURE;
}

/**
 * write data to uart in App-layer
 * @para *msg pointer to the message to write
 * 		 length the length of the message
 * @return SUCCESS success to write;FAILURE fail to write.
 */
void writeToUart(unsigned char* msg, unsigned char length) {
	writeToUart_BSP(msg, length);
}

/**
 * get data from uart in App-layer
 * @para *msg pointer to the message
 * (according to Uart.h, msg must point to unsigned char msg[4])
 * @return SUCCESS success to write;FAILURE fail to write.
 */
unsigned char getFromUart(unsigned char* msg) {
	if (getFromUart_BSP(msg) == GETUART_SUCCESS) {
		return SUCCESS;
	} else {
		return FAILURE;
	}
}
