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
		//		FEED_WDT;
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
void cal_Sensor_Volt() {
	AdValueAccTop = voltage[AD_INCH_A15]*0.0024414;
	AdValueGyroRoll = voltage[AD_INCH_A12]*0.0024414;
	AdValueGyroYaw = voltage[AD_INCH_A7]*0.0024414;
	AdValueGyroNick = voltage[AD_INCH_A6]*0.0024414;
	AdValueAccNick = voltage[AD_INCH_A4] *0.0024414;
	AdValueAccRoll = voltage[AD_INCH_A3]*0.0024414;
}
void getADValues() {
	msg[MsgBegin] = 'B';
	msg[AD_ACC_NICK_POSITION] = voltage[AD_INCH_A3] >> 8;
	msg[AD_ACC_NICK_POSITION + 1] = voltage[AD_INCH_A3];
	msg[AD_GYRO_YAW_POSITION] = voltage[AD_INCH_A7] >> 8;
	msg[AD_GYRO_YAW_POSITION + 1] = voltage[AD_INCH_A7];
	msg[AD_GYRO_NICK_POSITION] = voltage[AD_INCH_A6] >> 8;
	msg[AD_GYRO_NICK_POSITION + 1] = voltage[AD_INCH_A6];
	msg[AD_GYRO_ROLL_POSITION] = voltage[AD_INCH_A12] >> 8;
	msg[AD_GYRO_ROLL_POSITION + 1] = voltage[AD_INCH_A12];
	msg[AD_ACC_ROLL_POSITION] = voltage[AD_INCH_A4] >> 8;
	msg[AD_ACC_ROLL_POSITION + 1] = voltage[AD_INCH_A4];
	msg[AD_ACC_TOP_POSITION] = voltage[AD_INCH_A15] >> 8;
	msg[AD_ACC_TOP_POSITION + 1] = voltage[AD_INCH_A15];
	msg[MsgEnd] = 'E';
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
