/*
 * QuadroUtils.h
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */

#ifndef QUADROUTILS_H_
#define QUADROUTILS_H_

#include "BspDef.h"
#include "Led.h"
#include "SMPL.h"
#include "analog.h"
#include <string.h>


#define QUADRO_VERSION 1

#define INVALID_ADDRESS {0xFF, 0xFF}

#define ROOT_ADDRESS {0xAA, 0x00}

#define RELAY_LOW 0x02


#define COMMAND_MAX_LENGTH_IN_BYTES 14

#define WRITE_TRY_TIMES_IN_SEND 100
/* Program Status */
#define SUCCESS 1
#define FAILURE 0
/* Length of node address */
#define ADDR_LENGTH 2

#define QUADRO_ADDRESS {0xAB,0x00}
#define RECEIVER_ADDRESS {0xAC,0x00}

//ADC msg information
#define AD_GYRO_ROLL_POSITION	0
#define AD_GYRO_NICK_POSITION	2
#define AD_GYRO_YAW_POSITION	4
#define AD_ACC_ROLL_POSITION	6
#define AD_ACC_NICK_POSITION	8
#define AD_ACC_TOP_POSITION		10


static addr_t addrOfSenderR = {INVALID_ADDRESS};
static addr_t addrOfSenderS = {RECEIVER_ADDRESS};
/**
 * addresses of this nodes.
 */
static addr_t addrOfQuadro = QUADRO_ADDRESS;
static unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES] = { 0 };



unsigned char send(addr_t *, unsigned char *);
unsigned char receive(addr_t *, unsigned char *);
void getADValues();
/**
 * 出错提示 using LED
 */
void errorLog(void);

/**
 * 警告提示 using LED
 */
void warningLog(void);

void writeToUart(unsigned char *, unsigned char);
unsigned char getFromUart(unsigned char *);
#endif /* QUADROUTILS_H_ */
