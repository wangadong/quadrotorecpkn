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

static addr_t addrOfSenderR = INVALID_ADDRESS;
static addr_t addrOfSenderS = RSSI_RECEIVER_ADDRESS;
/**
 * addresses of this nodes.
 */
static addr_t addrOfQuadro = QUADRO_ADDRESS;

unsigned char send(addr_t *, unsigned char *);
unsigned char receive(addr_t *, unsigned char *);

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
