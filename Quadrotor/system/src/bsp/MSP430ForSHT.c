/*
 * Use MSP430F2012 As TMP
 */

#define MSP430F2012
#include "BspDef.h"
#include "Led.h"
#include "HWI2CSlave.h"
#include <string.h>

#define SLAVE_ADDR 0x92

//For Test propose, let it be same as a TMP.
#define BUFFER_LENGTH 2
#define FINISH_FLAG BUFFER_LENGTH+1

unsigned char transmitterBuffer[BUFFER_LENGTH];
unsigned char receiverBuffer[BUFFER_LENGTH + 1];

static void measure();
static unsigned char getCommand();

/**
 * do a measure
 * 1. clear transmitter buffer at first
 * 2. write defined temperature in the transmitter
 */
void measure(void) {
  if(transmitterBuffer[0] <= 0x28 && transmitterBuffer[0] >= 0x12){
	transmitterBuffer[0] = transmitterBuffer[0] + 0x05;
	transmitterBuffer[1] = 0x00;
  }else{
    	transmitterBuffer[0] = 0x12;
	transmitterBuffer[1] = 0x00;
  }
}

/**
 * get content from receiver buffer,
 * if no content, return 0.
 * @return rc: 	0 no content;
 * 				1 ACCESS_TEMERATURE_REG;
 * 				2 ACCESS_CONFIGURATION_REG;
 * 				99 unknown;
 */
unsigned char getCommand(void) {
	unsigned char rc = 0;
	if (!receiverBuffer[FINISH_FLAG - 1]) {
		/* 没有数据 */
		return rc;
	}
	switch (receiverBuffer[0]) {
	case 0x00:
		/* ACCESS_TEMERATURE_REG */
		rc = 1;
		break;
	case 0x01:
		/* ACCESS_CONFIGURATION_REG */
		rc = 2;
		break;
	default:
		rc = 99;
		break;
	}
	receiverBuffer[FINISH_FLAG - 1] = 0x00;
	return rc;
}
/**
 * excute command as tmp
 */
//ToCheck __monitor to critical
critical void executeCommand(void) {
	if (getCommand()) {
		measure();
	}
}

void main(void) {
	__disable_interrupt();

	/* init */
	BSP_Init();
	initLeds();
	turnOnLeds();

	memset(transmitterBuffer, 0xFF, BUFFER_LENGTH);
	memset(receiverBuffer, 0xFF, BUFFER_LENGTH);
	receiverBuffer[FINISH_FLAG - 1] = 0x00;

	initialize_HWI2CSlave(receiverBuffer, 2, transmitterBuffer, BUFFER_LENGTH,
			SLAVE_ADDR);

	turnOffLeds();
	__enable_interrupt(); //Enable interrupt


	while (1) {
		LPM0;
	}
}

