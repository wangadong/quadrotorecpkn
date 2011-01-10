/*
 * SMPLQ.c
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */

#include "SMPLQ.h"

void quadroInit(void) {
	BSP_Init();
	initLeds();
	initUart();
	ADC_Init();
}
