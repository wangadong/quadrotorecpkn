/*
 * QuadrotorMain.c
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */

#include "QuadroUtils.h"
int main(void) {

	quadroInit();
	if(ADReady!=0){
		getADValues();
		writeToUart(msg,COMMAND_MAX_LENGTH_IN_BYTES);
		ADReady=0;
	}
}
