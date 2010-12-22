#ifndef BSP_I2CTEMP_H
#define BSP_I2CTEMP_H

#include "BspDef.h"
#include "GPIOSimI2CMaster.h"

unsigned char executeShutdown_TMP275(unsigned char address);
unsigned char executeOneShotDetection_TMP275(unsigned char address);
unsigned char executeContinuousDetection_TMP275(unsigned char address);
unsigned char readTemperature_TMP(unsigned char address,
		unsigned char * bufferPointer);
#endif
