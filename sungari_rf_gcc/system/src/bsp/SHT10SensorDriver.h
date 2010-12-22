#ifndef __BSP_SHT1XDRV_H__
#define __BSP_SHT1XDRV_H__

#include "BspDef.h"

#ifdef MSP430F2012
/* for msp430f2012 */
#define SHT_I2C_PxOUT          P1OUT
#define SHT_I2C_PxDIR          P1DIR
#define SHT_I2C_PxIN           P1IN

#define SHT_SDA_PIN            BIT4
#define SHT_SCL_PIN            BIT5

#else

/* for msp430f2232 */
#define SHT_I2C_PxOUT          P4OUT
#define SHT_I2C_PxDIR          P4DIR
#define SHT_I2C_PxIN           P4IN

#define SHT_SDA_PIN            BIT4
#define SHT_SCL_PIN            BIT5

#endif

unsigned char detectTemperature_SHT(unsigned char *, unsigned char *);
unsigned char detectHumidity_SHT(unsigned char *, unsigned char *);
unsigned char checkHumidity_SHT(unsigned char *p_value,
		unsigned char *p_checksum);
unsigned char checkTemperature_SHT(unsigned char *p_value,
		unsigned char *p_checksum);

#endif
