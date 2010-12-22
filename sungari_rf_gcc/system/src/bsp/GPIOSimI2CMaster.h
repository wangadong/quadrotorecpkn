/*
 * The driver of I2C Master that simulated by GPIO.
 *
 * we use the post-fix _GSI2CM to tag this driver.
 * @author Boreal Mao
 */
#ifndef BSP_GPIOI2C_H
#define BSP_GPIOI2C_H

#include "BspDef.h"
#include "Led.h"

unsigned char read_GSI2CM(unsigned char Address, unsigned char* Buffer, unsigned short nBufLen);
unsigned char write_GSI2CM(unsigned char Address, unsigned char* Buffer, unsigned short nBufLen);
void init_GSI2CM(void);

#endif
