#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "BspDef.h"

#define GETUART_SUCCESS     101
#define GETUART_NOT_READY  201

/**
 * buffer length for uart receiving.
 */
#define UART_BUFFER_NUM  30

void initUart(void);
void writeToUart_BSP(unsigned char*, unsigned char length);
unsigned char getFromUart_BSP(unsigned char*);

#endif
