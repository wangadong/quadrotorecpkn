/*
 * 本文件为写430内部INFO. MEMORY区flash单元的实现
 * 注意：不要使用A段
 * 
 */

#include "Flash.h"

/**
 * 一次写入Flash多个字节.
 * @param dest 目标Flash地址
 * @param src 源地址,可以是Flash,也可以是Ram
 * @param len 数据长度(字节数)
 */
void writeBytesToFlash(unsigned short *dest, unsigned short *src, unsigned char len) {


	FCTL2 = FWKEY + FSSEL_0 + 20; // Flash clock source is MCLK, divisor is 20
	FCTL3 = FWKEY; // Clear Lock bit
	FCTL1 = FWKEY + ERASE; // Set Erase bit
	FCTL1 = FWKEY | WRT; // set WRT for single acces

	*dest = 0;
	for (register unsigned char i = 0; i < len; i++) {
		*dest++ = *src++; // Write value to flash
	}

	FCTL1 = FWKEY; // Clear WRT bit
	FCTL3 = FWKEY | LOCK; // Set LOCK bit
}
