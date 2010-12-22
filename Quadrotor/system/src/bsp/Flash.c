/*
 * ���ļ�Ϊд430�ڲ�INFO. MEMORY��flash��Ԫ��ʵ��
 * ע�⣺��Ҫʹ��A��
 * 
 */

#include "Flash.h"

/**
 * һ��д��Flash����ֽ�.
 * @param dest Ŀ��Flash��ַ
 * @param src Դ��ַ,������Flash,Ҳ������Ram
 * @param len ���ݳ���(�ֽ���)
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
