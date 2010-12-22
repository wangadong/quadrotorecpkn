#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "BspDef.h"

#define SEG_D_ADDR          0x1000      // SegmentD�Ŀ�ʼ��ַ
#define SEG_C_ADDR          0x1040      // SegmentC�Ŀ�ʼ��ַ
#define SEG_B_ADDR          0x1080      // SegmentB�Ŀ�ʼ��ַ
#define SEG_A_ADDR          0x10C0      // SegmentA�Ŀ�ʼ��ַ

void writeBytesToFlash( unsigned short *dest, unsigned short *src, unsigned char len );

#endif
