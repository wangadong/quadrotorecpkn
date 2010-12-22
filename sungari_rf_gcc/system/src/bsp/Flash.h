#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "BspDef.h"

#define SEG_D_ADDR          0x1000      // SegmentD的开始地址
#define SEG_C_ADDR          0x1040      // SegmentC的开始地址
#define SEG_B_ADDR          0x1080      // SegmentB的开始地址
#define SEG_A_ADDR          0x10C0      // SegmentA的开始地址

void writeBytesToFlash( unsigned short *dest, unsigned short *src, unsigned char len );

#endif
