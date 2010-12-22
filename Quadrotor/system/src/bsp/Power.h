#ifndef __BSP_POWER_H__
#define __BSP_POWER_H__

#include "BspDef.h"

/** 
 * 电量使用1个字节表示,最高位为参考电压,低7位为真正的数值.
 */
#define PWR_REF_2_5V  0x80        // 参考电压2.5v
#define PWR_REF_1_5V  0x00        // 参考电压1.5v
#define PWR_VAL_MASK  0x7F

/* 电量高低阈值 */
#define POWER_LOW 2.4f
#define POWER_HIGH 3.8f

/**
 * 我们设定的切换参考电压的域值为 2.8v
 * 当参考电压设为2.5v，测量目标为2.8v时，对应原始数据是0x023C
 *
 * 在初始化时，参考电压永远设为2.5v
 * 如果读回的数据 <= 0x023C，我们就将参考电压切换到1.5v重新读数
 */
#define POWER_2_5V_THRESHOLD 0x023C

void initPower_BSP(void);
unsigned short getBatteryMeter_BSP(void);

#endif
