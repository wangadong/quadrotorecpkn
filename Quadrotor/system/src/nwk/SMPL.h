#ifndef __SMPL_H__
#define __SMPL_H__

#include "nwk_api.h"

#include "Led.h"
#include "Flash.h"
#include "Power.h"
#include "GPIOSimI2CMaster.h"
#include "Uart.h"

/**
 * 系统总初始化. 整合了所有需要的初始化过程
 * 含板卡，协议
 */
void hyenaInit(void);


#endif
