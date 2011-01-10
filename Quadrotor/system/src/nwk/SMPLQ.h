/*
 * SMPLQ.h
 *
 *  Created on: 2011-1-5
 *      Author: WANGADONG
 */

#ifndef SMPLQ_H_
#define SMPLQ_H_


#include "BspDef.h"
#include "nwk_api.h"

#include "Led.h"
#include "GPIOSimI2CMaster.h"
#include "Uart.h"
#include "analog.h"
/**
 * 系统总初始化. 整合了所有需要的初始化过程
 * 含板卡，协议
 */
void quadroInit(void);


#endif /* SMPLQ_H_ */
