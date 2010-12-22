#ifndef _BSP_HYENA_WIRE_H_
#define _BSP_HYENA_WIRE_H_

#include "BspDef.h"
#include "GPIOSimI2CMaster.h"
#include "SHT10.h"
#include "TMP275.h"
#include "Led.h"
#include "SHT10SensorDriver.h"
#include "Power.h"
#include <string.h>

/* 控制电源芯片开关Sensor电源 */
#define SENSOR_POWER_INIT() st(P3DIR |= BIT7;)
#define SENSOR_POWER_ON()   st(P3OUT |= BIT7;)
#define SENSOR_POWER_OFF()  st(P3OUT &= ~BIT7;)

/* 传感器所能存储数据的最大byte数 */
/* 为了保留1byte message id 由原来的26改为25*/
#define BUFFER_LENGTH 25
#define BUFFER_SENSOR_NUMBER_POSITION 0

#define MESSAGE_LENGTH_PER_SHT10	5
#define MESSAGE_LENGTH_PER_TMP275	3

#define FIRST_ADDRESS_OF_SHT10 0x60
#define FIRST_ADDRESS_OF_TMP275 0x90

/**
 * 传感器地址的间隔
 * TMP: 90,92,94...
 * SHT: 60,62,64...
 * 间隔为2
 */
#define SENSOR_ADDRESS_INTERVAL_LENGTH 2

/* 延时>220ms用于A/D转换 */
#define DELAY_FOR_TMP275_CONVERTION 400

/* 延时200ms等待sensor板Powerup并完成一次SHT数据采集*/
#define DELAY_FOR_SENSOR_STARTUP 200


void getSensorValues(unsigned char *buf);
void readSensorsToCosumePowerLeft();
#endif

