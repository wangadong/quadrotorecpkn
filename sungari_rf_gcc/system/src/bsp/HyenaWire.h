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

/* ���Ƶ�ԴоƬ����Sensor��Դ */
#define SENSOR_POWER_INIT() st(P3DIR |= BIT7;)
#define SENSOR_POWER_ON()   st(P3OUT |= BIT7;)
#define SENSOR_POWER_OFF()  st(P3OUT &= ~BIT7;)

/* ���������ܴ洢���ݵ����byte�� */
/* Ϊ�˱���1byte message id ��ԭ����26��Ϊ25*/
#define BUFFER_LENGTH 25
#define BUFFER_SENSOR_NUMBER_POSITION 0

#define MESSAGE_LENGTH_PER_SHT10	5
#define MESSAGE_LENGTH_PER_TMP275	3

#define FIRST_ADDRESS_OF_SHT10 0x60
#define FIRST_ADDRESS_OF_TMP275 0x90

/**
 * ��������ַ�ļ��
 * TMP: 90,92,94...
 * SHT: 60,62,64...
 * ���Ϊ2
 */
#define SENSOR_ADDRESS_INTERVAL_LENGTH 2

/* ��ʱ>220ms����A/Dת�� */
#define DELAY_FOR_TMP275_CONVERTION 400

/* ��ʱ200ms�ȴ�sensor��Powerup�����һ��SHT���ݲɼ�*/
#define DELAY_FOR_SENSOR_STARTUP 200


void getSensorValues(unsigned char *buf);
void readSensorsToCosumePowerLeft();
#endif

