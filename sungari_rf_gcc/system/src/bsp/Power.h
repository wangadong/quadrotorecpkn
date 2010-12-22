#ifndef __BSP_POWER_H__
#define __BSP_POWER_H__

#include "BspDef.h"

/** 
 * ����ʹ��1���ֽڱ�ʾ,���λΪ�ο���ѹ,��7λΪ��������ֵ.
 */
#define PWR_REF_2_5V  0x80        // �ο���ѹ2.5v
#define PWR_REF_1_5V  0x00        // �ο���ѹ1.5v
#define PWR_VAL_MASK  0x7F

/* �����ߵ���ֵ */
#define POWER_LOW 2.4f
#define POWER_HIGH 3.8f

/**
 * �����趨���л��ο���ѹ����ֵΪ 2.8v
 * ���ο���ѹ��Ϊ2.5v������Ŀ��Ϊ2.8vʱ����Ӧԭʼ������0x023C
 *
 * �ڳ�ʼ��ʱ���ο���ѹ��Զ��Ϊ2.5v
 * ������ص����� <= 0x023C�����Ǿͽ��ο���ѹ�л���1.5v���¶���
 */
#define POWER_2_5V_THRESHOLD 0x023C

void initPower_BSP(void);
unsigned short getBatteryMeter_BSP(void);

#endif
