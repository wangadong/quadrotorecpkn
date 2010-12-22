/*
 * ������LED����������
 */

#ifndef __BSP_LEDS_H__
#define __BSP_LEDS_H__

#include "BspDef.h"


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                 LED #1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   Schematic :  D1
 *   Color     :  Green
 *   Polarity  :  Active High
 *   GPIO      :  P1.1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define __bsp_LED1_BIT__            1
#define __bsp_LED1_PORT__           P1OUT
#define __bsp_LED1_DDR__            P1DIR
#define __bsp_LED1_IS_ACTIVE_LOW__  0


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                 LED #2
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   Schematic :  D2
 *   Color     :  Red
 *   Polarity  :  Active High
 *   GPIO      :  P1.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define __bsp_LED2_BIT__            0
#define __bsp_LED2_PORT__           P1OUT
#define __bsp_LED2_DDR__            P1DIR
#define __bsp_LED2_IS_ACTIVE_LOW__  0

/**
 * ָLED1. ���淶���̵�
 * �����޸ģ�������������߼�����
 */
#define LED1	1

/**
 * ָLED2. ���淶�Ǻ��
 * �����޸ģ�������������߼�����
 */
#define LED2	0

void turnOnLeds();
void turnOffLeds();
void turnOnLed(unsigned char led);
void turnOffLed(unsigned char led);
void toggleLed(unsigned char led);
void initLeds(void);
void warningLog(void);
void errorLog(void);
void showVersion(unsigned char);

/**
 * ��ָ���ĵ���˸
 */
void twinkleLed(unsigned char led, unsigned char times, unsigned char idle);


#endif
