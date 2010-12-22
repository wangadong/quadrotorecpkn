/*
 * 定义了LED灯所在引脚
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
 * 指LED1. 按规范是绿灯
 * 不可修改，否则将引起程序逻辑错误
 */
#define LED1	1

/**
 * 指LED2. 按规范是红灯
 * 不可修改，否则将引起程序逻辑错误
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
 * 让指定的灯闪烁
 */
void twinkleLed(unsigned char led, unsigned char times, unsigned char idle);


#endif
