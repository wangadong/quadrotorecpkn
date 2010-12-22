/*
 * 指示灯的定义和亮、灭功能的实现
 */

#include "Led.h"

#define __bsp_LED_TURN_ON__(bit,port,ddr,low)  \
  st( if (low) { port &= ~BV(bit); } else { port |= BV(bit); } )

#define __bsp_LED_TURN_OFF__(bit,port,ddr,low)  \
  st( if (low) { port |= BV(bit); } else { port &= ~BV(bit); } )

#define __bsp_LED_IS_ON__(bit,port,ddr,low)  \
  ( (low) ? (!((port) & BV(bit))) : ((port) & BV(bit)) )

#define __bsp_LED_TOGGLE__(bit,port,ddr,low)     st( port ^= BV(bit); )
#define __bsp_LED_CONFIG__(bit,port,ddr,low)     st( ddr |= BV(bit); )

/**
 * 点亮所有灯，不管目前状态
 */
void turnOnLeds() {
	__bsp_LED_TURN_ON__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_ON__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * 关掉所有灯，不管目前状态
 */
void turnOffLeds() {
	__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);

}

/**
 * 点亮指定的灯，不管这个 灯目前状态
 * @param led 灯号，宏LED1 or LED2
 */
void turnOnLed(unsigned char led) {
	if (led) {
		__bsp_LED_TURN_ON__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
		return;
	}
	__bsp_LED_TURN_ON__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * 关掉指定的灯，不管这个 灯目前状态
 * @param led 灯号，宏LED1 or LED2
 */
void turnOffLed(unsigned char led) {
	if (led) {
		__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
		return;
	}
	__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * 转换指定的灯
 * @param led 灯号，宏LED1 or LED2
 */
void toggleLed(unsigned char led) {
	if (led) {
		__bsp_LED_TOGGLE__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
		return;
	}
	__bsp_LED_TOGGLE__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * Initialize LED hardware and driver.
 * configure LEDs
 */
void initLeds(void) {
	__bsp_LED_CONFIG__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_CONFIG__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * 让指定的灯闪烁
 * 不管这个灯目前的状态，都是马上灭亮灭亮
 * 退出方法时，一定会让该灯灭。
 * @param led 灯号，宏LED1 or LED2
 * @param times 闪烁次数
 * @param idle 每次亮/灭的持续毫秒数(milliseconds)
 */
void twinkleLed(unsigned char led, unsigned char times, unsigned char idle) {
	FEED_WDT;
	while (times--) {
		turnOffLed(led);
		delayInMs_BSP(idle);
		turnOnLed(led);
		delayInMs_BSP(idle);
	}
	FEED_WDT;
}

/**
 * 出错时提示
 * 用两灯一起 亮一秒灭半秒 重复3次，来表示出现一个错误
 * 要求是阻断的，即在提示出错时主芯片不进行其它工作
 */
void errorLog() {
	FEED_WDT;
	for (register unsigned char i = 0; i < 3; i++) {
		FEED_WDT;
		turnOnLeds();
		delayInMs_BSP(300);
		turnOffLeds();
		delayInMs_BSP(300);
	}
	FEED_WDT;
}

/**
 * 警告提示
 * 两灯交替亮，持续约2秒
 */
void warningLog() {
	turnOnLed(LED1);
	turnOffLed(LED2);
	FEED_WDT;
	for (register unsigned char i = 0; i < 6; i++) {
		FEED_WDT;
		delayInMs_BSP(300);
		toggleLed(LED1);
		toggleLed(LED2);
	}
	FEED_WDT;
}

/**
 * 先两灯一起亮5秒钟表示上电正常，
 * 然后LED2一直亮，LED1按版本号闪烁version次。
 * 然后两灯齐灭，2秒后退出
 */
void showVersion(unsigned char version) {
	turnOffLeds();
	toggleLed(LED1);
	toggleLed(LED2);
	delayInMs_BSP(1000);

	while (version--) {
		toggleLed(LED1);
		delayInMs_BSP(200);
		toggleLed(LED1);
		delayInMs_BSP(200);
	}

	turnOffLeds();
	delayInMs_BSP(1000);

}
