/*
 * ָʾ�ƵĶ�����������ܵ�ʵ��
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
 * �������еƣ�����Ŀǰ״̬
 */
void turnOnLeds() {
	__bsp_LED_TURN_ON__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_ON__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * �ص����еƣ�����Ŀǰ״̬
 */
void turnOffLeds() {
	__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);

}

/**
 * ����ָ���ĵƣ�������� ��Ŀǰ״̬
 * @param led �ƺţ���LED1 or LED2
 */
void turnOnLed(unsigned char led) {
	if (led) {
		__bsp_LED_TURN_ON__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
		return;
	}
	__bsp_LED_TURN_ON__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * �ص�ָ���ĵƣ�������� ��Ŀǰ״̬
 * @param led �ƺţ���LED1 or LED2
 */
void turnOffLed(unsigned char led) {
	if (led) {
		__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
		return;
	}
	__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
}

/**
 * ת��ָ���ĵ�
 * @param led �ƺţ���LED1 or LED2
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
 * ��ָ���ĵ���˸
 * ���������Ŀǰ��״̬������������������
 * �˳�����ʱ��һ�����øõ���
 * @param led �ƺţ���LED1 or LED2
 * @param times ��˸����
 * @param idle ÿ����/��ĳ���������(milliseconds)
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
 * ����ʱ��ʾ
 * ������һ�� ��һ������� �ظ�3�Σ�����ʾ����һ������
 * Ҫ������ϵģ�������ʾ����ʱ��оƬ��������������
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
 * ������ʾ
 * ���ƽ�����������Լ2��
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
 * ������һ����5���ӱ�ʾ�ϵ�������
 * Ȼ��LED2һֱ����LED1���汾����˸version�Ρ�
 * Ȼ����������2����˳�
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
