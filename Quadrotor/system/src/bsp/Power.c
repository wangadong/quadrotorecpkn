/*
 * 测量电压功能
 *
 * 支持两种参考电压的切换，以支持更大的测量范围
 * 自切换只做一次，我们假设在一次上电后的测量过程永远是从高到低
 *
 * @author Taurus Ning, Xu Tao
 */

#include "Power.h"

/**
 * 当前是否使用了默认的参考电压
 * 1：当前使用默认参考电压 (2.5v)
 * 2: 当前使用的是另一参考电压 (1.5v)
 */
static char defaultRef = 1;
unsigned short detectBatteryMeter_BSP();

/**
 * 读出VCC电压
 *
 * @return：电压值
 * power最高位标志参考电压:1:2.5v 0:1.5v
 */
unsigned short getBatteryMeter_BSP(void) {
	unsigned short power = detectBatteryMeter_BSP();

	if(defaultRef) {
		if(power <= POWER_2_5V_THRESHOLD) {
			/* 更改参考电压为1.5v */
			ADC10CTL0 &= ~REF2_5V;

			/* 切换参考电压标志 */
			defaultRef = 0;

			/* 再次读数，并打上1.5标记*/
			power = detectBatteryMeter_BSP();
			power &= 0x7FFF;
		} else {
			power |= 0x8000;
		}
	} else {
		power &= 0x7FFF;
	}
	return power;
}

/**
 * 初始化函数，设置AD转换通道，11通道测量VCC电压
 */
void initPower_BSP(void) {
	/* ADC设置:16 x ADC10CLKs, V(+)=V(Ref),REF on, V(Ref)=2.5V */
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REFON | REF2_5V;

	/* 通道选择A11,使用MSP430内部自带的电路测电量 */
	ADC10CTL1 = INCH_11;
}

/**
 * 基本检测过程，使用当前的设置，读出原始数据
 * @return：电压值
 * power最高位标志参考电压:1-2.5v 0-1.5v
 */
unsigned short detectBatteryMeter_BSP() {
	unsigned short power = 0;

	/* 打开AD,开始测量 */
	ADC10CTL0 |= REFON | ADC10ON | ENC | ADC10SC;

	/* 等待转换结束 */
	while (ADC10CTL1 & ADC10BUSY)
		;

	power = ADC10MEM;

	/* 关闭参考电源、屏蔽模块功能以便省电
	 * turn off A/D to save power */
	ADC10CTL0 &= ~(ENC | REFON | ADC10ON);

	return power;
}
