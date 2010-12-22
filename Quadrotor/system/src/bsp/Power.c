/*
 * ������ѹ����
 *
 * ֧�����ֲο���ѹ���л�����֧�ָ���Ĳ�����Χ
 * ���л�ֻ��һ�Σ����Ǽ�����һ���ϵ��Ĳ���������Զ�ǴӸߵ���
 *
 * @author Taurus Ning, Xu Tao
 */

#include "Power.h"

/**
 * ��ǰ�Ƿ�ʹ����Ĭ�ϵĲο���ѹ
 * 1����ǰʹ��Ĭ�ϲο���ѹ (2.5v)
 * 2: ��ǰʹ�õ�����һ�ο���ѹ (1.5v)
 */
static char defaultRef = 1;
unsigned short detectBatteryMeter_BSP();

/**
 * ����VCC��ѹ
 *
 * @return����ѹֵ
 * power���λ��־�ο���ѹ:1:2.5v 0:1.5v
 */
unsigned short getBatteryMeter_BSP(void) {
	unsigned short power = detectBatteryMeter_BSP();

	if(defaultRef) {
		if(power <= POWER_2_5V_THRESHOLD) {
			/* ���Ĳο���ѹΪ1.5v */
			ADC10CTL0 &= ~REF2_5V;

			/* �л��ο���ѹ��־ */
			defaultRef = 0;

			/* �ٴζ�����������1.5���*/
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
 * ��ʼ������������ADת��ͨ����11ͨ������VCC��ѹ
 */
void initPower_BSP(void) {
	/* ADC����:16 x ADC10CLKs, V(+)=V(Ref),REF on, V(Ref)=2.5V */
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REFON | REF2_5V;

	/* ͨ��ѡ��A11,ʹ��MSP430�ڲ��Դ��ĵ�·����� */
	ADC10CTL1 = INCH_11;
}

/**
 * ���������̣�ʹ�õ�ǰ�����ã�����ԭʼ����
 * @return����ѹֵ
 * power���λ��־�ο���ѹ:1-2.5v 0-1.5v
 */
unsigned short detectBatteryMeter_BSP() {
	unsigned short power = 0;

	/* ��AD,��ʼ���� */
	ADC10CTL0 |= REFON | ADC10ON | ENC | ADC10SC;

	/* �ȴ�ת������ */
	while (ADC10CTL1 & ADC10BUSY)
		;

	power = ADC10MEM;

	/* �رղο���Դ������ģ�鹦���Ա�ʡ��
	 * turn off A/D to save power */
	ADC10CTL0 &= ~(ENC | REFON | ADC10ON);

	return power;
}
