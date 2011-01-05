/*
 * RSSI测试recevier文件
 * 用于RSSI测试的接收端
 * @author Xu Tao
 */
#include "HyenaUtils.h"
// clear ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define ADC_Disable() ADC10CTL0 &=~ ENC + ADC10SC+ADC10IE;
//// set ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define ADC_Enable() ADC10CTL0 |= ENC + ADC10SC+ADC10IE;
//unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES] = { 0 };
//unsigned char POSITION=1;
static unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES] = { 0 };

static unsigned int POSITION = 1;
void main(void) {
	//	unsigned char writeFlag = 0;

	/* initial the addr */
	//	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &addrOfNode);

	/* Board Init */
	BSP_Init();
	initLeds();
	initUart();
	//	BSP_DISABLE_INTERRUPTS();
	//ADC0 ... ADC7 is connected to PortA pin 0 ... 7
	ADC10CTL1 |= INCH_15 + ADC10DIV_7 + ADC10SSEL_0 + CONSEQ_3;
	ADC10CTL0 |= SREF_1 + ADC10SHT_3 + MSC + REF2_5V + REFON + ADC10ON
			+ ADC10IE;

	//Auto Trigger Enable, Prescaler Select Bits to Division Factor 128, i.e. ADC clock = SYSCKL/128 = 156.25 kHz
	ADC10AE0 |= BIT3 + BIT4 + BIT6 + BIT7;
	ADC10AE1 |= BIT4 + BIT7;
	//	ADC10AE0 |= BIT6 + BIT7;
	//	BSP_ENABLE_INTERRUPTS();
	ADC10CTL1 &= ~ADC10IFG;
	// Start AD conversion

	showVersion(HYENA_VERSION);

	/* RX on */
	//	SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	turnOnLeds();
	BSP_ENABLE_INTERRUPTS();
	ADC10CTL0 |= ENC + ADC10SC;

	while (1) {
		FEED_WDT;
		/*while (!(ADC10IFG & ADC10CTL0))
		 ;

		 turnOnLed(LED1);
		 msg[0] = ADC10MEM >> 8;
		 msg[1] = ADC10MEM;
		 ADC10CTL1 &= ~ADC10IFG;
		 writeToUart_BSP(msg, 2);*/

	}
}
/*BSP_ISR_FUNCTION( adcFun, ADC10_VECTOR) {
 FEED_WDT;
 msg[0] = ADC10MEM >> 8;
 msg[1] = ADC10MEM;
 ADC10CTL1 &= ~ADC10IFG;
 writeToUart_BSP(msg, 2);
 ADC10CTL0 |= ENC + ADC10SC;
 }*/
BSP_ISR_FUNCTION( adcFun, ADC10_VECTOR) {
	static volatile int state = 0;
	static volatile unsigned int gyroyaw, gyroroll, gyronick, accroll, accnick,
			accyaw;

	switch (state++) {
	case 0:
		FEED_WDT;
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);
		accnick = ADC10MEM;
		msg[0] = 'a';
		msg[POSITION] = accnick >> 8;
		msg[POSITION + 1] = accnick;
		POSITION++;
		break;
	case 1:
		FEED_WDT;

		gyroyaw = ADC10MEM;
		msg[POSITION] = gyroyaw >> 8;
		msg[POSITION + 1] = gyroyaw;
		POSITION++;
		break;
	case 2:
		FEED_WDT;

		gyronick = ADC10MEM;
		msg[POSITION] = gyronick >> 8;
		msg[POSITION + 1] = gyronick;
		POSITION++;
		break;
	case 3:
		FEED_WDT;

		gyroroll = ADC10MEM;
		msg[POSITION] = gyroroll >> 8;
		msg[POSITION + 1] = gyroroll;
		POSITION++;
		break;
	case 4:
		FEED_WDT;

		accroll = ADC10MEM;
		msg[POSITION] = accroll >> 8;
		msg[POSITION + 1] = accroll;
		POSITION++;
		break;
	case 5:
		FEED_WDT;

		accyaw = ADC10MEM;
		msg[POSITION] = accyaw >> 8;
		msg[POSITION + 1] = accyaw;
		msg[POSITION + 2] = 'b';
		writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
		POSITION = 1;
		state = 0;
		ADC_Enable()
		break;
	default:
		FEED_WDT;

		state = 0;
		ADC_Enable()

	}
}
