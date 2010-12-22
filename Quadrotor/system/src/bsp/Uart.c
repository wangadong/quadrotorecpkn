/*
 * UART��������˵��:
 * 1 �����Զ�����buf�е���������
 * 2 ����ʱ�����յ������Զ�����RXBuf��ֱ��RXBuf��Ϊֹ
 * 3 RXBuf������UART���ٽ����κ����ݣ�ֱ��APP��ȡ��RXBuf����Ϊֹ
 *
 * @author Li Shi
 */

#include "Uart.h"

/* RX���ջ��� */
static unsigned char RXBuf[UART_BUFFER_NUM];

/**
 * ��¼��ǰ���յ����ֽ���
 */
static unsigned char RecNum = 0;

/**
 * Initiate the UART.
 * 1. ��ʼ��UART�Ĵ�����ʹ�乤��
 * 2. ͨѶ�������ã�9600
 */
void initUart(void) {
	P3SEL |= 0x30; // P3.4,5 = USCI_A0 TXD/RXD
	UCA0CTL1 |= UCSSEL_2; // SMCLK
	UCA0BR0 = 0x41; // 8MHz 9600
	UCA0BR1 = 0x03; // 8MHz 9600
	UCA0MCTL = UCBRS0; // Modulation UCBRFx = 0,UCBRSx = 4 see User Manaul Table 15-4
	UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**

	IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt
	IFG2 &= ~UCA0RXIFG;

	RecNum = 0;
}

/**
 * �򴮿�д����
 * @param *buf ����ָ��
 * @param length Ҫд�����ݳ���
 */
void writeToUart_BSP(unsigned char* buf, unsigned char length) {
	while (length--) {
		/* Wait for TX buffer ready to receive new byte */
		/* Ӳ�����ڻ�����������Ϻ��Զ��ñ��λ */
		while (!(IFG2 & UCA0TXIFG))
			;

		/* д��һ��Ҫ���͵��� */
		UCA0TXBUF = *buf;

		/* ָ���¸�Ҫ���͵��� */
		buf++;
	}
}

/**
 * �Ӵ��ڶ�ȡ����
 * @param *target ���ص����ݴ��ָ�룬ָ��ָ����ڴ�����Ҫ��4��byte�ĵ�Ԫ����
 * @return GETUART_SUCCESS���ɹ���GETUART_NOT_READY�����ݻ�û׼���á�
 */
unsigned char getFromUart_BSP(unsigned char* target) {

	/*TODO by TN:���￴�Źֵֹģ�Ҫ�ٿ�һ�� */
	if (RecNum == UART_BUFFER_NUM) {
		for (register unsigned char i = 0; i < RecNum; i++)
			target[i] = RXBuf[i];
		RecNum = 0; //Clear RxBuf;
		return GETUART_SUCCESS;
	}
	return GETUART_NOT_READY;
}

/**
 * UART ���жϣ�����Ҫ��UCA0RXBUF�е�����ȡ�ߣ������µ���������ϵͳ������
 */
BSP_ISR_FUNCTION( BSPRXISR, USCIAB0RX_VECTOR) {
	unsigned char temp;

	/* �Ȱ�Ӳ������������ȡ�� */
	temp = UCA0RXBUF;

	if (RecNum < UART_BUFFER_NUM) {
		RXBuf[RecNum] = temp;
		RecNum++;
	}
}

