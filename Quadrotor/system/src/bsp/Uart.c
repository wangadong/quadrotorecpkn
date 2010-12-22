/*
 * UART驱动机制说明:
 * 1 可以自动发送buf中的所有数据
 * 2 接收时，接收的数据自动存入RXBuf，直到RXBuf满为止
 * 3 RXBuf已满，UART不再接收任何数据，直到APP层取走RXBuf数据为止
 *
 * @author Li Shi
 */

#include "Uart.h"

/* RX接收缓存 */
static unsigned char RXBuf[UART_BUFFER_NUM];

/**
 * 记录当前接收到的字节数
 */
static unsigned char RecNum = 0;

/**
 * Initiate the UART.
 * 1. 初始化UART寄存器，使其工作
 * 2. 通讯速率设置：9600
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
 * 向串口写数据
 * @param *buf 数据指针
 * @param length 要写的数据长度
 */
void writeToUart_BSP(unsigned char* buf, unsigned char length) {
	while (length--) {
		/* Wait for TX buffer ready to receive new byte */
		/* 硬件会在缓冲区发送完毕后自动置标记位 */
		while (!(IFG2 & UCA0TXIFG))
			;

		/* 写入一个要发送的数 */
		UCA0TXBUF = *buf;

		/* 指向下个要发送的数 */
		buf++;
	}
}

/**
 * 从串口读取数据
 * @param *target 读回的内容存放指针，指针指向的内存至少要有4个byte的单元可用
 * @return GETUART_SUCCESS，成功；GETUART_NOT_READY，数据还没准备好。
 */
unsigned char getFromUart_BSP(unsigned char* target) {

	/*TODO by TN:这里看着怪怪的，要再看一下 */
	if (RecNum == UART_BUFFER_NUM) {
		for (register unsigned char i = 0; i < RecNum; i++)
			target[i] = RXBuf[i];
		RecNum = 0; //Clear RxBuf;
		return GETUART_SUCCESS;
	}
	return GETUART_NOT_READY;
}

/**
 * UART 收中断，必须要把UCA0RXBUF中的内容取走，否则新的内容来，系统会死机
 */
BSP_ISR_FUNCTION( BSPRXISR, USCIAB0RX_VECTOR) {
	unsigned char temp;

	/* 先把硬件缓冲区的数取出 */
	temp = UCA0RXBUF;

	if (RecNum < UART_BUFFER_NUM) {
		RXBuf[RecNum] = temp;
		RecNum++;
	}
}

