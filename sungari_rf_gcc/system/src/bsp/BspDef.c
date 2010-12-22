/*
 * ���ļ�Ϊ�弶�ײ㹦�ܵ�ʵ�֣��������ù���Ƶ�ʣ���װ��ײ����ʱ������
 * 
 */
#include "BspDef.h"

#define BSP_TIMER_CLK_MHZ   (BSP_CONFIG_CLOCK_MHZ_SELECT)

#ifdef SW_TIMER
static unsigned char sIterationsPerUsec = 0;
#endif

static void BSP_InitBoard();

/**
 * �弶��ʼ��(�ṩ����һ��)
 *
 */
void BSP_Init(void) {

	/*
	 * set ACLK = VLO
	 * and set WDT using ACLK for lower frequency.
	 */
	BCSCTL3 |= LFXT1S_2;

	/* Disable watchdog timer */
	WDTCTL = WDTPW | WDTHOLD;

	BSP_InitBoard();

	/* Run time integrity checks.  Perform only if asserts are enabled. */
#ifdef BSP_ASSERTS_ARE_ON
	{
		/* verify endianess is correctly specified */
		unsigned short test = 0x00AA; /* first storage byte of 'test' is non-zero for little endian */
		BSP_ASSERT(!(*((unsigned char *)&test)) == !BSP_LITTLE_ENDIAN); /* endianess mismatch */
	}
#endif
}


/**
 * ��ʼ���弶�ײ��һЩ����: �ڲ�����,ʱ��
 */
static void BSP_InitBoard(void) {
	/* �����ڲ�����Ƶ�� */
	DCOCTL = BSP_CONFIG_MSP430_DCOCTL;
	BCSCTL1 = BSP_CONFIG_MSP430_BCSCTL1;

	/* ��λʱ��A */
	TACTL |= TACLR; /* Set the TACLR */

	/* ��ʼ��ʱ��A�Ŀ��ƼĴ��� */
	TACTL = 0x0;

	/* ��ƾ���Դ*/
	TACTL |= TASSEL_2;

#ifdef SW_TIMER
#define MHZ_CLOCKS_PER_USEC      BSP_CLOCK_MHZ
#define MHZ_CLOCKS_PER_ITERATION 10
	sIterationsPerUsec = (unsigned char)(((MHZ_CLOCKS_PER_USEC)/(MHZ_CLOCKS_PER_ITERATION))+.5);
	if (!sIterationsPerUsec) {
		sIterationsPerUsec = 1;
	}
#endif   /* SW_TIMER */
}

/**
 * ��ʱ��������λ��΢�뼶
 * @param usec Ҫ��ʱ��΢������
 */
void BSP_Delay(unsigned short usec)
#ifndef SW_TIMER
{

	TAR = 0; /* initial count */
	TACCR0 = BSP_TIMER_CLK_MHZ * usec; /* compare count. (delay in ticks) */

	/* Start the timer in UP mode */
	TACTL |= MC_1;

	/* Loop till compare interrupt flag is set */
	while (!(TACCTL0 & CCIFG))
		;

	/* Stop the timer */
	TACTL &= ~(MC_1);

	/* Clear the interrupt flag */
	TACCTL0 &= ~CCIFG;
}

#else  /* !SW_TIMER */

{
	/*
	 *  Declared 'volatile' in case User optimizes for speed. This will
	 *  prevent the optimizer from eliminating the loop completely. But
	 *  it also generates more code...
	 */
	volatile unsigned short repeatCount = sIterationsPerUsec*usec;

	while (repeatCount--);

	return;
}

#endif  /* !SW_TIMER */

/**
 * ���뼶��ʱ����
 * @param ms Ҫ��ʱ�ĺ�������
 */
void delayInMs_BSP(unsigned short ms) {
	while(ms--) {
		BSP_Delay(1000);
	}
}

/**
 * �뼶��ʱ����
 * @param seconds  Ҫ��ʱ��������
 */
void delayInSecond_BSP(unsigned char seconds) {
	while(seconds--) {
		delayInMs_BSP(1000);
	}
}

/* Compile Time Integrity Checks */
BSP_STATIC_ASSERT( sizeof( unsigned char ) == 1 );
BSP_STATIC_ASSERT( sizeof( signed char ) == 1 );
BSP_STATIC_ASSERT( sizeof( unsigned short ) == 2 );
BSP_STATIC_ASSERT( sizeof( signed short ) == 2 );
BSP_STATIC_ASSERT( sizeof( unsigned long ) == 4 );
BSP_STATIC_ASSERT( sizeof( long ) == 4 );
