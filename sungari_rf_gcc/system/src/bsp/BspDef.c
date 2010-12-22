/*
 * 本文件为板级底层功能的实现，包括设置工作频率，封装最底层的延时函数等
 * 
 */
#include "BspDef.h"

#define BSP_TIMER_CLK_MHZ   (BSP_CONFIG_CLOCK_MHZ_SELECT)

#ifdef SW_TIMER
static unsigned char sIterationsPerUsec = 0;
#endif

static void BSP_InitBoard();

/**
 * 板级初始化(提供给上一层)
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
 * 初始化板级底层的一些参数: 内部晶振,时钟
 */
static void BSP_InitBoard(void) {
	/* 设置内部晶振频率 */
	DCOCTL = BSP_CONFIG_MSP430_DCOCTL;
	BCSCTL1 = BSP_CONFIG_MSP430_BCSCTL1;

	/* 复位时钟A */
	TACTL |= TACLR; /* Set the TACLR */

	/* 初始化时钟A的控制寄存器 */
	TACTL = 0x0;

	/* 设计晶振源*/
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
 * 延时函数，单位是微秒级
 * @param usec 要延时的微秒数量
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
 * 毫秒级延时函数
 * @param ms 要延时的毫秒数量
 */
void delayInMs_BSP(unsigned short ms) {
	while(ms--) {
		BSP_Delay(1000);
	}
}

/**
 * 秒级延时函数
 * @param seconds  要延时的秒数量
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
