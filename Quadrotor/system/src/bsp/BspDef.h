/*
 * 板卡级驱动的最底层支持
 * 1. 包含基础定义和函数
 * 2. 使用SW_TIMER，但保留了支持HW_TIMER的代码
 */
#ifndef BSP_H
#define BSP_H
#define __GNUC__
/**
 * for mspgcc include.
 */
#ifdef __GNUC__
#include <io.h>
#include <signal.h>
#include <iomacros.h>
#define __bsp_ISTATE_T__ uint16_t
#define __bsp_ISR_FUNCTION__(f,v) interrupt (v) f(void)
#define __bsp_ENABLE_INTERRUPTS__() eint()
#define __bsp_DISABLE_INTERRUPTS__() dint()
#define __bsp_INTERRUPTS_ARE_ENABLED__() (READ_SR & 0x8)
#define __bsp_GET_ISTATE__() (READ_SR & 0x8)
#define __bsp_RESTORE_ISTATE__(x) st(if((x&GIE))_BIS_SR(GIE);)
#endif


/**
 * in sungari, we use 430 2232 or 2012.
 * so, by default, 430x22x2 will be included.
 * when MSP430F2012 defined, 430x20x2 will be included as instead.
 */
#ifdef MSP430F2012
#include <msp430x20x2.h>
#else
#include <msp430x22x2.h>
#endif


#ifndef __GNUC__
/* Initialization call provided in IAR environment before standard C-startup */
#include <intrinsics.h>
#endif

/* used in the whole sungari system, including bsp layer. */
#define TRUE 1
#define FALSE 0

/* 激活对SW_TIMER的使用 */
#define SW_TIMER

/* bit value */
#ifndef BV
#define BV(n)      (1 << (n))
#endif

/*TODO by TN:这是啥意思？*/
#define st(x)      do { x } while (__LINE__ == -1)

#ifdef __GNUC__
/* Critical Sections */
typedef __bsp_ISTATE_T__  bspIState_t;
#else
/* Critical Sections */
typedef istate_t  bspIState_t;
#endif


/* MCU */
#define BSP_MCU_MSP430

#ifndef __GNUC__
/* IAR Compiler */
#define BSP_COMPILER_IAR
#endif

/*TODO by TN: 这句我看不懂，但这个int是什么？*/
#define BSP_EARLY_INIT(void) __intrinsic int __low_level_init(void)
#define __bsp_LITTLE_ENDIAN__   1
#define __bsp_CODE_MEMSPACE__   /* blank */
#define __bsp_XDATA_MEMSPACE__  /* blank */

/**
 *  Supported clock speeds : 1, 2, 4, 6, 8, 10, 12, and 16 MHz.
 *  The clock configuration values for 1, 8, 12, 16 MHz are taken from configuration flash
 *  set at the factory and should be the most accurate.
 *
 *  NOTE!  The clock speeds are approximate as they are derived from an internal
 *         digitally controlled oscillator.
 */
#define BSP_CONFIG_CLOCK_MHZ            8
#define BSP_CONFIG_CLOCK_MHZ_SELECT     8  /* approximate MHz */

#define BSP_CONFIG_MSP430_BCSCTL1   CALBC1_8MHZ   /* factory calibrated value from flash */
#define BSP_CONFIG_MSP430_DCOCTL    CALDCO_8MHZ   /* factory calibrated value from flash */
#define BSP_CLOCK_MHZ   BSP_CONFIG_CLOCK_MHZ
#define BSP_LITTLE_ENDIAN   __bsp_LITTLE_ENDIAN__
#define CODE    __bsp_CODE_MEMSPACE__
#define XDATA   __bsp_XDATA_MEMSPACE__

#ifdef __GNUC__
#define BSP_ISR_FUNCTION(func,vect)   __bsp_ISR_FUNCTION__(func,vect)
#define BSP_ENABLE_INTERRUPTS()        __bsp_ENABLE_INTERRUPTS__()
#define BSP_DISABLE_INTERRUPTS()        __bsp_DISABLE_INTERRUPTS__()
#define BSP_INTERRUPTS_ARE_ENABLED()     __bsp_INTERRUPTS_ARE_ENABLED__()


#define BSP_ENTER_CRITICAL_SECTION(x)   st( x = __bsp_GET_ISTATE__(); __disable_interrupt(); )
#define BSP_EXIT_CRITICAL_SECTION(x)    __bsp_RESTORE_ISTATE__(x)
#define BSP_CRITICAL_STATEMENT(x)       st( bspIState_t s;                    \
                                            BSP_ENTER_CRITICAL_SECTION(s);    \
                                            x;                                \
                                            BSP_EXIT_CRITICAL_SECTION(s); )
#else
#define __bsp_QUOTED_PRAGMA__(x)          _Pragma(#x)
#define BSP_ISR_FUNCTION(func,vect)   __bsp_QUOTED_PRAGMA__(vector=vect) __interrupt void func(void); \
                                      __bsp_QUOTED_PRAGMA__(vector=vect) __interrupt void func(void)
#define BSP_ENABLE_INTERRUPTS()         __enable_interrupt()
#define BSP_DISABLE_INTERRUPTS()        __disable_interrupt()
#define BSP_INTERRUPTS_ARE_ENABLED()    (__get_SR_register() & GIE)


#define BSP_ENTER_CRITICAL_SECTION(x)   st( x = __get_interrupt_state(); __disable_interrupt(); )
#define BSP_EXIT_CRITICAL_SECTION(x)    __set_interrupt_state(x)
#define BSP_CRITICAL_STATEMENT(x)       st( bspIState_t s;                    \
                                            BSP_ENTER_CRITICAL_SECTION(s);    \
                                            x;                                \
                                            BSP_EXIT_CRITICAL_SECTION(s); )
#endif

/**
 *  BSP_ASSERT( expression ) - The given expression must evaluate as "true" or else the assert
 *  handler is called.  From here, the call stack feature of the debugger can pinpoint where
 * the problem occurred.
 *
 *  BSP_FORCE_ASSERT() - If asserts are in use, immediately calls the assert handler.
 *
 *  BSP_ASSERTS_ARE_ON - can use #ifdef to see if asserts are enabled
 *
 *  Asserts can be disabled for optimum performance and minimum code size (ideal for
 *  finalized, debugged production code).
 */
#if (!defined BSP_NO_DEBUG)
#ifndef BSP_ASSERT_HANDLER
#define BSP_ASSERT_HANDLER()      st( __disable_interrupt();  while(1); )
#endif
#define BSP_ASSERT(expr)          st( if (!(expr)) BSP_ASSERT_HANDLER(); )
#define BSP_FORCE_ASSERT()        BSP_ASSERT_HANDLER()
#define BSP_ASSERTS_ARE_ON
#else
#define BSP_ASSERT(expr)          /* empty */
#define BSP_FORCE_ASSERT()        /* empty */
#endif

/* static assert */
#define BSP_STATIC_ASSERT(expr)   void bspDummyPrototype( char dummy[1/((expr)!=0)] )

void BSP_Init(void);

void BSP_Delay(unsigned short);
void delayInMs_BSP(unsigned short);
void delayInSecond_BSP(unsigned char);

#define BSP_DELAY_USECS(x)        BSP_Delay(x)


//ToLearn the concept of ENDIAN, LITTLE ENDIAN. (TN 2010-11-15)
/****************************************************************************************
 *                                 BEGIN ENDIAN SUPPORT
 *
 * Security encrypt/decrypt operates on unsigned long quantities. These must match on
 * source and destination platforms. These macros enforce the standard conversions.
 * Currently all platforms (CC2520/CC2x30 and MSP430) are little endian.
 *
 *******************   Network order for encryption is LITTLE ENDIAN   ******************
 *
 ****************************************************************************************/

#if (BSP_LITTLE_ENDIAN != 0)
#define   ntohs(x)    (x)
#define   htons(x)    (x)
#define   ntohl(x)    (x)
#define   htonl(x)    (x)
#else
#define   ntohs(x)    (((x>>8) & 0xFF) | ((x & 0xFF)<<8))
#define   htons(x)    (((x>>8) & 0xFF) | ((x & 0xFF)<<8))

#define   ntohl(x)    ( ((x>>24) & 0xFF) | ((x>>8) & 0xFF00) | \
                        ((x & 0xFF00)<<8) | ((x & 0xFF)<<24)   \
                      )
#define   htonl(x)    ( ((x>>24) & 0xFF) | ((x>>8) & 0xFF00) | \
                        ((x & 0xFF00)<<8) | ((x & 0xFF)<<24)   \
                      )
#endif  /* (BSP_LITTLE_ENDIAN != 0) */

/* For the usage of Watch Dog Timer.  The feeding dog action. */
/* we defined this macro to simplify code. */
#define FEED_WDT (WDTCTL = WDT_ARST_1000)

#endif
