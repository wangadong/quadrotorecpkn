/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Definition and abstraction for radio targets.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */
#ifndef MRFI_DEFS_H
#define MRFI_DEFS_H

#include "BspDef.h"

/* GDO0 Pin Configuration P2.1 */
#define __mrfi_GDO0_BIT__                     1
#define MRFI_CONFIG_GDO0_PIN_AS_INPUT()       st( P2SEL &= ~BV(__mrfi_GDO0_BIT__); ) /* clear pin special function default */
#define MRFI_GDO0_PIN_IS_HIGH()               (P2IN & BV(__mrfi_GDO0_BIT__))

#define MRFI_GDO0_INT_VECTOR                  PORT2_VECTOR
#define MRFI_ENABLE_GDO0_INT()                st( P2IE  |=  BV(__mrfi_GDO0_BIT__); ) /* atomic operation */
#define MRFI_DISABLE_GDO0_INT()               st( P2IE  &= ~BV(__mrfi_GDO0_BIT__); ) /* atomic operation */
#define MRFI_GDO0_INT_IS_ENABLED()             (  P2IE  &   BV(__mrfi_GDO0_BIT__) )
#define MRFI_CLEAR_GDO0_INT_FLAG()            st( P2IFG &= ~BV(__mrfi_GDO0_BIT__); ) /* atomic operation */
#define MRFI_GDO0_INT_FLAG_IS_SET()            (  P2IFG &   BV(__mrfi_GDO0_BIT__) )
#define MRFI_CONFIG_GDO0_RISING_EDGE_INT()    st( P2IES &= ~BV(__mrfi_GDO0_BIT__); ) /* atomic operation */
#define MRFI_CONFIG_GDO0_FALLING_EDGE_INT()   st( P2IES |=  BV(__mrfi_GDO0_BIT__); ) /* atomic operation */


/* GDO2 Pin Configuration P2.2 */
#define __mrfi_GDO2_BIT__                     2
#define MRFI_CONFIG_GDO2_PIN_AS_INPUT()       st( P2SEL &= ~BV(__mrfi_GDO2_BIT__); ) /* clear pin special function default */
#define MRFI_GDO2_PIN_IS_HIGH()               (P2IN & BV(__mrfi_GDO2_BIT__))

#define MRFI_GDO2_INT_VECTOR                  PORT2_VECTOR
#define MRFI_ENABLE_GDO2_INT()                st( P2IE  |=  BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_DISABLE_GDO2_INT()               st( P2IE  &= ~BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_GDO2_INT_IS_ENABLED()             (  P2IE  &   BV(__mrfi_GDO2_BIT__) )
#define MRFI_CLEAR_GDO2_INT_FLAG()            st( P2IFG &= ~BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_GDO2_INT_FLAG_IS_SET()            (  P2IFG &   BV(__mrfi_GDO2_BIT__) )
#define MRFI_CONFIG_GDO2_RISING_EDGE_INT()    st( P2IES &= ~BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_CONFIG_GDO2_FALLING_EDGE_INT()   st( P2IES |=  BV(__mrfi_GDO2_BIT__); ) /* atomic operation */


/* SPI Configuration */

/* CSn Pin Configuration P3.0 */
#define __mrfi_SPI_CSN_GPIO_BIT__             0
#define MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT()   st( P3DIR |=  BV(__mrfi_SPI_CSN_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_CSN_HIGH()             st( P3OUT |=  BV(__mrfi_SPI_CSN_GPIO_BIT__); ) /* atomic operation */
#define MRFI_SPI_DRIVE_CSN_LOW()              st( P3OUT &= ~BV(__mrfi_SPI_CSN_GPIO_BIT__); ) /* atomic operation */
#define MRFI_SPI_CSN_IS_HIGH()                 (  P3OUT &   BV(__mrfi_SPI_CSN_GPIO_BIT__) )

/* SCLK Pin Configuration P3.3 */
#define __mrfi_SPI_SCLK_GPIO_BIT__            3
#define MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()  st( P3DIR |=  BV(__mrfi_SPI_SCLK_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SCLK_HIGH()            st( P3OUT |=  BV(__mrfi_SPI_SCLK_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SCLK_LOW()             st( P3OUT &= ~BV(__mrfi_SPI_SCLK_GPIO_BIT__); )

/* SI Pin Configuration P3.1 */
#define __mrfi_SPI_SI_GPIO_BIT__              1
#define MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT()    st( P3DIR |=  BV(__mrfi_SPI_SI_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SI_HIGH()              st( P3OUT |=  BV(__mrfi_SPI_SI_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SI_LOW()               st( P3OUT &= ~BV(__mrfi_SPI_SI_GPIO_BIT__); )

/* SO Pin Configuration P3.2 */
#define __mrfi_SPI_SO_GPIO_BIT__              2
#define MRFI_SPI_CONFIG_SO_PIN_AS_INPUT()     /* nothing to required */
#define MRFI_SPI_SO_IS_HIGH()                 ( P3IN & BV(__mrfi_SPI_SO_GPIO_BIT__) )

/* SPI Port Configuration */
#define MRFI_SPI_CONFIG_PORT()                st( P3SEL |= BV(__mrfi_SPI_SCLK_GPIO_BIT__) |  \
                                                           BV(__mrfi_SPI_SI_GPIO_BIT__)   |  \
                                                           BV(__mrfi_SPI_SO_GPIO_BIT__); )

/* read/write macros */
#define MRFI_SPI_WRITE_BYTE(x)                st( IFG2 &= ~UCB0RXIFG;  UCB0TXBUF = x; )
#define MRFI_SPI_READ_BYTE()                  UCB0RXBUF
#define MRFI_SPI_WAIT_DONE()                  while(!(IFG2 & UCB0RXIFG));

/* SPI critical section macros */
typedef bspIState_t mrfiSpiIState_t;
#define MRFI_SPI_ENTER_CRITICAL_SECTION(x)    BSP_ENTER_CRITICAL_SECTION(x)
#define MRFI_SPI_EXIT_CRITICAL_SECTION(x)     BSP_EXIT_CRITICAL_SECTION(x)


/*
 *  Radio SPI Specifications
 * -----------------------------------------------
 *    Max SPI Clock   :  10 MHz
 *    Data Order      :  MSB transmitted first
 *    Clock Polarity  :  low when idle
 *    Clock Phase     :  sample leading edge
 */

/* initialization macro */
#define MRFI_SPI_INIT() \
st ( \
  UCB0CTL1 = UCSWRST;                           \
  UCB0CTL1 = UCSWRST | UCSSEL_2;                \
  UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;   \
  UCB0BR0  = 2;                                 \
  UCB0BR1  = 0;                                 \
  MRFI_SPI_CONFIG_PORT();                       \
  UCB0CTL1 &= ~UCSWRST;                         \
)

#define MRFI_SPI_IS_INITIALIZED()         (UCB0CTL0 & UCMST)

#define MRFI_CCA_RETRIES        4

#define MRFI_ASSERT(x)          BSP_ASSERT(x)
#define MRFI_FORCE_ASSERT()     BSP_FORCE_ASSERT()
#define MRFI_ASSERTS_ARE_ON     BSP_ASSERTS_ARE_ON


/* verify that only one supported radio is selected */
#define MRFI_NUM_SUPPORTED_RADIOS_SELECTED   1

#define MRFI_RADIO_FAMILY1

/* Radio States */
#define MRFI_RADIO_STATE_UNKNOWN  0
#define MRFI_RADIO_STATE_OFF      1
#define MRFI_RADIO_STATE_IDLE     2
#define MRFI_RADIO_STATE_RX       3
#define MRFI_RADIO_STATE_WOR      4

 
#define __mrfi_NUM_LOGICAL_CHANS__      4
#define MRFI_NUM_LOGICAL_CHANS          __mrfi_NUM_LOGICAL_CHANS__
#define __mrfi_NUM_POWER_SETTINGS__      3
#define MRFI_NUM_POWER_SETTINGS         __mrfi_NUM_POWER_SETTINGS__

/* return values for MRFI_Transmit */
#define MRFI_TX_RESULT_SUCCESS        0
#define MRFI_TX_RESULT_FAILED         1

/* transmit type parameter for MRFI_Transmit */
#define MRFI_TX_TYPE_FORCED           0
#define MRFI_TX_TYPE_CCA              1

/* Network header size definition */
#define  NWK_HDR_SIZE   3
#define  NWK_PAYLOAD    9            //Defined by LISHI in Jun. 7, 2010

/* if external code has defined a maximum payload, use that instead of default */
#define MAX_PAYLOAD     30             //Defined by LISHI in Jun. 7, 2010

#define MRFI_MAX_PAYLOAD_SIZE  (MAX_PAYLOAD+NWK_HDR_SIZE) /* SimpliciTI payload size plus six byte overhead */

#define MRFI_ADDR_SIZE              2

#define __mrfi_LENGTH_FIELD_SIZE__      1
#define __mrfi_HEADER_SIZE__            (2 * MRFI_ADDR_SIZE)
#define __mrfi_FRAME_OVERHEAD_SIZE__    (__mrfi_LENGTH_FIELD_SIZE__ + __mrfi_HEADER_SIZE__)

/* frame definitions */
#define MRFI_MAX_FRAME_SIZE         (MRFI_MAX_PAYLOAD_SIZE + __mrfi_FRAME_OVERHEAD_SIZE__)


#define MRFI_RX_METRICS_SIZE        2
#define MRFI_RX_METRICS_RSSI_OFS    0
#define MRFI_RX_METRICS_CRC_LQI_OFS 1

#define __mrfi_RX_METRICS_CRC_OK_MASK__ 0x80
#define __mrfi_RX_METRICS_LQI_MASK__    0x7F


#define __mrfi_BACKOFF_PERIOD_USECS__   250

/* Platform constant used to calculate worst-case for an application
 * acknowledgment delay. Used in the NWK_REPLY_DELAY() macro.
 *

                                      processing time on peer
                                      |   round trip
                                      |   |      max number of replays
                                      |   |      |             number of backoff opportunities
                                      |   |      |             |         average number of backoffs
                                      |   |      |             |         |                                    */
#define   PLATFORM_FACTOR_CONSTANT   (2 + 2*(    1 *(MRFI_CCA_RETRIES*(8*MRFI_BACKOFF_PERIOD_USECS)/1000)))



#define __mrfi_LENGTH_FIELD_OFS__       0
#define __mrfi_DST_ADDR_OFS__           (__mrfi_LENGTH_FIELD_OFS__ + __mrfi_LENGTH_FIELD_SIZE__)
#define __mrfi_SRC_ADDR_OFS__           (__mrfi_DST_ADDR_OFS__ + MRFI_ADDR_SIZE)
#define __mrfi_PAYLOAD_OFS__            (__mrfi_SRC_ADDR_OFS__ + MRFI_ADDR_SIZE)


#define MRFI_GET_PAYLOAD_LEN(p)         ((p)->frame[__mrfi_LENGTH_FIELD_OFS__] - __mrfi_HEADER_SIZE__)
#define MRFI_SET_PAYLOAD_LEN(p,x)       st( (p)->frame[__mrfi_LENGTH_FIELD_OFS__] = x + __mrfi_HEADER_SIZE__; )


#define MRFI_P_DST_ADDR(p)              (&((p)->frame[__mrfi_DST_ADDR_OFS__]))
#define MRFI_P_SRC_ADDR(p)              (&((p)->frame[__mrfi_SRC_ADDR_OFS__]))
#define MRFI_P_PAYLOAD(p)               (&((p)->frame[__mrfi_PAYLOAD_OFS__]))

#endif
