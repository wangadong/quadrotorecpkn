#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

extern volatile uint16_t CountMilliseconds;
extern volatile uint8_t UpdateMotor;
extern volatile uint16_t cntKompass;
extern volatile uint16_t BeepModulation;
extern volatile uint16_t BeepTime;
#ifdef USE_NAVICTRL
extern volatile uint8_t SendSPI;
#endif

extern void TIMER0_Init(void);
extern void Delay_ms(uint16_t w);
extern void Delay_ms_Mess(uint16_t w);
extern uint16_t SetDelay (uint16_t t);
extern int8_t CheckDelay (uint16_t t);

#endif //_TIMER0_H
