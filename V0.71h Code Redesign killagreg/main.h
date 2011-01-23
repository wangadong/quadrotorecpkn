#ifndef _MAIN_H
#define _MAIN_H

#include <avr/io.h>

//set crystal frequency here
#if defined (__AVR_ATmega644__)
#define SYSCLK	20000000L	//crystal freqency in Hz
#endif

#if defined (__AVR_ATmega644P__)
#define SYSCLK	20000000L	//crystal freqency in Hz
#endif

#define F_CPU SYSCLK


// neue Hardware
#define RED_OFF   {if((BoardRelease == 10)||(BoardRelease == 20)) PORTB &=~(1<<PORTB0); else  PORTB |= (1<<PORTB0);}
#define RED_ON    {if((BoardRelease == 10)||(BoardRelease == 20)) PORTB |= (1<<PORTB0); else  PORTB &=~(1<<PORTB0);}
#define RED_FLASH PORTB ^= (1<<PORTB0)
#define GRN_OFF   {if(BoardRelease  < 12) PORTB &=~(1<<PORTB1); else PORTB |= (1<<PORTB1);}
#define GRN_ON    {if(BoardRelease  < 12) PORTB |= (1<<PORTB1); else PORTB &=~(1<<PORTB1);}
#define GRN_FLASH PORTB ^= (1<<PORTB1)

#include <inttypes.h>

extern uint8_t BoardRelease;

#endif //_MAIN_H






