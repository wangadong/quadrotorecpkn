/*#######################################################################################
Decodieren eines RC Summen Signals
#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 04.2007 Holger Buss
// + only for non-profit use
// + www.MikroKopter.com
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "rc.h"
#include "main.h"

volatile int16_t PPM_in[15]; //PPM24 supports 12 channels per frame
volatile int16_t PPM_diff[15];
volatile uint8_t NewPpmData = 1;
volatile int16_t RC_Quality = 0;

volatile uint8_t NewRCFrames = 0;


/***************************************************************/
/*  16bit timer 1 is used to decode the PPM-Signal            */
/***************************************************************/
void RC_Init (void)
{
	uint8_t sreg = SREG;

	// disable all interrupts before reconfiguration
	cli();

	// PPM-signal is connected to the Input Capture Pin (PD6) of timer 1
	DDRD &= ~(1<<DDD6);
	PORTD |= (1<<PORTD6);

	// Channel 5,6,7 is decoded to servo signals at pin PD5 (J3), PD4(J4), PD3(J5)
	// set as output
	DDRD |= (1<<DDD5)|(1<<DDD4);
	// low level
	PORTD &= ~((1<<PORTD5)|(1<<PORTD4));

	// PD3 can't be used in FC 1.1 if 2nd UART is activated
	// because TXD1 is at that port
	if(BoardRelease == 10)
	{
		DDRD |= (1<<PORTD3);
		PORTD &= ~(1<<PORTD3);
	}

	// Timer/Counter1 Control Register A, B, C

	// Normal Mode (bits: WGM13=0, WGM12=0, WGM11=0, WGM10=0)
	// Compare output pin A & B is disabled (bits: COM1A1=0, COM1A0=0, COM1B1=0, COM1B0=0)
	// Set clock source to SYSCLK/64 (bit: CS12=0, CS11=1, CS10=1)
	// Enable input capture noise cancler (bit: ICNC1=1)
	// Trigger on positive edge of the input capture pin (bit: ICES1=1),
	// Therefore the counter incremets at a clock of 20 MHz/64 = 312.5 kHz or 3.2µs
    // The longest period is 0xFFFF / 312.5 kHz = 0.209712 s.
	TCCR1A &= ~((1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM11)|(1<<WGM10));
	TCCR1B &= ~((1<<WGM13)|(1<<WGM12)|(1<<CS12));
	TCCR1B |= (1<<CS11)|(1<<CS10)|(1<<ICES1)|(1<<ICNC1);
	TCCR1C &= ~((1<<FOC1A)|(1<<FOC1B));

	// Timer/Counter1 Interrupt Mask Register

	// Enable Input Capture Interrupt (bit: ICIE1=1)
	// Disable Output Compare A & B Match Interrupts (bit: OCIE1B=0, OICIE1A=0)
	// Enable Overflow Interrupt (bit: TOIE1=0)
	TIMSK1 &= ~((1<<OCIE1B)|(1<<OCIE1A));
    TIMSK1 |= (1<<ICIE1)|(1<<TOIE1);

    RC_Quality = 0;

    SREG = sreg;
}


// happens every 0.209712 s.
// check for at least one new frame per timer overflow (timeout)
ISR(TIMER1_OVF_vect)
{
	if (NewRCFrames == 0) RC_Quality -= RC_Quality/8;
	NewRCFrames = 0;
}


/********************************************************************/
/*         Every time a positive edge is detected at PD6            */
/********************************************************************/
/*                               t-Frame
       <----------------------------------------------------------------------->
         ____   ______   _____   ________                ______    sync gap      ____
        |    | |      | |     | |        |              |      |                |
        |    | |      | |     | |        |              |      |                |
     ___|    |_|      |_|     |_|        |_.............|      |________________|
        <-----><-------><------><-------->              <------>                <---
          t0       t1      t2       t4                     tn                     t0

The PPM-Frame length is 22.5 ms.
Channel high pulse width range is 0.7 ms to 1.7 ms completed by an 0.3 ms low pulse.
The mininimum time delay of two events coding a channel is ( 0.7 + 0.3) ms = 1 ms.
The maximum time delay of two events coding a chanel is ( 1.7 + 0.3) ms = 2 ms.
The minimum duration of all channels at minimum value is  8 * 1 ms = 8 ms.
The maximum duration of all channels at maximum value is  8 * 2 ms = 16 ms.
The remaining time of (22.5 - 8 ms) ms = 14.5 ms  to (22.5 - 16 ms) ms = 6.5 ms is
the syncronization gap.
*/
ISR(TIMER1_CAPT_vect) // typical rate of 1 ms to 2 ms
{
    int16_t signal = 0, tmp;
	static int16_t index;
	static uint16_t oldICR1 = 0;

	// 16bit Input Capture Register ICR1 contains the timer value TCNT1
	// at the time the edge was detected

	// calculate the time delay to the previous event time which is stored in oldICR1
	// calculatiing the difference of the two uint16_t and converting the result to an int16_t
	// implicit handles a timer overflow 65535 -> 0 the right way.
	signal = (uint16_t) ICR1 - oldICR1;
	oldICR1 = ICR1;

    //sync gap? (3.52 ms < signal < 25.6 ms)
	if((signal > 1100) && (signal < 8000))
	{
		// if a sync gap happens and there where at least 4 channels decoded before
		// then the NewPpmData flag is reset indicating valid data in the PPM_in[] array.
		if(index >= 4)
		{
			NewPpmData = 0;  // Null means NewData for the first 4 channels
			NewRCFrames++;
		}
		// synchronize channel index
		index = 1;
	}
 	else // within the PPM frame
    {
        if(index < 14) // PPM24 supports 12 channels
        {
			// check for valid signal length (0.8 ms < signal < 2.1984 ms)
			// signal range is from 1.0ms/3.2us = 312 to 2.0ms/3.2us = 625
            if((signal > 250) && (signal < 687))
            {
				// shift signal to zero symmetric range  -154 to 159
                signal -= 466; // offset of 1.4912 ms ??? (469 * 3.2µs = 1.5008 ms)
                // check for stable signal
                if(abs(signal-PPM_in[index]) < 6)
                {
					if(RC_Quality < 200) RC_Quality +=10;
				}
				// calculate exponential history for signal
                tmp = (3 * (PPM_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                // calculate signal difference on good signal level
                if(RC_Quality >= 195)  PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3; // cut off lower 3 bit for nois reduction
                else PPM_diff[index] = 0;
                PPM_in[index] = tmp; // update channel value
            }
            index++; // next channel
            // demux sum signal for channels 5 to 7 to J3, J4, J5
         	if(index == 5) PORTD |= (1<<PORTD5); else PORTD &= ~(1<<PORTD5);
         	if(index == 6) PORTD |= (1<<PORTD4); else PORTD &= ~(1<<PORTD4);
         	if(BoardRelease == 10)
         	{
				if(index == 7) PORTD |= (1<<PORTD3); else PORTD &= ~(1<<PORTD3);
         	}
        }
	}
	if(RC_Quality) RC_Quality--;
}





