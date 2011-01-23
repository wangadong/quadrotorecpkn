#include <avr/io.h>
#include <avr/interrupt.h>
#include "fc.h"
#include "eeprom.h"
#include "uart.h"
#include "main.h"

volatile int16_t ServoValue = 0;

#define HEF4017R_ON     PORTC |=  (1<<PORTC6)
#define HEF4017R_OFF    PORTC &= ~(1<<PORTC6)


/*****************************************************/
/*              Initialize Timer 2                   */
/*****************************************************/
// The timer 2 is used to generate the PWM at PD7 (J7)
// to control a camera servo for nick compensation.
void TIMER2_Init(void)
{
	uint8_t sreg = SREG;

	// disable all interrupts before reconfiguration
	cli();

	// set PD7 as output of the PWM for nick servo
	DDRD  |= (1<<DDD7);
	PORTD &= ~(1<<PORTD7); 	// set PD7 to low

	DDRC  |= (1<<DDC6);     // set PC6 as output (Reset for HEF4017)
	PORTC &= ~(1<<PORTC6);	// set PC6 to low

	// Timer/Counter 2 Control Register A

	// Waveform Generation Mode is Fast PWM (Bits: WGM22 = 0, WGM21 = 1, WGM20 = 1)
    // PD7: Normal port operation, OC2A disconnected, (Bits: COM2A1 = 0, COM2A0 = 0)
    // PD6: Normal port operation, OC2B disconnected, (Bits: COM2B1 = 0, COM2B0 = 0)
	TCCR2A &= ~((1<<COM2A1)|(1<<COM2A0)|(1<<COM2B1)|(1<<COM2B0));
    TCCR2A |= (1<<WGM21)|(1<<WGM20);

    // Timer/Counter 2 Control Register B

	// Set clock divider for timer 2 to SYSKLOCK/64 = 20MHz / 64 = 312.5 kHz
	// The timer increments from 0x00 to 0xFF with an update rate of 312.5 kHz or 3.2 us
	// hence the timer overflow interrupt frequency is 312.5 kHz / 256 = 1220.7 Hz or 0.8192 ms

    // divider 64 (Bits: CS022 = 1, CS21 = 0, CS20 = 0)
	TCCR2B &= ~((1<<FOC2A)|(1<<FOC2B)|(1<<CS21)|(1<<CS20)|(1<<WGM22));
    TCCR2B |= (1<<CS22);

	// Initialize the Timer/Counter 2 Register
    TCNT2 = 0;

	// Initialize the Output Compare Register A used for PWM generation on port PD7.
	OCR2A = 10;

	// Timer/Counter 2 Interrupt Mask Register
	// Enable timer output compare match A Interrupt only
	TIMSK2 &= ~((1<<OCIE2B)|(1<<TOIE2));
	TIMSK2 |= (1<<OCIE2A);

    SREG = sreg;
}


/*****************************************************/
/*              Control Servo Position               */
/*****************************************************/

ISR(TIMER2_COMPA_vect)  // every  256 * 3.2 us = 0.819 us ( on compare match of TCNT2 and OC2A)
{
	static  uint8_t PostPulse = 0x80;	// value for last pwm cycle in non inverting mode (clear pin on compare match)
	static uint16_t FilterServo = 100; 	// initial value, after some iterations it becomes the average value of 2 * FCParam.ServoNickControl
	static uint16_t ServoState = 40;  	// cycle down counter for this ISR

	#define MULTIPLIER 4

	if(BoardRelease < 99)
	{
		switch(ServoState)
		{
			case 4:
				// recalculate new ServoValue
				ServoValue = 0x0030; // Offset (part 1)
				FilterServo = (3 * FilterServo + (uint16_t)FCParam.ServoNickControl * 2) / 4; // lowpass static offset
				ServoValue += FilterServo; // add filtered static offset
				if(ParamSet.ServoNickCompInvert & 0x01)
				{	// inverting movement of servo
					ServoValue += (int16_t)( ( (int32_t)ParamSet.ServoNickComp * (IntegralNick / 128L ) ) / (512L/MULTIPLIER) );
				}
				else
				{	// non inverting movement of servo
					ServoValue -= (int16_t)( ( (int32_t)ParamSet.ServoNickComp * (IntegralNick / 128L ) ) / (512L/MULTIPLIER) );
				}
				// limit servo value to its parameter range definition
				if(ServoValue < ((int16_t)ParamSet.ServoNickMin * 3) )
				{
					ServoValue = (int16_t)ParamSet.ServoNickMin * 3;
				}
				else
				if(ServoValue > ((int16_t)ParamSet.ServoNickMax * 3) )
				{
					ServoValue = (int16_t)ParamSet.ServoNickMax * 3;
				}
				DebugOut.Analog[20] = ServoValue;

				// determine prepulse width (remaining part of ServoValue/Timer Cycle)
				if ((ServoValue % 255) < 45)
				{	// if prepulse width is to short the execution time of this ISR is longer than the next compare match
					// so balance with postpulse width
					ServoValue += 77;
					PostPulse = 0x60 - 77;
				}
				else
				{
					PostPulse = 0x60;
				}
				// set output compare register to 255 - prepulse width
				OCR2A = 255 - (ServoValue % 256);
				// connect OC2A in inverting mode (Clear pin on overflow, Set pin on compare match)
				TCCR2A=(1<<COM2A1)|(1<<COM2A0)|(1<<WGM21)|(1<<WGM20);

				break;

			case 3:
			case 2:
			case 1:

				if(ServoValue > 255)        // is larger than a full timer 2 cycle
				{
					PORTD |= (1<<PORTD7); 			// set PD7 to high
					TCCR2A = (1<<WGM21)|(1<<WGM20);	// disconnect OC2A
					ServoValue -= 255;          	// substract full timer cycle
				}
				else // the post pule must be generated
				{
					TCCR2A=(1<<COM2A1)|(0<<COM2A0)|(1<<WGM21)|(1<<WGM20); // connect OC2A in non inverting mode
					OCR2A = PostPulse; // Offset Part2
					ServoState = 1;    // jump to ServoState 0 with next ISR call
				}
			break;

			case 0:
				ServoState  = (uint16_t) ParamSet.ServoNickRefresh * MULTIPLIER;	// reload ServoState
				PORTD &= ~(1<<PORTD7); 												// set PD7 to low
				TCCR2A = (1<<WGM21)|(1<<WGM20);             						// disconnect OC2A
				break;

			default:
				// do nothing
				break;
		}
	ServoState--;
	} // EOF BoardRelease < 20
	else // output to HEF4014
	{
		// code for HEF4014 output must be placed here
	}
}

