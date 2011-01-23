#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "eeprom.h"
#include "analog.h"
#include "main.h"
#include "fc.h"
#ifdef USE_KILLAGREG
#include "mm3.h"
#endif
#ifdef USE_MK3MAG
#include "mk3mag.h"
#endif

volatile uint16_t CountMilliseconds = 0;
volatile uint8_t UpdateMotor = 0;
volatile uint16_t cntKompass = 0;
volatile uint16_t BeepTime = 0;
volatile uint16_t BeepModulation = 0xFFFF;

#ifdef USE_NAVICTRL
volatile uint8_t SendSPI = 0;
#endif



/*****************************************************/
/*              Initialize Timer 0                   */
/*****************************************************/
// timer 0 is used for the PWM generation to control the offset voltage at the air pressure sensor
// Its overflow interrupt routine is used to generate the beep signal and the flight control motor update rate
void TIMER0_Init(void)
{
	uint8_t sreg = SREG;

	// disable all interrupts before reconfiguration
	cli();

	// configure speaker port as output
	if(BoardRelease == 10)
	{	// Speaker at PD2
		DDRD |= (1<<DDD2);
		PORTD &= ~(1<<PORTD2);
	}
	else
	{	// Speaker at PC7
		DDRC |= (1<<DDC7);
		PORTC &= ~(1<<PORTC7);
	}

	// set PB3 and PB4 as output for the PWM used as offset for the pressure sensor
	DDRB |= (1<<DDB4)|(1<<DDB3);
	PORTB &= ~((1<<PORTB4)|(1<<PORTB3));

	// Timer/Counter 0 Control Register A

	// Waveform Generation Mode is Fast PWM (Bits WGM02 = 0, WGM01 = 1, WGM00 = 1)
    // Clear OC0A on Compare Match, set OC0A at BOTTOM, noninverting PWM (Bits COM0A1 = 1, COM0A0 = 0)
    // Clear OC0B on Compare Match, set OC0B at BOTTOM, (Bits COM0B1 = 1, COM0B0 = 0)
    TCCR0A &= ~((1<<COM0A0)|(1<<COM0B0));
    TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);

	// Timer/Counter 0 Control Register B

	// set clock devider for timer 0 to SYSKLOCK/8 = 20MHz / 8 = 2.5MHz
	// i.e. the timer increments from 0x00 to 0xFF with an update rate of 2.5 MHz
	// hence the timer overflow interrupt frequency is 2.5 MHz / 256 = 9.765 kHz

	// divider 8 (Bits CS02 = 0, CS01 = 1, CS00 = 0)
	TCCR0B &= ~((1<<FOC0A)|(1<<FOC0B)|(1<<WGM02));
    TCCR0B = (TCCR0B & 0xF8)|(0<<CS02)|(1<<CS01)|(0<<CS00);

	// initialize the Output Compare Register A & B used for PWM generation on port PB3 & PB4
    OCR0A =  0;  // for PB3
    OCR0B = 120; // for PB4

	// init Timer/Counter 0 Register
    TCNT0 = 0;

	// Timer/Counter 0 Interrupt Mask Register
	// enable timer overflow interrupt only
	TIMSK0 &= ~((1<<OCIE0B)|(1<<OCIE0A));
	TIMSK0 |= (1<<TOIE0);

	SREG = sreg;
}



/*****************************************************/
/*          Interrupt Routine of Timer 0             */
/*****************************************************/
ISR(TIMER0_OVF_vect)    // 9.765 kHz
{
    static uint8_t cnt_1ms = 1,cnt = 0;
    uint8_t Beeper_On = 0;

#ifdef USE_NAVICTRL
	if(SendSPI) SendSPI--; // if SendSPI is 0, the transmit of a byte via SPI bus to and from The Navicontrol is done
#endif

	if(!cnt--) // every 10th run (9.765kHz/10 = 976Hz)
	{
	 cnt = 9;
	 cnt_1ms++;
	 cnt_1ms %= 2;
	 if(!cnt_1ms) UpdateMotor = 1; // every 2nd run (976Hz/2 = 488 Hz)
	 CountMilliseconds++; // increment millisecond counter
	}


	// beeper on if duration is not over
	if(BeepTime)
	{
	   BeepTime--; // decrement BeepTime
	   if(BeepTime & BeepModulation) Beeper_On = 1;
	   else Beeper_On = 0;
	}
	else // beeper off if duration is over
	{
	   Beeper_On = 0;
	   BeepModulation = 0xFFFF;
	}

	// if beeper is on
	if(Beeper_On)
	{
		// set speaker port to high
		if(BoardRelease == 10) PORTD |= (1<<PORTD2); // Speaker at PD2
		else                   PORTC |= (1<<PORTC7); // Speaker at PC7
	}
	else // beeper is off
	{
		// set speaker port to low
		if(BoardRelease == 10) PORTD &= ~(1<<PORTD2);// Speaker at PD2
		else                   PORTC &= ~(1<<PORTC7);// Speaker at PC7
	}

	#ifndef USE_NAVICTRL
	// update compass value if this option is enabled in the settings
	if(ParamSet.GlobalConfig & (CFG_COMPASS_ACTIVE|CFG_GPS_ACTIVE))
	{
	#ifdef USE_KILLAGREG
		MM3_Update(); // read out mm3 board
	#endif
	#ifdef USE_MK3MAG
		MK3MAG_Update(); // read out mk3mag pwm
	#endif
	}
	#endif
}



// -----------------------------------------------------------------------
uint16_t SetDelay (uint16_t t)
{
  return(CountMilliseconds + t - 1);
}

// -----------------------------------------------------------------------
int8_t CheckDelay(uint16_t t)
{
  return(((t - CountMilliseconds) & 0x8000) >> 8); // check sign bit
}

// -----------------------------------------------------------------------
void Delay_ms(uint16_t w)
{
 unsigned int t_stop;
 t_stop = SetDelay(w);
 while (!CheckDelay(t_stop));
}

// -----------------------------------------------------------------------
void Delay_ms_Mess(uint16_t w)
{
 uint16_t t_stop;
 t_stop = SetDelay(w);
 while (!CheckDelay(t_stop)) ADC_Enable();
}

