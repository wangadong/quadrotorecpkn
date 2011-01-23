#include <avr/io.h>
#include <stdlib.h>
#include <inttypes.h>
#include "timer0.h"
#include "fc.h"
#include "rc.h"
#include "eeprom.h"
#include "mk3mag.h"

uint8_t PWMTimeout = 12;
ToMk3Mag_t ToMk3Mag;


/*********************************************/
/*  Initialize Interface to MK3MAG Compass   */
/*********************************************/
void MK3MAG_Init(void)
{
	// Port PC4 connected to PWM output from compass module
	DDRC &= ~(1<<DDC4); // set as input
	PORTC |= (1<<PORTC4); // pull up  to increase PWM counter also if nothing is connected

	PWMTimeout = 0;

	ToMk3Mag.CalState = 0;
	ToMk3Mag.Orientation = 1;
}


/*********************************************/
/*  Get PWM from MK3MAG                      */
/*********************************************/
void MK3MAG_Update(void) // called every 102.4 us by timer 0 ISR
{
	static uint16_t PWMCount = 0;
	static uint16_t BeepDelay = 0;
	// The pulse width varies from 1ms (0°) to 36.99ms (359.9°)
	// in other words 100us/° with a +1ms offset.
	// The signal goes low for 65ms between pulses,
	// so the cycle time is 65mS + the pulse width.

	// pwm is high

	if(PINC & (1<<PINC4))
	{	// If PWM signal is high increment PWM high counter
		// This counter is incremented by a periode of 102.4us,
		// i.e. the resoluton of pwm coded heading is approx. 1 deg.
		PWMCount++;
		// pwm overflow?
		if (PWMCount > 400)
		{
			if(PWMTimeout) PWMTimeout--; // decrement timeout
			CompassHeading = -1; // unknown heading
			PWMCount = 0; // reset PWM Counter
		}

	}
	else // pwm is low
	{   // ignore pwm values values of 0 and higher than 37 ms;
		if((PWMCount) && (PWMCount < 362)) // 362 * 102.4us = 37.0688 ms
		{
			if(PWMCount <10) CompassHeading = 0;
			else CompassHeading = ((uint32_t)(PWMCount - 10) * 1049L)/1024; // correct timebase and offset
			CompassOffCourse = ((540 + CompassHeading - CompassCourse) % 360) - 180;
	 		PWMTimeout = 12; // if 12 periodes long no valid PWM was detected the data are invalid
	 		// 12 * 362 counts * 102.4 us
		}
		PWMCount = 0; // reset pwm counter
	}
	if(!PWMTimeout)
	{
		if(CheckDelay(BeepDelay))
		{
			if(!BeepTime) BeepTime = 100; // make noise with 10Hz to signal the compass problem
			BeepDelay = SetDelay(100);
		}
	}
}



