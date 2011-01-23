#include <inttypes.h>
#include "led.h"
#include "fc.h"
#include "eeprom.h"

uint8_t J16Blinkcount = 0, J16Mask = 1;
uint8_t J17Blinkcount = 0, J17Mask = 1;

// initializes the LED control outputs J16, J17
void LED_Init(void)
{
    // set PC2 & PC3 as output (control of J16 & J17)
	DDRC |= (1<<DDC2)|(1<<DDC3);
	J16_OFF;
	J17_OFF;
	J16Blinkcount = 0; J16Mask = 128;
	J17Blinkcount = 0; J17Mask = 128;
}


// called in main loop every 2ms
void LED_Update(void)
{
	static int8_t delay = 0;

	if(!delay--) // 10 ms intervall
	{
		delay = 4;


		if ((ParamSet.J16Timing > 250) && (FCParam.J16Timing > 230))
		{
			if(ParamSet.J16Bitmask & 128) J16_ON;
			else J16_OFF;
		}
		else if ((ParamSet.J16Timing > 250) && (FCParam.J16Timing <  10))
		{
			if(ParamSet.J16Bitmask & 128) J16_OFF;
			else J16_ON;
		}
		else if(!J16Blinkcount--)
		{
			J16Blinkcount = FCParam.J16Timing - 1;
			if(J16Mask == 1) J16Mask = 128; else J16Mask /= 2;
			if(J16Mask & ParamSet.J16Bitmask) J16_ON; else J16_OFF;
		}

		if ((ParamSet.J17Timing > 250) && (FCParam.J17Timing > 230))
		{
			if(ParamSet.J17Bitmask & 128) J17_ON;
			else J17_OFF;
		}
		else if ((ParamSet.J17Timing > 250) && (FCParam.J17Timing <  10))
		{
			if(ParamSet.J17Bitmask & 128) J17_OFF;
			else J17_ON;
		}
		else if(!J17Blinkcount--)
		{
			J17Blinkcount = FCParam.J17Timing - 1;
			if(J17Mask == 1) J17Mask = 128; else J17Mask /= 2;
			if(J17Mask & ParamSet.J17Bitmask) J17_ON; else J17_OFF;
		}
	}
}
