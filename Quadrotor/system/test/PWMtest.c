#include "msp430x22x4.h"

void main(void) {
	WDTCTL = WDTPW + WDTHOLD; // Stop WDT
	P4DIR |= BIT4 + BIT5; // P4.1 and P4.2 output
	P4SEL |= BIT4 + BIT5; // P4.1 and P4.2 TB1/2 otions
	TBCCR0 = 128; // PWM Period/2
	TBCCTL1 = OUTMOD_6; // TBCCR1 toggle/set
	TBCCR1 = 32; // TBCCR1 PWM Duty Cycle
	TBCCTL2 = OUTMOD_6; // TBCCR2 toggle/set
	TBCCR2 = 96; // TBCCR2 PWM duty cycle
	TBCTL = TBSSEL_1 + MC_3; // ACLK, updown mode
	while (1)
		;
}

