#include "msp430x22x4.h"

void main(void){
WDTCTL = WDTPW + WDTHOLD; // Stop WDT
P4DIR |= BIT4; // P1.2 and P1.3 output
P4SEL |= BIT4; // P1.2 and P1.3 TA1/2 otions
TBCCR0 = 128; // PWM Period/2
TBCCTL1 = OUTMOD_6; // TACCR1 toggle/set
TBCCR1 = 96; // TACCR1 PWM duty cycle

TBCTL = TBSSEL_2 + MC_3; // SMCLK, up-down mode
while(1);
}

