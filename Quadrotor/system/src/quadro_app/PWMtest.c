#include "msp430x22x4.h"

void main(void){
WDTCTL = WDTPW + WDTHOLD; // Stop WDT
P4DIR |= (1<<4); // P1.2 and P1.3 output
P4SEL |= (1<<4); // P1.2 and P1.3 TA1/2 otions
TACCR0 = 128; // PWM Period/2
TACCTL1 = OUTMOD_6; // TACCR1 toggle/set
TACCR1 = 96; // TACCR1 PWM duty cycle

TACTL = TASSEL_2 + MC_3; // SMCLK, up-down mode

__bis_SR_register(LPM0_bits); // Enter LPM0
}

