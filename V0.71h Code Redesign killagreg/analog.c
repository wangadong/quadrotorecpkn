// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 04.2007 Holger Buss
// + only for non-profit use
// + www.MikroKopter.com
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "analog.h"
#include "main.h"
#include "timer0.h"
#include "fc.h"
#include "printf_P.h"
#include "eeprom.h"
#include "twimaster.h"

volatile int16_t Current_AccZ = 0;
volatile int16_t UBat = 100;
volatile int16_t AdValueGyrNick = 0, AdValueGyrRoll = 0,  AdValueGyrYaw = 0;
uint8_t AnalogOffsetNick = 115, AnalogOffsetRoll = 115, AnalogOffsetYaw = 115;
uint8_t GyroDefectNick = 0, GyroDefectRoll = 0, GyroDefectYaw = 0;
volatile int16_t AdValueAccRoll = 0,  AdValueAccNick = 0, AdValueAccTop = 0;
volatile int32_t AirPressure = 32000;
volatile uint8_t average_pressure = 0;
volatile int16_t StartAirPressure;
volatile uint16_t ReadingAirPressure = 1023;
int8_t ExpandBaro = 0;
uint8_t PressureSensorOffset;
volatile int16_t HeightD = 0;
volatile uint16_t MeasurementCounter = 0;

/*****************************************************/
/*     Initialize Analog Digital Converter           */
/*****************************************************/
void ADC_Init(void)
{
	uint8_t sreg = SREG;
	// disable all interrupts before reconfiguration
	cli();
	//ADC0 ... ADC7 is connected to PortA pin 0 ... 7
	DDRA = 0x00;
	PORTA = 0x00;
	// Digital Input Disable Register 0
	// Disable digital input buffer for analog adc_channel pins
	DIDR0 = 0xFF;
	// external reference, adjust data to the right
    ADMUX &= ~((1 << REFS1)|(1 << REFS0)|(1 << ADLAR));
    // set muxer to ADC adc_channel 0 (0 to 7 is a valid choice)
    ADMUX = (ADMUX & 0xE0) | 0x00;
    //Set ADC Control and Status Register A
    //Auto Trigger Enable, Prescaler Select Bits to Division Factor 128, i.e. ADC clock = SYSCKL/128 = 156.25 kHz
    ADCSRA = (1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//Set ADC Control and Status Register B
	//Trigger Source to Free Running Mode
	ADCSRB &= ~((1 << ADTS2)|(1 << ADTS1)|(1 << ADTS0));
	// Enable AD conversion
	ADC_Enable();
    // restore global interrupt flags
    SREG = sreg;
}

void SearchAirPressureOffset(void)
{
	uint8_t off;
	off = GetParamByte(PID_PRESSURE_OFFSET);
	if(off > 20) off -= 10;
	OCR0A = off;
	ExpandBaro = 0;
	Delay_ms_Mess(100);
	if(ReadingAirPressure < 850) off = 0;
	for(; off < 250;off++)
	{
		OCR0A = off;
		Delay_ms_Mess(50);
		printf(".");
		if(ReadingAirPressure < 850) break;
	}
	SetParamByte(PID_PRESSURE_OFFSET, off);
	PressureSensorOffset = off;
	Delay_ms_Mess(300);
}


void SearchGyroOffset(void)
{
	uint8_t i, ready = 0;

 	GyroDefectNick = 0; GyroDefectRoll = 0; GyroDefectYaw = 0;
 	for(i = 140; i != 0; i--)
  	{
		if(ready == 3 && i > 10) i = 9;
		ready = 0;
		if(AdValueGyrNick < 1020) AnalogOffsetNick--; else if(AdValueGyrNick > 1030) AnalogOffsetNick++; else ready++;
		if(AdValueGyrRoll < 1020) AnalogOffsetRoll--; else if(AdValueGyrRoll > 1030) AnalogOffsetRoll++; else ready++;
		if(AdValueGyrYaw  < 1020) AnalogOffsetYaw-- ; else if(AdValueGyrYaw  > 1030) AnalogOffsetYaw++ ; else ready++;
		twi_state = TWI_STATE_GYRO_OFFSET_TX; // set twi_state in TWI ISR to start of Gyro Offset
		I2C_Start();   // initiate data transmission
		if(AnalogOffsetNick < 10)  { GyroDefectNick = 1; AnalogOffsetNick = 10;}; if(AnalogOffsetNick > 245) { GyroDefectNick = 1; AnalogOffsetNick = 245;};
		if(AnalogOffsetRoll < 10)  { GyroDefectRoll = 1; AnalogOffsetRoll = 10;}; if(AnalogOffsetRoll > 245) { GyroDefectRoll = 1; AnalogOffsetRoll = 245;};
		if(AnalogOffsetYaw  < 10)  { GyroDefectYaw  = 1; AnalogOffsetYaw  = 10;}; if(AnalogOffsetYaw  > 245) { GyroDefectYaw  = 1; AnalogOffsetYaw  = 245;};
		while(twi_state); // wait for end of data transmission
		average_pressure = 0;
		ADC_Enable();
		while(average_pressure == 0);
		if(i < 10) Delay_ms_Mess(10);
	}
	Delay_ms_Mess(70);
}




/*****************************************************/
/*     Interrupt Service Routine for ADC             */
/*****************************************************/
// runs at 156.25 kHz or 6.4 µs
// if after (70.4µs) all 11 states are processed the interrupt is disabled
// and the update of further ads is stopped
// The routine changes the ADC input muxer running
// thru the state machine by the following order.
// state 0: ch0 (yaw gyro)
// state 1: ch1 (roll gyro)
// state 2: ch2 (nick gyro)
// state 3: ch4 (battery voltage -> UBat)
// state 4: ch6 (acc y -> Current_AccY)
// state 5: ch7 (acc x -> Current_AccX)
// state 6: ch0 (yaw gyro average with first reading   -> AdValueGyrYaw)
// state 7: ch1 (roll gyro average with first reading  -> AdValueGyrRoll)
// state 8: ch2 (nick gyro average with first reading -> AdValueGyrNick)
// state 9: ch5 (acc z add also 4th part of acc x and acc y to reading)
// state10: ch3 (air pressure averaging over 5 single readings -> tmpAirPressure)

ISR(ADC_vect)
{
    static uint8_t adc_channel = 0, state = 0;
    static uint16_t yaw1, roll1, nick1;
    static int16_t tmpAirPressure = 0;
    // disable further AD conversion
    ADC_Disable();
    // state machine
    switch(state++)
        {
        case 0:
            yaw1 = ADC; // get Gyro Yaw Voltage 1st sample
            adc_channel = 1; // set next channel to ADC1 = ROLL GYRO
            MeasurementCounter++; // increment total measurement counter
            break;
        case 1:
            roll1 = ADC; // get Gyro Roll Voltage 1st sample
            adc_channel = 2; // set next channel to ADC2 = NICK GYRO
            break;
        case 2:
            nick1 = ADC; // get Gyro Nick Voltage 1st sample
            adc_channel = 4; // set next channel to ADC4 = UBAT
            break;
        case 3:
        	// get actual UBat (Volts*10) is ADC*30V/1024*10 = ADC/3
            UBat = (3 * UBat + ADC / 3) / 4; // low pass filter updates UBat only to 1 quater with actual ADC value
            adc_channel = 6; // set next channel to ADC6 = ACC_Y
            break;
        case 4:
            AdValueAccRoll = NeutralAccY - ADC; // get acceleration in Y direction
            adc_channel = 7; // set next channel to ADC7 = ACC_X
            break;
        case 5:
            AdValueAccNick = ADC - NeutralAccX; // get acceleration in X direction
		    adc_channel = 0; // set next channel to ADC7 = YAW GYRO
            break;
        case 6:
        	// average over two samples to create current AdValueGyrYaw
            if(BoardRelease == 10)  AdValueGyrYaw = (ADC + yaw1) / 2;
            else if (BoardRelease == 20)  AdValueGyrYaw = 1023 - (ADC + yaw1);
			else 					AdValueGyrYaw = ADC + yaw1; // gain is 2 times lower on FC 1.1
            adc_channel = 1; // set next channel to ADC7 = ROLL GYRO
            break;
        case 7:
       		// average over two samples to create current ADValueGyrRoll
            if(BoardRelease == 10)  AdValueGyrRoll = (ADC + roll1) / 2;
			else 					AdValueGyrRoll = ADC + roll1; // gain is 2 times lower on FC 1.1
            adc_channel = 2; // set next channel to ADC2 = NICK GYRO
            break;
        case 8:
        	// average over two samples to create current ADValueNick
            if(BoardRelease == 10)  AdValueGyrNick = (ADC + nick1) / 2;
			else 					AdValueGyrNick = ADC + nick1; // gain is 2 times lower on FC 1.1
            adc_channel = 5; // set next channel to ADC5 = ACC_Z
            break;
       case 9:
       		// get z acceleration
            AdValueAccTop =  (int16_t) ADC - NeutralAccZ; // get plain acceleration in Z direction
            AdValueAccTop += abs(AdValueAccNick) / 4 + abs(AdValueAccRoll) / 4;
            if(AdValueAccTop > 1)
             {
             	if(NeutralAccZ < 750)
             	{
					NeutralAccZ += 0.02;
					if(Model_Is_Flying < 500) NeutralAccZ += 0.1;
				}
             }
             else if(AdValueAccTop < -1)
             {
             	if(NeutralAccZ > 550)
             	{
					NeutralAccZ-= 0.02;
					if(Model_Is_Flying < 500) NeutralAccZ -= 0.1;
				}
             }
            Current_AccZ = ADC;
            Reading_Integral_Top += AdValueAccTop;      // Integrieren
            Reading_Integral_Top -= Reading_Integral_Top / 1024; // dämfen
 	        adc_channel = 3; // set next channel to ADC3 = air pressure
            break;
        case 10:
            tmpAirPressure += ADC; // sum vadc values
            if(++average_pressure >= 5) // if 5 values are summerized for averaging
            {
                ReadingAirPressure = ADC; // update measured air pressure
				HeightD = (7 * HeightD + (int16_t)FCParam.Height_D * (int16_t)(255 * ExpandBaro + StartAirPressure - tmpAirPressure - ReadingHeight))/8;  // D-Part = CurrentValue - OldValue
                AirPressure = (tmpAirPressure + 3 * AirPressure) / 4; // averaging using history
                ReadingHeight = 255 * ExpandBaro + StartAirPressure - AirPressure;
                average_pressure = 0; // reset air pressure measurement counter
                tmpAirPressure = 0;
            }
            adc_channel = 0; // set next channel to ADC0 = GIER GYRO
            state = 0; // reset state machine
            break;
        default:
            adc_channel = 0;
            state = 0;
            break;
        }
    // set adc muxer to next adc_channel
    ADMUX = (ADMUX & 0xE0) | adc_channel;
    // after full cycle stop further interrupts
    if(state != 0) ADC_Enable();
}
