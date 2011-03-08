/**
 * Create by WangXindong
 * ADC Control
 * 2011/1/1
 */
#include "BspDef.h"
#include "Led.h"
#include "analog.h"
#include "Uart.h"
/*volatile uint16_t Test = 0;

 volatile int16_t UBat = 100;
 volatile int16_t AdValueGyroNick = 0, AdValueGyroRoll = 0, AdValueGyroYaw = 0;
 volatile int16_t FilterHiResGyroNick = 0, FilterHiResGyroRoll = 0;
 volatile int16_t HiResGyroNick = 2500, HiResGyroRoll = 2500;
 volatile int16_t AdValueAccRoll = 0, AdValueAccNick = 0, AdValueAccTop = 0,
 AdValueAccZ = 0;
 volatile int32_t AirPressure = 32000;
 volatile uint8_t average_pressure = 0;
 volatile int16_t StartAirPressure;
 volatile uint16_t ReadingAirPressure = 1023;
 volatile int16_t HeightD = 0;
 volatile uint16_t MeasurementCounter = 0;
 volatile uint8_t ADReady = 1;

 uint8_t DacOffsetGyroNick = 115, DacOffsetGyroRoll = 115, DacOffsetGyroYaw =
 115;
 uint8_t GyroDefectNick = 0, GyroDefectRoll = 0, GyroDefectYaw = 0;
 int8_t ExpandBaro = 0;
 uint8_t PressureSensorOffset;*/
static volatile unsigned char state = MAX_AD_INCH;
static volatile unsigned int counterOfInterrupt = 0;

/****************************************************
 Initialize Analog Digital Converter
 ****************************************************/
void ADC_Init(void) {
	/*
	 // disable all interrupts before reconfiguration
	 ADC10CTL1 |= INCH_15 + ADC10DIV_7 + ADC10SSEL_0 + CONSEQ_3;
	 ADC10CTL0 |= SREF_1 + ADC10SHT_3 + MSC + REF2_5V + REFON + ADC10ON
	 + ADC10IE;
	 //Auto Trigger Enable, Prescaler Select Bits to Division Factor 128, i.e. ADC clock = SYSCKL/128 = 156.25 kHz
	 ADC10AE0 |= BIT3 + BIT4 + BIT6 + BIT7;
	 ADC10AE1 |= BIT4 + BIT7;
	 ADC10CTL1 &= ~ADC10IFG;
	 // Start AD conversion
	 // restore global interrupt flags
	 */
	//	ADC10CTL1 |= ADC10DIV_3 + ADC10SSEL_0 + CONSEQ_0;
	/* ADCÉèÖÃ:16 x ADC10CLKs, V(+)=V(Ref),REF on, V(Ref)=2.5V */
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REFON | REF2_5V | MSC | ADC10IE | ADC10ON;

	/* enable INCH_* analog input */
	ADC10AE0|=BIT3+BIT4+BIT6+BIT7;
	ADC10AE1|=BIT4+BIT7;

	/* ADCÉèÖÃ:INCH=A12, Sequence-of-Channels Mode, ADC10OSC, ADC10SC*/
	ADC10CTL1 = INCH_15 | CONSEQ_1 | ADC10SSEL_0 | SHS_0;

}

/*

 void SearchAirPressureOffset(void) {
 uint8_t off;
 off = GetParamByte(PID_PRESSURE_OFFSET);
 if (off > 20)
 off -= 10;
 OCR0A = off;
 ExpandBaro = 0;
 Delay_ms_Mess(100);
 if (ReadingAirPressure < 850)
 off = 0;
 for (; off < 250; off++) {
 OCR0A = off;
 Delay_ms_Mess(50);
 printf(".");
 if (ReadingAirPressure < 850)
 break;
 }
 SetParamByte(PID_PRESSURE_OFFSET, off);
 PressureSensorOffset = off;
 Delay_ms_Mess(300);
 }

 void SearchDacGyroOffset(void) {
 uint8_t i, ready = 0;
 uint16_t timeout;

 GyroDefectNick = 0;
 GyroDefectRoll = 0;
 GyroDefectYaw = 0;

 timeout = SetDelay(2000);
 if (BoardRelease == 13) // the auto offset calibration is available only at board release 1.3
 {
 for (i = 140; i != 0; i--) {
 if (ready == 3 && i > 10)
 i = 9;
 ready = 0;
 if (AdValueGyroNick < 1020)
 DacOffsetGyroNick--;
 else if (AdValueGyroNick > 1030)
 DacOffsetGyroNick++;
 else
 ready++;
 if (AdValueGyroRoll < 1020)
 DacOffsetGyroRoll--;
 else if (AdValueGyroRoll > 1030)
 DacOffsetGyroRoll++;
 else
 ready++;
 if (AdValueGyroYaw < 1020)
 DacOffsetGyroYaw--;
 else if (AdValueGyroYaw > 1030)
 DacOffsetGyroYaw++;
 else
 ready++;
 I2C_Start(TWI_STATE_GYRO_OFFSET_TX); // initiate data transmission
 if (DacOffsetGyroNick < 10) {
 GyroDefectNick = 1;
 DacOffsetGyroNick = 10;
 };
 if (DacOffsetGyroNick > 245) {
 GyroDefectNick = 1;
 DacOffsetGyroNick = 245;
 };
 if (DacOffsetGyroRoll < 10) {
 GyroDefectRoll = 1;
 DacOffsetGyroRoll = 10;
 };
 if (DacOffsetGyroRoll > 245) {
 GyroDefectRoll = 1;
 DacOffsetGyroRoll = 245;
 };
 if (DacOffsetGyroYaw < 10) {
 GyroDefectYaw = 1;
 DacOffsetGyroYaw = 10;
 };
 if (DacOffsetGyroYaw > 245) {
 GyroDefectYaw = 1;
 DacOffsetGyroYaw = 245;
 };
 while (twi_state) {
 if (CheckDelay(timeout)) {
 printf(
 "\r\n DAC or I2C Error1 check I2C, 3Vref, DAC, and BL-Ctrl");
 break;
 }
 } // wait for end of data transmission
 average_pressure = 0;
 ADC_Enable();
 while (average_pressure == 0)
 ;
 if (i < 10)
 Delay_ms_Mess(10);
 }
 Delay_ms_Mess(70);
 }
 }
 */

/****************************************************
 Interrupt Service Routine for ADC
 ****************************************************/
//roll axis:ENCX---A12
//pitch axis(nick axis):ENCY----A6
//yaw axis:ENCZ---A7
//roll acc---A3
//pitch(nick) acc---A4
//top acc---A15
BSP_ISR_FUNCTION( adcFun, ADC10_VECTOR)
{   counterOfInterrupt++;
switch (state) {
case AD_INCH_A15:
    voltage[AD_INCH_A15] = ADC10MEM;
    state--;
    break;
case AD_INCH_A12:
   voltage[AD_INCH_A12] = ADC10MEM;
   state--;
   break;
case AD_INCH_A7:
    voltage[AD_INCH_A7] = ADC10MEM;
    state--;
    break;
case AD_INCH_A6:
    voltage[AD_INCH_A6] = ADC10MEM;
    state--;
    break;
 case AD_INCH_A3:
    voltage[AD_INCH_A3] = ADC10MEM;
    state--;
    break;
case AD_INCH_A4:
    voltage[AD_INCH_A4] = ADC10MEM;
    state--;
    break;
case AD_INCH_A0:
    state = MAX_AD_INCH;
    ADReady=1;
    break;

default:
    state--;
    break;
}
/*	counterOfInterrupt++;
	switch (state) {
	case AD_INCH_A12:
		AdValueGyroRoll = ADC10MEM;
		state--;
		break;
	case AD_INCH_A7:
		AdValueGyroYaw = ADC10MEM;
		state--;
		break;
	case AD_INCH_A6:
		AdValueGyroNick = ADC10MEM;
		state--;
		break;
	case AD_INCH_A15:
		AdValueAccTop = ADC10MEM;
		state--;
		break;
	case AD_INCH_A4:
		AdValueAccNick = ADC10MEM;
		state--;
		break;
	case AD_INCH_A3:
		AdValueAccRoll = ADC10MEM;
		state--;
		break;
	case 0:
		state = MAX_AD_INCH;
		ADReady = 1;
		break;
	default:
		state--;
		break;

	}*/
}
