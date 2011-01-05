/**
 * Create by WangXindong
 * ADC Control
 * 2011/1/1
 */
#include "analog.h"


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

/****************************************************
 Initialize Analog Digital Converter
 ****************************************************/
void ADC_Init(void) {
	// disable all interrupts before reconfiguration
	BSP_DISABLE_INTERRUPTS();
	ADC10CTL1 |= INCH_15 + ADC10DIV_7 + ADC10SSEL_0 + CONSEQ_3;
	ADC10CTL0 |= SREF_1 + ADC10SHT_3 + MSC + REF2_5V + REFON + ADC10ON + ADC10IE;
	//Auto Trigger Enable, Prescaler Select Bits to Division Factor 128, i.e. ADC clock = SYSCKL/128 = 156.25 kHz
	ADC10AE0 |= BIT3 + BIT4 + BIT6 + BIT7;
	ADC10AE1 |= BIT4 + BIT7;
	BSP_ENABLE_INTERRUPTS();
	ADC10CTL1 &= ~ADC10IFG;
	// Start AD conversion
	ADC_Enable();
	// restore global interrupt flags
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
{
	FEED_WDT;
	static volatile unsigned int state = 0;

	switch (state++) {
	case 0:
		AdValueAccNick = ADC10MEM;
		break;
	case 1:
		AdValueGyroYaw = ADC10MEM;
		break;
	case 2:
		AdValueGyroNick = ADC10MEM;
		break;
	case 3:
		AdValueGyroRoll = ADC10MEM;
		break;
	case 4:
		AdValueAccRoll = ADC10MEM;
		break;
	case 5:
		AdValueAccTop = ADC10MEM;
		ADReady=1;
		state = 0;
		ADC_Enable()
		break;
	default:
		state = 0;
		ADC_Enable()

	}
}
