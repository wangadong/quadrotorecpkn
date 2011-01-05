#ifndef _ANALOG_H
#define _ANALOG_H

#include <inttypes.h>
#include "BspDef.h"
/*extern volatile uint16_t MeasurementCounter;
 extern volatile int16_t UBat;
 extern volatile int16_t AdValueGyroNick, AdValueGyroRoll, AdValueGyroYaw;
 #define HIRES_GYRO_AMPLIFY 8 // the offset corrected HiResGyro values are a factor of 8 scaled to the AdValues
 extern volatile int16_t HiResGyroNick, HiResGyroRoll;
 extern volatile int16_t FilterHiResGyroNick, FilterHiResGyroRoll;
 extern volatile int16_t AdValueAccRoll, AdValueAccNick, AdValueAccTop,
 AdValueAccZ;
 extern volatile int32_t AirPressure;
 extern volatile int16_t HeightD;
 extern volatile uint16_t ReadingAirPressure;
 extern volatile int16_t StartAirPressure;
 extern volatile uint8_t ADReady;

 extern uint8_t DacOffsetGyroNick, DacOffsetGyroRoll, DacOffsetGyroYaw;
 extern uint8_t PressureSensorOffset;
 extern int8_t ExpandBaro;

 void SearchAirPressureOffset(void);
 void SearchDacGyroOffset(void);*/
static volatile int AdValueGyroNick = 0, AdValueGyroRoll = 0, AdValueGyroYaw = 0;
static volatile int AdValueAccRoll = 0, AdValueAccNick = 0, AdValueAccTop = 0;
static volatile unsigned char ADReady = 0;
void ADC_Init(void);

// clear ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define ADC_Disable() ADC10CTL0 &=~ ENC + ADC10SC+ADC10IE;
// set ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define ADC_Enable() ADC10CTL0 |= ENC + ADC10SC+ADC10IE;
#endif //_ANALOG_H
