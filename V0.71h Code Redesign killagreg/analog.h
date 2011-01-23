#ifndef _ANALOG_H
#define _ANALOG_H

#include <inttypes.h>

extern volatile int16_t UBat;
extern volatile int16_t AdValueGyrNick, AdValueGyrRoll, AdValueGyrYaw;
extern uint8_t AnalogOffsetNick, AnalogOffsetRoll, AnalogOffsetYaw;
extern volatile int16_t AdValueAccRoll, AdValueAccNick, AdValueAccTop;
extern volatile int16_t Current_AccZ;
extern volatile int32_t AirPressure;
extern volatile uint16_t MeasurementCounter;
extern int8_t ExpandBaro;
extern uint8_t PressureSensorOffset;
extern volatile int16_t HeightD;
extern volatile uint16_t ReadingAirPressure;
extern volatile int16_t  StartAirPressure;

void SearchAirPressureOffset(void);
void SearchGyroOffset(void);
void ADC_Init(void);

// clear ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define ADC_Disable() (ADCSRA &= ~((1<<ADEN)|(1<<ADSC)|(1<<ADIE)))
// set ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define ADC_Enable() (ADCSRA |= (1<<ADEN)|(1<<ADSC)|(1<<ADIE))


#endif //_ANALOG_H


