// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Konstanten
// + 0-250 -> normale Werte
// + 251 -> Poti1
// + 252 -> Poti2
// + 253 -> Poti3
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef EEMEM
#define EEMEM __attribute__ ((section (".eeprom")))
#endif


#include <avr/eeprom.h>
#include <string.h>
#include "eeprom.h"
#include "printf_P.h"
#include "led.h"


// byte array in eeprom
uint8_t EEPromArray[E2END+1] EEMEM;

paramset_t ParamSet;



/***************************************************/
/*    Default Values for parameter set 1           */
/***************************************************/
void ParamSet_DefaultSet1(void) // sport
{
	ParamSet.ChannelAssignment[CH_NICK]  = 1;
	ParamSet.ChannelAssignment[CH_ROLL]  = 2;
	ParamSet.ChannelAssignment[CH_GAS]   = 3;
	ParamSet.ChannelAssignment[CH_YAW]  = 4;
	ParamSet.ChannelAssignment[CH_POTI1] = 5;
	ParamSet.ChannelAssignment[CH_POTI2] = 6;
	ParamSet.ChannelAssignment[CH_POTI3] = 7;
	ParamSet.ChannelAssignment[CH_POTI4] = 8;
	ParamSet.GlobalConfig = CFG_AXIS_COUPLING_ACTIVE | CFG_COMPASS_ACTIVE | CFG_GPS_ACTIVE;//CFG_HEIGHT_CONTROL | CFG_HEIGHT_SWITCH | CFG_COMPASS_FIX;
	ParamSet.Height_MinGas = 30;
	ParamSet.MaxHeight     = 251;      // Wert : 0-250   251 -> Poti1
	ParamSet.Height_P      = 10;       // Wert : 0-32
	ParamSet.Height_D  = 30;       // Wert : 0-250
	ParamSet.Height_ACC_Effect = 30;  // Wert : 0-250
	ParamSet.Height_Gain = 4;  // Wert : 0-50
	ParamSet.Stick_P = 15;         // Wert : 1-24
	ParamSet.Stick_D = 30;         // Wert : 0-250
	ParamSet.Yaw_P = 12;             // Wert : 1-20
	ParamSet.Gas_Min = 8;            // Wert : 0-32
	ParamSet.Gas_Max = 230;           // Wert : 33-250
	ParamSet.GyroAccFactor = 30;      // Wert : 1-64
	ParamSet.CompassYawEffect = 128;    // Wert : 0-250
	ParamSet.Gyro_P = 80;             // Wert : 0-250
	ParamSet.Gyro_I = 150;               // Wert : 0-250
	ParamSet.LowVoltageWarning = 94;	// Wert : 0-250
	ParamSet.EmergencyGas = 35;                // Wert : 0-250     // Gaswert bei Empangsverlust
	ParamSet.EmergencyGasDuration = 30;            // Wert : 0-250     // Zeit bis auf EmergencyGas geschaltet wird, wg. Rx-Problemen
	ParamSet.UfoArrangement = 0;         // X oder + Formation
	ParamSet.I_Factor = 32;
	ParamSet.UserParam1 = 0;             //zur freien Verwendung
	ParamSet.UserParam2 = 0;             //zur freien Verwendung
	ParamSet.UserParam3 = 0;             //zur freien Verwendung
	ParamSet.UserParam4 = 0;             //zur freien Verwendung
	ParamSet.UserParam5 = 0;             // zur freien Verwendung
	ParamSet.UserParam6 = 0;             // zur freien Verwendung
	ParamSet.UserParam7 = 0;             // zur freien Verwendung
	ParamSet.UserParam8 = 0;             // zur freien Verwendung
	ParamSet.ServoNickControl = 100;     // Wert : 0-250     // Stellung des Servos
	ParamSet.ServoNickComp = 40;         // Wert : 0-250     // Einfluss Gyro/Servo
	ParamSet.ServoNickCompInvert = 0;    // Wert : 0-250     // Richtung Einfluss Gyro/Servo
	ParamSet.ServoNickMin = 50;          // Wert : 0-250     // Anschlag
	ParamSet.ServoNickMax = 150;         // Wert : 0-250     // Anschlag
	ParamSet.ServoNickRefresh = 5;
	ParamSet.LoopGasLimit = 50;
	ParamSet.LoopThreshold = 90;         // Wert: 0-250  Schwelle für Stickausschlag
	ParamSet.LoopHysteresis = 50;
	ParamSet.BitConfig = 0;             // Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts / wird getrennt behandelt
	ParamSet.Yaw_PosFeedback = 90;
	ParamSet.Yaw_NegFeedback = 5;
	ParamSet.AngleTurnOverNick = 85;
	ParamSet.AngleTurnOverRoll = 85;
	ParamSet.GyroAccTrim = 16;        // 1/k
	ParamSet.DriftComp = 4;
	ParamSet.DynamicStability = 100;
	ParamSet.J16Bitmask = 95;
	ParamSet.J17Bitmask = 243;
	ParamSet.J16Timing = 15;
	ParamSet.J17Timing = 15;
	ParamSet.NaviGpsModeControl = 253;
	ParamSet.NaviGpsGain = 100;
	ParamSet.NaviGpsP = 90;
	ParamSet.NaviGpsI = 90;
	ParamSet.NaviGpsD = 90;
	ParamSet.NaviGpsACC = 0;
	ParamSet.NaviGpsMinSat = 6;
	ParamSet.NaviStickThreshold = 8;
	ParamSet.NaviWindCorrection = 90;
	ParamSet.NaviSpeedCompensation = 30;
	ParamSet.NaviOperatingRadius = 100;
	ParamSet.NaviAngleLimitation = 60;
	memcpy(ParamSet.Name, "Sport\0",6);
}


/***************************************************/
/*    Default Values for parameter set 2           */
/***************************************************/
void ParamSet_DefaultSet2(void) // normal
{
	ParamSet.ChannelAssignment[CH_NICK]  = 1;
	ParamSet.ChannelAssignment[CH_ROLL]  = 2;
	ParamSet.ChannelAssignment[CH_GAS]   = 3;
	ParamSet.ChannelAssignment[CH_YAW]  = 4;
	ParamSet.ChannelAssignment[CH_POTI1] = 5;
	ParamSet.ChannelAssignment[CH_POTI2] = 6;
	ParamSet.ChannelAssignment[CH_POTI3] = 7;
	ParamSet.ChannelAssignment[CH_POTI4] = 8;
	ParamSet.GlobalConfig = CFG_AXIS_COUPLING_ACTIVE | CFG_COMPASS_ACTIVE | CFG_GPS_ACTIVE;//CFG_HEIGHT_CONTROL | CFG_HEIGHT_SWITCH | CFG_COMPASS_FIX;
	ParamSet.Height_MinGas = 30;
	ParamSet.MaxHeight     = 251;         // Wert : 0-250   251 -> Poti1
	ParamSet.Height_P      = 10;          // Wert : 0-32
	ParamSet.Height_D  = 30;          // Wert : 0-250
	ParamSet.Height_ACC_Effect = 30;     // Wert : 0-250
	ParamSet.Height_Gain = 3;     // Wert : 0-50
	ParamSet.Stick_P = 12;            // Wert : 1-24
	ParamSet.Stick_D = 16;           // Wert : 0-250
	ParamSet.Yaw_P = 6;                 // Wert : 1-20
	ParamSet.Gas_Min = 8;               // Wert : 0-32
	ParamSet.Gas_Max = 230;              // Wert : 33-250
	ParamSet.GyroAccFactor = 30;         // Wert : 1-64
	ParamSet.CompassYawEffect = 128;        // Wert : 0-250
	ParamSet.Gyro_P = 80;                // Wert : 0-250
	ParamSet.Gyro_I = 120;               // Wert : 0-250
	ParamSet.LowVoltageWarning = 94; // Wert : 0-250
	ParamSet.EmergencyGas = 35;                // Wert : 0-250     // Gaswert bei Empangsverlust
	ParamSet.EmergencyGasDuration = 30;            // Wert : 0-250     // Zeit bis auf EmergencyGas geschaltet wird, wg. Rx-Problemen
	ParamSet.UfoArrangement = 0;         // X oder + Formation
	ParamSet.I_Factor = 32;
	ParamSet.UserParam1 = 0;             // zur freien Verwendung
	ParamSet.UserParam2 = 0;             // zur freien Verwendung
	ParamSet.UserParam3 = 0;             // zur freien Verwendung
	ParamSet.UserParam4 = 0;             // zur freien Verwendung
	ParamSet.UserParam5 = 0;             // zur freien Verwendung
	ParamSet.UserParam6 = 0;             // zur freien Verwendung
	ParamSet.UserParam7 = 0;             // zur freien Verwendung
	ParamSet.UserParam8 = 0;             // zur freien Verwendung
	ParamSet.ServoNickControl = 100;     // Wert : 0-250     // Stellung des Servos
	ParamSet.ServoNickComp = 40;         // Wert : 0-250     // Einfluss Gyro/Servo
	ParamSet.ServoNickCompInvert = 0;    // Wert : 0-250     // Richtung Einfluss Gyro/Servo
	ParamSet.ServoNickMin = 50;          // Wert : 0-250     // Anschlag
	ParamSet.ServoNickMax = 150;         // Wert : 0-250     // Anschlag
	ParamSet.ServoNickRefresh = 5;
	ParamSet.LoopGasLimit = 50;
	ParamSet.LoopThreshold = 90;         // Wert: 0-250  Schwelle für Stickausschlag
	ParamSet.LoopHysteresis = 50;
	ParamSet.BitConfig = 0;             // Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts
	ParamSet.Yaw_PosFeedback = 90;        // Faktor, mit dem Yaw die Achsen Roll und Nick verkoppelt
	ParamSet.Yaw_NegFeedback = 5;
	ParamSet.AngleTurnOverNick = 85;
	ParamSet.AngleTurnOverRoll = 85;
	ParamSet.GyroAccTrim = 32;        // 1/k
	ParamSet.DriftComp = 4;
	ParamSet.DynamicStability = 75;
	ParamSet.J16Bitmask = 95;
	ParamSet.J17Bitmask = 243;
	ParamSet.J16Timing = 20;
	ParamSet.J17Timing = 20;
	ParamSet.NaviGpsModeControl = 253;
	ParamSet.NaviGpsGain = 100;
	ParamSet.NaviGpsP = 90;
	ParamSet.NaviGpsI = 90;
	ParamSet.NaviGpsD = 90;
	ParamSet.NaviGpsACC = 0;
	ParamSet.NaviGpsMinSat = 6;
	ParamSet.NaviStickThreshold = 8;
	ParamSet.NaviWindCorrection = 90;
	ParamSet.NaviSpeedCompensation = 30;
	ParamSet.NaviOperatingRadius = 100;
	ParamSet.NaviAngleLimitation = 60;
	memcpy(ParamSet.Name, "Normal\0", 7);
}


/***************************************************/
/*    Default Values for parameter set 3           */
/***************************************************/
void ParamSet_DefaultSet3(void) // beginner
{
	ParamSet.ChannelAssignment[CH_NICK]  = 1;
	ParamSet.ChannelAssignment[CH_ROLL]  = 2;
	ParamSet.ChannelAssignment[CH_GAS]   = 3;
	ParamSet.ChannelAssignment[CH_YAW]  = 4;
	ParamSet.ChannelAssignment[CH_POTI1] = 5;
	ParamSet.ChannelAssignment[CH_POTI2] = 6;
	ParamSet.ChannelAssignment[CH_POTI3] = 7;
	ParamSet.ChannelAssignment[CH_POTI4] = 8;
	ParamSet.GlobalConfig = CFG_ROTARY_RATE_LIMITER | CFG_AXIS_COUPLING_ACTIVE | CFG_COMPASS_ACTIVE | CFG_GPS_ACTIVE;//CFG_HEIGHT_CONTROL | CFG_HEIGHT_SWITCH | CFG_COMPASS_FIX;
	ParamSet.Height_MinGas = 30;
	ParamSet.MaxHeight     = 251;         // Wert : 0-250   251 -> Poti1
	ParamSet.Height_P      = 10;          // Wert : 0-32
	ParamSet.Height_D  = 30;              // Wert : 0-250
	ParamSet.Height_ACC_Effect = 30;     // Wert : 0-250
	ParamSet.Height_Gain = 3;            // Wert : 0-50
	ParamSet.Stick_P = 8;              // Wert : 1-24
	ParamSet.Stick_D = 16;              // Wert : 0-250
	ParamSet.Yaw_P  = 6;                // Wert : 1-20
	ParamSet.Gas_Min = 8;               // Wert : 0-32
	ParamSet.Gas_Max = 230;              // Wert : 33-250
	ParamSet.GyroAccFactor = 30;         // Wert : 1-64
	ParamSet.CompassYawEffect = 128;       // Wert : 0-250
	ParamSet.Gyro_P = 100;               // Wert : 0-250
	ParamSet.Gyro_I = 120;               // Wert : 0-250
	ParamSet.LowVoltageWarning = 94; // Wert : 0-250
	ParamSet.EmergencyGas = 35;                // Wert : 0-250     // Gaswert bei Empangsverlust
	ParamSet.EmergencyGasDuration = 20;            // Wert : 0-250     // Zeit bis auf EmergencyGas geschaltet wird, wg. Rx-Problemen
	ParamSet.UfoArrangement = 0;         // X oder + Formation
	ParamSet.I_Factor = 16;
	ParamSet.UserParam1 = 0;             // zur freien Verwendung
	ParamSet.UserParam2 = 0;             // zur freien Verwendung
	ParamSet.UserParam3 = 0;             // zur freien Verwendung
	ParamSet.UserParam4 = 0;             // zur freien Verwendung
	ParamSet.UserParam5 = 0;             // zur freien Verwendung
	ParamSet.UserParam6 = 0;             // zur freien Verwendung
	ParamSet.UserParam7 = 0;             // zur freien Verwendung
	ParamSet.UserParam8 = 0;             // zur freien Verwendung
	ParamSet.ServoNickControl = 100;     // Wert : 0-250     // Stellung des Servos
	ParamSet.ServoNickComp = 40;         // Wert : 0-250     // Einfluss Gyro/Servo
	ParamSet.ServoNickCompInvert = 0;    // Wert : 0-250     // Richtung Einfluss Gyro/Servo
	ParamSet.ServoNickMin = 50;          // Wert : 0-250     // Anschlag
	ParamSet.ServoNickMax = 150;         // Wert : 0-250     // Anschlag
	ParamSet.ServoNickRefresh = 5;
	ParamSet.LoopGasLimit = 50;
	ParamSet.LoopThreshold = 90;         // Wert: 0-250  Schwelle für Stickausschlag
	ParamSet.LoopHysteresis = 50;
	ParamSet.BitConfig = 0;             // Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts
	ParamSet.Yaw_PosFeedback = 90;        // Faktor, mit dem Yaw die Achsen Roll und Nick verkoppelt
	ParamSet.Yaw_NegFeedback = 5;
	ParamSet.AngleTurnOverNick = 85;
	ParamSet.AngleTurnOverRoll = 85;
	ParamSet.GyroAccTrim = 32;        // 1/k
	ParamSet.DriftComp = 4;
	ParamSet.DynamicStability = 50;
	ParamSet.J16Bitmask = 95;
	ParamSet.J17Bitmask = 243;
	ParamSet.J16Timing = 30;
	ParamSet.J17Timing = 30;
	ParamSet.NaviGpsModeControl = 253;
	ParamSet.NaviGpsGain = 100;
	ParamSet.NaviGpsP = 90;
	ParamSet.NaviGpsI = 90;
	ParamSet.NaviGpsD = 90;
	ParamSet.NaviGpsACC = 0;
	ParamSet.NaviGpsMinSat = 6;
	ParamSet.NaviStickThreshold = 8;
	ParamSet.NaviWindCorrection = 90;
	ParamSet.NaviSpeedCompensation = 30;
	ParamSet.NaviOperatingRadius = 100;
	ParamSet.NaviAngleLimitation = 60;
	memcpy(ParamSet.Name, "Beginner\0", 9);
}

/***************************************************/
/*       Read Parameter from EEPROM as byte        */
/***************************************************/
uint8_t GetParamByte(uint8_t param_id)
{
	return eeprom_read_byte(&EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id]);
}

/***************************************************/
/*       Write Parameter to EEPROM as byte         */
/***************************************************/
void SetParamByte(uint8_t param_id, uint8_t value)
{
	eeprom_write_byte(&EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id], value);
}

/***************************************************/
/*       Read Parameter from EEPROM as word        */
/***************************************************/
uint16_t GetParamWord(uint8_t param_id)
{
	return eeprom_read_word((uint16_t *) &EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id]);
}

/***************************************************/
/*       Write Parameter to EEPROM as word         */
/***************************************************/
void SetParamWord(uint8_t param_id, uint16_t value)
{
	eeprom_write_word((uint16_t *) &EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id], value);
}


/***************************************************/
/*       Read Parameter Set from EEPROM            */
/***************************************************/
// number [1..5]
void ParamSet_ReadFromEEProm(uint8_t setnumber)
{
	if((1 > setnumber) || (setnumber > 5)) setnumber = 3;
	eeprom_read_block((uint8_t *) &ParamSet.ChannelAssignment[0], &EEPromArray[EEPROM_ADR_PARAMSET_BEGIN + PARAMSET_STRUCT_LEN * (setnumber - 1)], PARAMSET_STRUCT_LEN);
	LED_Init();
}

/***************************************************/
/*        Write Parameter Set to EEPROM            */
/***************************************************/
// number [1..5]
void ParamSet_WriteToEEProm(uint8_t setnumber)
{
	if(setnumber > 5) setnumber = 5;
	if(setnumber < 1) return;
	eeprom_write_block((uint8_t *) &ParamSet.ChannelAssignment[0], &EEPromArray[EEPROM_ADR_PARAMSET_BEGIN + PARAMSET_STRUCT_LEN * (setnumber - 1)], PARAMSET_STRUCT_LEN);
	// set this parameter set to active set
	SetActiveParamSet(setnumber);
	LED_Init();
}

/***************************************************/
/*       Get active parameter set                  */
/***************************************************/
uint8_t GetActiveParamSet(void)
{
	uint8_t setnumber;
	setnumber = eeprom_read_byte(&EEPromArray[PID_ACTIVE_SET]);
	if(setnumber > 5)
	{
		setnumber = 3;
		eeprom_write_byte(&EEPromArray[PID_ACTIVE_SET], setnumber);
	}
	return(setnumber);
}

/***************************************************/
/*       Set active parameter set                  */
/***************************************************/
void SetActiveParamSet(uint8_t setnumber)
{
	if(setnumber > 5) setnumber = 5;
	if(setnumber < 1) setnumber = 1;
	eeprom_write_byte(&EEPromArray[PID_ACTIVE_SET], setnumber);
}

/***************************************************/
/*       Initialize EEPROM Parameter Sets          */
/***************************************************/
void ParamSet_Init(void)
{
	// version  check
	if(eeprom_read_byte(&EEPromArray[PID_VERSION]) != EEPARAM_VERSION)
	{
		// if version check faild
		printf("\n\rInit. EEPROM: Generating Default-Parameter...");
		ParamSet_DefaultSet1(); // Fill ParamSet Structure to default parameter set 1 (Sport)
		// fill all 5 parameter settings with set 1 except otherwise defined
		for (uint8_t i = 1;i < 6; i++)
		{
			if(i==2) ParamSet_DefaultSet2(); // Kamera
			if(i==3) ParamSet_DefaultSet3(); // Beginner
			if(i>3)  ParamSet_DefaultSet2(); // Kamera
			ParamSet_WriteToEEProm(i);
		}
		// default-Setting is parameter set 3
		SetActiveParamSet(3);
		// update version info
		SetParamByte(PID_VERSION, EEPARAM_VERSION);
	}
	// read active parameter set to ParamSet stucture
	ParamSet_ReadFromEEProm(GetActiveParamSet());
	printf("\n\rUsing Parameter Set %d", GetActiveParamSet());
}
