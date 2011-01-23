#ifndef _EEPROM_H
#define _EEPROM_H

#include <inttypes.h>

#define EEPROM_ADR_PARAM_BEGIN 			0
#define PID_VERSION          1 // byte
#define PID_ACTIVE_SET       2 // byte
#define PID_PRESSURE_OFFSET  3 // byte
#define PID_ACC_NICK         4 // word
#define PID_ACC_ROLL         6 // word
#define PID_ACC_Z            8 // word

#ifdef USE_KILLAGREG
#define PID_MM3_X_OFF		10 // byte
#define PID_MM3_Y_OFF		11 // byte
#define PID_MM3_Z_OFF		12 // byte
#define PID_MM3_X_RANGE		13 // word
#define PID_MM3_Y_RANGE		15 // word
#define PID_MM3_Z_RANGE		17 // word
#endif

#define EEPROM_ADR_PARAMSET_BEGIN      100

// bit mask for ParamSet.GlobalConfig
#define CFG_HEIGHT_CONTROL			0x01
#define CFG_HEIGHT_SWITCH			0x02
#define CFG_HEADING_HOLD			0x04
#define CFG_COMPASS_ACTIVE			0x08
#define CFG_COMPASS_FIX				0x10
#define CFG_GPS_ACTIVE				0x20
#define CFG_AXIS_COUPLING_ACTIVE	0x40
#define CFG_ROTARY_RATE_LIMITER		0x80

// bit mask for ParamSet.BitConfig
#define CFG_LOOP_UP			0x01
#define CFG_LOOP_DOWN		0x02
#define CFG_LOOP_LEFT		0x04
#define CFG_LOOP_RIGHT		0x08
#define CFG_HEIGHT_3SWITCH	0x10

// defines for lookup ParamSet.ChannelAssignment
#define CH_NICK		0
#define CH_ROLL		1
#define CH_GAS		2
#define CH_YAW		3
#define CH_POTI1	4
#define CH_POTI2	5
#define CH_POTI3	6
#define CH_POTI4 	7

#define EEPARAM_VERSION 73 // is count up, if EE_Paramater stucture has changed (compatibility)

// values above 250 representing poti1 to poti4
typedef struct
 {
   uint8_t ChannelAssignment[8];   // see upper defines for details
   uint8_t GlobalConfig;           // see upper defines for bitcoding
   uint8_t Height_MinGas;           // Wert : 0-100
   uint8_t Height_D;            // Wert : 0-250
   uint8_t MaxHeight;               // Wert : 0-32
   uint8_t Height_P;                // Wert : 0-32
   uint8_t Height_Gain;             // Wert : 0-50
   uint8_t Height_ACC_Effect;      // Wert : 0-250
   uint8_t Stick_P;                // Wert : 1-6
   uint8_t Stick_D;                // Wert : 0-64
   uint8_t Yaw_P;                 // Wert : 1-20
   uint8_t Gas_Min;                // Wert : 0-32
   uint8_t Gas_Max;                // Wert : 33-250
   uint8_t GyroAccFactor;          // Wert : 1-64
   uint8_t CompassYawEffect;         // Wert : 0-32
   uint8_t Gyro_P;                 // Wert : 10-250
   uint8_t Gyro_I;                 // Wert : 0-250
   uint8_t LowVoltageWarning;      // Wert : 0-250
   uint8_t EmergencyGas;           // Wert : 0-250     //Gaswert bei Empängsverlust
   uint8_t EmergencyGasDuration;   // Wert : 0-250     // Zeitbis auf EmergencyGas geschaltet wird, wg. Rx-Problemen
   uint8_t UfoArrangement;         // x oder + Formation
   uint8_t I_Factor;               // Wert : 0-250
   uint8_t UserParam1;             // Wert : 0-250
   uint8_t UserParam2;             // Wert : 0-250
   uint8_t UserParam3;             // Wert : 0-250
   uint8_t UserParam4;             // Wert : 0-250
   uint8_t ServoNickControl;       // Wert : 0-250     // Stellung des Servos
   uint8_t ServoNickComp;          // Wert : 0-250     // Einfluss Gyro/Servo
   uint8_t ServoNickMin;           // Wert : 0-250     // Anschlag
   uint8_t ServoNickMax;           // Wert : 0-250     // Anschlag
   uint8_t ServoNickRefresh;       //
   uint8_t LoopGasLimit;           // Wert: 0-250  max. Gas während Looping
   uint8_t LoopThreshold;          // Wert: 0-250  Schwelle für Stickausschlag
   uint8_t LoopHysteresis;         // Wert: 0-250  Hysterese für Stickausschlag
   uint8_t Yaw_PosFeedback;        // Wert: 0-250  Faktor, mit dem Yaw die Achsen Roll und Nick koppelt (NickRollMitkopplung)
   uint8_t Yaw_NegFeedback;        // Wert: 0-250  Faktor, mit dem Yaw die Achsen Roll und Nick Gegenkoppelt (NickRollGegenkopplung)
   uint8_t AngleTurnOverNick;     // Wert: 0-250  180°-Punkt
   uint8_t AngleTurnOverRoll;      // Wert: 0-250  180°-Punkt
   uint8_t GyroAccTrim;            // 1/k  (Koppel_ACC_Wirkung)
   uint8_t DriftComp;
   uint8_t DynamicStability;
   uint8_t UserParam5;             // Wert : 0-250
   uint8_t UserParam6;             // Wert : 0-250
   uint8_t UserParam7;             // Wert : 0-250
   uint8_t UserParam8;             // Wert : 0-250
   uint8_t J16Bitmask;			   // for the J16 Output
   uint8_t J16Timing;			   // for the J16 Output
   uint8_t J17Bitmask;			   // for the J17 Output
   uint8_t J17Timing;			   // for the J17 Output
   uint8_t NaviGpsModeControl;     // Parameters for the Naviboard
   uint8_t NaviGpsGain;			   // overall gain for GPS-PID controller
   uint8_t NaviGpsP;			   // P gain for GPS-PID controller
   uint8_t NaviGpsI;			   // I gain for GPS-PID controller
   uint8_t NaviGpsD;               // D gain for GPS-PID controller
   uint8_t NaviGpsACC;             // ACC gain for GPS-PID controller
   uint8_t NaviGpsMinSat;          // number of sattelites neccesary for GPS functions
   uint8_t NaviStickThreshold;     // activation threshild for detection of manual stick movements
   uint8_t NaviWindCorrection;     // streng of wind course correction
   uint8_t NaviSpeedCompensation;
   uint8_t NaviOperatingRadius;	   // Radius limit in m around start position for GPS flights
   uint8_t NaviAngleLimitation;	   // limitation of attitude angle controlled by the gps algorithm
   uint8_t ExternalControl;        // for serial Control
   uint8_t BitConfig;              // see upper defines for bitcoding
   uint8_t ServoNickCompInvert;    // Wert : 0-250   0 oder 1  // WICHTIG!!! am Ende lassen
   uint8_t Reserved[4];
   int8_t Name[12];
 } paramset_t;

#define  PARAMSET_STRUCT_LEN  sizeof(paramset_t)

extern paramset_t ParamSet;

extern void ParamSet_Init(void);
extern void ParamSet_ReadFromEEProm(uint8_t setnumber);
extern void ParamSet_WriteToEEProm(uint8_t setnumber);
extern uint8_t GetActiveParamSet(void);
extern void SetActiveParamSet(uint8_t setnumber);


extern uint8_t GetParamByte(uint8_t param_id);
extern void SetParamByte(uint8_t param_id, uint8_t value);
extern uint16_t GetParamWord(uint8_t param_id);
extern void SetParamWord(uint8_t param_id, uint16_t value);

#endif //_EEPROM_H
