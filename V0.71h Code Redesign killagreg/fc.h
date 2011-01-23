/*#######################################################################################
Flight Control
#######################################################################################*/

#ifndef _FC_H
#define _FC_H

#include <inttypes.h>

#define YAW_GYRO_DEG_FACTOR 1291L // Factor between Yaw Gyro Integral and HeadingAngle in deg
#define STICK_GAIN 4

typedef struct
{
	uint8_t Height_D;
	uint8_t MaxHeight;
	uint8_t Height_P;
	uint8_t Height_ACC_Effect;
	uint8_t CompassYawEffect;
	uint8_t Gyro_P;
	uint8_t Gyro_I;
	uint8_t Yaw_P;
	uint8_t I_Factor;
	uint8_t UserParam1;
	uint8_t UserParam2;
	uint8_t UserParam3;
	uint8_t UserParam4;
	uint8_t UserParam5;
	uint8_t UserParam6;
	uint8_t UserParam7;
	uint8_t UserParam8;
	uint8_t ServoNickControl;
	uint8_t LoopGasLimit;
	uint8_t Yaw_PosFeedback;
	uint8_t Yaw_NegFeedback;
	uint8_t DynamicStability;
	uint8_t ExternalControl;
	uint8_t J16Timing;
	uint8_t J17Timing;
	#if (defined (USE_KILLAGREG) || defined (USE_MK3MAG))
	uint8_t NaviGpsModeControl;
	uint8_t NaviGpsGain;
	uint8_t NaviGpsP;
	uint8_t NaviGpsI;
	uint8_t NaviGpsD;
	uint8_t NaviGpsACC;
	uint8_t NaviOperatingRadius;
	uint8_t NaviWindCorrection;
	uint8_t NaviSpeedCompensation;
	#endif
	int8_t Kalman_K;
	int8_t Kalman_MaxDrift;
	int8_t Kalman_MaxFusion;
} fc_param_t;

extern fc_param_t FCParam;

// attitude
extern  int32_t IntegralNick, IntegralRoll, IntegralYaw;
extern  int16_t Reading_GyroNick, Reading_GyroRoll, Reading_GyroYaw;

// offsets
extern  int16_t AdNeutralNick, AdNeutralRoll, AdNeutralYaw;
extern volatile int16_t NeutralAccX, NeutralAccY;
extern volatile float NeutralAccZ;


extern volatile int32_t Reading_Integral_Top; // calculated in analog.c

// compass navigation
extern volatile int16_t CompassHeading;
extern volatile int16_t CompassCourse;
extern volatile int16_t CompassOffCourse;
extern volatile uint8_t CompassCalState;
extern int32_t YawGyroHeading;
extern int16_t YawGyroHeadingInDeg;

// hight control
extern int ReadingHeight;
extern int SetPointHeight;

// mean accelerations
extern  int16_t Mean_AccNick, Mean_AccRoll, Mean_AccTop;

// acceleration send to navi board
extern int16_t NaviAccNick, NaviAccRoll, NaviCntAcc;


// looping params
extern long TurnOver180Nick, TurnOver180Roll;

// external control
extern int16_t ExternStickNick, ExternStickRoll, ExternStickYaw;


void MotorControl(void);
void SendMotorData(void);
void CalibMean(void);
void Mean(void);
void SetNeutral(void);
void Beep(uint8_t numbeeps);


extern int16_t  Poti1, Poti2, Poti3, Poti4, Poti5, Poti6, Poti7, Poti8;

// setpoints for motors
extern volatile uint8_t Motor_Front, Motor_Rear, Motor_Right, Motor_Left; //used by twimaster isr

// current stick values
extern int16_t StickNick;
extern int16_t StickRoll;
extern int16_t StickYaw;
extern int16_t GPS_Nick;
extern int16_t GPS_Roll;

// current stick elongations
extern int16_t MaxStickNick, MaxStickRoll, MaxStickYaw;


extern uint16_t Model_Is_Flying;


// MKFlags
#define MKFLAG_MOTOR_RUN  				0x01
#define MKFLAG_FLY        				0x02
#define MKFLAG_CALIBRATE  				0x04
#define MKFLAG_START      				0x08
#define MKFLAG_EMERGENCY_LANDING      	0x10
#define MKFLAG_RESERVE1		      		0x20
#define MKFLAG_RESERVE2		      		0x40
#define MKFLAG_RESERVE3		      		0x80

volatile extern uint8_t MKFlags;

#endif //_FC_H

