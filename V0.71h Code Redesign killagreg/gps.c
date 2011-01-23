#include <inttypes.h>
#include <stdlib.h>
#include "fc.h"
#include "ubx.h"
#include "mymath.h"
#include "timer0.h"
#include "uart.h"
#include "rc.h"
#include "eeprom.h"

typedef enum
{
	GPS_FLIGHT_MODE_UNDEF,
	GPS_FLIGHT_MODE_FREE,
	GPS_FLIGHT_MODE_AID,
	GPS_FLIGHT_MODE_HOME,
} FlightMode_t;

#define GPS_POSINTEGRAL_LIMIT 32000
#define GPS_STICK_LIMIT		45		// limit of gps stick control to avoid critical flight attitudes
#define GPS_P_LIMIT			25


typedef struct
{
	int32_t Longitude;
	int32_t Latitude;
	int32_t Altitude;
	Status_t Status;
} GPS_Pos_t;

// GPS coordinates for hold position
GPS_Pos_t HoldPosition  = {0,0,0,INVALID};
// GPS coordinates for home position
GPS_Pos_t HomePosition  = {0,0,0,INVALID};
// the current flight mode
FlightMode_t FlightMode = GPS_FLIGHT_MODE_UNDEF;


// ---------------------------------------------------------------------------------
void GPS_UpdateParameter(void)
{
	static FlightMode_t FlightModeOld = GPS_FLIGHT_MODE_UNDEF;

	if((RC_Quality < 100) || (MKFlags & MKFLAG_EMERGENCY_LANDING))
	{
		FlightMode = GPS_FLIGHT_MODE_FREE;
	}
	else
	{
		if     (FCParam.NaviGpsModeControl <  50) FlightMode = GPS_FLIGHT_MODE_AID;
		else if(FCParam.NaviGpsModeControl < 180) FlightMode = GPS_FLIGHT_MODE_FREE;
		else                               		  FlightMode = GPS_FLIGHT_MODE_HOME;
	}
	if (FlightMode != FlightModeOld)
	{
		BeepTime = 100;
	}
	FlightModeOld = FlightMode;
}



// ---------------------------------------------------------------------------------
// This function defines a good GPS signal condition
uint8_t GPS_IsSignalOK(void)
{
	static uint8_t GPSFix = 0;
	if( (GPSInfo.status != INVALID)  && (GPSInfo.satfix == SATFIX_3D) && (GPSInfo.flags & FLAG_GPSFIXOK) && ((GPSInfo.satnum >= ParamSet.NaviGpsMinSat) || GPSFix))
	{
		GPSFix = 1;
		return(1);

	}
	else return (0);

}
// ---------------------------------------------------------------------------------
// rescale xy-vector length to  limit
uint8_t GPS_LimitXY(int32_t *x, int32_t *y, int32_t limit)
{
	uint8_t retval = 0;
	int32_t len;
	len = (int32_t)c_sqrt(*x * *x + *y * *y);
	if (len > limit)
	{
		// normalize control vector components to the limit
		*x  = (*x  * limit) / len;
		*y  = (*y  * limit) / len;
		retval = 1;
	}
	return(retval);
}

// checks nick and roll sticks for manual control
uint8_t GPS_IsManualControlled(void)
{
	if ( (abs(PPM_in[ParamSet.ChannelAssignment[CH_NICK]]) < ParamSet.NaviStickThreshold) && (abs(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]]) < ParamSet.NaviStickThreshold)) return 0;
	else return 1;
}

// set given position to current gps position
uint8_t GPS_SetCurrPosition(GPS_Pos_t * pGPSPos)
{
	uint8_t retval = 0;
	if(pGPSPos == NULL) return(retval);	// bad pointer

	if(GPS_IsSignalOK())
	{	// is GPS signal condition is fine
		pGPSPos->Longitude	= GPSInfo.longitude;
		pGPSPos->Latitude	= GPSInfo.latitude;
		pGPSPos->Altitude	= GPSInfo.altitude;
		pGPSPos->Status 	= NEWDATA;
		retval = 1;
	}
	else
	{ 	// bad GPS signal condition
		pGPSPos->Status = INVALID;
		retval = 0;
	}
	return(retval);
}

// clear position
uint8_t GPS_ClearPosition(GPS_Pos_t * pGPSPos)
{
	uint8_t retval = 0;
	if(pGPSPos == NULL) return(retval);	// bad pointer
	else
	{
		pGPSPos->Longitude	= 0;
		pGPSPos->Latitude	= 0;
		pGPSPos->Altitude	= 0;
		pGPSPos->Status 	= INVALID;
		retval = 1;
	}
	return (retval);
}

// disable GPS control sticks
void GPS_Neutral(void)
{
	GPS_Nick = 0;
	GPS_Roll = 0;
}

// calculates the GPS control stick values from the deviation to target position
// if the pointer to the target positin is NULL or is the target position invalid
// then the P part of the controller is deactivated.
void GPS_PIDController(GPS_Pos_t *pTargetPos)
{
	static int32_t PID_Nick, PID_Roll;
	int32_t coscompass, sincompass;
	int32_t GPSPosDev_North, GPSPosDev_East; // Position deviation in cm
	int32_t P_North = 0, D_North = 0, P_East = 0, D_East = 0, I_North = 0, I_East = 0;
	int32_t PID_North = 0, PID_East = 0;
	static int32_t cos_target_latitude = 1;
	static int32_t GPSPosDevIntegral_North = 0, GPSPosDevIntegral_East = 0;
	static GPS_Pos_t *pLastTargetPos = 0;

	// if GPS data and Compass are ok
	if( GPS_IsSignalOK() && (CompassHeading >= 0) )
	{

		if(pTargetPos != NULL) // if there is a target position
		{
			if(pTargetPos->Status != INVALID) // and the position data are valid
			{
				// if the target data are updated or the target pointer has changed
				if ((pTargetPos->Status != PROCESSED) || (pTargetPos != pLastTargetPos) )
				{
					// reset error integral
					GPSPosDevIntegral_North = 0;
					GPSPosDevIntegral_East = 0;
					// recalculate latitude projection
					cos_target_latitude = (int32_t)c_cos_8192((int16_t)(pTargetPos->Latitude/10000000L));
					// remember last target pointer
					pLastTargetPos = pTargetPos;
					// mark data as processed
					pTargetPos->Status = PROCESSED;
				}
				// calculate position deviation from latitude and longitude differences
				GPSPosDev_North = (GPSInfo.latitude - pTargetPos->Latitude); // to calculate real cm we would need *111/100 additionally
				GPSPosDev_East  = (GPSInfo.longitude - pTargetPos->Longitude); // to calculate real cm we would need *111/100 additionally
				// calculate latitude projection
				GPSPosDev_East  *= cos_target_latitude;
				GPSPosDev_East /= 8192;
			}
			else // no valid target position available
			{
				// reset error
				GPSPosDev_North = 0;
				GPSPosDev_East = 0;
				// reset error integral
				GPSPosDevIntegral_North = 0;
				GPSPosDevIntegral_East = 0;
			}
		}
		else // no target position available
		{
			// reset error
			GPSPosDev_North = 0;
			GPSPosDev_East = 0;
			// reset error integral
			GPSPosDevIntegral_North = 0;
			GPSPosDevIntegral_East = 0;
		}

		//Calculate PID-components of the controller

		// D-Part
		D_North = ((int32_t)FCParam.NaviGpsD * GPSInfo.velnorth)/512;
		D_East =  ((int32_t)FCParam.NaviGpsD * GPSInfo.veleast)/512;

		// P-Part
		P_North = ((int32_t)FCParam.NaviGpsP * GPSPosDev_North)/2048;
		P_East =  ((int32_t)FCParam.NaviGpsP * GPSPosDev_East)/2048;

		// I-Part
		I_North = ((int32_t)FCParam.NaviGpsI * GPSPosDevIntegral_North)/8192;
		I_East =  ((int32_t)FCParam.NaviGpsI * GPSPosDevIntegral_East)/8192;


		// combine P & I
		PID_North = P_North + I_North;
		PID_East  = P_East  + I_East;
		if(!GPS_LimitXY(&PID_North, &PID_East, GPS_P_LIMIT))
		{
			GPSPosDevIntegral_North += GPSPosDev_North/16;
			GPSPosDevIntegral_East  += GPSPosDev_East/16;
			GPS_LimitXY(&GPSPosDevIntegral_North, &GPSPosDevIntegral_North, GPS_POSINTEGRAL_LIMIT);
		}

		// combine PI- and D-Part
		PID_North += D_North;
		PID_East  += D_East;


		// scale combination with gain.
		PID_North = (PID_North * (int32_t)FCParam.NaviGpsGain) / 100;
		PID_East  = (PID_East  * (int32_t)FCParam.NaviGpsGain) / 100;

		// GPS to nick and roll settings

		// A positive nick angle moves head downwards (flying forward).
		// A positive roll angle tilts left side downwards (flying left).
		// If compass heading is 0 the head of the copter is in north direction.
		// A positive nick angle will fly to north and a positive roll angle will fly to west.
		// In case of a positive north deviation/velocity the
		// copter should fly to south (negative nick).
		// In case of a positive east position deviation and a positive east velocity the
		// copter should fly to west (positive roll).
		// The influence of the GPS_Nick and GPS_Roll variable is contrarily to the stick values
		// in the fc.c. Therefore a positive north deviation/velocity should result in a positive
		// GPS_Nick and a positive east deviation/velocity should result in a negative GPS_Roll.

		coscompass = (int32_t)c_cos_8192(YawGyroHeading / YAW_GYRO_DEG_FACTOR);
		sincompass = (int32_t)c_sin_8192(YawGyroHeading / YAW_GYRO_DEG_FACTOR);
		PID_Nick =   (coscompass * PID_North + sincompass * PID_East) / 8192;
		PID_Roll  =  (sincompass * PID_North - coscompass * PID_East) / 8192;


		// limit resulting GPS control vector
		GPS_LimitXY(&PID_Nick, &PID_Roll, GPS_STICK_LIMIT);

		GPS_Nick = (int16_t)PID_Nick;
		GPS_Roll = (int16_t)PID_Roll;
	}
	else // invalid GPS data or bad compass reading
	{
		GPS_Neutral(); // do nothing
		// reset error integral
		GPSPosDevIntegral_North = 0;
		GPSPosDevIntegral_East = 0;
	}
}




void GPS_Main(void)
{
	static uint8_t GPS_P_Delay = 0;
	static uint16_t beep_rythm = 0;

	GPS_UpdateParameter();

	// store home position if start of flight flag is set
	if(MKFlags & MKFLAG_CALIBRATE)
	{
		if(GPS_SetCurrPosition(&HomePosition)) BeepTime = 700;
	}

	switch(GPSInfo.status)
	{
	case INVALID:  // invalid gps data
		GPS_Neutral();
		if(FlightMode != GPS_FLIGHT_MODE_FREE)
		{
			BeepTime = 100; // beep if signal is neccesary
		}
		break;
	case PROCESSED: // if gps data are already processed do nothing
		// downcount timeout
		if(GPSTimeout) GPSTimeout--;
		// if no new data arrived within timeout set current data invalid
		// and therefore disable GPS
		else
		{
			GPS_Neutral();
			GPSInfo.status = INVALID;
		}
		break;
	case NEWDATA: // new valid data from gps device
		// if the gps data quality is good
		beep_rythm++;

		if (GPS_IsSignalOK())
		{
			switch(FlightMode) // check what's to do
			{
				case GPS_FLIGHT_MODE_FREE:
					// update hold position to current gps position
					GPS_SetCurrPosition(&HoldPosition); // can get invalid if gps signal is bad
					// disable gps control
					GPS_Neutral();
					break;

				case GPS_FLIGHT_MODE_AID:
					if(HoldPosition.Status != INVALID)
					{
						if( GPS_IsManualControlled() ) // MK controlled by user
						{
							// update hold point to current gps position
							GPS_SetCurrPosition(&HoldPosition);
							// disable gps control
							GPS_Neutral();
							GPS_P_Delay = 0;
						}
						else // GPS control active
						{
							if(GPS_P_Delay < 7)
							{ // delayed activation of P-Part for 8 cycles (8*0.25s = 2s)
								GPS_P_Delay++;
								GPS_SetCurrPosition(&HoldPosition); // update hold point to current gps position
								GPS_PIDController(NULL); // activates only the D-Part
							}
							else GPS_PIDController(&HoldPosition);// activates the P&D-Part
						}
					}
					else // invalid Hold Position
					{  // try to catch a valid hold position from gps data input
						GPS_SetCurrPosition(&HoldPosition);
						GPS_Neutral();
					}
					break;

				case GPS_FLIGHT_MODE_HOME:
					if(HomePosition.Status != INVALID)
					{
						// update hold point to current gps position
						// to avoid a flight back if home comming is deactivated
						GPS_SetCurrPosition(&HoldPosition);
						if( GPS_IsManualControlled() ) // MK controlled by user
						{
							GPS_Neutral();
						}
						else // GPS control active
						{
							GPS_PIDController(&HomePosition);
						}
					}
					else // bad home position
					{
						BeepTime = 50; // signal invalid home position
						// try to hold at least the position as a fallback option

						if (HoldPosition.Status != INVALID)
						{
							if( GPS_IsManualControlled() ) // MK controlled by user
							{
								GPS_Neutral();
							}
							else // GPS control active
							{
								GPS_PIDController(&HoldPosition);
							}
						}
						else
						{ // try to catch a valid hold position
							GPS_SetCurrPosition(&HoldPosition);
							GPS_Neutral();
						}
					}
					break; // eof TSK_HOME
				default: // unhandled task
					GPS_Neutral();
					break; // eof default
			} // eof switch GPS_Task
		} // eof gps data quality is good
		else // gps data quality is bad
		{	// disable gps control
			GPS_Neutral();
			if(FlightMode != GPS_FLIGHT_MODE_FREE)
			{
				// beep if signal is not sufficient
				if(!(GPSInfo.flags & FLAG_GPSFIXOK) && !(beep_rythm % 5)) BeepTime = 100;
				else if (GPSInfo.satnum < ParamSet.NaviGpsMinSat && !(beep_rythm % 5)) BeepTime = 10;
			}
		}
		// set current data as processed to avoid further calculations on the same gps data
		GPSInfo.status = PROCESSED;
		break;
	} // eof GPSInfo.status
}

