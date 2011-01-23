/*#######################################################################################
Flight Control
#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 04.2007 Holger Buss
// + Nur für den privaten Gebrauch
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Es gilt für das gesamte Projekt (Hardware, Software, Binärfiles, Sourcecode und Dokumentation),
// + dass eine Nutzung (auch auszugsweise) nur für den privaten (nicht-kommerziellen) Gebrauch zulässig ist.
// + Sollten direkte oder indirekte kommerzielle Absichten verfolgt werden, ist mit uns (info@mikrokopter.de) Kontakt
// + bzgl. der Nutzungsbedingungen aufzunehmen.
// + Eine kommerzielle Nutzung ist z.B.Verkauf von MikroKoptern, Bestückung und Verkauf von Platinen oder Bausätzen,
// + Verkauf von Luftbildaufnahmen, usw.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Werden Teile des Quellcodes (mit oder ohne Modifikation) weiterverwendet oder veröffentlicht,
// + unterliegen sie auch diesen Nutzungsbedingungen und diese Nutzungsbedingungen incl. Copyright müssen dann beiliegen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Sollte die Software (auch auszugesweise) oder sonstige Informationen des MikroKopter-Projekts
// + auf anderen Webseiten oder sonstigen Medien veröffentlicht werden, muss unsere Webseite "http://www.mikrokopter.de"
// + eindeutig als Ursprung verlinkt werden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Keine Gewähr auf Fehlerfreiheit, Vollständigkeit oder Funktion
// + Benutzung auf eigene Gefahr
// + Wir übernehmen keinerlei Haftung für direkte oder indirekte Personen- oder Sachschäden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Portierung der Software (oder Teile davon) auf andere Systeme (ausser der Hardware von www.mikrokopter.de) ist nur
// + mit unserer Zustimmung zulässig
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Funktion printf_P() unterliegt ihrer eigenen Lizenz und ist hiervon nicht betroffen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Redistributions of source code (with or without modifications) must retain the above copyright notice,
// + this list of conditions and the following disclaimer.
// +   * Neither the name of the copyright holders nor the names of contributors may be used to endorse or promote products derived
// +     from this software without specific prior written permission.
// +   * The use of this project (hardware, software, binary files, sources and documentation) is only permittet
// +     for non-commercial use (directly or indirectly)
// +     Commercial use (for excample: selling of MikroKopters, selling of PCBs, assembly, ...) is only permitted
// +     with our written permission
// +   * If sources or documentations are redistributet on other webpages, out webpage (http://www.MikroKopter.de) must be
// +     clearly linked as origin
// +   * porting to systems other than hardware from www.mikrokopter.de is not allowed
// +  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// +  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// +  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// +  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// +  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// +  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// +  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// +  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN// +  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// +  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// +  POSSIBILITY OF SUCH DAMAGE.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdlib.h>
#include <avr/io.h>

#include "main.h"
#include "eeprom.h"
#include "timer0.h"
#include "_Settings.h"
#include "analog.h"
#include "fc.h"
#include "uart.h"
#include "rc.h"
#include "twimaster.h"
#include "timer2.h"
#ifdef USE_KILLAGREG
#include "mm3.h"
#include "gps.h"
#endif
#ifdef USE_MK3MAG
#include "mk3mag.h"
#include "gps.h"
#endif
#include "led.h"
#ifdef USE_NAVICTRL
#include "spi.h"
#endif
// gyro readings
int16_t Reading_GyroNick, Reading_GyroRoll, Reading_GyroYaw;
// gyro neutral readings
int16_t AdNeutralNick = 0, AdNeutralRoll = 0, AdNeutralYaw = 0;
int16_t StartNeutralRoll = 0, StartNeutralNick = 0;
// mean accelerations
int16_t Mean_AccNick, Mean_AccRoll, Mean_AccTop;

// neutral acceleration readings
volatile int16_t NeutralAccX=0, NeutralAccY=0;
volatile float NeutralAccZ = 0;

// attitude gyro integrals
int32_t IntegralNick = 0,IntegralNick2 = 0;
int32_t IntegralRoll = 0,IntegralRoll2 = 0;
int32_t IntegralYaw = 0;
int32_t Reading_IntegralGyroNick = 0, Reading_IntegralGyroNick2 = 0;
int32_t Reading_IntegralGyroRoll = 0, Reading_IntegralGyroRoll2 = 0;
int32_t Reading_IntegralGyroYaw = 0;
int32_t MeanIntegralNick;
int32_t MeanIntegralRoll;

// attitude acceleration integrals
int32_t IntegralAccNick = 0, IntegralAccRoll = 0;
volatile int32_t Reading_Integral_Top = 0;

// compass course
volatile int16_t CompassHeading = -1; // negative angle indicates invalid data.
volatile int16_t CompassCourse = -1;
volatile int16_t CompassOffCourse = 0;
volatile uint8_t CompassCalState = 0;
uint8_t FunnelCourse = 0;
uint16_t BadCompassHeading = 500;
int32_t YawGyroHeading;
int16_t YawGyroDrift;


int16_t NaviAccNick = 0, NaviAccRoll = 0, NaviCntAcc = 0;


// MK flags
uint16_t Model_Is_Flying = 0;
volatile uint8_t MKFlags = 0;

int32_t TurnOver180Nick = 250000L, TurnOver180Roll = 250000L;

float Gyro_P_Factor;
float Gyro_I_Factor;

int16_t  DiffNick, DiffRoll;

int16_t  Poti1 = 0, Poti2 = 0, Poti3 = 0, Poti4 = 0, Poti5 = 0, Poti6 = 0, Poti7 = 0, Poti8 = 0;

// setpoints for motors
volatile uint8_t Motor_Front, Motor_Rear, Motor_Right, Motor_Left;

// stick values derived by rc channels readings
int16_t StickNick = 0, StickRoll = 0, StickYaw = 0, StickGas = 0;
int16_t GPS_Nick = 0, GPS_Roll = 0;

int16_t MaxStickNick = 0, MaxStickRoll = 0;
// stick values derived by uart inputs
int16_t ExternStickNick = 0, ExternStickRoll = 0, ExternStickYaw = 0, ExternHeightValue = -20;



int16_t ReadingHeight = 0;
int16_t SetPointHeight = 0;

int16_t AttitudeCorrectionRoll = 0, AttitudeCorrectionNick = 0;

float Ki =  FACTOR_I;

uint8_t Looping_Nick = 0, Looping_Roll = 0;
uint8_t Looping_Left = 0, Looping_Right = 0, Looping_Down = 0, Looping_Top = 0;


fc_param_t FCParam = {48,251,16,58,64,150,150,2,10,0,0,0,0,0,0,0,0,100,70,0,0,100};


/************************************************************************/
/*  Creates numbeeps beeps at the speaker                               */
/************************************************************************/
void Beep(uint8_t numbeeps)
{
	while(numbeeps--)
	{
		if(MKFlags & MKFLAG_MOTOR_RUN) return; //auf keinen Fall bei laufenden Motoren!
		BeepTime = 100; // 0.1 second
		Delay_ms(250); // blocks 250 ms as pause to next beep,
		// this will block the flight control loop,
		// therefore do not use this function if motors are running
	}
}

/************************************************************************/
/*  Neutral Readings                                                    */
/************************************************************************/
void SetNeutral(void)
{
	NeutralAccX = 0;
	NeutralAccY = 0;
	NeutralAccZ = 0;
    AdNeutralNick = 0;
	AdNeutralRoll = 0;
	AdNeutralYaw = 0;
    FCParam.Yaw_PosFeedback = 0;
    FCParam.Yaw_NegFeedback = 0;
    ExpandBaro = 0;
    CalibMean();
    Delay_ms_Mess(100);
	CalibMean();
    if((ParamSet.GlobalConfig & CFG_HEIGHT_CONTROL))  // Height Control activated?
    {
		if((ReadingAirPressure > 950) || (ReadingAirPressure < 750)) SearchAirPressureOffset();
    }
	AdNeutralNick = AdValueGyrNick;
	AdNeutralRoll  = AdValueGyrRoll;
	AdNeutralYaw   = AdValueGyrYaw;
	StartNeutralRoll  = AdNeutralRoll;
	StartNeutralNick = AdNeutralNick;
    if(GetParamWord(PID_ACC_NICK) > 1023)
    {
		NeutralAccY = abs(Mean_AccRoll) / ACC_AMPLIFY;
		NeutralAccX = abs(Mean_AccNick) / ACC_AMPLIFY;
		NeutralAccZ = Current_AccZ;
    }
    else
    {
		NeutralAccX = (int16_t)GetParamWord(PID_ACC_NICK);
	    NeutralAccY = (int16_t)GetParamWord(PID_ACC_ROLL);
	    NeutralAccZ = (int16_t)GetParamWord(PID_ACC_Z);
    }
	Reading_IntegralGyroNick = 0;
    Reading_IntegralGyroNick2 = 0;
    Reading_IntegralGyroRoll = 0;
    Reading_IntegralGyroRoll2 = 0;
    Reading_IntegralGyroYaw = 0;
    Reading_GyroNick = 0;
    Reading_GyroRoll = 0;
    Reading_GyroYaw = 0;
    Delay_ms_Mess(100);
    StartAirPressure = AirPressure;
    HeightD = 0;
    Reading_Integral_Top = 0;
    CompassCourse = CompassHeading;
    BeepTime = 50;
	TurnOver180Nick = ((int32_t) ParamSet.AngleTurnOverNick * 2500L) +15000L;
	TurnOver180Roll =  ((int32_t) ParamSet.AngleTurnOverRoll *  2500L) +15000L;
    ExternHeightValue = 0;
    GPS_Nick = 0;
    GPS_Roll = 0;
    YawGyroHeading = CompassHeading * YAW_GYRO_DEG_FACTOR;
    YawGyroDrift = 0;
    MKFlags |= MKFLAG_CALIBRATE;
   	FCParam.Kalman_K = -1;
	FCParam.Kalman_MaxDrift = ParamSet.DriftComp * 16;
	FCParam.Kalman_MaxFusion = 32;
}

/************************************************************************/
/*  Averaging Measurement Readings                                      */
/************************************************************************/
void Mean(void)
{
    static int32_t tmpl,tmpl2;

 // Get offset corrected gyro readings (~ to angular velocity)
    Reading_GyroYaw   = AdNeutralYaw   - AdValueGyrYaw;
    Reading_GyroRoll  = AdValueGyrRoll - AdNeutralRoll;
    Reading_GyroNick  = AdValueGyrNick - AdNeutralNick;

// Acceleration Sensor
	// sliding average sensor readings
	Mean_AccNick  = ((int32_t)Mean_AccNick * 1 + ((ACC_AMPLIFY * (int32_t)AdValueAccNick))) / 2L;
	Mean_AccRoll  = ((int32_t)Mean_AccRoll * 1 + ((ACC_AMPLIFY * (int32_t)AdValueAccRoll))) / 2L;
	Mean_AccTop   = ((int32_t)Mean_AccTop * 1 + ((int32_t)AdValueAccTop)) / 2L;

	// sum sensor readings for later averaging
    IntegralAccNick += ACC_AMPLIFY * AdValueAccNick;
    IntegralAccRoll  += ACC_AMPLIFY * AdValueAccRoll;

    NaviAccNick += AdValueAccNick;
    NaviAccRoll  += AdValueAccRoll;
    NaviCntAcc++;

// Yaw
	// calculate yaw gyro integral (~ to rotation angle)
	Reading_IntegralGyroYaw  += Reading_GyroYaw;
	YawGyroHeading += Reading_GyroYaw;
    if(YawGyroHeading >= (360L * YAW_GYRO_DEG_FACTOR)) YawGyroHeading -= 360L * YAW_GYRO_DEG_FACTOR;  // 360° Wrap
	if(YawGyroHeading < 0)                             YawGyroHeading += 360L * YAW_GYRO_DEG_FACTOR;


	// Coupling fraction
	if(!Looping_Nick && !Looping_Roll && (ParamSet.GlobalConfig & CFG_AXIS_COUPLING_ACTIVE))
	{
		tmpl = (Reading_GyroYaw * Reading_IntegralGyroNick) / 2048L;
		tmpl *= FCParam.Yaw_PosFeedback;
		tmpl /= 4096L;
		tmpl2 = ( Reading_GyroYaw * Reading_IntegralGyroRoll) / 2048L;
		tmpl2 *= FCParam.Yaw_PosFeedback;
		tmpl2 /= 4096L;
		if(labs(tmpl) > 128 || labs(tmpl2) > 128) FunnelCourse = 1;
	}
	else  tmpl = tmpl2 = 0;

// Roll
	Reading_GyroRoll += tmpl;
	Reading_GyroRoll += (tmpl2 * FCParam.Yaw_NegFeedback) / 512L;
	Reading_IntegralGyroRoll2 += Reading_GyroRoll;
	Reading_IntegralGyroRoll +=  Reading_GyroRoll - AttitudeCorrectionRoll;
	if(Reading_IntegralGyroRoll > TurnOver180Roll)
	{
		Reading_IntegralGyroRoll  = -(TurnOver180Roll - 10000L);
		Reading_IntegralGyroRoll2 = Reading_IntegralGyroRoll;
	}
	if(Reading_IntegralGyroRoll < -TurnOver180Roll)
	{
		Reading_IntegralGyroRoll =  (TurnOver180Roll - 10000L);
		Reading_IntegralGyroRoll2 = Reading_IntegralGyroRoll;
	}
	if(AdValueGyrRoll < 15)   Reading_GyroRoll = -1000;
	if(AdValueGyrRoll <  7)   Reading_GyroRoll = -2000;
	if(BoardRelease == 10)
	{
		if(AdValueGyrRoll > 1010) Reading_GyroRoll = +1000;
		if(AdValueGyrRoll > 1017) Reading_GyroRoll = +2000;
	}
	else
	{
		if(AdValueGyrRoll > 2020) Reading_GyroRoll = +1000;
		if(AdValueGyrRoll > 2034) Reading_GyroRoll = +2000;
	}
// Nick
	Reading_GyroNick -= tmpl2;
	Reading_GyroNick -= (tmpl*FCParam.Yaw_NegFeedback) / 512L;
	Reading_IntegralGyroNick2 += Reading_GyroNick;
	Reading_IntegralGyroNick  += Reading_GyroNick - AttitudeCorrectionNick;
	if(Reading_IntegralGyroNick > TurnOver180Nick)
	{
	 Reading_IntegralGyroNick = -(TurnOver180Nick - 25000L);
	 Reading_IntegralGyroNick2 = Reading_IntegralGyroNick;
	}
	if(Reading_IntegralGyroNick < -TurnOver180Nick)
	{
	 Reading_IntegralGyroNick =  (TurnOver180Nick - 25000L);
	 Reading_IntegralGyroNick2 = Reading_IntegralGyroNick;
	}
	if(AdValueGyrNick < 15)   Reading_GyroNick = -1000;
	if(AdValueGyrNick <  7)   Reading_GyroNick = -2000;
	if(BoardRelease == 10)
	{
		if(AdValueGyrNick > 1010) Reading_GyroNick = +1000;
		if(AdValueGyrNick > 1017) Reading_GyroNick = +2000;
	}
	else
	{
		if(AdValueGyrNick > 2020) Reading_GyroNick = +1000;
		if(AdValueGyrNick > 2034) Reading_GyroNick = +2000;
	}

// start ADC again to capture measurement values for the next loop
    ADC_Enable();

    IntegralYaw    = Reading_IntegralGyroYaw;
    IntegralNick  = Reading_IntegralGyroNick;
    IntegralRoll   = Reading_IntegralGyroRoll;
    IntegralNick2 = Reading_IntegralGyroNick2;
    IntegralRoll2  = Reading_IntegralGyroRoll2;

	if((ParamSet.GlobalConfig & CFG_ROTARY_RATE_LIMITER) && !Looping_Nick && !Looping_Roll)
	{
		if(Reading_GyroNick > 200)       Reading_GyroNick += 4 * (Reading_GyroNick - 200);
		else if(Reading_GyroNick < -200) Reading_GyroNick += 4 * (Reading_GyroNick + 200);
		if(Reading_GyroRoll > 200)        Reading_GyroRoll  += 4 * (Reading_GyroRoll - 200);
		else if(Reading_GyroRoll < -200)  Reading_GyroRoll  += 4 * (Reading_GyroRoll + 200);
	}
}

/************************************************************************/
/*  Averaging Measurement Readings  for Calibration                     */
/************************************************************************/
void CalibMean(void)
{
	if(BoardRelease == 13) SearchGyroOffset();
    // stop ADC to avoid changing values during calculation
	ADC_Disable();

	Reading_GyroNick = AdValueGyrNick;
	Reading_GyroRoll  = AdValueGyrRoll;
	Reading_GyroYaw   = AdValueGyrYaw;

	Mean_AccNick = ACC_AMPLIFY * (int32_t)AdValueAccNick;
	Mean_AccRoll  = ACC_AMPLIFY * (int32_t)AdValueAccRoll;
	Mean_AccTop   = (int32_t)AdValueAccTop;
    // start ADC (enables internal trigger so that the ISR in analog.c
    // updates the readings once)
    ADC_Enable();

	TurnOver180Nick = (int32_t) ParamSet.AngleTurnOverNick * 2500L;
	TurnOver180Roll =  (int32_t) ParamSet.AngleTurnOverRoll  * 2500L;
}

/************************************************************************/
/*  Transmit Motor Data via I2C                                         */
/************************************************************************/
void SendMotorData(void)
{
    if(!(MKFlags & MKFLAG_MOTOR_RUN))
    {
        Motor_Rear = 0;
        Motor_Front = 0;
        Motor_Right = 0;
        Motor_Left = 0;
        if(MotorTest[0]) Motor_Front = MotorTest[0];
        if(MotorTest[1]) Motor_Rear  = MotorTest[1];
        if(MotorTest[2]) Motor_Left  = MotorTest[2];
        if(MotorTest[3]) Motor_Right = MotorTest[3];
        MKFlags &= ~(MKFLAG_FLY|MKFLAG_START); // clear flag FLY and START if motors are off
    }
    DebugOut.Analog[12] = Motor_Front;
    DebugOut.Analog[13] = Motor_Rear;
    DebugOut.Analog[14] = Motor_Left;
    DebugOut.Analog[15] = Motor_Right;

    //Start I2C Interrupt Mode
    twi_state = TWI_STATE_MOTOR_TX;
    I2C_Start();
}



/************************************************************************/
/*  Maps the parameter to poti values                                   */
/************************************************************************/
void ParameterMapping(void)
{
	if(RC_Quality > 160) // do the mapping of RC-Potis only if the rc-signal is ok
	// else the last updated values are used
	{
		 //update poti values by rc-signals
		#define CHK_POTI_MM(b,a,min,max) { if(a > 250) { if(a == 251) b = Poti1; else if(a == 252) b = Poti2; else if(a == 253) b = Poti3; else if(a == 254) b = Poti4;} else b = a; if(b <= min) b = min; else if(b >= max) b = max;}
		#define CHK_POTI(b,a) { if(a > 250) { if(a == 251) b = Poti1; else if(a == 252) b = Poti2; else if(a == 253) b = Poti3; else if(a == 254) b = Poti4;} else b = a;}
		CHK_POTI(FCParam.MaxHeight,ParamSet.MaxHeight);
		CHK_POTI_MM(FCParam.Height_D,ParamSet.Height_D,0,100);
		CHK_POTI_MM(FCParam.Height_P,ParamSet.Height_P,0,100);
		CHK_POTI(FCParam.Height_ACC_Effect,ParamSet.Height_ACC_Effect);
		CHK_POTI(FCParam.CompassYawEffect,ParamSet.CompassYawEffect);
		CHK_POTI_MM(FCParam.Gyro_P,ParamSet.Gyro_P,10,255);
		CHK_POTI(FCParam.Gyro_I,ParamSet.Gyro_I);
		CHK_POTI(FCParam.I_Factor,ParamSet.I_Factor);
		CHK_POTI(FCParam.UserParam1,ParamSet.UserParam1);
		CHK_POTI(FCParam.UserParam2,ParamSet.UserParam2);
		CHK_POTI(FCParam.UserParam3,ParamSet.UserParam3);
		CHK_POTI(FCParam.UserParam4,ParamSet.UserParam4);
		CHK_POTI(FCParam.UserParam5,ParamSet.UserParam5);
		CHK_POTI(FCParam.UserParam6,ParamSet.UserParam6);
		CHK_POTI(FCParam.UserParam7,ParamSet.UserParam7);
		CHK_POTI(FCParam.UserParam8,ParamSet.UserParam8);
		CHK_POTI(FCParam.ServoNickControl,ParamSet.ServoNickControl);
		CHK_POTI(FCParam.LoopGasLimit,ParamSet.LoopGasLimit);
		CHK_POTI(FCParam.Yaw_PosFeedback,ParamSet.Yaw_PosFeedback);
		CHK_POTI(FCParam.Yaw_NegFeedback,ParamSet.Yaw_NegFeedback);
		CHK_POTI(FCParam.DynamicStability,ParamSet.DynamicStability);
		CHK_POTI_MM(FCParam.J16Timing,ParamSet.J16Timing,1,255);
		CHK_POTI_MM(FCParam.J17Timing,ParamSet.J17Timing,1,255);
		#if (defined (USE_KILLAGREG) || defined (USE_MK3MAG))
		CHK_POTI(FCParam.NaviGpsModeControl,ParamSet.NaviGpsModeControl);
		CHK_POTI(FCParam.NaviGpsGain,ParamSet.NaviGpsGain);
		CHK_POTI(FCParam.NaviGpsP,ParamSet.NaviGpsP);
		CHK_POTI(FCParam.NaviGpsI,ParamSet.NaviGpsI);
		CHK_POTI(FCParam.NaviGpsD,ParamSet.NaviGpsD);
		CHK_POTI(FCParam.NaviGpsACC,ParamSet.NaviGpsACC);
		CHK_POTI_MM(FCParam.NaviOperatingRadius,ParamSet.NaviOperatingRadius,10, 255);
		CHK_POTI(FCParam.NaviWindCorrection,ParamSet.NaviWindCorrection);
		CHK_POTI(FCParam.NaviSpeedCompensation,ParamSet.NaviSpeedCompensation);
		#endif
		CHK_POTI(FCParam.ExternalControl,ParamSet.ExternalControl);
		Ki = (float) FCParam.I_Factor * FACTOR_I;
	}
}


void SetCompassCalState(void)
{
	static uint8_t stick = 1;

    // if nick is centered or top set stick to zero
	if(PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > -20) stick = 0;
	// if nick is down trigger to next cal state
	if((PPM_in[ParamSet.ChannelAssignment[CH_NICK]] < -70) && !stick)
	{
		stick = 1;
		CompassCalState++;
		if(CompassCalState < 5) Beep(CompassCalState);
		else BeepTime = 1000;
	}
}



/************************************************************************/
/*  MotorControl                                                        */
/************************************************************************/
void MotorControl(void)
{
	int16_t MotorValue, pd_result, h, tmp_int;
	int16_t YawMixFraction, GasMixFraction;
	static int32_t SumNick = 0, SumRoll = 0;
	static int32_t SetPointYaw = 0;
	static int32_t IntegralErrorNick = 0;
	static int32_t IntegralErrorRoll = 0;
	static uint16_t RcLostTimer;
	static uint8_t delay_neutral = 0, delay_startmotors = 0, delay_stopmotors = 0;
	static uint8_t HeightControlActive = 0;
	static int16_t HeightControlGas = 0;
	static int8_t TimerDebugOut = 0;
	static uint16_t UpdateCompassCourse = 0;
	static int32_t CorrectionNick, CorrectionRoll;

	Mean();
	GRN_ON;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// determine gas value
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   	GasMixFraction = StickGas;
    if(GasMixFraction < ParamSet.Gas_Min + 10) GasMixFraction = ParamSet.Gas_Min + 10;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// RC-signal is bad
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(RC_Quality < 120)  // the rc-frame signal is not reveived or noisy
	{
		if(!PcAccess) // if also no PC-Access via UART
		{
			if(BeepModulation == 0xFFFF)
			{
			 BeepTime = 15000; // 1.5 seconds
			 BeepModulation = 0x0C00;
			}
		}
		if(RcLostTimer) RcLostTimer--; // decremtent timer after rc sigal lost
		else // rc lost countdown finished
		{
		  MKFlags &= ~(MKFLAG_MOTOR_RUN|MKFLAG_EMERGENCY_LANDING); // clear motor run flag that stop the motors in SendMotorData()
		}
		RED_ON; // set red led
		if(Model_Is_Flying > 1000)  // wahrscheinlich in der Luft --> langsam absenken
		{
			GasMixFraction = ParamSet.EmergencyGas; // set emergency gas
			MKFlags |= (MKFLAG_EMERGENCY_LANDING); // ser flag fpr emergency landing
			// set neutral rc inputs
			PPM_diff[ParamSet.ChannelAssignment[CH_NICK]] = 0;
			PPM_diff[ParamSet.ChannelAssignment[CH_ROLL]] = 0;
			PPM_diff[ParamSet.ChannelAssignment[CH_YAW]] = 0;
			PPM_in[ParamSet.ChannelAssignment[CH_NICK]] = 0;
			PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] = 0;
			PPM_in[ParamSet.ChannelAssignment[CH_YAW]] = 0;
		}
		else MKFlags &= ~(MKFLAG_MOTOR_RUN); // clear motor run flag that stop the motors in SendMotorData()
	} // eof RC_Quality < 120
	else
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// RC-signal is good
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(RC_Quality > 140)
	{
		MKFlags &= ~(MKFLAG_EMERGENCY_LANDING); // clear flag for emergency landing
		// reset emergency timer
		RcLostTimer = ParamSet.EmergencyGasDuration * 50;
		if(GasMixFraction > 40 && (MKFlags & MKFLAG_MOTOR_RUN) )
		{
			if(Model_Is_Flying < 0xFFFF) Model_Is_Flying++;
		}
		if(Model_Is_Flying < 256)
		{
			SumNick = 0;
			SumRoll = 0;
			StickYaw = 0;
			if(Model_Is_Flying == 250)
			{
				UpdateCompassCourse = 1;
				Reading_IntegralGyroYaw = 0;
				SetPointYaw = 0;
			}
		}
		else MKFlags |= (MKFLAG_FLY); // set fly flag

		if(Poti1 < PPM_in[ParamSet.ChannelAssignment[CH_POTI1]] + 110) Poti1++; else if(Poti1 > PPM_in[ParamSet.ChannelAssignment[CH_POTI1]] + 110 && Poti1) Poti1--;
		if(Poti2 < PPM_in[ParamSet.ChannelAssignment[CH_POTI2]] + 110) Poti2++; else if(Poti2 > PPM_in[ParamSet.ChannelAssignment[CH_POTI2]] + 110 && Poti2) Poti2--;
		if(Poti3 < PPM_in[ParamSet.ChannelAssignment[CH_POTI3]] + 110) Poti3++; else if(Poti3 > PPM_in[ParamSet.ChannelAssignment[CH_POTI3]] + 110 && Poti3) Poti3--;
		if(Poti4 < PPM_in[ParamSet.ChannelAssignment[CH_POTI4]] + 110) Poti4++; else if(Poti4 > PPM_in[ParamSet.ChannelAssignment[CH_POTI4]] + 110 && Poti4) Poti4--;
		//PPM24-Extension
		if(Poti5 < PPM_in[9] + 110)  Poti5++; else if(Poti5 >  PPM_in[9] + 110 && Poti5) Poti5--;
		if(Poti6 < PPM_in[10] + 110) Poti6++; else if(Poti6 > PPM_in[10] + 110 && Poti6) Poti6--;
		if(Poti7 < PPM_in[11] + 110) Poti7++; else if(Poti7 > PPM_in[11] + 110 && Poti7) Poti7--;
		if(Poti8 < PPM_in[12] + 110) Poti8++; else if(Poti8 > PPM_in[12] + 110 && Poti8) Poti8--;
		//limit poti values
		if(Poti1 < 0) Poti1 = 0; else if(Poti1 > 255) Poti1 = 255;
		if(Poti2 < 0) Poti2 = 0; else if(Poti2 > 255) Poti2 = 255;
		if(Poti3 < 0) Poti3 = 0; else if(Poti3 > 255) Poti3 = 255;
		if(Poti4 < 0) Poti4 = 0; else if(Poti4 > 255) Poti4 = 255;
		//PPM24-Extension
		if(Poti5 < 0) Poti5 = 0; else if(Poti5 > 255) Poti5 = 255;
		if(Poti6 < 0) Poti6 = 0; else if(Poti6 > 255) Poti6 = 255;
		if(Poti7 < 0) Poti7 = 0; else if(Poti7 > 255) Poti7 = 255;
		if(Poti8 < 0) Poti8 = 0; else if(Poti8 > 255) Poti8 = 255;

		// if motors are off and the gas stick is in the upper position
		if((PPM_in[ParamSet.ChannelAssignment[CH_GAS]] > 80) && !(MKFlags & MKFLAG_MOTOR_RUN) )
		{
			// and if the yaw stick is in the leftmost position
			if(PPM_in[ParamSet.ChannelAssignment[CH_YAW]] > 75)
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// calibrate the neutral readings of all attitude sensors
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			{
				// gas/yaw joystick is top left
				//  _________
				// |x        |
				// |         |
				// |         |
				// |         |
				// |         |
				//  ¯¯¯¯¯¯¯¯¯
				if(++delay_neutral > 200)  // not immediately (wait 200 loops = 200 * 2ms = 0.4 s)
				{
					delay_neutral = 0;
					GRN_OFF;
					Model_Is_Flying = 0;
					// check roll/nick stick position
					// if nick stick is top or roll stick is left or right --> change parameter setting
					// according to roll/nick stick position
					if(PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > 70 || abs(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]]) > 70)
					{
						 uint8_t setting = 1; // default
						 // nick/roll joystick
						 //  _________
						 // |2   3   4|
						 // |         |
						 // |1       5|
						 // |         |
						 // |         |
						 //  ¯¯¯¯¯¯¯¯¯
						 // roll stick leftmost and nick stick centered --> setting 1
						 if(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] > 70 && PPM_in[ParamSet.ChannelAssignment[CH_NICK]] < 70) setting = 1;
						 // roll stick leftmost and nick stick topmost --> setting 2
						 if(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] > 70 && PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > 70) setting = 2;
						 // roll stick centered an nick stick topmost --> setting 3
						 if(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] < 70 && PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > 70) setting = 3;
						 // roll stick rightmost and nick stick topmost --> setting 4
						 if(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] <-70 && PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > 70) setting = 4;
						 // roll stick rightmost and nick stick centered --> setting 5
						 if(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] <-70 && PPM_in[ParamSet.ChannelAssignment[CH_NICK]] < 70) setting = 5;
						 // update active parameter set in eeprom
						 SetActiveParamSet(setting);
						 ParamSet_ReadFromEEProm(GetActiveParamSet());
						 SetNeutral();
						 Beep(GetActiveParamSet());
					}
					else
					{
						if(ParamSet.GlobalConfig & (CFG_COMPASS_ACTIVE|CFG_GPS_ACTIVE))
						{
							// if roll stick is centered and nick stick is down
							if (abs(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]]) < 30 && PPM_in[ParamSet.ChannelAssignment[CH_NICK]] < -70)
							{
								// nick/roll joystick
								//  _________
								// |         |
								// |         |
								// |         |
								// |         |
								// |    x    |
								//  ¯¯¯¯¯¯¯¯¯
								// enable calibration state of compass
								CompassCalState = 1;
								BeepTime = 1000;
							}
							else // nick and roll are centered
							{
								ParamSet_ReadFromEEProm(GetActiveParamSet());
								SetNeutral();
								Beep(GetActiveParamSet());
							}
						}
						else // nick and roll are centered
						{
							ParamSet_ReadFromEEProm(GetActiveParamSet());
							SetNeutral();
							Beep(GetActiveParamSet());
						}
					}
				}
			}
			// and if the yaw stick is in the rightmost position
			// save the ACC neutral setting to eeprom
			else if(PPM_in[ParamSet.ChannelAssignment[CH_YAW]] < -75)
			{
				if(++delay_neutral > 200)  // not immediately (wait 200 loops = 200 * 2ms = 0.4 s)
				{
					delay_neutral = 0;
					GRN_OFF;
					SetParamWord(PID_ACC_NICK, 0xFFFF); // make value invalid
					Model_Is_Flying = 0;
					SetNeutral();
					// Save ACC neutral settings to eeprom
					SetParamWord(PID_ACC_NICK, (uint16_t)NeutralAccX);
					SetParamWord(PID_ACC_ROLL,  (uint16_t)NeutralAccY);
					SetParamWord(PID_ACC_Z,     (uint16_t)NeutralAccZ);
					Beep(GetActiveParamSet());
				}
			}
			else delay_neutral = 0;
		}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// gas stick is down
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(PPM_in[ParamSet.ChannelAssignment[CH_GAS]] < -85)
		{
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// and yaw stick is rightmost --> start motors
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			if(PPM_in[ParamSet.ChannelAssignment[CH_YAW]] < -75)
			{
				if(++delay_startmotors > 200) // not immediately (wait 200 loops = 200 * 2ms = 0.4 s)
				{
					delay_startmotors = 200; // do not repeat if once executed
					Model_Is_Flying = 1;
					MKFlags |= (MKFLAG_MOTOR_RUN|MKFLAG_START); // set flag RUN and START
					SetPointYaw = 0;
					Reading_IntegralGyroYaw = 0;
					Reading_IntegralGyroNick = 0;
					Reading_IntegralGyroRoll = 0;
					Reading_IntegralGyroNick2 = IntegralNick;
					Reading_IntegralGyroRoll2 = IntegralRoll;
					SumNick = 0;
					SumRoll = 0;
				}
			}
			else delay_startmotors = 0; // reset delay timer if sticks are not in this position
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// and yaw stick is leftmost --> stop motors
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			if(PPM_in[ParamSet.ChannelAssignment[CH_YAW]] > 75)
				{
				if(++delay_stopmotors > 200)  // not immediately (wait 200 loops = 200 * 2ms = 0.4 s)
				{
					delay_stopmotors = 200; // do not repeat if once executed
					Model_Is_Flying = 0;
					MKFlags &= ~(MKFLAG_MOTOR_RUN);
				}
			}
			else delay_stopmotors = 0; // reset delay timer if sticks are not in this position
		}
			// remapping of paameters only if the signal rc-sigbnal conditions are good
	} // eof RC_Quality > 150
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// new values from RC
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(!NewPpmData-- || (MKFlags & MKFLAG_EMERGENCY_LANDING) ) // NewData = 0 means new data from RC
	{
		ParameterMapping(); // remapping params (online poti replacement)
		// calculate Stick inputs by rc channels (P) and changing of rc channels (D)
		StickNick = (StickNick * 3 + PPM_in[ParamSet.ChannelAssignment[CH_NICK]] * ParamSet.Stick_P) / 4;
		StickNick += PPM_diff[ParamSet.ChannelAssignment[CH_NICK]] * ParamSet.Stick_D;
		StickNick -= (GPS_Nick);

		StickRoll = (StickRoll * 3 + PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] * ParamSet.Stick_P) / 4;
		StickRoll += PPM_diff[ParamSet.ChannelAssignment[CH_ROLL]] * ParamSet.Stick_D;
		StickRoll -= (GPS_Roll);

		// direct mapping of yaw and gas
		StickYaw = -PPM_in[ParamSet.ChannelAssignment[CH_YAW]];
		StickGas  = PPM_in[ParamSet.ChannelAssignment[CH_GAS]] + 120;// shift to positive numbers

		// update gyro control loop factors
		Gyro_P_Factor = ((float) FCParam.Gyro_P + 10.0) / (256.0 / STICK_GAIN);
		Gyro_I_Factor = ((float) FCParam.Gyro_I) / (44000 / STICK_GAIN);


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+ Analog control via serial communication
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		if(ExternControl.Config & 0x01 && FCParam.ExternalControl > 128)
		{
			 StickNick += (int16_t) ExternControl.Nick * (int16_t) ParamSet.Stick_P;
			 StickRoll += (int16_t) ExternControl.Roll * (int16_t) ParamSet.Stick_P;
			 StickYaw += ExternControl.Yaw;
			 ExternHeightValue =  (int16_t) ExternControl.Height * (int16_t)ParamSet.Height_Gain;
			 if(ExternControl.Gas < StickGas) StickGas = ExternControl.Gas;
		}
		if(StickGas < 0) StickGas = 0;

		// disable I part of gyro control feedback
		if(ParamSet.GlobalConfig & CFG_HEADING_HOLD) Gyro_I_Factor =  0;
		// avoid negative scaling factors
		if(Gyro_P_Factor < 0) Gyro_P_Factor = 0;
		if(Gyro_I_Factor < 0) Gyro_I_Factor = 0;


		// update max stick positions for nick and roll

		if(abs(StickNick / STICK_GAIN) > MaxStickNick)
		{
			MaxStickNick = abs(StickNick)/STICK_GAIN;
			if(MaxStickNick > 100) MaxStickNick = 100;
		}
		else MaxStickNick--;
		if(abs(StickRoll / STICK_GAIN) > MaxStickRoll)
		{
			MaxStickRoll = abs(StickRoll)/STICK_GAIN;
			if(MaxStickRoll > 100) MaxStickRoll = 100;
		}
		else MaxStickRoll--;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Looping?
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		if((PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] > ParamSet.LoopThreshold) && ParamSet.BitConfig & CFG_LOOP_LEFT)  Looping_Left = 1;
		else
		{
			if(Looping_Left) // Hysteresis
		 	{
		  		if((PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] < (ParamSet.LoopThreshold - ParamSet.LoopHysteresis))) Looping_Left = 0;
		 	}
		}
		if((PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] < -ParamSet.LoopThreshold) && ParamSet.BitConfig & CFG_LOOP_RIGHT) Looping_Right = 1;
		else
		{
			if(Looping_Right) // Hysteresis
		 	{
		  		if(PPM_in[ParamSet.ChannelAssignment[CH_ROLL]] > -(ParamSet.LoopThreshold - ParamSet.LoopHysteresis)) Looping_Right = 0;
		 	}
		}

		if((PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > ParamSet.LoopThreshold) && ParamSet.BitConfig & CFG_LOOP_UP) Looping_Top = 1;
		else
		{
			if(Looping_Top)  // Hysteresis
		 	{
		  		if((PPM_in[ParamSet.ChannelAssignment[CH_NICK]] < (ParamSet.LoopThreshold - ParamSet.LoopHysteresis))) Looping_Top = 0;
		 	}
		}
		if((PPM_in[ParamSet.ChannelAssignment[CH_NICK]] < -ParamSet.LoopThreshold) && ParamSet.BitConfig & CFG_LOOP_DOWN) Looping_Down = 1;
		else
		{
			if(Looping_Down) // Hysteresis
		 	{
		  		if(PPM_in[ParamSet.ChannelAssignment[CH_NICK]] > -(ParamSet.LoopThreshold - ParamSet.LoopHysteresis)) Looping_Down = 0;
		 	}
		}

		if(Looping_Left || Looping_Right)   Looping_Roll = 1; else Looping_Roll = 0;
		if(Looping_Top  || Looping_Down) {Looping_Nick = 1; Looping_Roll = 0; Looping_Left = 0; Looping_Right = 0;} else Looping_Nick = 0;
	} // End of new RC-Values or Emergency Landing


	if(Looping_Roll || Looping_Nick)
	{
		if(GasMixFraction > ParamSet.LoopGasLimit) GasMixFraction = ParamSet.LoopGasLimit;
	}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// in case of emergency landing
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// set all inputs to save values
	if(MKFlags & MKFLAG_EMERGENCY_LANDING)
	{
		StickYaw = 0;
		StickNick = 0;
		StickRoll = 0;
		Gyro_P_Factor  = (float) 100 / (256.0 / STICK_GAIN);
		Gyro_I_Factor = (float) 120 / (44000 / STICK_GAIN);
		Looping_Roll = 0;
		Looping_Nick = 0;
		MaxStickNick = 0;
		MaxStickRoll = 0;
	}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Trim Gyro-Integrals to ACC-Signals
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define BALANCE_NUMBER 256L
	// sum for averaging
	MeanIntegralNick  += IntegralNick;
	MeanIntegralRoll  += IntegralRoll;

	if(Looping_Nick || Looping_Roll) // if looping in any direction
	{
		// reset averaging for acc and gyro integral as well as gyro integral acc correction
		MeasurementCounter = 0;

		IntegralAccNick = 0;
		IntegralAccRoll = 0;

		MeanIntegralNick = 0;
		MeanIntegralRoll = 0;

		Reading_IntegralGyroNick2 = Reading_IntegralGyroNick;
		Reading_IntegralGyroRoll2 = Reading_IntegralGyroRoll;

		AttitudeCorrectionNick = 0;
		AttitudeCorrectionRoll = 0;
	}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(!Looping_Nick && !Looping_Roll) // if not lopping in any direction
	{
		int32_t tmp_long, tmp_long2;
		if(FCParam.Kalman_K != -1)
		{
			// determine the deviation of gyro integral from averaged acceleration sensor
			tmp_long   = (int32_t)(IntegralNick / ParamSet.GyroAccFactor - (int32_t)Mean_AccNick);
			tmp_long   = (tmp_long * FCParam.Kalman_K) / (32 * 16);
			tmp_long2  = (int32_t)(IntegralRoll   / ParamSet.GyroAccFactor - (int32_t)Mean_AccRoll);
			tmp_long2  = (tmp_long2 * FCParam.Kalman_K) / (32 * 16);

			if((MaxStickNick > 64) || (MaxStickRoll > 64)) // reduce effect during stick commands
			{
				tmp_long  /= 2;
				tmp_long2 /= 2;
			}
			if(abs(PPM_in[ParamSet.ChannelAssignment[CH_YAW]]) > 25) // reduce further if yaw stick is active
			{
				tmp_long  /= 3;
				tmp_long2 /= 3;
			}
			// limit correction effect
			if(tmp_long >  (int32_t)FCParam.Kalman_MaxFusion)  tmp_long  = (int32_t)FCParam.Kalman_MaxFusion;
			if(tmp_long < -(int32_t)FCParam.Kalman_MaxFusion)  tmp_long  =-(int32_t)FCParam.Kalman_MaxFusion;
			if(tmp_long2 > (int32_t)FCParam.Kalman_MaxFusion)  tmp_long2 = (int32_t)FCParam.Kalman_MaxFusion;
			if(tmp_long2 <-(int32_t)FCParam.Kalman_MaxFusion)  tmp_long2 =-(int32_t)FCParam.Kalman_MaxFusion;
		}
		else
		{
			// determine the deviation of gyro integral from averaged acceleration sensor
			tmp_long   =  (int32_t)(IntegralNick / ParamSet.GyroAccFactor - (int32_t)Mean_AccNick);
			tmp_long  /= 16;
			tmp_long2  = (int32_t)(IntegralRoll   / ParamSet.GyroAccFactor - (int32_t)Mean_AccRoll);
			tmp_long2 /= 16;

			if((MaxStickNick > 64) || (MaxStickRoll > 64)) // reduce effect during stick commands
			{
				tmp_long  /= 3;
				tmp_long2 /= 3;
			}
			if(abs(PPM_in[ParamSet.ChannelAssignment[CH_YAW]]) > 25) // reduce further if yaw stick is active
			{
				tmp_long  /= 3;
				tmp_long2 /= 3;
			}

			#define BALANCE 32
			// limit correction effect
			if(tmp_long >  BALANCE)  tmp_long  = BALANCE;
			if(tmp_long < -BALANCE)  tmp_long  =-BALANCE;
			if(tmp_long2 > BALANCE)  tmp_long2 = BALANCE;
			if(tmp_long2 <-BALANCE)  tmp_long2 =-BALANCE;
		}
		// correct current readings
		Reading_IntegralGyroNick -= tmp_long;
		Reading_IntegralGyroRoll -= tmp_long2;
	}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// MeasurementCounter is incremented in the isr of analog.c
	if(MeasurementCounter >= BALANCE_NUMBER) // averaging number has reached
	{
		static int16_t cnt = 0;
		static int8_t last_n_p, last_n_n, last_r_p, last_r_n;
		static int32_t MeanIntegralNick_old, MeanIntegralRoll_old;

		// if not lopping in any direction (this should be alwais the case,
		// because the Measurement counter is reset to 0 if looping in any direction is active.)
		if(!Looping_Nick && !Looping_Roll && !FunnelCourse)
		{
			// Calculate mean value of the gyro integrals
			MeanIntegralNick /= BALANCE_NUMBER;
			MeanIntegralRoll  /= BALANCE_NUMBER;

			// Calculate mean of the acceleration values
			IntegralAccNick = (ParamSet.GyroAccFactor * IntegralAccNick) / BALANCE_NUMBER;
			IntegralAccRoll  = (ParamSet.GyroAccFactor * IntegralAccRoll ) / BALANCE_NUMBER;

			// Nick ++++++++++++++++++++++++++++++++++++++++++++++++
			// Calculate deviation of the averaged gyro integral and the averaged acceleration integral
			IntegralErrorNick = (int32_t)(MeanIntegralNick - (int32_t)IntegralAccNick);
			CorrectionNick = IntegralErrorNick / ParamSet.GyroAccTrim;
			AttitudeCorrectionNick = CorrectionNick / BALANCE_NUMBER;
			// Roll ++++++++++++++++++++++++++++++++++++++++++++++++
			// Calculate deviation of the averaged gyro integral and the averaged acceleration integral
			IntegralErrorRoll = (int32_t)(MeanIntegralRoll - (int32_t)IntegralAccRoll);
			CorrectionRoll  = IntegralErrorRoll / ParamSet.GyroAccTrim;
			AttitudeCorrectionRoll  = CorrectionRoll  / BALANCE_NUMBER;

			if(((MaxStickNick > 64) || (MaxStickRoll > 64) || (abs(PPM_in[ParamSet.ChannelAssignment[CH_YAW]]) > 25)) && (FCParam.Kalman_K == -1) )
			{
				AttitudeCorrectionNick /= 2;
				AttitudeCorrectionRoll /= 2;
			}

	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Gyro-Drift ermitteln
	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			// deviation of gyro nick integral (IntegralNick is corrected by averaged acc sensor)
			IntegralErrorNick  = IntegralNick2 - IntegralNick;
			Reading_IntegralGyroNick2 -= IntegralErrorNick;
			// deviation of gyro nick integral (IntegralNick is corrected by averaged acc sensor)
			IntegralErrorRoll = IntegralRoll2 - IntegralRoll;
			Reading_IntegralGyroRoll2 -= IntegralErrorRoll;

			if(YawGyroDrift >  BALANCE_NUMBER/2) AdNeutralYaw++;
			if(YawGyroDrift < -BALANCE_NUMBER/2) AdNeutralYaw--;
			YawGyroDrift = 0;

			#define ERROR_LIMIT  (BALANCE_NUMBER * 4)
			#define ERROR_LIMIT2 (BALANCE_NUMBER * 16)
			#define MOVEMENT_LIMIT 20000
	// Nick +++++++++++++++++++++++++++++++++++++++++++++++++
			cnt = 1;// + labs(IntegralErrorNick) / 4096;
			CorrectionNick = 0;
			if((labs(MeanIntegralNick_old - MeanIntegralNick) < MOVEMENT_LIMIT) || (FCParam.Kalman_MaxDrift > 3* 16))
			{
				if(IntegralErrorNick >  ERROR_LIMIT2)
				{
					if(last_n_p)
					{
						cnt += labs(IntegralErrorNick) / ERROR_LIMIT2;
						CorrectionNick = IntegralErrorNick / 8;
						if(CorrectionNick > 5000) CorrectionNick = 5000;
						AttitudeCorrectionNick += CorrectionNick / BALANCE_NUMBER;
					}
					else last_n_p = 1;
				}
				else  last_n_p = 0;
				if(IntegralErrorNick < -ERROR_LIMIT2)
				{
					if(last_n_n)
					{
						cnt += labs(IntegralErrorNick) / ERROR_LIMIT2;
						CorrectionNick = IntegralErrorNick / 8;
						if(CorrectionNick < -5000) CorrectionNick = -5000;
						AttitudeCorrectionNick += CorrectionNick / BALANCE_NUMBER;
					}
					else last_n_n = 1;
				}
				else  last_n_n = 0;
			}
			else
			{
				cnt = 0;
				BadCompassHeading = 1000;
			}
			if(cnt > ParamSet.DriftComp) cnt = ParamSet.DriftComp;
			if(cnt * 16 > FCParam.Kalman_MaxDrift) cnt = FCParam.Kalman_MaxDrift / 16;
			// correct Gyro Offsets
			if(IntegralErrorNick >  ERROR_LIMIT)   AdNeutralNick += cnt;
			if(IntegralErrorNick < -ERROR_LIMIT)   AdNeutralNick -= cnt;

	// Roll +++++++++++++++++++++++++++++++++++++++++++++++++
			cnt = 1;// + labs(IntegralErrorNick) / 4096;
			CorrectionRoll = 0;
			if((labs(MeanIntegralRoll_old - MeanIntegralRoll) < MOVEMENT_LIMIT) || (FCParam.Kalman_MaxDrift > 3 * 16))
			{
				if(IntegralErrorRoll >  ERROR_LIMIT2)
				{
					if(last_r_p)
					{
						cnt += labs(IntegralErrorRoll) / ERROR_LIMIT2;
						CorrectionRoll = IntegralErrorRoll / 8;
						if(CorrectionRoll > 5000) CorrectionRoll = 5000;
						AttitudeCorrectionRoll += CorrectionRoll / BALANCE_NUMBER;
					}
					else last_r_p = 1;
				}
				else  last_r_p = 0;
				if(IntegralErrorRoll < -ERROR_LIMIT2)
				{
					if(last_r_n)
					{
						cnt += labs(IntegralErrorRoll) / ERROR_LIMIT2;
						CorrectionRoll = IntegralErrorRoll / 8;
						if(CorrectionRoll < -5000) CorrectionRoll = -5000;
						AttitudeCorrectionRoll += CorrectionRoll / BALANCE_NUMBER;
					}
					else last_r_n = 1;
				}
				else  last_r_n = 0;
			}
			else
			{
				cnt = 0;
				BadCompassHeading = 1000;
			}
			// correct Gyro Offsets
			if(cnt > ParamSet.DriftComp) cnt = ParamSet.DriftComp;
			if(cnt * 16 > FCParam.Kalman_MaxDrift) cnt = FCParam.Kalman_MaxDrift / 16;
			if(IntegralErrorRoll >  ERROR_LIMIT)   AdNeutralRoll += cnt;
			if(IntegralErrorRoll < -ERROR_LIMIT)   AdNeutralRoll -= cnt;

		}
		else // looping is active
		{
			AttitudeCorrectionRoll  = 0;
			AttitudeCorrectionNick = 0;
			FunnelCourse = 0;
		}

		// if Gyro_I_Factor == 0 , for example at Heading Hold, ignore attitude correction
		if(!Gyro_I_Factor)
		{
			AttitudeCorrectionRoll  = 0;
			AttitudeCorrectionNick = 0;
		}
	// +++++++++++++++++++++++++++++++++++++++++++++++++++++
		MeanIntegralNick_old = MeanIntegralNick;
		MeanIntegralRoll_old  = MeanIntegralRoll;
	// +++++++++++++++++++++++++++++++++++++++++++++++++++++
		// reset variables used for averaging
		IntegralAccNick = 0;
		IntegralAccRoll = 0;
		MeanIntegralNick = 0;
		MeanIntegralRoll = 0;
		MeasurementCounter = 0;
	} // end of averaging


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Yawing
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(abs(StickYaw) > 15 ) // yaw stick is activated
	{
		BadCompassHeading = 1000;
		if(!(ParamSet.GlobalConfig & CFG_COMPASS_FIX))
		{
			UpdateCompassCourse = 1;
		}
	}
	// exponential stick sensitivity in yawring rate
	tmp_int  = (int32_t) ParamSet.Yaw_P * ((int32_t)StickYaw * abs(StickYaw)) / 512L; // expo  y = ax + bx²
	tmp_int += (ParamSet.Yaw_P * StickYaw) / 4;
	SetPointYaw = tmp_int;
	// trimm drift of Reading_IntegralGyroYaw with SetPointYaw(StickYaw)
	Reading_IntegralGyroYaw -= tmp_int;
	// limit the effect
	if(Reading_IntegralGyroYaw > 50000) Reading_IntegralGyroYaw = 50000;
	if(Reading_IntegralGyroYaw <-50000) Reading_IntegralGyroYaw =-50000;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Compass
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // compass code is used if Compass option is selected
	if(ParamSet.GlobalConfig & (CFG_COMPASS_ACTIVE|CFG_GPS_ACTIVE))
	{
		int16_t w, v, r,correction, error;

		if(CompassCalState && !(MKFlags & MKFLAG_MOTOR_RUN) )
		{
			SetCompassCalState();
			#ifdef USE_KILLAGREG
			MM3_Calibrate();
			#endif
		}
		else
		{
			#ifdef USE_KILLAGREG
			static uint8_t updCompass = 0;
			if (!updCompass--)
			{
				updCompass = 49; // update only at 2ms*50 = 100ms (10Hz)
				MM3_Heading();
			}
			#endif

			// get maximum attitude angle
			w = abs(IntegralNick / 512);
			v = abs(IntegralRoll / 512);
			if(v > w) w = v;
			correction = w / 8 + 1;
			// calculate the deviation of the yaw gyro heading and the compass heading
			if (CompassHeading < 0) error = 0; // disable yaw drift compensation if compass heading is undefined
			else error = ((540 + CompassHeading - (YawGyroHeading / YAW_GYRO_DEG_FACTOR)) % 360) - 180;
			if(UpdateCompassCourse)
			{
				error = 0;
				YawGyroHeading = CompassHeading * YAW_GYRO_DEG_FACTOR;
			}
			if(!BadCompassHeading && w < 25)
			{
				YawGyroDrift += error;
				if(UpdateCompassCourse)
				{
					BeepTime = 200;
					CompassCourse = (YawGyroHeading / YAW_GYRO_DEG_FACTOR);
					UpdateCompassCourse = 0;
				}
			}
			YawGyroHeading += (error * 8) / correction;
			w = (w * FCParam.CompassYawEffect) / 32;
			w = FCParam.CompassYawEffect - w;
			if(w >= 0)
			{
				if(!BadCompassHeading)
				{
					v = 64 + (MaxStickNick + MaxStickRoll) / 8;
					// calc course deviation
					r = ((540 + (YawGyroHeading / YAW_GYRO_DEG_FACTOR) - CompassCourse) % 360) - 180;
					v = (r * w) / v; // align to compass course
					// limit yaw rate
					w = 3 * FCParam.CompassYawEffect;
					if (v > w) v = w;
					else if (v < -w) v = -w;
					Reading_IntegralGyroYaw += v;
				}
				else
				{ // wait a while
					BadCompassHeading--;
				}
			}
			else
			{  // ignore compass at extreme attitudes for a while
				BadCompassHeading = 500;
			}
		}
	}

	#if (defined (USE_KILLAGREG) || defined (USE_MK3MAG))
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  GPS
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(ParamSet.GlobalConfig & CFG_GPS_ACTIVE)
	{
		GPS_Main();
		MKFlags &= ~(MKFLAG_CALIBRATE | MKFLAG_START);
	}
	else
	{
		GPS_Nick = 0;
		GPS_Roll = 0;
	}
	#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Debugwerte zuordnen
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(!TimerDebugOut--)
	{
		TimerDebugOut = 24; // update debug outputs every 25*2ms = 50 ms (20Hz)
		DebugOut.Analog[0]  = IntegralNick / ParamSet.GyroAccFactor;
		DebugOut.Analog[1]  = IntegralRoll / ParamSet.GyroAccFactor;
		DebugOut.Analog[2]  = Mean_AccNick;
		DebugOut.Analog[3]  = Mean_AccRoll;
		DebugOut.Analog[4]  = Reading_GyroYaw;
		DebugOut.Analog[5]  = ReadingHeight;
		DebugOut.Analog[6]  = (Reading_Integral_Top / 512);
		DebugOut.Analog[8]  = CompassHeading;
		DebugOut.Analog[9]  = UBat;
		DebugOut.Analog[10] = RC_Quality;
		DebugOut.Analog[11] = YawGyroHeading / YAW_GYRO_DEG_FACTOR;
		//DebugOut.Analog[16] = Mean_AccTop;
		//DebugOut.Analog[17] = FromNaviCtrl_Value.Distance;
		//DebugOut.Analog[18] = FromNaviCtrl_Value.OsdBar;
		DebugOut.Analog[27] = (int16_t)FCParam.Kalman_MaxDrift;
		DebugOut.Analog[29] = (int16_t)FCParam.Kalman_K;
		DebugOut.Analog[30] = GPS_Nick;
		DebugOut.Analog[31] = GPS_Roll;

	}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  calculate control feedback from angle (gyro integral) and agular velocity (gyro signal)
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if(Looping_Nick) 
		Reading_GyroNick = Reading_GyroNick * Gyro_P_Factor;
	else 
		Reading_GyroNick = IntegralNick * Gyro_I_Factor + Reading_GyroNick * Gyro_P_Factor;

	
	if(Looping_Roll) Reading_GyroRoll = Reading_GyroRoll * Gyro_P_Factor;
	else Reading_GyroRoll = IntegralRoll * Gyro_I_Factor + Reading_GyroRoll * Gyro_P_Factor;
	Reading_GyroYaw = Reading_GyroYaw * (2 * Gyro_P_Factor) + IntegralYaw * Gyro_I_Factor / 2;

	DebugOut.Analog[21] = Reading_GyroNick;
	DebugOut.Analog[22] = Reading_GyroRoll;

	// limit control feedback
	#define MAX_SENSOR  (4096 * STICK_GAIN)
	if(Reading_GyroNick >  MAX_SENSOR) Reading_GyroNick =  MAX_SENSOR;
	if(Reading_GyroNick < -MAX_SENSOR) Reading_GyroNick = -MAX_SENSOR;
	if(Reading_GyroRoll >  MAX_SENSOR) Reading_GyroRoll =  MAX_SENSOR;
	if(Reading_GyroRoll < -MAX_SENSOR) Reading_GyroRoll = -MAX_SENSOR;
	if(Reading_GyroYaw  >  MAX_SENSOR) Reading_GyroYaw  =  MAX_SENSOR;
	if(Reading_GyroYaw  < -MAX_SENSOR) Reading_GyroYaw  = -MAX_SENSOR;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Height Control
// The height control algorithm reduces the gas but does not increase the gas.
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	GasMixFraction *= STICK_GAIN;

	// if height control is activated and no emergency landing is active
	if((ParamSet.GlobalConfig & CFG_HEIGHT_CONTROL) && !(MKFlags & MKFLAG_EMERGENCY_LANDING) )
	{
		int tmp_int;
		static uint8_t delay = 100;
		// if height control is activated by an rc channel
		if(ParamSet.GlobalConfig & CFG_HEIGHT_SWITCH)
		{	// check if parameter is less than activation threshold
			if(
				( (ParamSet.BitConfig & CFG_HEIGHT_3SWITCH) && ( (FCParam.MaxHeight > 80) && (FCParam.MaxHeight < 140) ) )|| // for 3-state switch height control is only disabled in center position
				(!(ParamSet.BitConfig & CFG_HEIGHT_3SWITCH) && (FCParam.MaxHeight < 50) ) // for 2-State switch height control is disabled in lower position
			)
			{   //hight control not active
				if(!delay--)
				{
					// measurement of air pressure close to upper limit
					if(ReadingAirPressure > 1000)
					{   // lower offset
						ExpandBaro -= 10;
						OCR0A = PressureSensorOffset - ExpandBaro;
						BeepTime = 300;
    	   				delay = 250;
					}
					// measurement of air pressure close to lower limit
					else if(ReadingAirPressure < 100)
					{   // increase offset
						ExpandBaro += 10;
						OCR0A = PressureSensorOffset - ExpandBaro;
						BeepTime = 300;
    	   				delay = 250;
					}
					else
					{
						SetPointHeight = ReadingHeight - 20;  // update SetPoint with current reading
						HeightControlActive = 0; // disable height control
						delay = 1;
					}
				}
			}
			else
			{	//hight control not active
				HeightControlActive = 1; // enable height control
				delay = 200;
			}
		}
		else // no switchable height control
		{
			SetPointHeight = ((int16_t) ExternHeightValue + (int16_t) FCParam.MaxHeight) * (int16_t)ParamSet.Height_Gain - 20;
			HeightControlActive = 1;
		}
		// get current height
		h = ReadingHeight;
		// if current height is above the setpoint reduce gas
		if((h > SetPointHeight) && HeightControlActive)
		{
			// GasMixFraction - HightDeviation * P  - HeightChange * D - ACCTop * DACC
			// height difference -> P control part
			h = ((h - SetPointHeight) * (int16_t) FCParam.Height_P) / (16 / STICK_GAIN);
			h = GasMixFraction - h; // reduce gas
			// height gradient --> D control part
			//h -= (HeightD * FCParam.Height_D) / (8 / STICK_GAIN);  // D control part
			h -= (HeightD) / (8 / STICK_GAIN);  // D control part
			// acceleration sensor effect
			tmp_int = ((Reading_Integral_Top / 128) * (int32_t) FCParam.Height_ACC_Effect) / (128 / STICK_GAIN);
			if(tmp_int > 70 * STICK_GAIN)        tmp_int =   70 * STICK_GAIN;
			else if(tmp_int < -(70 * STICK_GAIN)) tmp_int = -(70 * STICK_GAIN);
			h -= tmp_int;
			// update height control gas
			HeightControlGas = (HeightControlGas*15 + h) / 16;
			// limit gas reduction
			if(HeightControlGas < ParamSet.Height_MinGas * STICK_GAIN)
			{
				if(GasMixFraction >= ParamSet.Height_MinGas * STICK_GAIN) HeightControlGas = ParamSet.Height_MinGas * STICK_GAIN;
				// allows landing also if gas stick is reduced below min gas on height control
				if(GasMixFraction < ParamSet.Height_MinGas * STICK_GAIN) HeightControlGas = GasMixFraction;
			}
			// limit gas to stick setting
			if(HeightControlGas > GasMixFraction) HeightControlGas = GasMixFraction;
			GasMixFraction = HeightControlGas;
		}
	}
	// limit gas to parameter setting
	if(GasMixFraction > (ParamSet.Gas_Max - 20) * STICK_GAIN) GasMixFraction = (ParamSet.Gas_Max - 20) * STICK_GAIN;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Mixer and PI-Controller
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	DebugOut.Analog[7] = GasMixFraction;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Yaw-Fraction
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    YawMixFraction = Reading_GyroYaw - SetPointYaw * STICK_GAIN;     // yaw controller
	#define MIN_YAWGAS (40 * STICK_GAIN)  // yaw also below this gas value
	// limit YawMixFraction
	if(GasMixFraction > MIN_YAWGAS)
	{
		if(YawMixFraction >  (GasMixFraction / 2)) YawMixFraction = GasMixFraction / 2;
		if(YawMixFraction < -(GasMixFraction / 2)) YawMixFraction = -(GasMixFraction / 2);
	}
	else
	{
		if(YawMixFraction >  (MIN_YAWGAS / 2)) YawMixFraction = MIN_YAWGAS / 2;
		if(YawMixFraction < -(MIN_YAWGAS / 2)) YawMixFraction = -(MIN_YAWGAS / 2);
	}
	tmp_int = ParamSet.Gas_Max * STICK_GAIN;
    if(YawMixFraction >  ((tmp_int - GasMixFraction))) YawMixFraction =  ((tmp_int - GasMixFraction));
    if(YawMixFraction < -((tmp_int - GasMixFraction))) YawMixFraction = -((tmp_int - GasMixFraction));

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Nick-Axis
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    DiffNick = Reading_GyroNick - StickNick;	// get difference
    if(Gyro_I_Factor) SumNick += IntegralNick * Gyro_I_Factor - StickNick; // I-part for attitude control
    else SumNick += DiffNick; // I-part for head holding
    if(SumNick >  (STICK_GAIN * 16000L)) SumNick =  (STICK_GAIN * 16000L);
    if(SumNick < -(STICK_GAIN * 16000L)) SumNick = -(STICK_GAIN * 16000L);
    pd_result = DiffNick + Ki * SumNick; // PI-controller for nick

    tmp_int = (int32_t)((int32_t)FCParam.DynamicStability * (int32_t)(GasMixFraction + abs(YawMixFraction)/2)) / 64;
    if(pd_result >  tmp_int) pd_result =  tmp_int;
    if(pd_result < -tmp_int) pd_result = -tmp_int;

	// Motor Front
    MotorValue = GasMixFraction + pd_result + YawMixFraction;	  // Mixer
    MotorValue /= STICK_GAIN;
	if ((MotorValue < 0)) MotorValue = 0;
	else if(MotorValue > ParamSet.Gas_Max)  	  MotorValue = ParamSet.Gas_Max;
	if (MotorValue < ParamSet.Gas_Min)            MotorValue = ParamSet.Gas_Min;
	Motor_Front = MotorValue;

 // Motor Rear
	MotorValue = GasMixFraction - pd_result + YawMixFraction;     // Mixer
	MotorValue /= STICK_GAIN;
	if ((MotorValue < 0)) MotorValue = 0;
	else if(MotorValue > ParamSet.Gas_Max)	    MotorValue = ParamSet.Gas_Max;
	if (MotorValue < ParamSet.Gas_Min)          MotorValue = ParamSet.Gas_Min;
	Motor_Rear = MotorValue;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Roll-Axis
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	DiffRoll = Reading_GyroRoll - StickRoll;	// get difference
    if(Gyro_I_Factor) SumRoll += IntegralRoll * Gyro_I_Factor - StickRoll; // I-part for attitude control
    else SumRoll += DiffRoll;  // I-part for head holding
    if(SumRoll >  (STICK_GAIN * 16000L)) SumRoll =  (STICK_GAIN * 16000L);
    if(SumRoll < -(STICK_GAIN * 16000L)) SumRoll = -(STICK_GAIN * 16000L);
    pd_result = DiffRoll + Ki * SumRoll;	 // PI-controller for roll
    tmp_int = (int32_t)((int32_t)FCParam.DynamicStability * (int32_t)(GasMixFraction + abs(YawMixFraction)/2)) / 64;
    if(pd_result >  tmp_int) pd_result =  tmp_int;
    if(pd_result < -tmp_int) pd_result = -tmp_int;

    // Motor Left
    MotorValue = GasMixFraction + pd_result - YawMixFraction;  // Mixer
    MotorValue /= STICK_GAIN;
	if ((MotorValue < 0)) MotorValue = 0;
	else if(MotorValue > ParamSet.Gas_Max)		MotorValue = ParamSet.Gas_Max;
	if (MotorValue < ParamSet.Gas_Min)          MotorValue = ParamSet.Gas_Min;
    Motor_Left = MotorValue;

 // Motor Right
	MotorValue = GasMixFraction - pd_result - YawMixFraction;  // Mixer
	MotorValue /= STICK_GAIN;
	if ((MotorValue < 0)) MotorValue = 0;
	else if(MotorValue > ParamSet.Gas_Max)	   	MotorValue = ParamSet.Gas_Max;
	if (MotorValue < ParamSet.Gas_Min)          MotorValue = ParamSet.Gas_Min;
    Motor_Right = MotorValue;
}

