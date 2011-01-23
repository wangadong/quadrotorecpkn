/*

Copyright 2008, by Killagreg

This program (files mm3.c and mm3.h) is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation;
either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details. You should have received a copy of the GNU Lesser General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

Please note: The original implementation was done by Niklas Nold.
All the other files for the project "Mikrokopter" by H. Buss are under the license (license_buss.txt) published by www.mikrokopter.de
*/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include "mm3.h"
#include "main.h"
#include "mymath.h"
#include "fc.h"
#include "timer0.h"
#include "rc.h"
#include "eeprom.h"
#include "printf_P.h"

#define MAX_AXIS_VALUE		500


typedef struct
{
	uint8_t STATE;
	uint16_t DRDY;
	uint8_t AXIS;
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
} MM3_working_t;


// MM3 State Machine
#define MM3_STATE_RESET				0
#define MM3_STATE_START_TRANSFER	1
#define MM3_STATE_WAIT_DRDY			2
#define MM3_STATE_DRDY				3
#define MM3_STATE_BYTE2				4

#define MM3_X_AXIS		0x01
#define MM3_Y_AXIS		0x02
#define MM3_Z_AXIS		0x03


#define MM3_PERIOD_32	0x00
#define MM3_PERIOD_64	0x10
#define MM3_PERIOD_128	0x20
#define MM3_PERIOD_256	0x30
#define MM3_PERIOD_512	0x40
#define MM3_PERIOD_1024 0x50
#define MM3_PERIOD_2048 0x60
#define MM3_PERIOD_4096 0x70

#if defined(USE_WALTER_EXT) // walthers board
	// Output Pins (J9)PC6->MM3_SS ,(J8)PB2->MM3_RESET
	#define MM3_SS_PORT    PORTC //J9->MM3_SS
	#define MM3_SS_DDR     DDRC
	#define MM3_SS_PIN     PC6
	#define MM3_RESET_PORT PORTB //J8->MM3_RESET
	#define MM3_RESET_DDR  DDRB
	#define MM3_RESET_PIN  PB2
#elif defined(USE_NICK666) // nick666 version 0.67g
	#define MM3_SS_PORT    PORTD //J5->MM3_SS
	#define MM3_SS_DDR     DDRD
	#define MM3_SS_PIN     PD3
	#define MM3_RESET_PORT PORTB //J8->MM3_RESET
	#define MM3_RESET_DDR  DDRB
	#define MM3_RESET_PIN  PB2
#else // killagregs board
	// Output Pins PC4->MM3_SS ,PC5->MM3_RESET
	#define MM3_SS_PORT    PORTC
	#define MM3_SS_DDR     DDRC
	#define MM3_SS_PIN     PC4
	#define MM3_RESET_PORT PORTC
	#define MM3_RESET_DDR  DDRC
	#define MM3_RESET_PIN  PC5
#endif
	
#define MM3_SS_ON      MM3_SS_PORT    &= ~(1<<MM3_SS_PIN);
#define MM3_SS_OFF     MM3_SS_PORT    |=  (1<<MM3_SS_PIN);
#define MM3_RESET_ON   MM3_RESET_PORT |=  (1<<MM3_RESET_PIN);
#define MM3_RESET_OFF  MM3_RESET_PORT  &= ~(1<<MM3_RESET_PIN);



MM3_calib_t MM3_calib;
volatile MM3_working_t MM3;
volatile uint8_t MM3_Timeout = 0;



/*********************************************/
/*  Initialize Interface to MM3 Compass      */
/*********************************************/
void MM3_Init(void)
{
	uint8_t sreg = SREG;

	cli();

	// Configure Pins for SPI
	// set SCK (PB7), MOSI (PB5) as output
	DDRB |= (1<<DDB7)|(1<<DDB5);
	// set MISO (PB6) as input
	DDRB &= ~(1<<DDB6);


	// Output Pins MM3_SS ,MM3_RESET
	MM3_SS_DDR    |= (1<<MM3_SS_PIN);
	MM3_RESET_DDR |= (1<<MM3_RESET_PIN);
	// set pins permanent to low
	MM3_SS_PORT    &= ~((1<<MM3_SS_PIN));
	MM3_RESET_PORT &= ~((1<<MM3_RESET_PIN));

	// Initialize SPI-Interface
	// Enable interrupt (SPIE=1)
	// Enable SPI bus (SPE=1)
	// MSB transmitted first (DORD = 0)
	// Master SPI Mode (MSTR=1)
	// Clock polarity low when idle (CPOL=0)
	// Clock phase sample at leading edge (CPHA=0)
	// Clock rate = SYSCLK/128 (SPI2X=0, SPR1=1, SPR0=1) 20MHz/128 = 156.25kHz
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(1<<SPR0);
	SPSR &= ~(1<<SPI2X);

    // Init Statemachine
	MM3.AXIS = MM3_X_AXIS;
	MM3.STATE = MM3_STATE_RESET;

	// Read calibration from EEprom
	MM3_calib.X_off = (int8_t)GetParamByte(PID_MM3_X_OFF);
	MM3_calib.Y_off = (int8_t)GetParamByte(PID_MM3_Y_OFF);
	MM3_calib.Z_off = (int8_t)GetParamByte(PID_MM3_Z_OFF);
	MM3_calib.X_range = (int16_t)GetParamWord(PID_MM3_X_RANGE);
	MM3_calib.Y_range = (int16_t)GetParamWord(PID_MM3_Y_RANGE);
	MM3_calib.Z_range = (int16_t)GetParamWord(PID_MM3_Z_RANGE);

	MM3_Timeout = 0;

	SREG = sreg;
}


/*********************************************/
/*  Get Data from MM3                        */
/*********************************************/
void MM3_Update(void) // called every 102.4 탎 by timer 0 ISR
{
	switch (MM3.STATE)
	{
	case MM3_STATE_RESET:
		MM3_SS_ON  // select slave
		MM3_RESET_ON	// RESET to High, MM3 Reset
		MM3.STATE = MM3_STATE_START_TRANSFER;
		return;

	case MM3_STATE_START_TRANSFER:
		MM3_RESET_OFF	// RESET auf Low (was 102.4 탎 at high level)
		// write to SPDR triggers automatically the transfer MOSI MISO
		// MM3 Period, + AXIS code
		switch(MM3.AXIS)
		{
		case MM3_X_AXIS:
			SPDR = MM3_PERIOD_256 + MM3_X_AXIS;
			break;
		case MM3_Y_AXIS:
			SPDR = MM3_PERIOD_256 + MM3_Y_AXIS;
			break;
		case MM3_Z_AXIS:
			SPDR = MM3_PERIOD_256 + MM3_Z_AXIS;
			break;
		default:
			MM3.AXIS = MM3_X_AXIS;
			MM3.STATE = MM3_STATE_RESET;
			return;
		}

		// DRDY line is not connected, therefore
		// wait before reading data back
		MM3.DRDY = SetDelay(8); // wait 8ms for data ready
		MM3.STATE = MM3_STATE_WAIT_DRDY;
		return;

	case MM3_STATE_WAIT_DRDY:
		if (CheckDelay(MM3.DRDY))
		{
			// write something into SPDR to trigger data reading
			SPDR = 0x00;
			MM3.STATE = MM3_STATE_DRDY;
		}
		return;
	}
}


/*********************************************/
/*  Interrupt SPI transfer complete          */
/*********************************************/
ISR(SPI_STC_vect)
{
	static int8_t tmp;
	int16_t value;

	switch (MM3.STATE)
	{
	// 1st byte received
	case MM3_STATE_DRDY:
		tmp = SPDR;     // store 1st byte
		SPDR = 0x00;	// trigger transfer of 2nd byte
		MM3.STATE = MM3_STATE_BYTE2;
		return;

	case MM3_STATE_BYTE2:		// 2nd byte received
		value = (int16_t)tmp;	// combine the 1st and 2nd byte to a word
		value <<= 8;	        // shift 1st byte to MSB-Position
		value |= (int16_t)SPDR;	// add 2nd byte

		if(abs(value) < MAX_AXIS_VALUE)		// ignore spikes
		{
			switch (MM3.AXIS)
			{
			case MM3_X_AXIS:
				MM3.x_axis = value;
				MM3.AXIS = MM3_Y_AXIS;
				break;
			case MM3_Y_AXIS:
				MM3.y_axis = value;
				MM3.AXIS = MM3_Z_AXIS;
				break;
			case MM3_Z_AXIS:
				MM3.z_axis = value;
				MM3.AXIS = MM3_X_AXIS;
				break;
			default:
				MM3.AXIS = MM3_X_AXIS;
				break;
			}
		}
		MM3_SS_OFF // deselect slave
		MM3.STATE = MM3_STATE_RESET;
		// Update timeout is called every 102.4 탎.
		// It takes 2 cycles to write a measurement data request for one axis and
		// at at least 8 ms / 102.4 탎 = 79 cycles to read the requested data back.
		// I.e. 81 cycles * 102.4 탎 = 8.3ms per axis.
		// The two function accessing the MM3 Data - MM3_Calibrate() and MM3_Heading() -
		// decremtent the MM3_Timeout every 100 ms.
		// incrementing the counter by 1 every 8.3 ms is sufficient to avoid a timeout.
		if ((MM3.x_axis != MM3.y_axis) || (MM3.x_axis != MM3.z_axis) || (MM3.y_axis != MM3.z_axis))
		{	// if all axis measurements give diffrent readings the data should be valid
			if(MM3_Timeout < 20) MM3_Timeout++;
		}
		else // something is very strange here
		{
			if(MM3_Timeout ) MM3_Timeout--;
		}
		return;

	default:
		return;
	}
}


/*********************************************/
/*  Calibrate Compass                        */
/*********************************************/
void MM3_Calibrate(void)
{
	static int16_t x_min, x_max, y_min, y_max, z_min, z_max;

	switch(CompassCalState)
	{
		case 1: // change to x-y axis
			x_min =  10000;
			x_max = -10000;
			y_min =  10000;
			y_max = -10000;
			z_min =  10000;
			z_max = -10000;
			break;
		case 2:
			// find Min and Max of the X- and Y-Axis
			if(MM3.x_axis < x_min) x_min = MM3.x_axis;
			if(MM3.x_axis > x_max) x_max = MM3.x_axis;
			if(MM3.y_axis < y_min) y_min = MM3.y_axis;
			if(MM3.y_axis > y_max) y_max = MM3.y_axis;
			break;
		case 3:
			// change to z-Axis
		break;
		case 4:
			RED_ON;  // find Min and Max of the Z-axis
			if(MM3.z_axis < z_min) z_min = MM3.z_axis;
			if(MM3.z_axis > z_max) z_max = MM3.z_axis;
		break;
		case 5:
			// calc range of all axis
			MM3_calib.X_range = (x_max - x_min);
			MM3_calib.Y_range = (y_max - y_min);
			MM3_calib.Z_range = (z_max - z_min);

			// calc offset of all axis
			MM3_calib.X_off = (x_max + x_min) / 2;
			MM3_calib.Y_off = (y_max + y_min) / 2;
			MM3_calib.Z_off = (z_max + z_min) / 2;

			// save to EEProm
			SetParamByte(PID_MM3_X_OFF,   (uint8_t)MM3_calib.X_off);
			SetParamByte(PID_MM3_Y_OFF,   (uint8_t)MM3_calib.Y_off);
			SetParamByte(PID_MM3_Z_OFF,   (uint8_t)MM3_calib.Z_off);
			SetParamWord(PID_MM3_X_RANGE, (uint16_t)MM3_calib.X_range);
			SetParamWord(PID_MM3_Y_RANGE, (uint16_t)MM3_calib.Y_range);
			SetParamWord(PID_MM3_Z_RANGE, (uint16_t)MM3_calib.Z_range);

			CompassCalState = 0;
			break;
		default:
			CompassCalState = 0;
			break;
	}
}


/*
void MM3_Calibrate(void)
{
	static uint8_t debugcounter = 0;
	int16_t x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;
	uint8_t measurement = 50, beeper = 0;
	uint16_t timer;

	GRN_ON;
	RED_OFF;

	// get maximum and minimum reading of all axis
	while (measurement)
	{
		// reset range markers if yawstick ist leftmost
		if(PPM_in[ParamSet.ChannelAssignment[CH_YAW]] > 100)
		{
			x_min = 0;
			x_max = 0;
			y_min = 0;
			y_max = 0;
			z_min = 0;
			z_max = 0;
		}

		if (MM3.x_axis > x_max) x_max = MM3.x_axis;
		else if (MM3.x_axis < x_min) x_min = MM3.x_axis;

		if (MM3.y_axis > y_max) y_max = MM3.y_axis;
		else if (MM3.y_axis < y_min) y_min = MM3.y_axis;

		if (MM3.z_axis > z_max) z_max = MM3.z_axis;
		else if (MM3.z_axis < z_min) z_min = MM3.z_axis;

		if (!beeper)
		{
			RED_FLASH;
			GRN_FLASH;
			BeepTime = 50;
			beeper = 50;
		}
		beeper--;
		// loop with period of 10 ms / 100 Hz
		timer = SetDelay(10);
		while(!CheckDelay(timer));

		if(debugcounter++ > 30)
		{
			printf("\n\rXMin:%4d, XMax:%4d, YMin:%4d, YMax:%4d, ZMin:%4d, ZMax:%4d",x_min,x_max,y_min,y_max,z_min,z_max);
			debugcounter = 0;
		}

		// If gas is less than 100, stop calibration with a delay of 0.5 seconds
		if (PPM_in[ParamSet.ChannelAssignment[CH_GAS]] < 100) measurement--;
	}
	// Rage of all axis
	MM3_calib.X_range = (x_max - x_min);
	MM3_calib.Y_range = (y_max - y_min);
	MM3_calib.Z_range = (z_max - z_min);

	// Offset of all axis
	MM3_calib.X_off = (x_max + x_min) / 2;
	MM3_calib.Y_off = (y_max + y_min) / 2;
	MM3_calib.Z_off = (z_max + z_min) / 2;

	// save to EEProm
	SetParamByte(PID_MM3_X_OFF,   (uint8_t)MM3_calib.X_off);
	SetParamByte(PID_MM3_Y_OFF,   (uint8_t)MM3_calib.Y_off);
	SetParamByte(PID_MM3_Z_OFF,   (uint8_t)MM3_calib.Z_off);
	SetParamWord(PID_MM3_X_RANGE, (uint16_t)MM3_calib.X_range);
	SetParamWord(PID_MM3_Y_RANGE, (uint16_t)MM3_calib.Y_range);
	SetParamWord(PID_MM3_Z_RANGE, (uint16_t)MM3_calib.Z_range);

}
*/

/*********************************************/
/*  Calculate north direction (heading)      */
/*********************************************/
void MM3_Heading(void)
{
	int32_t sin_nick, cos_nick, sin_roll, cos_roll, sin_yaw, cos_yaw;
	int32_t  Hx, Hy, Hz, Hx_corr, Hy_corr;
	int16_t angle;
	uint16_t div_factor;
	int16_t heading;

	if (MM3_Timeout)
	{
		// Offset correction and normalization (values of H are +/- 512)
		Hx = (((int32_t)(MM3.x_axis - MM3_calib.X_off)) * 1024) / (int32_t)MM3_calib.X_range;
		Hy = (((int32_t)(MM3.y_axis - MM3_calib.Y_off)) * 1024) / (int32_t)MM3_calib.Y_range;
		Hz = (((int32_t)(MM3.z_axis - MM3_calib.Z_off)) * 1024) / (int32_t)MM3_calib.Z_range;

		// Compensate the angle of the MM3-arrow to the head of the MK by a yaw rotation transformation
		// assuming the MM3 board is mounted parallel to the frame.
		// User Param 4 is used to define the positive angle from the MM3-arrow to the MK heading
		// in a top view counter clockwise direction.
		// North is in opposite direction of the small arrow on the MM3 board.
		// Therefore 180 deg must be added to that angle.
		angle = ((int16_t)ParamSet.UserParam4 + 180);
		// wrap angle to interval of 0�- 359�
		angle += 360;
		angle %= 360;
		sin_yaw = (int32_t)(c_sin_8192(angle));
		cos_yaw = (int32_t)(c_cos_8192(angle));

		Hx_corr = Hx;
		Hy_corr = Hy;

		// rotate
		Hx = (Hx_corr * cos_yaw - Hy_corr  * sin_yaw) / 8192;
		Hy = (Hx_corr * sin_yaw + Hy_corr  * cos_yaw) / 8192;


		// tilt compensation

		// calibration factor for transforming Gyro Integrals to angular degrees
		div_factor = (uint16_t)ParamSet.UserParam3 * 8;

		// calculate sinus cosinus of nick and tilt angle
		angle = (IntegralNick/div_factor);
		sin_nick = (int32_t)(c_sin_8192(angle));
		cos_nick = (int32_t)(c_cos_8192(angle));

		angle = (IntegralRoll/div_factor);
		sin_roll = (int32_t)(c_sin_8192(angle));
		cos_roll = (int32_t)(c_cos_8192(angle));

		Hx_corr = Hx * cos_nick;
		Hx_corr -= Hz * sin_nick;
		Hx_corr /= 8192;

		Hy_corr = Hy * cos_roll;
		Hy_corr += Hz * sin_roll;
		Hy_corr /= 8192;

		// calculate Heading
		heading = c_atan2(Hy_corr, Hx_corr);

		// atan returns angular range from -180 deg to 180 deg in counter clockwise notation
		// but the compass course is defined in a range from 0 deg to 360 deg clockwise notation.
		if (heading < 0) heading = -heading;
		else heading = 360 - heading;
	}
	else // MM3_Timeout = 0 i.e now new data from external board
	{
		if(!BeepTime) BeepTime = 100; // make noise to signal the compass problem
		heading = -1;
	}
	// update compass values in fc variables
	CompassHeading = heading;
	if (CompassHeading < 0) CompassOffCourse = 0;
	else CompassOffCourse = ((540 + CompassHeading - CompassCourse) % 360) - 180;
}
