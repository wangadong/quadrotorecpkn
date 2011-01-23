// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 04.2007 Holger Buss
// + only for non-profit use
// + www.MikroKopter.com
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <stdlib.h>
#include <inttypes.h>
#include "main.h"
#include "eeprom.h"
#include "timer2.h"
#include "fc.h"
#include "rc.h"
#include "uart.h"
#include "printf_P.h"
#include "analog.h"

#ifdef USE_KILLAGREG
#include "mm3.h"
#endif

#if (defined (USE_KILLAGREG) || defined (USE_MK3MAG))
#include "ubx.h"
#endif

#include "_Settings.h"

#if (!defined (USE_KILLAGREG) && !defined (USE_MK3MAG))
uint8_t MaxMenuItem = 11;
#else
#ifdef USE_MK3MAG
uint8_t MaxMenuItem = 12;
#endif

#ifdef USE_KILLAGREG
uint8_t MaxMenuItem = 14;
#endif
#endif
uint8_t MenuItem = 0;
uint8_t RemoteKeys = 0;

#define KEY1    0x01
#define KEY2    0x02
#define KEY3    0x04
#define KEY4    0x08
#define KEY5    0x10



#define DISPLAYBUFFSIZE 80
int8_t DisplayBuff[DISPLAYBUFFSIZE] = "Hello World";
uint8_t DispPtr = 0;


/************************************/
/*        Clear LCD Buffer          */
/************************************/
void LCD_Clear(void)
{
 uint8_t i;
 for( i = 0; i < DISPLAYBUFFSIZE; i++) DisplayBuff[i] = ' ';
}


/************************************/
/*        Update Menu on LCD        */
/************************************/
// Display with 20 characters in 4 lines
void LCD_PrintMenu(void)
{
	if(RemoteKeys & KEY1)
	{
		if(MenuItem) MenuItem--;
		else MenuItem = MaxMenuItem;
	}
	if(RemoteKeys  & KEY2)
	{
		if(MenuItem == MaxMenuItem) MenuItem = 0;
		else MenuItem++;
	}
	if((RemoteKeys  & KEY1) && (RemoteKeys  & KEY2)) MenuItem = 0;

	LCD_Clear();

	if(MenuItem > MaxMenuItem) MenuItem = MaxMenuItem;
	// print menu item number in the upper right corner
	if(MenuItem < 10)
	{
	  LCD_printfxy(17,0,"[%i]",MenuItem);
	}
	else
	{
	  LCD_printfxy(16,0,"[%i]",MenuItem);
	}

	switch(MenuItem)
	{
    case 0:// Version Info Menu Item
           LCD_printfxy(0,0,"+ MikroKopter +");
           LCD_printfxy(0,1,"HW:V%d.%d SW:%d.%d%c",BoardRelease/10,BoardRelease%10,VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH+'a');
           LCD_printfxy(0,2,"Setting: %d ", GetActiveParamSet());
           LCD_printfxy(0,3,"(c) Holger Buss");
           break;
    case 1:// Height Control Menu Item
          if(ParamSet.GlobalConfig & CFG_HEIGHT_CONTROL)
           {
           LCD_printfxy(0,0,"Height:    %5i",ReadingHeight);
           LCD_printfxy(0,1,"Set Point: %5i",SetPointHeight);
           LCD_printfxy(0,2,"Air Press.:%5i",ReadingAirPressure);
           LCD_printfxy(0,3,"Offset    :%5i",PressureSensorOffset);
           }
           else
           {
           LCD_printfxy(0,1,"No ");
           LCD_printfxy(0,2,"Height Control");
           }

           break;
    case 2:// Attitude Menu Item
           LCD_printfxy(0,0,"Attitude");
           LCD_printfxy(0,1,"Nick:      %5i",IntegralNick/1024);
           LCD_printfxy(0,2,"Roll:      %5i",IntegralRoll/1024);
           LCD_printfxy(0,3,"Heading:   %5i",CompassHeading);
           break;
    case 3:// Remote Control Channel Menu Item
           LCD_printfxy(0,0,"C1:%4i  C2:%4i ",PPM_in[1],PPM_in[2]);
           LCD_printfxy(0,1,"C3:%4i  C4:%4i ",PPM_in[3],PPM_in[4]);
           LCD_printfxy(0,2,"C5:%4i  C6:%4i ",PPM_in[5],PPM_in[6]);
           LCD_printfxy(0,3,"C7:%4i  C8:%4i ",PPM_in[7],PPM_in[8]);
           break;
    case 4:// Remote Control Mapping Menu Item
           LCD_printfxy(0,0,"Ni:%4i  Ro:%4i ",PPM_in[ParamSet.ChannelAssignment[CH_NICK]],PPM_in[ParamSet.ChannelAssignment[CH_ROLL]]);
           LCD_printfxy(0,1,"Gs:%4i  Ya:%4i ",PPM_in[ParamSet.ChannelAssignment[CH_GAS]],PPM_in[ParamSet.ChannelAssignment[CH_YAW]]);
           LCD_printfxy(0,2,"P1:%4i  P2:%4i ",PPM_in[ParamSet.ChannelAssignment[CH_POTI1]],PPM_in[ParamSet.ChannelAssignment[CH_POTI2]]);
           LCD_printfxy(0,3,"P3:%4i  P4:%4i ",PPM_in[ParamSet.ChannelAssignment[CH_POTI3]],PPM_in[ParamSet.ChannelAssignment[CH_POTI4]]);
           break;
	case 5:// Gyro Sensor Menu Item
           LCD_printfxy(0,0,"Gyro - Sensor");
           switch(BoardRelease)
           {
			case 10:
			   LCD_printfxy(0,1,"Nick %4i (%3i)",AdValueGyrNick - AdNeutralNick, AdNeutralNick);
			   LCD_printfxy(0,2,"Roll %4i (%3i)",AdValueGyrRoll - AdNeutralRoll, AdNeutralRoll);
			   LCD_printfxy(0,3,"Yaw  %4i (%3i)",AdNeutralYaw  - AdValueGyrYaw , AdNeutralYaw);
			   break;

			case 11:
			case 12:
			case 20:
			   LCD_printfxy(0,1,"Nick %4i (%3i)",AdValueGyrNick - AdNeutralNick, AdNeutralNick/2);
			   LCD_printfxy(0,2,"Roll %4i (%3i)",AdValueGyrRoll - AdNeutralRoll, AdNeutralRoll/2);
			   LCD_printfxy(0,3,"Yaw  %4i (%3i)",AdNeutralYaw  - AdValueGyrYaw , AdNeutralYaw/2);
			   break;

			case 13:
			default:
			   LCD_printfxy(0,1,"Nick %4i (%3i)(%3i)",AdValueGyrNick - AdNeutralNick, AdNeutralNick/2, AnalogOffsetNick);
			   LCD_printfxy(0,2,"Roll %4i (%3i)(%3i)",AdValueGyrRoll - AdNeutralRoll, AdNeutralRoll/2, AnalogOffsetRoll);
			   LCD_printfxy(0,3,"Yaw  %4i (%3i)(%3i)",AdNeutralYaw  - AdValueGyrYaw , AdNeutralYaw/2 , AnalogOffsetYaw );
			   break;
          }
          break;
    case 6:// Acceleration Sensor Menu Item
           LCD_printfxy(0,0,"ACC - Sensor");
           LCD_printfxy(0,1,"Nick   %4i (%3i)",AdValueAccNick, NeutralAccX);
           LCD_printfxy(0,2,"Roll   %4i (%3i)",AdValueAccRoll, NeutralAccY);
           LCD_printfxy(0,3,"Height %4i (%3i)",Mean_AccTop, (int)NeutralAccZ);
           break;
    case 7:// Accumulator Voltage / Remote Control Level
           LCD_printfxy(0,1,"Voltage:  %5i",UBat);
           LCD_printfxy(0,2,"RC-Level: %5i",RC_Quality);
           break;
    case 8:// Compass Menu Item
           LCD_printfxy(0,0,"Compass       ");
           LCD_printfxy(0,1,"Course:    %5i",CompassCourse);
           LCD_printfxy(0,2,"Heading:   %5i",CompassHeading);
           LCD_printfxy(0,3,"OffCourse: %5i",CompassOffCourse);
           break;
    case 9:// Poti Menu Item
		   LCD_printfxy(0,0,"Po1: %3i Po5: %3i" ,Poti1,Poti5); //PPM24-Extesion
		   LCD_printfxy(0,1,"Po2: %3i Po6: %3i" ,Poti2,Poti6); //PPM24-Extesion
		   LCD_printfxy(0,2,"Po3: %3i Po7: %3i" ,Poti3,Poti7); //PPM24-Extesion
		   LCD_printfxy(0,3,"Po4: %3i Po8: %3i" ,Poti4,Poti8); //PPM24-Extesion
           break;
    case 10:// Servo Menu Item
           LCD_printfxy(0,0,"Servo  " );
           LCD_printfxy(0,1,"Setpoint  %3i",FCParam.ServoNickControl);
           LCD_printfxy(0,2,"Position: %3i",ServoValue);
           LCD_printfxy(0,3,"Range:%3i-%3i",ParamSet.ServoNickMin, ParamSet.ServoNickMax);
           break;
    case 11://Extern Control
           LCD_printfxy(0,0,"ExternControl  " );
           LCD_printfxy(0,1,"Ni:%4i  Ro:%4i ",ExternControl.Nick, ExternControl.Roll);
           LCD_printfxy(0,2,"Gs:%4i  Ya:%4i ",ExternControl.Gas, ExternControl.Yaw);
           LCD_printfxy(0,3,"Hi:%4i  Cf:%4i ",ExternControl.Height, ExternControl.Config);
           break;

    #if (defined (USE_KILLAGREG) || defined (USE_MK3MAG))
	case 12://GPS Lat/Lon coords
			if (GPSInfo.status == INVALID)
			{
				LCD_printfxy(0,0,"No GPS data!");
			}
			else
			{
				switch (GPSInfo.satfix)
				{
				case SATFIX_NONE:
					LCD_printfxy(0,0,"Sats: %d Fix: No", GPSInfo.satnum);
					break;
				case SATFIX_2D:
					LCD_printfxy(0,0,"Sats: %d Fix: 2D", GPSInfo.satnum);
					break;
				case SATFIX_3D:
					LCD_printfxy(0,0,"Sats: %d Fix: 3D", GPSInfo.satnum);
					break;
				default:
					LCD_printfxy(0,0,"Sats: %d Fix: ??", GPSInfo.satnum);
					break;
				}
				int16_t i1,i2,i3;
				i1 = (int16_t)(GPSInfo.longitude/10000000L);
				i2 = abs((int16_t)((GPSInfo.longitude%10000000L)/10000L));
				i3 = abs((int16_t)(((GPSInfo.longitude%10000000L)%10000L)/10L));
				LCD_printfxy(0,1,"Lon: %d.%.3d%.3d deg",i1, i2, i3);
				i1 = (int16_t)(GPSInfo.latitude/10000000L);
				i2 = abs((int16_t)((GPSInfo.latitude%10000000L)/10000L));
				i3 = abs((int16_t)(((GPSInfo.latitude%10000000L)%10000L)/10L));
				LCD_printfxy(0,2,"Lat: %d.%.3d%.3d deg",i1, i2, i3);
				i1 = (int16_t)(GPSInfo.altitude/1000L);
				i2 = abs((int16_t)(GPSInfo.altitude%1000L));
				LCD_printfxy(0,3,"Alt: %d.%.3d m",i1, i2);
			}
			break;
	#endif
	#ifdef USE_KILLAGREG
	case 13:// MM3 Kompass
			LCD_printfxy(0,0,"MM3 Offset");
			LCD_printfxy(0,1,"X_Offset:  %3i",MM3_calib.X_off);
			LCD_printfxy(0,2,"Y_Offset:  %3i",MM3_calib.Y_off);
			LCD_printfxy(0,3,"Z_Offset:  %3i",MM3_calib.Z_off);
			break;
	case 14://MM3 Range
			LCD_printfxy(0,0,"MM3 Range");
			LCD_printfxy(0,1,"X_Range:  %4i",MM3_calib.X_range);
			LCD_printfxy(0,2,"Y_Range:  %4i",MM3_calib.Y_range);
			LCD_printfxy(0,3,"Z_Range:  %4i",MM3_calib.Z_range);
			break;
	#endif

    default: MaxMenuItem = MenuItem - 1;
             MenuItem = 0;
           break;
    }
    RemoteKeys = 0;
}
