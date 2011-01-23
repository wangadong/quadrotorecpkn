// ######################## SPI - FlightCtrl ###################
#ifndef _SPI_H
#define _SPI_H

//#include <util/delay.h>
#include <inttypes.h>


#define SPI_PROTOCOL_COMP   1

//-----------------------------------------
#define DDR_SPI DDRB
#define DD_SS   PB4
#define DD_SCK  PB7
#define DD_MOSI PB5
#define DD_MISO PB6

// for compatibility reasons gcc3.x <-> gcc4.x
#ifndef SPCR
#define SPCR   SPCR0
#endif
#ifndef SPE
#define SPE   SPE0
#endif
#ifndef MSTR
#define MSTR   MSTR0
#endif
#ifndef SPR1
#define SPR1   SPR01
#endif
#ifndef SPR0
#define SPR0   SPR00
#endif
#ifndef SPIE
#define SPIE   SPIE0
#endif
#ifndef SPDR
#define SPDR   SPDR0
#endif
#ifndef SPIF
#define SPIF   SPIF0
#endif
#ifndef SPSR
#define SPSR   SPSR0
#endif
// -------------------------

#define SLAVE_SELECT_DDR_PORT   DDRC
#define SLAVE_SELECT_PORT       PORTC
#define SPI_SLAVE_SELECT        PC5


#define SPI_CMD_USER        10
#define SPI_CMD_STICK       11
#define SPI_CMD_MISC		12
#define SPI_CMD_PARAMETER1	13
#define SPI_CMD_VERSION		14

typedef struct
{
	uint8_t Sync1;
	uint8_t Sync2;
	uint8_t Command;
	int16_t IntegralNick;
	int16_t IntegralRoll;
	int16_t AccNick;
	int16_t AccRoll;
	int16_t GyroHeading;
	int16_t GyroNick;
	int16_t GyroRoll;
	int16_t GyroYaw;
	union
	{
		int8_t  sByte[12];
		uint8_t Byte[12];
		int16_t Int[6];
		int32_t Long[3];
		float   Float[3];
	} Param;
	uint8_t Chksum;
} __attribute__((packed)) ToNaviCtrl_t;



#define SPI_CMD_OSD_DATA	100
#define SPI_CMD_GPS_POS		101
#define SPI_CMD_GPS_TARGET	102
#define SPI_KALMAN			103

typedef struct
{
	uint8_t Command;
	int16_t GPS_Nick;
	int16_t GPS_Roll;
	int16_t GPS_Yaw;
	int16_t CompassHeading;
	int16_t Status;
	uint16_t BeepTime;
	union
	{
		int8_t  Byte[12];
		int16_t Int[6];
		int32_t Long[3];
		float   Float[3];
	} Param;
	uint8_t Chksum;
} __attribute__((packed)) FromNaviCtrl_t;


typedef struct
{
	uint8_t Major;
	uint8_t Minor;
	uint8_t Patch;
	uint8_t Compatible;
} __attribute__((packed)) SPI_VersionInfo_t;


extern ToNaviCtrl_t   			ToNaviCtrl;
extern FromNaviCtrl_t 			FromNaviCtrl;

void SPI_MasterInit(void);
void SPI_StartTransmitPacket(void);
void SPI_TransmitByte(void);



#endif //_SPI_H
