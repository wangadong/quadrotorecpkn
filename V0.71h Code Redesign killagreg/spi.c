// ######################## SPI - FlightCtrl ###################
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "_Settings.h"
#include "spi.h"
#include "fc.h"
#include "rc.h"
#include "eeprom.h"
#include "uart.h"
#include "timer0.h"
#include "analog.h"

#define SPI_TXSYNCBYTE1 0xAA
#define SPI_TXSYNCBYTE2 0x83
#define SPI_RXSYNCBYTE1 0x81
#define SPI_RXSYNCBYTE2 0x55

typedef enum
{
	SPI_SYNC1,
	SPI_SYNC2,
	SPI_DATA
} SPI_RXState_t;


// data exchange packets to and From NaviCtrl
ToNaviCtrl_t			ToNaviCtrl;
FromNaviCtrl_t			FromNaviCtrl;

SPI_VersionInfo_t SPI_VersionInfo;

// rx packet buffer
#define SPI_RXBUFFER_LEN sizeof(FromNaviCtrl)
uint8_t SPI_RxBuffer[SPI_RXBUFFER_LEN];
uint8_t	SPI_RxBufferIndex = 0;
uint8_t SPI_RxBuffer_Request = 0;

// tx packet buffer
#define SPI_TXBUFFER_LEN sizeof(ToNaviCtrl)
uint8_t *SPI_TxBuffer;
uint8_t	SPI_TxBufferIndex = 0;

uint8_t SPITransferCompleted, SPI_ChkSum;
uint8_t SPI_RxDataValid;

uint8_t SPI_CommandSequence[] = { SPI_CMD_USER, SPI_CMD_STICK, SPI_CMD_PARAMETER1, SPI_CMD_STICK, SPI_CMD_MISC, SPI_CMD_VERSION };
uint8_t SPI_CommandCounter = 0;

/*********************************************/
/*  Initialize SPI interface to NaviCtrl     */
/*********************************************/
void SPI_MasterInit(void)
{
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);    // Set MOSI and SCK output, all others input
	SLAVE_SELECT_DDR_PORT |= (1 << SPI_SLAVE_SELECT); // set Slave select port as output port

	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(0<<SPR0)|(0<<SPIE);   // Enable SPI, Master, set clock rate fck/64
	SPSR = 0;//(1<<SPI2X);

	SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT); // Deselect Slave

	SPI_TxBuffer = (uint8_t *) &ToNaviCtrl; // set pointer to tx-buffer
	SPITransferCompleted = 1;
	// initialize data packet to NaviControl
	ToNaviCtrl.Sync1 = SPI_TXSYNCBYTE1;
	ToNaviCtrl.Sync2 = SPI_TXSYNCBYTE2;

	ToNaviCtrl.Command = SPI_CMD_USER;
	ToNaviCtrl.IntegralNick = 0;
  	ToNaviCtrl.IntegralRoll = 0;

	SPI_RxDataValid = 0;

	SPI_VersionInfo.Major = VERSION_MAJOR;
	SPI_VersionInfo.Minor = VERSION_MINOR;
	SPI_VersionInfo.Patch = VERSION_PATCH;
	SPI_VersionInfo.Compatible = NC_SPI_COMPATIBLE;
}


/**********************************************************/
/*  Update Data transferd by the SPI from/to NaviCtrl     */
/**********************************************************/
void UpdateSPI_Buffer(void)
{
	int16_t tmp;
	cli(); // stop all interrupts to avoid writing of new data during update of that packet.

	// update content of packet to NaviCtrl
	ToNaviCtrl.IntegralNick = (int16_t) (IntegralNick / 130L);   // convert to multiple of 0.1°
	ToNaviCtrl.IntegralRoll  = (int16_t) (IntegralRoll  / 130L); // convert to multiple of 0.1°
	ToNaviCtrl.GyroHeading = YawGyroHeading / YAW_GYRO_DEG_FACTOR;
	ToNaviCtrl.GyroNick = Reading_GyroNick;
	ToNaviCtrl.GyroRoll = Reading_GyroRoll;
	ToNaviCtrl.GyroYaw = Reading_GyroYaw;
	ToNaviCtrl.AccNick = (int16_t) ACC_AMPLIFY * (NaviAccNick / NaviCntAcc);
	ToNaviCtrl.AccRoll = (int16_t) ACC_AMPLIFY * (NaviAccRoll / NaviCntAcc);
	NaviCntAcc = 0; NaviAccNick = 0; NaviAccRoll = 0;

	switch(ToNaviCtrl.Command)
	{
		case SPI_CMD_USER:
			ToNaviCtrl.Param.Byte[0] = FCParam.UserParam1;
			ToNaviCtrl.Param.Byte[1] = FCParam.UserParam2;
			ToNaviCtrl.Param.Byte[2] = FCParam.UserParam3;
			ToNaviCtrl.Param.Byte[3] = FCParam.UserParam4;
			ToNaviCtrl.Param.Byte[4] = FCParam.UserParam5;
			ToNaviCtrl.Param.Byte[5] = FCParam.UserParam6;
			ToNaviCtrl.Param.Byte[6] = FCParam.UserParam7;
			ToNaviCtrl.Param.Byte[7] = FCParam.UserParam8;
			ToNaviCtrl.Param.Byte[8] = MKFlags;
			MKFlags &= ~(MKFLAG_CALIBRATE | MKFLAG_START); // calibrate and start are temporal states that are cleared immediately after transmitting
			ToNaviCtrl.Param.Byte[9] = (uint8_t)UBat;
			ToNaviCtrl.Param.Byte[10] = ParamSet.LowVoltageWarning;
			ToNaviCtrl.Param.Byte[11] = GetActiveParamSet();
			break;

		case SPI_CMD_PARAMETER1:
			ToNaviCtrl.Param.Byte[0] = ParamSet.NaviGpsModeControl;     // Parameters for the Naviboard
			ToNaviCtrl.Param.Byte[1] = ParamSet.NaviGpsGain;
			ToNaviCtrl.Param.Byte[2] = ParamSet.NaviGpsP;
			ToNaviCtrl.Param.Byte[3] = ParamSet.NaviGpsI;
			ToNaviCtrl.Param.Byte[4] = ParamSet.NaviGpsD;
			ToNaviCtrl.Param.Byte[5] = ParamSet.NaviGpsACC;
			ToNaviCtrl.Param.Byte[6] = ParamSet.NaviGpsMinSat;
			ToNaviCtrl.Param.Byte[7] = ParamSet.NaviStickThreshold;
			ToNaviCtrl.Param.Byte[8] = ParamSet.NaviOperatingRadius;
			ToNaviCtrl.Param.Byte[9] = ParamSet.NaviWindCorrection;
            ToNaviCtrl.Param.Byte[10] = ParamSet.NaviSpeedCompensation;
            ToNaviCtrl.Param.Byte[11] = ParamSet.NaviAngleLimitation;
			break;


		case SPI_CMD_STICK:
			tmp = PPM_in[ParamSet.ChannelAssignment[CH_GAS]];  if(tmp > 127) tmp = 127; else if(tmp < -128) tmp = -128;
			ToNaviCtrl.Param.Byte[0] = (int8_t) tmp;
			tmp = PPM_in[ParamSet.ChannelAssignment[CH_YAW]]; if(tmp > 127) tmp = 127; else if(tmp < -128) tmp = -128;
			ToNaviCtrl.Param.Byte[1] = (int8_t) tmp;
			tmp = PPM_in[ParamSet.ChannelAssignment[CH_ROLL]]; if(tmp > 127) tmp = 127; else if(tmp < -128) tmp = -128;
			ToNaviCtrl.Param.Byte[2] = (int8_t) tmp;
			tmp = PPM_in[ParamSet.ChannelAssignment[CH_NICK]]; if(tmp > 127) tmp = 127; else if(tmp < -128) tmp = -128;
			ToNaviCtrl.Param.Byte[3] = (int8_t) tmp;
			ToNaviCtrl.Param.Byte[4] = (uint8_t) Poti1;
			ToNaviCtrl.Param.Byte[5] = (uint8_t) Poti2;
			ToNaviCtrl.Param.Byte[6] = (uint8_t) Poti3;
			ToNaviCtrl.Param.Byte[7] = (uint8_t) Poti4;
			ToNaviCtrl.Param.Byte[8] = (uint8_t) RC_Quality;
			break;

		case SPI_CMD_MISC:
			ToNaviCtrl.Param.Byte[0] = CompassCalState;
			if(CompassCalState > 4)
			{ // jump from 5 to 0
				CompassCalState  = 0;
			}
			ToNaviCtrl.Param.Int[1] = ReadingHeight;
			break;

		case SPI_CMD_VERSION:
			ToNaviCtrl.Param.Byte[0] = SPI_VersionInfo.Major;
			ToNaviCtrl.Param.Byte[1] = SPI_VersionInfo.Minor;
			ToNaviCtrl.Param.Byte[2] = SPI_VersionInfo.Patch;
			ToNaviCtrl.Param.Byte[3] = SPI_VersionInfo.Compatible;
			break;

		default:
			break;
	}


	sei(); // enable all interrupts

	// analyze content of packet from NaviCtrl if valid
	if (SPI_RxDataValid)
	{
		// update gps controls
		if(abs(FromNaviCtrl.GPS_Nick) < 512 && abs(FromNaviCtrl.GPS_Roll) < 512 && (ParamSet.GlobalConfig & CFG_GPS_ACTIVE))
		{
			GPS_Nick	= FromNaviCtrl.GPS_Nick;
			GPS_Roll	= FromNaviCtrl.GPS_Roll;
		}
		// update compass readings
		if(FromNaviCtrl.CompassHeading <= 360)
		{
			CompassHeading = FromNaviCtrl.CompassHeading;
		}
		if(CompassHeading < 0) CompassOffCourse = 0;
		else CompassOffCourse = ((540 + CompassHeading - CompassCourse) % 360) - 180;
        // NaviCtrl wants to beep?
		if (FromNaviCtrl.BeepTime > BeepTime && !CompassCalState) BeepTime = FromNaviCtrl.BeepTime;

		switch (FromNaviCtrl.Command)
		{
			case SPI_KALMAN:
				FCParam.Kalman_K = FromNaviCtrl.Param.Byte[0];
				FCParam.Kalman_MaxFusion = FromNaviCtrl.Param.Byte[1];
				FCParam.Kalman_MaxDrift = FromNaviCtrl.Param.Byte[2];
				break;

			default:
				break;
		}
	}
	else // no valid data from NaviCtrl
	{
		// disable GPS control
		GPS_Nick = 0;
		GPS_Roll = 0;
	}
}



/*********************************************/
/*  Start Transmission of packet to NaviCtrl */
/*********************************************/
void SPI_StartTransmitPacket(void)
{

	if (!SPITransferCompleted) return; // return immediately if transfer is in progress
	else // transmission was completed
	{
		SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // Select slave

		// cyclic commands
		ToNaviCtrl.Command = SPI_CommandSequence[SPI_CommandCounter++];
		if (SPI_CommandCounter >= sizeof(SPI_CommandSequence)) SPI_CommandCounter = 0;

		SPITransferCompleted = 0; // transfer is in progress
		UpdateSPI_Buffer();    // update data in ToNaviCtrl

		SPI_TxBufferIndex = 1; //proceed with 2nd byte

		// -- Debug-Output ---
		//----
		asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
		asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
		asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
		ToNaviCtrl.Chksum = ToNaviCtrl.Sync1; // init checksum
		SPDR = ToNaviCtrl.Sync1; // send first byte
	}
}

//------------------------------------------------------
// This is the spi data transfer between FlightCtrl and NaviCtrl
// Every time this routine is called within the mainloop one byte of the packet to
// the NaviCtrl and one byte of the packet from the NaviCtrl is possible transfered

void SPI_TransmitByte(void)
{
	static SPI_RXState_t SPI_RXState = SPI_SYNC1;
	uint8_t rxdata;
	static uint8_t rxchksum;

	if (SPITransferCompleted) return;  // return immediatly if transfer was completed
	if (!(SPSR & (1 << SPIF))) return; // return if no SPI-IRQ pending
	SendSPI = 4; // mait 4 * 0.102 ms for the next call of SPI_TransmitByte() in the main loop

	SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);   // DeselectSlave

	rxdata = SPDR; // save spi data register

	switch (SPI_RXState)
	{
  		case SPI_SYNC1: // first sync byte
			SPI_RxBufferIndex = 0; // set pointer to start of rx buffer
			rxchksum = rxdata; // initialize checksum
			if (rxdata == SPI_RXSYNCBYTE1 )
			{	// 1st Syncbyte found
				SPI_RXState  = SPI_SYNC2; // trigger to state for second sync byte
			}
			break;

		case SPI_SYNC2: // second sync byte
			if (rxdata == SPI_RXSYNCBYTE2)
			{	// 2nd Syncbyte found
				rxchksum += rxdata; // update checksum
				SPI_RXState  = SPI_DATA;   // trigger to state for second sync byte
			}
			else // 2nd Syncbyte not found
			{
				SPI_RXState  = SPI_SYNC1; // jump back to 1st sync byte
			}
			break;

		case SPI_DATA: // data bytes
			SPI_RxBuffer[SPI_RxBufferIndex++] = rxdata;  // copy data byte to spi buffer
			// if all bytes are received of a packet from the NaviCtrl
			if (SPI_RxBufferIndex >= SPI_RXBUFFER_LEN)
			{   // last byte transfered is the checksum of the packet
				if (rxdata == rxchksum) // checksum matching?
				{
				  	// copy SPI_RxBuffer -> FromFlightCtrl
				  	uint8_t *ptr = (uint8_t *)&FromNaviCtrl;
				  	cli();
				  	memcpy(ptr, (uint8_t *) SPI_RxBuffer, sizeof(FromNaviCtrl));
				  	sei();
				  	SPI_RxDataValid = 1;
				  	//DebugOut.Analog[18]++;
				}
				else
				{   // checksum does not match
					//DebugOut.Analog[17]++;
					SPI_RxDataValid = 0; // reset valid flag
				}
				SPI_RXState  = SPI_SYNC1; // reset state sync
			}
			else // not all bytes transfered
			{
				rxchksum += rxdata; // update checksum
			}
			break;
	}// eof switch(SPI_RXState)

    // if still some bytes left for transmission to NaviCtrl
	if (SPI_TxBufferIndex < SPI_TXBUFFER_LEN)
	{
		SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
		asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
		asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
		asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");	asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");

		SPDR = SPI_TxBuffer[SPI_TxBufferIndex]; // transmit byte
		ToNaviCtrl.Chksum += SPI_TxBuffer[SPI_TxBufferIndex]; // update checksum for everey byte that was sent
		SPI_TxBufferIndex++;
	}
	else
	{
		//Transfer of all bytes of the packet to NaviCtrl completed
		SPITransferCompleted = 1;
	}
}



