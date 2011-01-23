#ifndef _UART_H
#define _UART_H

#define RXD_BUFFER_LEN  150
// must be at least 4('#'+Addr+'CmdID'+'\r')+ (80 * 4)/3 = 111 bytes
#define TXD_BUFFER_LEN  150
#define RXD_BUFFER_LEN  150

#include <inttypes.h>

//Baud rate of the USART
#define USART0_BAUD 57600


extern void USART0_Init (void);
extern void USART0_TransmitTxData(void);
extern void USART0_ProcessRxData(void);
extern int16_t uart_putchar(int8_t c);

extern uint8_t PcAccess;
extern uint8_t RemotePollDisplayLine;
extern uint8_t MotorTest[4];

typedef struct
{
	uint8_t Digital[2];
	uint16_t Analog[32];    // Debugvalues
} __attribute__((packed)) DebugOut_t;

extern DebugOut_t DebugOut;

typedef struct
{
	uint8_t	Digital[2];
	uint8_t	RemoteButtons;
	int8_t		Nick;
	int8_t		Roll;
	int8_t		Yaw;
	uint8_t	Gas;
	int8_t		Height;
	uint8_t	free;
	uint8_t	Frame;
	uint8_t	Config;
} __attribute__((packed)) ExternControl_t;

extern ExternControl_t ExternControl;

typedef struct
{
	uint8_t SWMajor;
	uint8_t SWMinor;
	uint8_t ProtoMajor;
	uint8_t ProtoMinor;
	uint8_t SWPatch;
	uint8_t Reserved[5];
}  __attribute__((packed)) UART_VersionInfo_t;


#endif //_UART_H
