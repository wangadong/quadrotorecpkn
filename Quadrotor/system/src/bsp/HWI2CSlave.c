/*
 * hardware i2c slave driver.
 * ONLY FOR MSP430F2012
 *
 * current: add code of using MSP4302012 As TMP.
 * @author Boreal Mao, Xu Tao
 *
 */
#define MSP430F2012
#include "HWI2CSlave.h"

static unsigned char * rxBuffer = 0;
static unsigned char rxBufferLength = 0x00;
static unsigned char rxCounter = 0x00;
static unsigned char * txBuffer = 0;
static unsigned char txBufferLength = 0x00;
static unsigned char txCounter = 0x00;
static unsigned char slaveAddress = 0x00;
static unsigned char RW = 0; // Address with R/W bit; 1 for read, 0 for write
static unsigned char I2C_State = 0;
static void reset_WaitStart(void);

//ToCheck  __monitor to critical
critical extern void executeCommand(void);

//ToCheck  __monitor to critical
critical unsigned char is_I2CFree(void) {
	if (I2C_State == 0)
		return TRUE;
	return FALSE;
	//FIXME TN should use TRUE/FALSE
}

void initialize_HWI2CSlave(unsigned char* rb, unsigned char rbLength,
		unsigned char* tb, unsigned char tbLength, unsigned char address) {
	rxBuffer = rb;
	rxBufferLength = rbLength; // maximum receive data length
	txBuffer = tb;
	txBufferLength = tbLength; // transmit data length
	slaveAddress = address;
	USICTL0 = USIPE6 + USIPE7 + USISWRST; // Port & USI mode setup
	USICTL1 = USII2C + USIIE + USISTTIE; // Enable I2C mode & USI interrupts
	USICKCTL = USICKPL; // Setup clock polarity
	USICNT |= USIIFGCC; // Disable automatic clear control
	USICTL0 &= ~USISWRST; // Enable USI
	USICTL1 &= ~USIIFG; // Clear pending flag

	/*
	 * this is a trick for the fisrt I2C communication
	 * modified by xt
	 */
	I2C_State = 2; // Reset state machine

	txCounter = 0; // clear bufferCounter
	rxCounter = 0; // clear bufferCounter
#ifdef WITH_STOP
	USICTL1 &= ~USISTP; // clear STOP flag
#endif
}

static void reset_WaitStart(void) {
	USICTL0 &= ~USIOE; // SDA = input
#ifdef WITH_STOP
	USICTL1 &= ~USISTP; // clear STOP flag
#endif
	I2C_State = 0; // Reset state machine
	txCounter = 0; // clear bufferCounter
	rxCounter = 0; // clear bufferCounter
}

BSP_ISR_FUNCTION(USI_TXRX,USI_VECTOR) {
	if (rxBuffer[rxBufferLength]) {
		LPM0_EXIT;
		return;
	}
	if (USICTL1 & USISTTIFG) { // Start entry?
		I2C_State = 2; // Enter 1st state on start
	}

	switch (I2C_State)//__even_in_range(I2C_State,18))
	{
	case 0: //Idle, should not get here
		break;

	case 2: //RX Address

		USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX Address
		USICTL1 &= ~USISTTIFG; // Clear start flag
		I2C_State = 4; // Go to next state: check address
		break;

	case 4: // Process Address and send (N)Ack
		RW = 0; // assume receive
		if (USISRL & 0x01) {
			RW = 1; // if transmit
		}
		if ((USISRL & 0xFE) == slaveAddress) // Address matched
		{
			P1OUT |= 0x01; //LED2 on
			USICTL0 |= USIOE; // SDA = output
			USISRL = 0x00; // Send Ack
			if (RW == 0) {
				I2C_State = 6; // Go to next state: RX data
			} else {
				I2C_State = 10; // Go to next state: TX data
			}
			USICNT |= 0x01; // Bit counter = 1, send (N)Ack bit
		} else {
			I2C_State = 18; // Go directly to next state: prepare for next Start
		}
		break;

	case 6: // Receive data
		USICTL0 &= ~USIOE; // SDA = input
		USICNT |= 0x08; // Bit counter = 8, receive 8 bit data
		I2C_State = 8; // Go to next state: save data and send (N)Ack
		break;
	case 8:
		USICTL0 |= USIOE; // SDA = output
		if (rxCounter < rxBufferLength - 1) {
			rxBuffer[rxCounter++] = USISRL;
			USISRL = 0x00; // Send Ack
			I2C_State = 6; // Continue receiving
		} else {
			rxBuffer[rxCounter++] = USISRL;// rxBuffer full
			USISRL = 0xFF; // Send NAck
			I2C_State = 16; // Terminate transaction, prepare for next
		}
		USICNT |= 0x01; // Bit counter = 1, send (N)Ack bit
		break;
	case 10: // Transmit Data byte
		/* Attention: there must exist at least one byte to send*/
		USICTL0 |= USIOE; // SDA = output
		USISRL = txBuffer[txCounter++]; // Send data byte
		USICNT |= 0x08; // Bit counter = 8, TX data
		I2C_State = 12; // Go to next state: receive (N)Ack
		break;

	case 12: // Receive Data (N)Ack
		USICTL0 &= ~USIOE; // SDA = input
		USICNT |= 0x01; // Bit counter = 1, receive (N)Ack
		I2C_State = 14; // Go to next state: check (N)Ack
		break;

	case 14:// Process Data Ack/NAck
		if (USISRL & 0x01) {// NAck received
			I2C_State = 18; // Terminate transaction, prepare for next
		} else if (txCounter < txBufferLength) { // Ack received && data transmit not finished
			USICTL0 |= USIOE; // SDA = output
			USISRL = txBuffer[txCounter++]; // Send data byte
			USICNT |= 0x08; // Bit counter = 8, TX data
			I2C_State = 12; // Go to next state: receive (N)Ack
		} else {
			I2C_State = 18; // Terminate transaction, prepare for next
		}
		break;
	case 16: // prepare for next start
		/* finish flag */
		rxBuffer[rxBufferLength] = 0x01;

		executeCommand();
		reset_WaitStart();
		LPM0_EXIT;
		break;
	}
#ifdef WITH_STOP
	if((I2C_State == 18)) || (USICTL1 & USISTP)) {
#else
	if (I2C_State == 18) {
#endif
		executeCommand();
		reset_WaitStart();
		LPM0_EXIT;
	}
	USICTL1 &= ~USIIFG; // Clear pending flags manually (because USIIFGCC is set)
}
