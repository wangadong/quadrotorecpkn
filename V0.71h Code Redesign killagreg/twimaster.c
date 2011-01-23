/*############################################################################
############################################################################*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "main.h"
#include "twimaster.h"
#include "fc.h"
#include "analog.h"

volatile uint8_t twi_state 		= 0;
volatile uint8_t motor_write 	= 0;
volatile uint8_t motor_read 	= 0;
volatile uint8_t dac_channel 	= 0;
volatile uint8_t motor_rx[8];
volatile uint16_t I2CTimeout = 100;


#define SCL_CLOCK  200000L
#define I2C_TIMEOUT 30000

#define TWSR_STATUS_MASK			0xF8
// for Master Transmitter Mode

#define I2C_STATUS_START			0x08
#define I2C_STATUS_REPEATSTART		0x10
#define I2C_STATUS_TX_SLA_ACK		0x18
#define I2C_STATUS_SLAW_NOACK		0x20
#define I2C_STATUS_TX_DATA_ACK		0x28
#define I2C_STATUS_TX_DATA_NOTACK	0x30
#define I2C_STATUS_RX_DATA_ACK		0x50
#define I2C_STATUS_RX_DATA_NOTACK	0x58

/**************************************************/
/*   Initialize I2C (TWI)                         */
/**************************************************/
void I2C_Init(void)
{
	uint8_t sreg = SREG;
	cli();

	// SDA is INPUT
	DDRC  &= ~(1<<DDC1);
	// SCL is output
	DDRC |= (1<<DDC0);
	// pull up SDA
	PORTC |= (1<<PORTC0)|(1<<PORTC1);

	// TWI Status Register
	// prescaler 1 (TWPS1 = 0, TWPS0 = 0)
	TWSR &= ~((1<<TWPS1)|(1<<TWPS0));

	// set TWI Bit Rate Register
	TWBR = ((SYSCLK/SCL_CLOCK)-16)/2;

	twi_state 		= 0;
	motor_write 	= 0;
	motor_read 		= 0;

	SREG = sreg;
}

/****************************************/
/*   Start I2C                          */
/****************************************/
void I2C_Start(void)
{
	// TWI Control Register
	// clear TWI interrupt flag (TWINT=1)
	// disable TWI Acknowledge Bit (TWEA = 0)
	// enable TWI START Condition Bit (TWSTA = 1), MASTER
	// disable TWI STOP Condition Bit (TWSTO = 0)
	// disable TWI Write Collision Flag (TWWC = 0)
	// enable i2c (TWEN = 1)
	// enable TWI Interrupt (TWIE = 1)
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
}

/****************************************/
/*    Stop I2C                          */
/****************************************/
void I2C_Stop(void)
{
	// TWI Control Register
	// clear TWI interrupt flag (TWINT=1)
	// disable TWI Acknowledge Bit (TWEA = 0)
	// diable TWI START Condition Bit (TWSTA = 1), no MASTER
	// enable TWI STOP Condition Bit (TWSTO = 1)
	// disable TWI Write Collision Flag (TWWC = 0)
	// enable i2c (TWEN = 1)
	// disable TWI Interrupt (TWIE = 0)
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}


/****************************************/
/*    Write to I2C                      */
/****************************************/
void I2C_WriteByte(int8_t byte)
{
    // move byte to send into TWI Data Register
    TWDR = byte;
    // clear interrupt flag (TWINT = 1)
    // enable i2c bus (TWEN = 1)
    // enable interrupt (TWIE = 1)
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}


/****************************************/
/*    Receive byte and send ACK         */
/****************************************/
void I2C_ReceiveByte(void)
{
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
}

/****************************************/
/* I2C receive last byte and send no ACK*/
/****************************************/
void I2C_ReceiveLastByte(void)
{
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}


/****************************************/
/*    Reset I2C                         */
/****************************************/
void I2C_Reset(void)
{
	// stop i2c bus
	I2C_Stop();
	twi_state 		= 0;
	motor_write 	= TWDR;
	motor_write  	= 0;
	motor_read 		= 0;
	TWCR = (1<<TWINT); // reset to original state incl. interrupt flag reset
	TWAMR = 0;
	TWAR = 0;
	TWDR = 0;
	TWSR = 0;
	TWBR = 0;
	I2C_Init();
	I2C_Start();
	I2C_WriteByte(0);
}

/****************************************/
/*        I2C ISR                       */
/****************************************/
ISR (TWI_vect)

{

    switch (twi_state++) // First i2c_start from SendMotorData()
	{
		// Master Transmit
        case 0: // Send SLA-W
                I2C_WriteByte(0x52 + (motor_write * 2) );
                break;
        case 1: // Send Data to Slave
                switch(motor_write)
                {
                    case 0:
                            I2C_WriteByte(Motor_Front);
                            break;
                    case 1:
                            I2C_WriteByte(Motor_Rear);
                            break;
                    case 2:
                            I2C_WriteByte(Motor_Right);
                            break;
                    case 3:
                            I2C_WriteByte(Motor_Left);
                            break;
                }
                break;
        case 2: // repeat case 0+1 for all motors
        		I2C_Stop();
                if (motor_write < 3)
                {
					motor_write++; // jump to next motor
					twi_state = 0; // and repeat from state 0
				}
                else
                {	// data to last motor send
					motor_write = 0; // reset motor write counter
				}
                I2C_Start(); // Repeated start -> switch slave or switch Master Transmit -> Master Receive
                break;

        // Master Receive
        case 3: // Send SLA-R
                I2C_WriteByte(0x53 + (motor_read * 2) );
                break;
        case 4:
                //Transmit 1st byte
				I2C_ReceiveByte();
                break;
        case 5: //Read 1st byte and transmit 2nd Byte
                motor_rx[motor_read] = TWDR;
				I2C_ReceiveLastByte();
				break;
        case 6:
                //Read 2nd byte
				motor_rx[motor_read + 4] = TWDR;
				motor_read++;
                if (motor_read > 3) motor_read = 0;
                I2C_Stop();
				twi_state = 0;
                I2CTimeout = 10;
                break;

		// Gyro-Offsets
		case 7:
				I2C_WriteByte(0x98); // Address the DAC
				break;

		case 8:
				I2C_WriteByte(0x10 + (dac_channel * 2)); // Select DAC Channel (0x10 = A, 0x12 = B, 0x14 = C)
				break;

		case 9:
				switch(dac_channel)
				{
					case 0:
							I2C_WriteByte(AnalogOffsetNick); // 1st byte for Channel A
							break;
					case 1:
							I2C_WriteByte(AnalogOffsetRoll); // 1st byte for Channel B
							break;
					case 2:
							I2C_WriteByte(AnalogOffsetYaw ); // 1st byte for Channel C
							break;
				}
				break;

		case 10:
				I2C_WriteByte(0x80); // 2nd byte for all channels is 0x80
				break;

		case 11:
				I2C_Stop();
				I2CTimeout = 10;
				// repeat case 7...10 until all DAC Channels are updated
				if(dac_channel < 2)
				{
					dac_channel ++; 	// jump to next channel
					twi_state = 7; 		// and repeat from state 7
					I2C_Start(); 		// start transmission for next channel
				}
				else
				{	// data to last motor send
					dac_channel = 0; // reset dac channel counter
					twi_state = 0;   // reset twi_state
				}
                break;

        default:
                I2C_Stop();
                twi_state = 0;
                I2CTimeout = 10;
                motor_write = 0;
                motor_read = 0;
	}
}
