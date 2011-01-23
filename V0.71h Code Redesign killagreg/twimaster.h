
#ifndef _I2C_MASTER_H
#define _I2C_MASTER_H

#include <inttypes.h>

#define TWI_STATE_MOTOR_TX 			0
#define TWI_STATE_GYRO_OFFSET_TX	7

extern volatile uint8_t twi_state;
extern volatile uint8_t motor_rx[8];
extern volatile uint16_t I2CTimeout;

extern void I2C_Init (void); // Initialize I2C
extern void I2C_Start(void); // Start I2C
extern void I2C_Stop (void); // Stop I2C
extern void I2C_Reset(void); // Reset I2C

#endif
