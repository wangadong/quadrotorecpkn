/*
 * Driver for TMP275 conversion.
 *
 * Notice: Lots of TMP275 features we do NOT use in Sungari. such as:
 * 1. ALERT mechanism.
 *     including TM/FQ/POL etc.
 * 2. Shutdown Mode: tell TMP275 to shutdown after current conversion
 *
 * We use
 * 1. One Shot Mode or Continuous Mode
 * 2. 12 bit Resolution only.
 *
 * @author Taurus Ning
 */

#include "TMP275.h"

/* value of Pointer Register to access Temperature Register */
#define ACCESS_TEMERATURE_REG 0x00

/* value of Pointer Register to access Configuration Register */
#define ACCESS_CONFIGURATION_REG 0x01

/* TODO try to refine code to better bit operation to use them.*/
#define RWBIT_WRITE 0
#define RWBIT_READ 1

/**
 * Sungari Configuration:
 * Use One-Shot, Highest Resolution.
 * Don't use alert mechanism.
 * so it should be: 1 11 00 000 = 0xE0
 * OS(D7)=1: USE One Shot Mode
 * Resolution(D6/D5)=11: 12bits
 * Fault Queue(D4/D3)=00: consecutive faults  set to 1
 * POL(D2)=0: alert pin active LOW
 * TM(D1)=0: comparator mode
 * SD(D0)=0: continuous conversion state
 *
 * change to 0xE1 for enter shutdown mode
 */
#define SUNGARI_TMP_CONFIGURATION 0xE0

static unsigned char parseAddress_TMP(unsigned char address);

/**
 * parse 3 bits address to TMP bus address.
 * "1001 A2-A1 0"
 * @param 0-7, means A2-A1
 */
unsigned char parseAddress_TMP(unsigned char address) {
	/* shift to left 1 bit. */
	address = address << 1;

	/* set High 4 bit to 1001(TMP required)*/
	address &= 0x0F; //SET High 4 bits to 0, don't change Low 4 bits.
	address |= 0x90; //SET High 4 bits to 1001 safely, don't change Low 4 bits.

	return address;
}

/**
 * Don't need to know the state of TMP,
 * write a config of SD immediately.
 *
 * After execute this feature, the caller should wait a while for shutdown taking effects.
 *
 * @param address 0-7, means A2,A1,A0
 * @return TRUE/FALSE means successfully written or not.
 */
unsigned char executeShutdown_TMP275(unsigned char address) {
	unsigned char bytesToTMP[2];

	address = parseAddress_TMP(address);

	bytesToTMP[0] = ACCESS_CONFIGURATION_REG;
	bytesToTMP[1] = 0x01; //shutdown

	if (!write_GSI2CM(address, bytesToTMP, 2))
		return FALSE;

	return TRUE;
}

/**
 * Tell TMP to execute OneShot mode detection.
 *
 * After execute this feature, the caller should wait 320ms
 * for the conversion before reading result.
 *
 * @param address 0-7, means A2,A1,A0
 * @return TRUE/FALSE means successful or not.
 */
unsigned char executeOneShotDetection_TMP275(unsigned char address) {
	unsigned char bytesToTMP[2];

	address = parseAddress_TMP(address);

	bytesToTMP[0] = ACCESS_CONFIGURATION_REG;
	bytesToTMP[1] = 0xE0; //111 0000 0 OneShot+12bits

	if (!write_GSI2CM(address, bytesToTMP, 2))
		return FALSE;

	return TRUE;
}

/**
 * Tell TMP to execute continuous detection.
 *
 * After execute this feature, the caller should wait 320ms
 * for the conversion before reading result.
 *
 * @param address 0-7, means A2,A1,A0
 * @return TRUE/FALSE means successful or not.
 */
unsigned char executeContinuousDetection_TMP275(unsigned char address) {
	unsigned char bytesToTMP[2];

	address = parseAddress_TMP(address);

	bytesToTMP[0] = ACCESS_CONFIGURATION_REG;
	bytesToTMP[1] = 0x60; //0 11 00 00 0 12bits

	if (!write_GSI2CM(address, bytesToTMP, 2))
		return FALSE;

	return TRUE;
}

/**
 *
 * Notice: I2C driver will take care of the RWBIT,
 *         so it needn't to deal the RWBIT in this function.
 * @param address 0-7 is legal input.
 *                means 0000 0 A2 A1 A0
 * @param *bufferPointer where to store the reading result.
 *                       2 bytes required.
 */
unsigned char readTemperature_TMP(unsigned char address,
		unsigned char * bufferPointer) {
	unsigned char bytesToTMP[1];

	address = parseAddress_TMP(address);

	bytesToTMP[0] = ACCESS_TEMERATURE_REG;
	if (!write_GSI2CM(address, bytesToTMP, 1))
		return FALSE;

	/* Read temperature */
	if (!read_GSI2CM(address, bufferPointer, 2))
		return FALSE;

	return TRUE;
}

