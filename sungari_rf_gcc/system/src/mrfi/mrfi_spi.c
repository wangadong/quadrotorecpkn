/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   SPI interface code for Radio FamRadios: CC1100, CC1101, CC2500
 *   SPI interface code.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#include "mrfi_spi.h"

/* ���� mrfi debug*/
#define MRFI_SPI_DEBUG

#define DUMMY_BYTE                  0xDB
#define READ_BIT                    0x80
#define BURST_BIT                   0x40

#define MRFI_SPI_TURN_CHIP_SELECT_ON()        MRFI_SPI_DRIVE_CSN_LOW()
#define MRFI_SPI_TURN_CHIP_SELECT_OFF()       MRFI_SPI_DRIVE_CSN_HIGH()
#define MRFI_SPI_CHIP_SELECT_IS_OFF()         MRFI_SPI_CSN_IS_HIGH()

#ifdef MRFI_SPI_DEBUG
#define MRFI_SPI_ASSERT(x)      BSP_ASSERT(x)
#else
#define MRFI_SPI_ASSERT(x)
#endif

static unsigned char spiRegAccess(unsigned char addrByte,
		unsigned char writeValue);
static void spiBurstFifoAccess(unsigned char addrByte, unsigned char * pData,
		unsigned char len);

/**
 * SPI�ӿڳ�ʼ��
 */
void mrfiSpiInit(void) {
	/* configure all SPI related pins */
	MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT();
	MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT();
	MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT(); MRFI_SPI_CONFIG_SO_PIN_AS_INPUT();

	/* set CSn to default high level */
	MRFI_SPI_DRIVE_CSN_HIGH();

	/* initialize the SPI registers */
	MRFI_SPI_INIT();
}

/**
 * ��strobe��ʽͨ��SPI����mrfi���ָ��
 * @param addr Ҫ������ָ��Ĵ�����ַ
 * @return оƬ��״ֵ̬
 */
unsigned char mrfiSpiCmdStrobe(unsigned char addr) {
	unsigned char statusByte;
	mrfiSpiIState_t s;

	MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() ); /* SPI is not initialized */
	MRFI_SPI_ASSERT((addr >= 0x30) && (addr <= 0x3D)); /* invalid address */

	/* disable interrupts that use SPI */
	MRFI_SPI_ENTER_CRITICAL_SECTION(s);

	/* turn chip select "off" and then "on" to clear any current SPI access */
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
	MRFI_SPI_TURN_CHIP_SELECT_ON();

	/* send the command strobe, wait for SPI access to complete */
	MRFI_SPI_WRITE_BYTE(addr);
	MRFI_SPI_WAIT_DONE();

	/* read the readio status byte returned by the command strobe */
	statusByte = MRFI_SPI_READ_BYTE();

	/* turn off chip select; enable interrupts that call SPI functions */
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);

	/* return the status byte */
	return (statusByte);
}

/**
 * ͨ��SPI�ڶ�ȡ����оƬ�Ĵ�����ֵ
 * @param addr Ҫ�����ļĴ�����ַ
 * @return �üĴ�����ֵ
 */
unsigned char mrfiSpiReadReg(unsigned char addr) {
	MRFI_SPI_ASSERT(addr <= 0x3B); /* invalid address */

	/*
	 *  The burst bit is set to allow access to read-only status registers.
	 *  This does not affect normal register reads.
	 */
	return (spiRegAccess(addr | BURST_BIT | READ_BIT, DUMMY_BYTE));
}

/**
 * ��װ�õ�ͨ��SPI��д����оƬ�Ĵ���
 * @param addr Ҫ�����ļĴ�����ַ
 * @param value Ҫд���ֵ
 */
void mrfiSpiWriteReg(unsigned char addr, unsigned char value) {
	MRFI_SPI_ASSERT((addr <= 0x2E) || (addr == 0x3E)); /* invalid address */

	spiRegAccess(addr, value);
}

/**
 * ͨ��SPI�ڷ�������оƬ�Ĵ������Ƕ�����д�����Ǹ���addrByte�Ķ�дλ������
 * @param addrByte Ҫ�����ļĴ�����ַ
 * @param writeValue Ҫд���ֵ
 * @return �ӼĴ���������ֵ
 */
static unsigned char spiRegAccess(unsigned char addrByte,
		unsigned char writeValue) {
	unsigned char readValue;
	mrfiSpiIState_t s;

	MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() ); /* SPI is not initialized */

	/* disable interrupts that use SPI */
	MRFI_SPI_ENTER_CRITICAL_SECTION(s);

	/* turn chip select "off" and then "on" to clear any current SPI access */
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
	MRFI_SPI_TURN_CHIP_SELECT_ON();

	/* send register address byte, the read/write bit is already configured */
	MRFI_SPI_WRITE_BYTE(addrByte);
	MRFI_SPI_WAIT_DONE();

	/*
	 *  Send the byte value to write.  If this operation is a read, this value
	 *  is not used and is just dummy data.  Wait for SPI access to complete.
	 */
	MRFI_SPI_WRITE_BYTE(writeValue);
	MRFI_SPI_WAIT_DONE();

	/*
	 *  If this is a read operation, SPI data register now contains the register
	 *  value which will be returned.  For a read operation, it contains junk info
	 *  that is not used.
	 */
	readValue = MRFI_SPI_READ_BYTE();

	/* turn off chip select; enable interrupts that call SPI functions */
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);

	/* return the register value */
	return (readValue);
}

/**
 * ��burst��ʽͨ��SPI������д�����ֽ����ݵ�����оƬ�ķ��ͻ�����
 * @param pData �������ݵ�ָ��
 * @param len �����ֽ���
 */
void mrfiSpiWriteTxFifo(unsigned char * pData, unsigned char len) {
	spiBurstFifoAccess(TXFIFO | BURST_BIT, pData, len);
}

/**
 * ��burst��ʽͨ��SPI������������оƬ�Ľ��ջ�����������ȡ����
 * @param pData �������ݵĴ��ָ��
 * @param len ��ȡ�ֽ���
 */
void mrfiSpiReadRxFifo(unsigned char * pData, unsigned char len) {
	spiBurstFifoAccess(RXFIFO | BURST_BIT | READ_BIT, pData, len);
}

/**
 * ��burst��ʽͨ��SPI�ڷ�������оƬ�ļĴ�����burst��ʽ��������д��/������ֵ
 * �ȵ��β���spiRegAccessЧ��Ҫ�ߵö࣬����ÿ�ζ�����д�Ĵ�����ַ
 * @param addrByte Ҫ�����ļĴ�����ַ
 * @param pData   Ҫ���������ݴ��ָ��
 * @param len �����ֽ���
 */
static void spiBurstFifoAccess(unsigned char addrByte, unsigned char * pData,
		unsigned char len) {
	mrfiSpiIState_t s;

	MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() ); /* SPI is not initialized */
	MRFI_SPI_ASSERT(len != 0); /* zero length is not allowed */
	MRFI_SPI_ASSERT(addrByte & BURST_BIT); /* only burst mode supported */

	/* disable interrupts that use SPI */
	MRFI_SPI_ENTER_CRITICAL_SECTION(s);

	/* turn chip select "off" and then "on" to clear any current SPI access */
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
	MRFI_SPI_TURN_CHIP_SELECT_ON();

	/**
	 *  Main loop.  If the SPI access is interrupted, execution comes back to
	 *  the start of this loop.  Loop exits when nothing left to transfer.
	 */
	do {
		/* send FIFO access command byte, wait for SPI access to complete */
		MRFI_SPI_WRITE_BYTE(addrByte);
		MRFI_SPI_WAIT_DONE();

		/**
		 *  Inner loop.  This loop executes as long as the SPI access is not interrupted.
		 *  Loop completes when nothing left to transfer.
		 */
		do {
			MRFI_SPI_WRITE_BYTE(*pData);

			/**
			 *  Use idle time.  Perform increment/decrement operations before pending on
			 *  completion of SPI access.
			 *
			 *  Decrement the length counter.  Wait for SPI access to complete.
			 */
			len--;
			MRFI_SPI_WAIT_DONE();

			/**
			 *  SPI data register holds data just read.  If this is a read operation,
			 *  store the value into memory.
			 */
			if (addrByte & READ_BIT) {
				*pData = MRFI_SPI_READ_BYTE();
			}

			/**
			 *  At least one byte of data has transferred.  Briefly enable (and then disable)
			 *  interrupts that can call SPI functions.  This provides a window for any timing
			 *  critical interrupts that might be pending.
			 *
			 *  To improve latency, take care of pointer increment within the interrupt
			 *  enabled window.
			 */
			MRFI_SPI_EXIT_CRITICAL_SECTION(s);
			pData++;
			MRFI_SPI_ENTER_CRITICAL_SECTION(s);

			/**
			 *  If chip select is "off" the SPI access was interrupted (all SPI access
			 *  functions leave chip select in the "off" state).  In this case, turn
			 *  back on chip select and break to the main loop.  The main loop will
			 *  pick up where the access was interrupted.
			 */
			if (MRFI_SPI_CHIP_SELECT_IS_OFF()) {
				MRFI_SPI_TURN_CHIP_SELECT_ON();
				break;
			}
		} while (len); /* inner loop */
	} while (len); /* main loop */

	/* turn off chip select; enable interrupts that call SPI functions */
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);
}

