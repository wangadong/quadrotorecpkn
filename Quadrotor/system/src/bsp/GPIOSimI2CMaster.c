/*
 *  @author Boreal Mao
 */
#include "GPIOSimI2CMaster.h"

#define DELAY_STEPS_IN_OPERATE 5

#define SDA_1       P4OUT |=  BIT4; delay_GSI2CM(DELAY_STEPS_IN_OPERATE);
#define SDA_0       P4OUT &=~ BIT4; delay_GSI2CM(DELAY_STEPS_IN_OPERATE);
#define SCL_1       P4OUT |=  BIT5; delay_GSI2CM(DELAY_STEPS_IN_OPERATE);
#define SCL_0       P4OUT &=~ BIT5; delay_GSI2CM(DELAY_STEPS_IN_OPERATE);

/* control SDA mode */
#define DIR_IN      P4DIR &=~ BIT4
#define DIR_OUT     P4DIR |=  BIT4
#define DIR_OUT_SCL		P4DIR |=  BIT5

/* read 1 bit from SDA */
#define SDA_IN      ((P4IN >> 4) & 0x01)

static void delay_GSI2CM(unsigned int n);
static void start_GSI2CM(void);
static void stop_GSI2CM(void);
static unsigned char writeByte_GSI2CM(unsigned char data);
static unsigned char readByte_GSI2CM(void);
static unsigned char receiveAck_GSI2CM(void);
static void ack_GSI2CM(void);

/**
 * ��ʱ�ú���
 * The requirement is to delay some us, according to I2C Spec.
 * (wait the V on SDA or SCL be stable after a change)
 * It doesn't need so precious here, so we use the 430clock unit as us.
 * @param n delay time in us.
 */
static void delay_GSI2CM(unsigned int n) {
	while (n)
		n--;
}

/**
 * Initiate for i2c usage at very beginning.
 */
void init_GSI2CM(void) {
	DIR_OUT_SCL;
	DIR_OUT;
	SCL_1;
	SDA_1;
}

/**
 * I2C START�ź�
 */
static void start_GSI2CM(void) {
	SDA_1;
	SCL_1;
	SDA_0;
	SCL_0;
}

/**
 * I2C STOP�ź�
 */
static void stop_GSI2CM(void) {
	SDA_0;
	SCL_1;
	SDA_1;
}

/**
 * ��I2C����д1byte����
 * @Param data Ҫд������
 * @Return
 * FASLE д����ʧ�� No ack returned.
 * TRUE �յ�ACK��д�����ɹ�
 */
static unsigned char writeByte_GSI2CM(unsigned char data) {
	for (register unsigned char i = 0; i < 8; i++) {
		SCL_0;

		if (((data >> 7) & 0x01) == 0x01) {
			SDA_1;
		} else {
			SDA_0;
		}

		SCL_1;
		data = data << 1;
	}

	SCL_0;

	/*when the slave finish receiving 1 byte, it will give ack back*/
	return receiveAck_GSI2CM();
}

/**
 * ��I2C���߶�1byte����
 * @Return ����������
 */
static unsigned char readByte_GSI2CM(void) {

	unsigned char tempBit = 0;
	unsigned char tempData = 0;

	SCL_0;
	DIR_IN;

	for (register unsigned char i = 0; i < 8; i++) {
		SCL_0;
		SCL_1;

		if (SDA_IN == 0x01) {
			tempBit = 1;
		} else {
			tempBit = 0;
		}
		tempData = (tempData << 1) | tempBit;
	}
	SCL_0;
	DIR_OUT;
	return (tempData);
}

/**
 * receive ack from Slave??
 * ����ACK
 * @Return
 * FALSE û���յ�ACK
 * TRUE �յ�ACK
 */
static unsigned char receiveAck_GSI2CM(void) {
	SCL_0;
	DIR_IN;
	SCL_1;
	delay_GSI2CM(DELAY_STEPS_IN_OPERATE); // Wait for SLAVE to prepare the ACK signal
	if (SDA_IN) {
		SCL_0;
		DIR_OUT;
		return FALSE;
	}

	SCL_0;
	DIR_OUT;
	return TRUE;
}

/**
 * send ACK signal to the slave
 */
static void ack_GSI2CM(void) {
	SCL_0;
	DIR_OUT;
	SDA_0;
	SCL_1;
}

/**
 * send NACK signal to the slave
 */
static void nack_GSI2CM(void) {
	SCL_0;
	DIR_OUT;
	SDA_1;
	SCL_1;
}
/**
 * �������϶������ɸ�byte
 * @param Address ���豸��ַ
 * @param Buffer ��Ŷ�����ָ��
 * @param nBufLen �����ĸ���
 * @return��
 * FALSE ���豸ûӦ��ʧ��
 * TRUE �ɹ�
 */
unsigned char read_GSI2CM(unsigned char Address, unsigned char* Buffer,
		unsigned short nBufLen) {

	/* ��дλ��1��Ϊ������ */
	Address |= 0x01;

	/* ����START�ź� */
	start_GSI2CM();

	/* ���ʹ��豸��ַ�����û��Ӧ��˵�����豸������ */
	if (writeByte_GSI2CM(Address) != TRUE) {
		stop_GSI2CM();
		return FALSE;
	}

	/* �������һ��byte����������͵���ACK�ź� */
	for (register unsigned char i = 0; i < nBufLen - 1; i++) {
		Buffer[i] = readByte_GSI2CM();
		ack_GSI2CM();
	}
	Buffer[nBufLen - 1] = readByte_GSI2CM();
	nack_GSI2CM();

	/* ����STOP�ź� */
	stop_GSI2CM();
	return TRUE;
}

/**
 * д���ɸ�byte��������
 * @param Address ���豸��ַ
 * @param Buffer ���д���ݵ�ָ��
 * @param nBufLen д���ĸ���
 * @return��
 * FALSE ���������У����豸�쳣ûӦ�� TODO this is not same with the code!!
 * TRUE �ɹ�
 */
unsigned char write_GSI2CM(unsigned char Address, unsigned char* Buffer,
		unsigned short nBufLen) {
	/* ��дλ��0��Ϊд���� */
	Address &= ~0x01;

	/* ����START�ź� */
	start_GSI2CM();

	/* ���ʹ��豸��ַ�����NoACK����ʾ���豸������ */
	if (writeByte_GSI2CM(Address) != TRUE) {
		stop_GSI2CM();
		return FALSE;
	}

	/* д��ǰnBufLen-1�ֽ� ��if error occurs while write any byte, return false immediately. */
	for (register unsigned char i = 0; i < nBufLen - 1; i++) {
		if (writeByte_GSI2CM(Buffer[i]) != TRUE) {
			stop_GSI2CM();
			return FALSE;
		}
	}
	/* д�����һ���ֽڣ�����SLAVE����NACK��ACK����Ϊ���ͳɹ� */
	writeByte_GSI2CM(Buffer[nBufLen - 1]);
	/* ����STOP�ź� */
	stop_GSI2CM();
	return TRUE;
}
