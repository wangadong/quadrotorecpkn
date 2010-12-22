/*
 * SHT10����������������
 * @author Boreal Mao
 */

#include "SHT10SensorDriver.h"

#define SHT_NOACK 0
#define SHT_ACK   1

#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
/* 1 ms */
#define SHT_DELAY_CYCLES    BSP_Delay(1000)

const unsigned char CRC_Table[256] = { 0, 49, 98, 83, 196, 245, 166, 151, 185,
		136, 219, 234, 125, 76, 31, 46, 67, 114, 33, 16, 135, 182, 229, 212,
		250, 203, 152, 169, 62, 15, 92, 109, 134, 183, 228, 213, 66, 115, 32,
		17, 63, 14, 93, 108, 251, 202, 153, 168, 197, 244, 167, 150, 1, 48, 99,
		82, 124, 77, 30, 47, 184, 137, 218, 235, 61, 12, 95, 110, 249, 200,
		155, 170, 132, 181, 230, 215, 64, 113, 34, 19, 126, 79, 28, 45, 186,
		139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80, 187, 138, 217, 232,
		127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149, 248, 201, 154, 171,
		60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214, 122, 75, 24, 41,
		190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84, 57, 8, 91, 106,
		253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23, 252, 205, 158,
		175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210, 191, 142,
		221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145, 71, 118,
		37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105, 4, 53,
		102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42, 193,
		240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
		130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157,
		172 };
static unsigned char reverseByte(unsigned char byte);

/**
 * ����SCLΪ0
 */
static void Scl_Out_0() {
	SHT_I2C_PxDIR |= SHT_SCL_PIN;
	SHT_I2C_PxOUT &= ~SHT_SCL_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * ����SCLΪ1
 */
static void Scl_Out_1() {
	SHT_I2C_PxDIR |= SHT_SCL_PIN;
	SHT_I2C_PxOUT |= SHT_SCL_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * ����SDA���0
 */
static void Sda_Out_0() {
	SHT_I2C_PxDIR |= SHT_SDA_PIN;
	SHT_I2C_PxOUT &= ~SHT_SDA_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * ����SDA���1
 * FIXME TN pull up DATA to 1 should be avoided due to the document.
 */
static void Sda_Out_1() {
	SHT_I2C_PxDIR |= SHT_SDA_PIN;
	SHT_I2C_PxOUT |= SHT_SDA_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * SDA����Ϊ���룬��������
 * @return ����������
 */
static unsigned char Sda_In() {
	SHT_I2C_PxDIR &= ~SHT_SDA_PIN;
	return (SHT_I2C_PxIN & SHT_SDA_PIN);
}

/**
 * ��SCL����Ϊ����
 * ToCheck TN why need to set SCK pin in?
 */
static unsigned char Scl_In() {
	SHT_I2C_PxDIR &= ~SHT_SCL_PIN;
	return (SHT_I2C_PxIN & SHT_SCL_PIN);
}

/**
 * ��SHT1x����START�źţ�����ź���SHT1xר�е�
 * generates a transmission start
 *       _____         ________
 * DATA:      |_______|
 *           ___     ___
 * SCK : ___|   |___|   |______
 */
void sendSTART_SHT(void) {
	Sda_Out_1(); //Initial state
	Scl_Out_0();
	Scl_Out_1();
	Sda_Out_0();
	Scl_Out_0();
	Scl_Out_1();
	Sda_Out_1();
	Scl_Out_0();
}

/**
 * �ͷ�SDA��SCL
 */
static void releaseBus_SHT() {
	/* Set SDA and SCL as input */
	Scl_In();
	Sda_In();
}

/**
 * ����ACK�ź�
 */
static void sendACK_SHT() {
	/* SDA��Ϊ�� */
	Sda_Out_0();

	/* ����һ��CLK */
	Scl_Out_1();
	Scl_Out_0();
}

/**
 * ����NoACK�ź�
 */
static void sendNOACK_SHT() {
	/* SDA��Ϊ��  */
	Sda_Out_1();

	/* ����һ��CLK */
	Scl_Out_1();
	Scl_Out_0();
}

/**
 * �������϶���ACK�ź�
 * @return
 *  TRUE��ACK�ź�
 *  FALSE��NoACK�ź�
 */
static unsigned char receiveACK_SHT() {
	/* SDA��Ϊ���� */
	Sda_In();

	/* SCL��Ϊ1*/
	Scl_Out_1();

	/* SDAΪ�ߣ���NoACK�ź� */
	if (Sda_In()) {
		releaseBus_SHT(); //ToCheck TN ???
		return FALSE;
	}
	/* SDAΪ�ͣ�ACK�ź� */
	/* SCL��Ϊ0 */
	Scl_Out_0();
	return TRUE;

}

/**
 * ��SHT1xдһ���ֽ�
 * @param value Ҫд����ֽ�
 * @return ACK�ź�
 * TRUE��ACK�ź�
 * FALSE��NoACK�źţ��쳣
 */
unsigned char writeBytes_SHT(unsigned char value) {

	sendSTART_SHT();

	/* Send byte */
	for (register unsigned char nC = 0; nC < 8; nC++) {
		/* Set data on SDA line (MSB first) */
		if (value & 0x80)
			/* Set SDA to high */
			Sda_Out_1();
		else
			/* Set SDA to low */
			Sda_Out_0();

		value <<= 1;

		/* Generate clock (SCL high to low transition) */
		Scl_Out_1();
		Scl_Out_0();
	}

	/* Read ACK */
	return receiveACK_SHT();
}

/**
 * ��SHT1x����һ���ֽ�
 * @param ack 0���������NoACK�źţ���0���������ACK�źţ���ʾ���滹Ҫ������
 * @return ����������
 */
unsigned char readByte_SHT(unsigned char ack) {
	unsigned char temp;

	/* SDA��Ϊ���� */
	//FIXME TN here has logic problem.  already be SDA_IN before call this function.
	Sda_In();

	/* ��ʼ��������ݵı��� */
	temp = 0;

	/* ��ʼ�� */
	for (register unsigned char nC = 0; nC < 8; nC++) {
		/* SCL���1 */
		Scl_Out_1();

		/* ��SDA�ϵ������� */
		//FIXME TN the |= is useless and confusing.
		if (Sda_In())
			temp |= 1;

		if (nC != 7)
			temp <<= 1;

		/* SCL���0������һ��CLK */
		Scl_Out_0();
	}

	/* ����ack������������ACK�źŻ���NoACK�ź� */
	if (ack)
		sendACK_SHT();
	else
		sendNOACK_SHT();

	return temp;
}

/**
 * ������ʪ��
 * @param p_value����Ų������ 2bytes
 *        p_checksum����Ž��У���� 1bytes
 * @return
 * 1���ɹ�
 * 0��ʧ�ܣ�����������ʱ������ĳ�β���û�л�Ack�ź�
 */
unsigned char detectTemperature_SHT(unsigned char *p_value,
		unsigned char *p_checksum) {

	if (writeBytes_SHT(MEASURE_TEMP) == FALSE)
		return FALSE;

	delayInMs_BSP(100);

	/* �ȴ���SHT1x������Ϻ���Sda��0��Ϊ������ϵ�ACK�ź� */
	for (register unsigned short i = 0; i < 200; i++) {
		delayInMs_BSP(10);
		if (Sda_In() == 0)
			break;
	}

	/* ���SDAһֱΪ1ֱ����ʱ(2s)����ô�ô����� */
	if (Sda_In())
		return FALSE;

	/* ������ֵ����8λ�ֽ� */
	p_value[0] = readByte_SHT(SHT_ACK);

	/* ������ֵ����8λ�ֽ� */
	p_value[1] = readByte_SHT(SHT_ACK);

	/* ��У���� *///ToCheck TN NOACK used correctly?
	*p_checksum = readByte_SHT(SHT_NOACK);

	releaseBus_SHT();
	return TRUE;
}
/**
 * ����ʪ��
 * @param p_value����Ų������ 2bytes
 *        p_checksum����Ž��У���� 1bytes
 * @return
 * 1���ɹ�
 * 0��ʧ�ܣ�����������ʱ������ĳ�β���û�л�Ack�ź�
 */
unsigned char detectHumidity_SHT(unsigned char *p_value,
		unsigned char *p_checksum) {

	if (writeBytes_SHT(MEASURE_HUMI) == FALSE)
		return FALSE;

	delayInMs_BSP(100);

	/* �ȴ���SHT1x������Ϻ���Sda��0��Ϊ������ϵ�ACK�ź� */
	for (register unsigned short i = 0; i < 200; i++) {
		delayInMs_BSP(10);
		if (Sda_In() == 0)
			break;
	}

	/* ���SDAһֱΪ1ֱ����ʱ(2s)����ô�ô����� */
	if (Sda_In())
		return FALSE;

	/* ������ֵ����8λ�ֽ� */
	p_value[0] = readByte_SHT(SHT_ACK);

	/* ������ֵ����8λ�ֽ� */
	p_value[1] = readByte_SHT(SHT_ACK);

	/* ��У���� *///ToCheck TN NOACK used correctly?
	*p_checksum = readByte_SHT(SHT_NOACK);

	releaseBus_SHT();
	return TRUE;
}

unsigned char checkHumidity_SHT(unsigned char *p_value,
		unsigned char *p_checksum) {
	volatile unsigned char crc = 0x00;

	/* check command */
	crc = MEASURE_HUMI ^ crc;
	crc = CRC_Table[crc];

	/* check first byte */
	crc = p_value[0] ^ crc;
	crc = CRC_Table[crc];

	/* check second byte */
	crc = p_value[1] ^ crc;
	crc = CRC_Table[crc];

	crc = reverseByte(crc);
	if (crc == *p_checksum) {
		return TRUE;
	} else {
		return FALSE;
	}

}

unsigned char checkTemperature_SHT(unsigned char *p_value,
		unsigned char *p_checksum) {
	volatile unsigned char crc = 0x00;

	/* check command */
	crc = MEASURE_TEMP ^ crc;
	crc = CRC_Table[crc];

	/* check first byte */
	crc = p_value[0] ^ crc;
	crc = CRC_Table[crc];

	/* check second byte */
	crc = p_value[1] ^ crc;
	crc = CRC_Table[crc];

	crc = reverseByte(crc);
	if (crc == *p_checksum) {
		return TRUE;
	} else {
		return FALSE;
	}

}

/**
 * reverse a byte
 * @param unsigned char byte:
 * @return unsigned char result:the reversed byte
 */
unsigned char reverseByte(unsigned char byte) {
	unsigned char result = 0;

	for (unsigned char i = 0; i < 8; i++) {
		result = (result << 1) + (byte & 1);
		byte >>= 1;
	}

	return result;
}
