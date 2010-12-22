/*
 * SHT10传感器的驱动程序
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
 * 设置SCL为0
 */
static void Scl_Out_0() {
	SHT_I2C_PxDIR |= SHT_SCL_PIN;
	SHT_I2C_PxOUT &= ~SHT_SCL_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * 设置SCL为1
 */
static void Scl_Out_1() {
	SHT_I2C_PxDIR |= SHT_SCL_PIN;
	SHT_I2C_PxOUT |= SHT_SCL_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * 设置SDA输出0
 */
static void Sda_Out_0() {
	SHT_I2C_PxDIR |= SHT_SDA_PIN;
	SHT_I2C_PxOUT &= ~SHT_SDA_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * 设置SDA输出1
 * FIXME TN pull up DATA to 1 should be avoided due to the document.
 */
static void Sda_Out_1() {
	SHT_I2C_PxDIR |= SHT_SDA_PIN;
	SHT_I2C_PxOUT |= SHT_SDA_PIN;
	SHT_DELAY_CYCLES;
}

/**
 * SDA设置为输入，读入数据
 * @return 读到的数据
 */
static unsigned char Sda_In() {
	SHT_I2C_PxDIR &= ~SHT_SDA_PIN;
	return (SHT_I2C_PxIN & SHT_SDA_PIN);
}

/**
 * 将SCL设置为输入
 * ToCheck TN why need to set SCK pin in?
 */
static unsigned char Scl_In() {
	SHT_I2C_PxDIR &= ~SHT_SCL_PIN;
	return (SHT_I2C_PxIN & SHT_SCL_PIN);
}

/**
 * 给SHT1x发送START信号，这个信号是SHT1x专有的
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
 * 释放SDA和SCL
 */
static void releaseBus_SHT() {
	/* Set SDA and SCL as input */
	Scl_In();
	Sda_In();
}

/**
 * 发送ACK信号
 */
static void sendACK_SHT() {
	/* SDA设为低 */
	Sda_Out_0();

	/* 产生一个CLK */
	Scl_Out_1();
	Scl_Out_0();
}

/**
 * 发送NoACK信号
 */
static void sendNOACK_SHT() {
	/* SDA设为高  */
	Sda_Out_1();

	/* 产生一个CLK */
	Scl_Out_1();
	Scl_Out_0();
}

/**
 * 从总线上读回ACK信号
 * @return
 *  TRUE，ACK信号
 *  FALSE，NoACK信号
 */
static unsigned char receiveACK_SHT() {
	/* SDA设为输入 */
	Sda_In();

	/* SCL设为1*/
	Scl_Out_1();

	/* SDA为高，则NoACK信号 */
	if (Sda_In()) {
		releaseBus_SHT(); //ToCheck TN ???
		return FALSE;
	}
	/* SDA为低，ACK信号 */
	/* SCL设为0 */
	Scl_Out_0();
	return TRUE;

}

/**
 * 给SHT1x写一个字节
 * @param value 要写入的字节
 * @return ACK信号
 * TRUE，ACK信号
 * FALSE，NoACK信号，异常
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
 * 从SHT1x读回一个字节
 * @param ack 0，读完后发送NoACK信号；非0，读完后发送ACK信号，表示后面还要继续读
 * @return 读到的数据
 */
unsigned char readByte_SHT(unsigned char ack) {
	unsigned char temp;

	/* SDA设为输入 */
	//FIXME TN here has logic problem.  already be SDA_IN before call this function.
	Sda_In();

	/* 初始化存放数据的变量 */
	temp = 0;

	/* 开始读 */
	for (register unsigned char nC = 0; nC < 8; nC++) {
		/* SCL输出1 */
		Scl_Out_1();

		/* 把SDA上的数读回 */
		//FIXME TN the |= is useless and confusing.
		if (Sda_In())
			temp |= 1;

		if (nC != 7)
			temp <<= 1;

		/* SCL输出0，产生一个CLK */
		Scl_Out_0();
	}

	/* 根据ack参数决定发送ACK信号还是NoACK信号 */
	if (ack)
		sendACK_SHT();
	else
		sendNOACK_SHT();

	return temp;
}

/**
 * 测量温湿度
 * @param p_value，存放测量结果 2bytes
 *        p_checksum，存放结果校验码 1bytes
 * @return
 * 1，成功
 * 0，失败，测量动作超时，或者某次操作没有回Ack信号
 */
unsigned char detectTemperature_SHT(unsigned char *p_value,
		unsigned char *p_checksum) {

	if (writeBytes_SHT(MEASURE_TEMP) == FALSE)
		return FALSE;

	delayInMs_BSP(100);

	/* 等待，SHT1x测量完毕后会把Sda清0作为测量完毕的ACK信号 */
	for (register unsigned short i = 0; i < 200; i++) {
		delayInMs_BSP(10);
		if (Sda_In() == 0)
			break;
	}

	/* 如果SDA一直为1直到超时(2s)，那么置错误标记 */
	if (Sda_In())
		return FALSE;

	/* 读测量值，高8位字节 */
	p_value[0] = readByte_SHT(SHT_ACK);

	/* 读测量值，低8位字节 */
	p_value[1] = readByte_SHT(SHT_ACK);

	/* 读校验码 *///ToCheck TN NOACK used correctly?
	*p_checksum = readByte_SHT(SHT_NOACK);

	releaseBus_SHT();
	return TRUE;
}
/**
 * 测量湿度
 * @param p_value，存放测量结果 2bytes
 *        p_checksum，存放结果校验码 1bytes
 * @return
 * 1，成功
 * 0，失败，测量动作超时，或者某次操作没有回Ack信号
 */
unsigned char detectHumidity_SHT(unsigned char *p_value,
		unsigned char *p_checksum) {

	if (writeBytes_SHT(MEASURE_HUMI) == FALSE)
		return FALSE;

	delayInMs_BSP(100);

	/* 等待，SHT1x测量完毕后会把Sda清0作为测量完毕的ACK信号 */
	for (register unsigned short i = 0; i < 200; i++) {
		delayInMs_BSP(10);
		if (Sda_In() == 0)
			break;
	}

	/* 如果SDA一直为1直到超时(2s)，那么置错误标记 */
	if (Sda_In())
		return FALSE;

	/* 读测量值，高8位字节 */
	p_value[0] = readByte_SHT(SHT_ACK);

	/* 读测量值，低8位字节 */
	p_value[1] = readByte_SHT(SHT_ACK);

	/* 读校验码 *///ToCheck TN NOACK used correctly?
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
