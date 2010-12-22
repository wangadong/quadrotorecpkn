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
 * 延时用函数
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
 * I2C START信号
 */
static void start_GSI2CM(void) {
	SDA_1;
	SCL_1;
	SDA_0;
	SCL_0;
}

/**
 * I2C STOP信号
 */
static void stop_GSI2CM(void) {
	SDA_0;
	SCL_1;
	SDA_1;
}

/**
 * 向I2C总线写1byte数据
 * @Param data 要写的数据
 * @Return
 * FASLE 写操作失败 No ack returned.
 * TRUE 收到ACK，写操作成功
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
 * 从I2C总线读1byte数据
 * @Return 读到的数据
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
 * 接收ACK
 * @Return
 * FALSE 没有收到ACK
 * TRUE 收到ACK
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
 * 从总线上读回若干个byte
 * @param Address 从设备地址
 * @param Buffer 存放读数的指针
 * @param nBufLen 读数的个数
 * @return：
 * FALSE 从设备没应答，失败
 * TRUE 成功
 */
unsigned char read_GSI2CM(unsigned char Address, unsigned char* Buffer,
		unsigned short nBufLen) {

	/* 读写位置1，为读操作 */
	Address |= 0x01;

	/* 发送START信号 */
	start_GSI2CM();

	/* 发送从设备地址，如果没回应则说明从设备不在线 */
	if (writeByte_GSI2CM(Address) != TRUE) {
		stop_GSI2CM();
		return FALSE;
	}

	/* 读除最后一个byte外的数，发送的是ACK信号 */
	for (register unsigned char i = 0; i < nBufLen - 1; i++) {
		Buffer[i] = readByte_GSI2CM();
		ack_GSI2CM();
	}
	Buffer[nBufLen - 1] = readByte_GSI2CM();
	nack_GSI2CM();

	/* 发送STOP信号 */
	stop_GSI2CM();
	return TRUE;
}

/**
 * 写若干个byte到总线上
 * @param Address 从设备地址
 * @param Buffer 存放写内容的指针
 * @param nBufLen 写数的个数
 * @return：
 * FALSE 操作过程中，从设备异常没应答 TODO this is not same with the code!!
 * TRUE 成功
 */
unsigned char write_GSI2CM(unsigned char Address, unsigned char* Buffer,
		unsigned short nBufLen) {
	/* 读写位置0，为写操作 */
	Address &= ~0x01;

	/* 发送START信号 */
	start_GSI2CM();

	/* 发送从设备地址，如果NoACK，表示从设备不存在 */
	if (writeByte_GSI2CM(Address) != TRUE) {
		stop_GSI2CM();
		return FALSE;
	}

	/* 写入前nBufLen-1字节 ，if error occurs while write any byte, return false immediately. */
	for (register unsigned char i = 0; i < nBufLen - 1; i++) {
		if (writeByte_GSI2CM(Buffer[i]) != TRUE) {
			stop_GSI2CM();
			return FALSE;
		}
	}
	/* 写入最后一个字节，不管SLAVE返回NACK或ACK均认为发送成功 */
	writeByte_GSI2CM(Buffer[nBufLen - 1]);
	/* 发送STOP信号 */
	stop_GSI2CM();
	return TRUE;
}
