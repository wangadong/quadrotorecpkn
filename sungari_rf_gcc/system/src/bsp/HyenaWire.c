/*
 * The driver of the "Module" of detect T/H.
 * includes controlling TMP sensor and SHT PCB, which provided by each driver.
 * includes how to control the multiple T/H on a cable.
 * <p>
 * The Rule of address on a cable contains here.
 *
 * @author Xu Tao, Boreal Mao, Taurus Ning
 */

#include "HyenaWire.h"

#define TMP_ONLY_MODE 0
#define SHT_DIRECTLY_ONLY_TEMPERATURE_MODE 0
#define SHT_DIRECTLY_ONLY_HUMIDITY_MODE 0
#define TBO_TMP_MODE 1

/* wait 400ms for tmp shutdown */
#define WAIT_FOR_SHUTDOWN_TMP 400

/* wait 320ms for tmp convertion */
#define WAIT_FOR_CONVERTION_TMP 320

/* wait 500ms for tmp convertion */
#define WAIT_FOR_CONTINUOUS_CONVERSION_TMP 500

#define CONTINUOUS_COUNT_REQUIRED 2
#define COUNT_LIMIT 4

#define TMP_ADDRESS_MASK 0x90

#define MAX_SENSOR_COUNT 8
#define WIRE_DEFAULT_VALUE_IN_BYTE 0xFF

static unsigned char readInContinuousMode(unsigned char sensorAddr,
		unsigned char *buf);

//FIXME XT 下面这段注释与代码不一致，暂时先保留。
/**
 * 轮询所有传感器包括TMP275,SHT10
 *
 * TMP275地址范围(8个): 0x90 0x92 0x94 0x96 0x98 0x9A 0x9C 0x9E
 * SHT10地址范围(8个): 0x60 0x62 0x64 0x66 0x68 0x6A 0x6C 0x6E
 * buf长度最大值为26
 *
 * 结果格式：
 * 第一个byte: 传感器个数
 * 后面依次是各个传感器的值
 *   TMP: 地址(1byte),值(2byte)　共3bytes
 *   SHT: 地址(1byte),值(4byte)　共5bytes
 *
 * Key Issue:
 * 当线缆上有地址冲突时
 * 本方法会视作只有其中一片存在。但取的是哪片未知。
 *
 * 如 --90---90---92---
 * 与 --90---92---
 * 返回结果一样
 *
 * @param unsigned char * buf存放传感器值的指针
 */
void getSensorValues(unsigned char * buf) {
	unsigned char result = 0;

	/*一旦调用，一定会写25位作为结果。初始值为FF*/
	memset(buf, WIRE_DEFAULT_VALUE_IN_BYTE, BUFFER_LENGTH);

	init_GSI2CM();
	FEED_WDT;

#if (TMP_ONLY_MODE == 1)
	/* tmpExist数组显示TMP传感器是否存在*/
	unsigned char isExistTMP[MAX_SENSOR_COUNT];
	//FIXME TN 这里存在含义混淆

	/* loop 0-7: start continuous detection */
	for (unsigned char i = 0; i < 8; i++) { //FIXME TN 这里写死8，谁能保证数组够大?
		isExistTMP[i] = executeContinuousDetection_TMP275(i);
	}
	FEED_WDT;

	/* wait for 500ms */
	delayInMs_BSP(WAIT_FOR_CONTINUOUS_CONVERSION_TMP);

	FEED_WDT;

	/* loop 0-7: read data */
	unsigned char sensorNum = 0;

	/* bit 0 for the number of sensors. start write from bit1 */
	unsigned char bufCount = 1;
	for (unsigned char i = 0; i < 8; i++) {
		if (!isExistTMP[i]) {
			continue;
		}
		/* parse TMP address(0-7) to 0x9* */
		buf[bufCount] = TMP_ADDRESS_MASK + i * 2;

		result = readInContinuousMode(i, &buf[bufCount + 1]);
		FEED_WDT;
		if (!result) {
			buf[bufCount + 1] = 0xFF;
			buf[bufCount + 2] = 0xFF;
		}
		bufCount += MESSAGE_LENGTH_PER_TMP275;
		sensorNum++;
	}

	buf[BUFFER_SENSOR_NUMBER_POSITION] = sensorNum;
	return; //under TMP_ONLY mode, no need to do further querying
#endif

#if(SHT_DIRECTLY_ONLY_TEMPERATURE_MODE == 1)
	unsigned char checksum;
	FEED_WDT;
	buf[1] = 0x60;
	result = detectTemperature_SHT(&buf[2],&checksum);

	//ToCheck how to use checksum.
	FEED_WDT;
	if (!result) {
		buf[2] = 0xFF;
		buf[3] = 0xFF;
	}
	buf[BUFFER_SENSOR_NUMBER_POSITION] = 1;
	return;
#endif

#if(SHT_DIRECTLY_ONLY_HUMIDITY_MODE == 1)
	unsigned char checksum;
	unsigned short power;
	FEED_WDT;
	buf[4] = 0x60;
	power = getBatteryMeter_BSP();
	FEED_WDT;
	buf[1] = 0xEE; // 标志电量，先写死
	buf[2] = power >> 8;
	buf[3] = power;

	/* 温度 */
	FEED_WDT;
	result = detectTemperature_SHT(&buf[5], &checksum);

	//ToCheck how to use checksum.
	FEED_WDT;
	if (result) {
		FEED_WDT;
		result = checkTemperature_SHT(&buf[5], &checksum);
		if (!result) {
			buf[9] = 0xEE;
		}
	} else {
		buf[5] = 0xFF;
		buf[6] = 0xFF;
	}

	/* 湿度 */
	FEED_WDT;
	result = detectHumidity_SHT(&buf[7], &checksum);

	//ToCheck how to use checksum.
	FEED_WDT;
	if (result) {
		FEED_WDT;
		result = checkHumidity_SHT(&buf[7], &checksum);
		if (!result) {
			buf[10] = 0xEE;
		}
	} else {
		buf[7] = 0xFF;
		buf[8] = 0xFF;
	}

	buf[BUFFER_SENSOR_NUMBER_POSITION] = 1;
	return;
#endif

#if(TBO_TMP_MODE == 1)
	unsigned char isExistTMP[MAX_SENSOR_COUNT];
	unsigned char TMPAddressNumber[3];

	/* 固定TMP地址 分别是90,94,96 */
	TMPAddressNumber[0] = 0;
	TMPAddressNumber[1] = 2;
	TMPAddressNumber[2] = 3;

	/* loop 0-7: start continuous detection */
	for (unsigned char i = 0; i < 3; i++) {
		isExistTMP[i] = executeContinuousDetection_TMP275(TMPAddressNumber[i]);
	}
	FEED_WDT;

	/* wait for 500ms */
	delayInMs_BSP(WAIT_FOR_CONTINUOUS_CONVERSION_TMP);

	FEED_WDT;

	/* loop 0-7: read data */
	unsigned char sensorNum = 0;

	/* bit 0 for the number of sensors. start write from bit1 */
	unsigned char bufCount = 1;
	for (unsigned char i = 0; i < 3; i++) {
		if (!isExistTMP[i]) {
			continue;
		}
		/* parse TMP address(0-7) to 0x9* */
		buf[bufCount] = TMP_ADDRESS_MASK + TMPAddressNumber[i] * 2;
		FEED_WDT;
		result = readInContinuousMode(TMPAddressNumber[i], &buf[bufCount + 1]);
		FEED_WDT;
		if (!result) {
			buf[bufCount + 1] = 0xFF;
			buf[bufCount + 2] = 0xFF;
		}
		bufCount += MESSAGE_LENGTH_PER_TMP275;
		sensorNum++;
	}

	buf[BUFFER_SENSOR_NUMBER_POSITION] = sensorNum;
	return;
#endif

}

/**
 * 在Continuous Mode下进行读取TMP数值，并且加入了滤波操作。
 * @param unsigned char sensorAddr TMP传感器地址：0-7
 * 		 unsigned char *buf 存放传感器值的指针
 * @return unsigned char result:TRUE 读取TMP温度数值成功；FALSE 读取TMP温度数值失败。
 */
unsigned char readInContinuousMode(unsigned char sensorAddr, unsigned char *buf) {
	unsigned int firstTemperature; //FIXME TN 都是temperature，一会儿int一会儿char? 命名也让人困惑
	unsigned int secondTemperature;
	unsigned char temperature[2];
	unsigned char result;
	unsigned char count = 0;
	unsigned char totalCount = 0;
	result = FALSE;
	count = 0;

	while (count < CONTINUOUS_COUNT_REQUIRED) {
		if (count == 0) {
			FEED_WDT;
			/* 获取第一次测量数据 */
			result = readTemperature_TMP(sensorAddr, temperature);

			if (!result) {
				return FALSE;
			}

			buf[0] = temperature[0];
			buf[1] = temperature[1];
			if (temperature[0] & 0x80) {
				firstTemperature = ((temperature[0] & 0x7F) << 8)
						+ temperature[1];
			} else {
				firstTemperature = (temperature[0] << 8) + temperature[1];
			}
		}
		/* wait for 320ms */
		delayInMs_BSP(WAIT_FOR_CONVERTION_TMP);
		FEED_WDT;
		/* 获取第二次测量数据 */
		result = readTemperature_TMP(sensorAddr, temperature);

		if (!result) {
			return FALSE;
		}

		if (temperature[0] & 0x80) {
			secondTemperature = ((temperature[0] & 0x7F) << 8) + temperature[1];
		} else {
			secondTemperature = (temperature[0] << 8) + temperature[1];
		}

		count++;
		totalCount++;

		if (secondTemperature >= firstTemperature) {
			if (secondTemperature - firstTemperature > 0x02) {
				count = 0;
			}
		} else {
			if (firstTemperature - secondTemperature > 0x02) {
				count = 0;
			}
		}

		if (totalCount == COUNT_LIMIT) {
			return FALSE;
		}

		FEED_WDT;
	}
	return TRUE;
}

/**
 * 因为电路板的设计问题，sensor板上电会出现END重启的问题
 * 此方法是用软件的方式解决这个问题。
 * 这个方法用于6月END板和新SENSOR板(10月份之前)
 *
 * 采取策略:
 * 把16个传感器的地址轮询一遍
 */
void readSensorsToCosumePowerLeft() {
	unsigned char sensorAddr;
	unsigned char value[5];
	init_GSI2CM();
	/*轮询TMP275传感器读取数据*/
	sensorAddr = FIRST_ADDRESS_OF_TMP275;
	for (register unsigned char i = 0; i < MAX_SENSOR_COUNT; ++i) {
		readTemperature_TMP(sensorAddr, value);
		/*下个传感器的地址·*/
		sensorAddr += SENSOR_ADDRESS_INTERVAL_LENGTH;
	}
	/*轮询SHT10传感器*/
	sensorAddr = FIRST_ADDRESS_OF_SHT10;
	for (register unsigned char i = 0; i < MAX_SENSOR_COUNT; ++i) {
		readTemperature_TMP(sensorAddr, value);
		/*下一个传感器地址*/
		sensorAddr += SENSOR_ADDRESS_INTERVAL_LENGTH;
	}
}
