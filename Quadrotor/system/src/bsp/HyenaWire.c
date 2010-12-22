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

//FIXME XT �������ע������벻һ�£���ʱ�ȱ�����
/**
 * ��ѯ���д���������TMP275,SHT10
 *
 * TMP275��ַ��Χ(8��): 0x90 0x92 0x94 0x96 0x98 0x9A 0x9C 0x9E
 * SHT10��ַ��Χ(8��): 0x60 0x62 0x64 0x66 0x68 0x6A 0x6C 0x6E
 * buf�������ֵΪ26
 *
 * �����ʽ��
 * ��һ��byte: ����������
 * ���������Ǹ�����������ֵ
 *   TMP: ��ַ(1byte),ֵ(2byte)����3bytes
 *   SHT: ��ַ(1byte),ֵ(4byte)����5bytes
 *
 * Key Issue:
 * ���������е�ַ��ͻʱ
 * ������������ֻ������һƬ���ڡ���ȡ������Ƭδ֪��
 *
 * �� --90---90---92---
 * �� --90---92---
 * ���ؽ��һ��
 *
 * @param unsigned char * buf��Ŵ�����ֵ��ָ��
 */
void getSensorValues(unsigned char * buf) {
	unsigned char result = 0;

	/*һ�����ã�һ����д25λ��Ϊ�������ʼֵΪFF*/
	memset(buf, WIRE_DEFAULT_VALUE_IN_BYTE, BUFFER_LENGTH);

	init_GSI2CM();
	FEED_WDT;

#if (TMP_ONLY_MODE == 1)
	/* tmpExist������ʾTMP�������Ƿ����*/
	unsigned char isExistTMP[MAX_SENSOR_COUNT];
	//FIXME TN ������ں������

	/* loop 0-7: start continuous detection */
	for (unsigned char i = 0; i < 8; i++) { //FIXME TN ����д��8��˭�ܱ�֤���鹻��?
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
	buf[1] = 0xEE; // ��־��������д��
	buf[2] = power >> 8;
	buf[3] = power;

	/* �¶� */
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

	/* ʪ�� */
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

	/* �̶�TMP��ַ �ֱ���90,94,96 */
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
 * ��Continuous Mode�½��ж�ȡTMP��ֵ�����Ҽ������˲�������
 * @param unsigned char sensorAddr TMP��������ַ��0-7
 * 		 unsigned char *buf ��Ŵ�����ֵ��ָ��
 * @return unsigned char result:TRUE ��ȡTMP�¶���ֵ�ɹ���FALSE ��ȡTMP�¶���ֵʧ�ܡ�
 */
unsigned char readInContinuousMode(unsigned char sensorAddr, unsigned char *buf) {
	unsigned int firstTemperature; //FIXME TN ����temperature��һ���intһ���char? ����Ҳ��������
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
			/* ��ȡ��һ�β������� */
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
		/* ��ȡ�ڶ��β������� */
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
 * ��Ϊ��·���������⣬sensor���ϵ�����END����������
 * �˷�����������ķ�ʽ���������⡣
 * �����������6��END�����SENSOR��(10�·�֮ǰ)
 *
 * ��ȡ����:
 * ��16���������ĵ�ַ��ѯһ��
 */
void readSensorsToCosumePowerLeft() {
	unsigned char sensorAddr;
	unsigned char value[5];
	init_GSI2CM();
	/*��ѯTMP275��������ȡ����*/
	sensorAddr = FIRST_ADDRESS_OF_TMP275;
	for (register unsigned char i = 0; i < MAX_SENSOR_COUNT; ++i) {
		readTemperature_TMP(sensorAddr, value);
		/*�¸��������ĵ�ַ��*/
		sensorAddr += SENSOR_ADDRESS_INTERVAL_LENGTH;
	}
	/*��ѯSHT10������*/
	sensorAddr = FIRST_ADDRESS_OF_SHT10;
	for (register unsigned char i = 0; i < MAX_SENSOR_COUNT; ++i) {
		readTemperature_TMP(sensorAddr, value);
		/*��һ����������ַ*/
		sensorAddr += SENSOR_ADDRESS_INTERVAL_LENGTH;
	}
}
