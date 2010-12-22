/*
 * The driver of SHT PCB for END.
 * includes reading data and controlling the SHT pcb.
 *
 * The SHT PCB has a F2012 as MCU and a SHT sensor controlled by F2012.
 * END must communicate to the SHT PCB by I2C.
 * we design some command based on I2C communication to implement the feature.
 *
 * ps. the driver of SHT Sensor which will run on F2012 is SHT10SensorDriver
 *
 * @author Hu Tianyang, Taurus Ning, Boreal Mao
 */

#include "SHT10.h"

/**
 * ��ȡSHT10���¶�ֵ��ʪ��ֵ
 *
 * @param address �������ĵ�ַ
 * @param bufferPointer �洢�Ӵ�������ַ�Ͷ���������ʪ����ֵ
 * @return TRUE read successfully. FALSE read failed.
 */
unsigned char readValueFromSHT10(unsigned char address, unsigned char * bufferPointer) {
	if (!read_GSI2CM(address, bufferPointer, 4)) {
		return FALSE;
	}
	return TRUE;

}

