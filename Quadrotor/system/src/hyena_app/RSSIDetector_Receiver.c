/*
 * RSSI测试recevier文件
 * 用于RSSI测试的接收端
 * @author Xu Tao
 */
#include "HyenaUtils.h"

#define RSSI_RECEIVER_ADDRESS {0xAB,0x00}
#define RSSI_SENDER_ADDRESS {0xAC,0x00}
#define RSSI_LENGTH_POSITION_IN_COMMAND 5
#define RSSI_POSITION_IN_COMMAND 6

#define IDLE_PER_LOOP 500
/**
 * store address of receive node.
 */
static addr_t addrOfSenderR = INVALID_ADDRESS;
static addr_t addrOfSenderS = RSSI_RECEIVER_ADDRESS;
/**
 * addresses of this nodes.
 */
static addr_t addrOfNode = RSSI_SENDER_ADDRESS;

void main(void) {
	unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES] = { 0 };
	unsigned char writeFlag = 0;
	ioctlRadioSiginfo_t radioSigInfo;
	rssi_t rssi;

	/* initial the addr */
	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &addrOfNode);

	/* Board Init */
	hyenaInit();
	showVersion(HYENA_VERSION);

	/* RX on */
	SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	turnOffLeds();
	while (1) {
		turnOnLeds();
		/* clear msg */
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);

		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

		/* clear msg */
		if (receive(&addrOfSenderR, msg) == SUCCESS) {
			/* RX on */
			SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
			/* get RSSI value */
			//if (writeFlag) {
				/* write to UART */
			if (send(&addrOfSenderS, msg) != SUCCESS) {
							warningLog();
			}			//}
		}
		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	}
}
