/*
 * RSSI测试recevier文件
 * 用于RSSI测试的接收端
 * @author Xu Tao
 */
#include "HyenaUtils.h"

#define RSSI_RECEIVER_ADDRESS {0xAB,0x00}

#define RSSI_LENGTH_POSITION_IN_COMMAND 5
#define RSSI_POSITION_IN_COMMAND 6

#define IDLE_PER_LOOP 500
/**
 * store address of receive node.
 */
static addr_t addrOfSender = INVALID_ADDRESS;

/**
 * addresses of this nodes.
 */
static addr_t addrOfNode = RSSI_RECEIVER_ADDRESS;

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
		FEED_WDT;
		/* clear msg */
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);

		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

		if (getFromUart_BSP(msg) == GETUART_SUCCESS) {
			writeFlag = 1;
		}

		/* clear msg */
		memset(msg, 0xFF, COMMAND_MAX_LENGTH_IN_BYTES);
		FEED_WDT;
		if (receive(&addrOfSender, msg) == SUCCESS) {
			FEED_WDT;
			SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SIGINFO, &radioSigInfo);
			/* RX on */
			SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
			FEED_WDT;
			/* get RSSI value */
			SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, &rssi);
			FEED_WDT;
			msg[RSSI_LENGTH_POSITION_IN_COMMAND] = 0x02;
			msg[RSSI_POSITION_IN_COMMAND] = radioSigInfo.sigInfo.rssi;
			msg[RSSI_POSITION_IN_COMMAND + 1] = rssi;
			//if (writeFlag) {
				/* write to UART */
				writeToUart(msg, COMMAND_MAX_LENGTH_IN_BYTES);
				writeFlag = 0;
			//}
			FEED_WDT;
		}
		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
	}
}
