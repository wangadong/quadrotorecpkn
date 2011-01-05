/*
 * RSSI测试sender文件
 * 用于RSSI测试的发送端
 * @author Xu Tao
 */
#include "HyenaUtils.h"

#define RSSI_RECEIVER_ADDRESS {0xAB,0x00}
#define RSSI_SENDER_ADDRESS {0xAC,0x00}

#define IDLE_PER_LOOP 500
#define RSSI_VALUE_POSITION_IN_COMMAND 5
/**
 * addresses of this nodes.
 */
static addr_t addrOfNode = { RSSI_SENDER_ADDRESS };

/**
 * addresses of receiver nodes.
 */
static addr_t addrOfReceiver = { RSSI_RECEIVER_ADDRESS };

unsigned char messageID = 0;

void main(void) {
	/* the buffer for sending or receiving message. */
	unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES] = { 0 };
	unsigned char receiveBuffer[UART_BUFFER_NUM];
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

		/* clear receiveBuffer */
		memset(receiveBuffer, 0xFF, UART_BUFFER_NUM);

		FEED_WDT;
		/* RX on */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
		/* get RSSI value */
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, &rssi);

		messageID++;
		msg[COMMAND_ID_IN_COMMAND] = 0x13;
		msg[POSITION_MESSAGE_ID_IN_COMMAND] = messageID;
		msg[POSITION_ADDRESS_LENGTH_IN_COMMAND] = 0x02;
		memcpy(&msg[POSITION_ADDRESS_HIGH_IN_COMMAND], &addrOfNode, ADDR_LENGTH);
		msg[RSSI_VALUE_POSITION_IN_COMMAND] = rssi;
		if (send(&addrOfReceiver, msg) != SUCCESS) {
			warningLog();
		}
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
		FEED_WDT;
		turnOffLeds();

	}
}
