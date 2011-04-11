#ifndef __HYENA_UTILS_H__
#define __HYENA_UTILS_H__

#include "SMPL.h"
#include "HyenaWire.h"
/**
 * build 101: sent to Liuzhou.  all features work.
 * build 102: refining for better code quality.
 * build 103: Wire detector
 * build 104: refine wire detector and wire driver.
 * build 105: detector finished. delete useless driver.
 * build 106: fix VERSION LED bug.
 * build 107: update Detector to support detecting unfinished wire.
 * build 108: study at restart puzzle:
 * 1.Shut down CC1101 in detector.
 * 2.Modify Wire driver to give up electricity operation ONLY FOR TEST PROPOSE!
 *
 * build 109: give up electricity operation due to the HW DESIGN ERROR!
 *    SHT read data time by time and wait the I2C master get it.
 *
 * build 110: continue debugging
 *
 * build 111: special mechanism to avoid END RESTART.
 * build 112: enable WDT
 *
 * build 113: continue debugging... add RSSI detector.
 *
 * build 115: prepare for long time test. with WDT.
 *
 * build 116: in dev.
 * build 117: for debug
 * build 118: for debug. slightly refine mrfi code.
 *
 * build 119: in dev.
 * build 121, 122: debug TMP
 *
 * build 123: MSP430forSHT -- pretend as TMP to debug I2C.
 *
 * build 124: for test SHT directly with the clean driver. HyenaeWires support 2 test mode.
 * build 125: for test SHT humidity. HyenaeWires support 3 test mode.
 */
#define HYENA_GRANARY_CORE_VERSION 1.1.0.126

/**
 * This number should be shown by LED.
 * It should be same with the last number of version.
 */
#define HYENA_VERSION 5




#define INVALID_ADDRESS {0xFF, 0xFF}

#define ROOT_ADDRESS {0xAA, 0x00}

#define RELAY_LOW 0x02

/**
 * The length of command message
 */
#define COMMAND_MAX_LENGTH_IN_BYTES 14

#define WRITE_TRY_TIMES_IN_SEND 100

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
 * Time Parameters
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
 */

/**
 * 切换WOR状态后为了稳定等待的时间
 */
#define IDLE_WOR 10

/**
 * 每次send失败后，再send的等待.
 */
#define IDLE_PER_SEND 5

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
 * end of Time Parameters
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
 */



/* Node Type */
#define	RELAY 1
#define END 2

/* Length of node address */
#define ADDR_LENGTH 2

/* Program Status */
#define SUCCESS 1
#define FAILURE 0

/* Command */
#define COMMAND_GET_SENSORS_VALUE 0x11
#define COMMAND_GET_ELECTRIC_AMOUNT 0x12

/* Action */
#define ACTION_PASS 0x0F
#define ACTION_GET_SENSORS_VALUE 0x01
#define ACTION_GET_ELECTRIC_AMOUNT 0x02

/* Direction */
#define DIRECTION_TO_NONE 0
#define DIRECTION_TO_FATHER 1
#define DIRECTION_TO_ALL_SONS 2
#define DIRECTION_TO_SPCIAL_SONS 3
#define DIRECTION_TO_ALL_RELAY_SONS 4


/* command id's position in command */
#define COMMAND_ID_IN_COMMAND 0
/* message id's position in command */
#define POSITION_MESSAGE_ID_IN_COMMAND 1
/* node address length's position in command */
#define POSITION_ADDRESS_LENGTH_IN_COMMAND 2
/* address's position in command  */
#define POSITION_ADDRESS_HIGH_IN_COMMAND 3
#define POSITION_ADDRESS_LOW_IN_COMMAND 4

/* address length's position in message */
#define POSITION_ADRRESS_LENGTH_IN_MESSAGE 2
/* address's position in command */
#define POSITION_ADDRESS_HIGH_IN_COMMAND 3
#define POSITION_ADDRESS_LOW_IN_COMMAND 4
/* power value length's position in message */
#define POSITION_POWER_VALUE_LENGTH_IN_MESSAGE 5
/* power value's position in message */
#define POSITION_POWER_VALUE_IN_MESSAGE 6
/* the position of number of sensor in message */
#define POSITION_SENSOR_NUMBER_IN_MESSAGE 5
/* sensor value's position in message */
#define POSITION_SENSOR_VALUE_IN_MESSAGE 6

typedef struct {
	unsigned char actionId;
	unsigned char direction;
	addr_t targetAddr;
} action_t;
#define RSSI_RECEIVER_ADDRESS {0xAB,0x00}
#define RSSI_SENDER_ADDRESS {0xAC,0x00}
#define RSSI_LENGTH_POSITION_IN_COMMAND 5
#define RSSI_POSITION_IN_COMMAND 6

#define IDLE_PER_LOOP 500
///**
// * store address of receive node.
// */
//static addr_t addrOfSenderR = INVALID_ADDRESS;
//static addr_t addrOfSenderS = RSSI_RECEIVER_ADDRESS;
///**
// * addresses of this nodes.
// */
//static addr_t addrOfNode = RSSI_SENDER_ADDRESS;
unsigned char send(addr_t *, unsigned char *);
unsigned char receive(addr_t *, unsigned char *);
void turnOnWOR(void);
void assignAddress(unsigned char *, addr_t *);


/**
 * 出错提示 using LED
 */
void errorLog(void);

/**
 * 警告提示 using LED
 */
void warningLog(void);

void writeToUart(unsigned char *, unsigned char);
unsigned char getFromUart(unsigned char *);
#endif

