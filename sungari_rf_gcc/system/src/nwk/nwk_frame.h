/**************************************************************************************************
 Filename:       nwk_frame.h

 Description:    This header file supports the SimpliciTI frame handling functions.

 **************************************************************************************************/

#include "mrfi.h"
#include "nwk_types.h"

#ifndef NWK_FRAME_H
#define NWK_FRAME_H

/* Frame field defines and masks. Mask name must be field name with '_MSK' appended
 * so the GET and PUT macros work correctly -- they use token pasting. Offset values
 * are with respect to the MRFI payload and not the entire frame.
 */
#define F_PORT_OS         0
#define F_PORT_OS_MSK     (0x3F)
#define F_ENCRYPT_OS      0
#define F_ENCRYPT_OS_MSK  (0x40)
#define F_FWD_FRAME       0
#define F_FWD_FRAME_MSK   (0x80)
#define F_RX_TYPE         1
#define F_RX_TYPE_MSK     (0x40)
#define F_ACK_REQ         1
#define F_ACK_REQ_MSK     (0x80)
#define F_ACK_RPLY        1
#define F_ACK_RPLY_MSK    (0x08)
#define F_TX_DEVICE       1
#define F_TX_DEVICE_MSK   (0x30)
#define F_HOP_COUNT       1
#define F_HOP_COUNT_MSK   (0x07)
#define F_TRACTID_OS      2
#define F_TRACTID_OS_MSK  (0xFF)
#define SMPL_NWK_HDR_SIZE 3

#define F_SECURE_OS       0

#define F_APP_PAYLOAD_OS  (SMPL_NWK_HDR_SIZE+F_SECURE_OS)

/* sub field details. they are in the correct bit locations (already shifted) */
#define F_RX_TYPE_USER_CTL       0x00    /* does not poll... */
#define F_RX_TYPE_POLLS          0x40    /* polls for held messages */

#define F_ACK_REQ_TYPE           0x80
#define F_ACK_RPLY_TYPE          0x08
#define F_FRAME_FWD_TYPE         0x80
#define F_FRAME_ENCRYPT_TYPE     0x40

/* device type fields */
#define F_TX_DEVICE_ED           0x00    /* End Device */
#define F_TX_DEVICE_RE           0x10    /* Range Extender */
#define F_TX_DEVICE_AP           0x20    /* Access Point */

/* macro to get a field from a frame buffer */
#define GET_FROM_FRAME(b,f)  ((b)[f] & (f##_MSK))

/* Macro to put a value 'v' into a frame buffer 'b'. 'v' value must already be shifted
 * if necessary. 'b' is a byte array
 */
#define PUT_INTO_FRAME(b,f,v)  do {(b)[f] = ((b)[f] & ~(f##_MSK)) | (v); } while(0)

/*       ****   frame information objects
 * info kept on each frame object
 */
//#define   FI_AVAILABLE         0   /* entry available for use */
//#define   FI_INUSE_UNTIL_DEL   1   /* in use. will be explicitly reclaimed */
//#define   FI_INUSE_UNTIL_TX    2   /* in use. will be reclaimed after Tx */
//#define   FI_INUSE_UNTIL_FWD   3   /* in use until forwarded by AP */
//#define   FI_INUSE_TRANSITION  4   /* being retrieved. do not delete in Rx ISR thread. */

typedef struct {
	unsigned char rssi;
	unsigned char lqi;
} sigInfo_t;

typedef struct {
	//volatile unsigned char      fi_usage;
	unsigned char fresh;
	mrfiPacket_t mrfiPkt;
} frameInfo_t;

/* prototypes */
frameInfo_t *nwk_buildFrame(unsigned char, unsigned char *msg,
		unsigned char len, unsigned char hops);
#ifdef APP_AUTO_ACK
frameInfo_t *nwk_buildAckReqFrame(unsigned char, unsigned char *, unsigned char, unsigned char, volatile unsigned char *);
#endif
void nwk_receiveFrame(void);
void nwk_frameInit(void);
smplStatus_t nwk_retrieveFrame(unsigned char *, unsigned char *, addr_t *);
smplStatus_t nwk_sendFrame(frameInfo_t *, unsigned char txOption);
frameInfo_t *nwk_getSandFFrame(mrfiPacket_t *, unsigned char);
unsigned char nwk_getMyRxType(void);
unsigned char nwk_setMyAddress(addr_t *addr);
addr_t const *nwk_getMyAddress(void);
void nwk_SendEmptyPollRspFrame(mrfiPacket_t *);
smplStatus_t nwk_getRadioSignInfo(ioctlRadioSiginfo_t * siginfo);
#ifdef APP_AUTO_ACK
void nwk_sendAckReply(mrfiPacket_t *, unsigned char);
#endif

#endif
