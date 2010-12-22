/**************************************************************************************************
 Filename:       nwk_frame.c

 Description:    This file supports the SimpliciTI frame handling functions.

 **************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include <string.h>
#include "nwk_frame.h"

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
static frameInfo_t sInFrameQ;

static frameInfo_t sOutFrameQ;

static unsigned char sTRACTID = 0;

static addr_t const *sMyAddr = NULL;

static unsigned char sMyRxType = 0, sMyTxType = 0;

static const addr_t sMyROMAddress = {THIS_DEVICE_ADDRESS};

static addr_t sMyRAMAddress;

static unsigned char sRAMAddressIsSet = 0;

/**
 * for rssi
 * add by xt.
 * 2010-11-04
 */
static ioctlRadioSiginfo_t radioSininfo;

#ifdef APP_AUTO_ACK
unsigned char ackReceived;
unsigned char ackTID;
#endif

#ifdef NWK_DEBUG
unsigned int ackAccumulation;
#endif
/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_frameInit
 *
 * @brief       Initialize network context.
 *
 * input parameters
 *       pF - Pointer to callback function. If none intended should be NULL.
 *
 * output parameters
 *
 * @return    void
 */

void nwk_frameInit(void) {
	/****** Fill static values for the DEVICEINFO byte that will go in each frame ******/
	/* Rx type when frame originates from this device. Set in nwk_buildFrame() */
	/* Tx type when frame sent from this device. Set in nwk_sendframe() */
	sMyTxType = F_TX_DEVICE_ED;
	sMyRxType = F_RX_TYPE_USER_CTL;
	/****** DONE fill static values for the DEVICEINFO byte that will go in each frame ******/

	/**
	 * for rssi
	 * add by xt.
	 * 2010-11-04
	 */
	radioSininfo.sigInfo.rssi =0x00;
	radioSininfo.sigInfo.lqi =0x00;
	//spCallback = pF;

	sMyAddr = nwk_getMyAddress();

	while (!(sTRACTID = MRFI_RandomByte()))
		;

	return;
}

/******************************************************************************
 * @fn          nwk_buildFrame
 *
 * @brief       Builds an output frame for the port and message enclosed.
 *              This routine prepends the frame header and populates the
 *              frame in the output queue.
 *
 * input parameters
 * @param   port    - port from application
 * @param   msg     - pointer to message from app to be sent
 * @param   len     - length of enclosed message
 * @param   hops    - number of hops allowed. this is less than MAX_HOPS
 *                    whenever the frame is being sent to the AP. this is to
 *                    help mitigate the (short) broadcast storms
 *
 * output parameters
 *
 * @return   pointer to frameInfo_t structure created. NULL if there is
 *           no room in output queue.
 */
frameInfo_t *nwk_buildFrame(unsigned char port, unsigned char *msg,
		unsigned char len, unsigned char hops) {
	frameInfo_t *fInfoPtr;

	fInfoPtr = &sOutFrameQ;

	MRFI_SET_PAYLOAD_LEN(&fInfoPtr->mrfiPkt, len+F_APP_PAYLOAD_OS);

	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_ENCRYPT_OS, 0);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_PORT_OS, port);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_TRACTID_OS, sTRACTID);
	while (!(++sTRACTID))
		; /* transaction ID can't be 0 */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_RX_TYPE, sMyRxType);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_HOP_COUNT, hops);

	/* reset ack-relevant bits */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_ACK_REQ, 0);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_ACK_RPLY, 0);

	/* reset forwarding bit */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_FWD_FRAME, 0);

	memcpy(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt) + F_APP_PAYLOAD_OS, msg, len);
	memcpy(MRFI_P_SRC_ADDR(&fInfoPtr->mrfiPkt), sMyAddr, NET_ADDR_SIZE);

	return fInfoPtr;
}

#ifdef APP_AUTO_ACK
/******************************************************************************
 * @fn          nwk_buildAckReqFrame
 *
 * @brief       Builds an output frame for the port and message enclosed.
 *              This routine prepends the frame header and populates the
 *              frame in the output queue. The frame is set to request that
 *              an ack frame be sent by the peer.
 *
 * input parameters
 * @param   port    - port from application
 * @param   msg     - pointer to message from app to be sent
 * @param   len     - length of enclosed message
 * @param   hops    - number of hops allowed. this is less than MAX_HOPS
 *                    whenever the frame is being sent to the AP. this is to
 *                    help mitigate the (short) broadcast storms
 * @param   tid     - Transaction ID to insert in NWK header used to match
 *                    the ack reply.
 *
 * output parameters
 *
 * @return   pointer to frameInfo_t structure created. NULL if there is
 *           no room in output queue.
 */
frameInfo_t *nwk_buildAckReqFrame(unsigned char port, unsigned char *msg, unsigned char len, unsigned char hops, volatile unsigned char *tid)
{
	frameInfo_t *fInfoPtr;

	/* Build a normal frame first. */
	if (!(fInfoPtr=nwk_buildFrame(port, msg, len, hops)))
	{
		return (frameInfo_t *)NULL;
	}

	/* save TID  */
	*tid = GET_FROM_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_TRACTID_OS);
	/* Set REQ_ACK bit */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_ACK_REQ, F_ACK_REQ_TYPE);

	return fInfoPtr;
}
#endif  /* APP_AUTO_ACK */

/******************************************************************************
 * @fn          MRFI_RxCompleteISR
 *
 * @brief       Here on Rx interrupt from radio. Process received frame from the
 *              radio Rx FIFO.
 *
 * input parameters
 *
 * output parameters
 *
 * @return      void
 */
void MRFI_RxCompleteISR() {

	frameInfo_t *fInfoPtr = &sInFrameQ;
#ifdef APP_AUTO_ACK
	mrfiPacket_t mrfiPkt;
	/* Receive ACK */
	if (ackReceived == 0xFF)
	{
		MRFI_Receive(&mrfiPkt);
		if (!memcmp(MRFI_P_SRC_ADDR(&mrfiPkt), sMyAddr, NET_ADDR_SIZE)) return;
		if (GET_FROM_FRAME(MRFI_P_PAYLOAD(&mrfiPkt), F_ACK_RPLY))
		{
			//Set by LISHI
			ackTID= GET_FROM_FRAME(MRFI_P_PAYLOAD(&mrfiPkt), F_TRACTID_OS);
			MRFI_PostKillSem();
		}
#ifdef NWK_DEBUG
		ackAccumulation++;
#endif
	}
	else /* Receive Packet */
	{

#endif
	//如果接收缓存中，Frame是空的
	if (fInfoPtr->fresh == 0x00) {
		MRFI_Receive(&fInfoPtr->mrfiPkt);
		fInfoPtr->fresh = 0xFF;
		/* be sure it's not an echo... */
		if (!memcmp(MRFI_P_SRC_ADDR(&fInfoPtr->mrfiPkt), sMyAddr, NET_ADDR_SIZE))
			return;

#ifdef APP_AUTO_ACK
		if (GET_FROM_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_ACK_REQ))
		{

			nwk_sendAckReply(&fInfoPtr->mrfiPkt, SMPL_PORT_USER_BCAST);
		}
#endif
		MRFI_PostKillSem();
	}
#ifdef APP_AUTO_ACK
}
#endif

}

/******************************************************************************
 * @fn          nwk_retrieveFrame
 *
 * @brief       Retrieve frame from Rx frame queue. Invoked by application-level
 *              code either app through SMPL_Receive() or IOCTL through raw Rx. This
 *              should run in a user thread, not an ISR thread.
 *
 * input parameters
 * @param    port    - port on which to get a frame
 *
 * output parameters
 * @param    msg     - pointer to where app payload should be copied. Buffer
 *                     allocated should be == MAX_APP_PAYLOAD.
 *
 * @param    len      - pointer to where payload length should be stored. Caller
 *                      can check for non-zero when polling the port. initialized
 *                      to 0 even if no frame is retrieved.
 * @param    srcAddr  - if non-NULL, a pointer to where to copy the source address
 *                      of the retrieved message.
 * @param    hopCount - if non-NULL, a pointer to where to copy the hop count
 of the retrieved message.
 *
 * @return    SMPL_SUCCESS
 *            SMPL_NO_FRAME  - no frame found for specified destination
 *            SMPL_BAD_PARAM - no valid connection info for the Link ID
 *
 */
smplStatus_t nwk_retrieveFrame(unsigned char *msg, unsigned char *len,
		addr_t *srcAddr) {
	frameInfo_t *fPtr;

	fPtr = &sInFrameQ;
	if (fPtr->fresh == 0xFF) {

		/* it's on the requested port. */
		*len = MRFI_GET_PAYLOAD_LEN(&fPtr->mrfiPkt) - F_APP_PAYLOAD_OS;
		memcpy(msg, MRFI_P_PAYLOAD(&fPtr->mrfiPkt) + F_APP_PAYLOAD_OS, *len);
		/* copy source address if requested */
		memcpy(srcAddr, MRFI_P_SRC_ADDR(&fPtr->mrfiPkt), NET_ADDR_SIZE);

		/**
		 * for rssi
		 * add by xt.
		 * 2010-11-04
		 */
		/* Save Rx metrics... */
		radioSininfo.sigInfo.rssi = fPtr->mrfiPkt.rxMetrics[MRFI_RX_METRICS_RSSI_OFS];
		radioSininfo.sigInfo.lqi = fPtr->mrfiPkt.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS];

		fPtr->fresh = 0x00;
		return SMPL_SUCCESS;
	}

	return SMPL_NO_FRAME;
}

/**
 * get Rx metrics:
 * rssi & lqi
 * add by xt.
 * 2010-11-04
 */
smplStatus_t nwk_getRadioSignInfo(ioctlRadioSiginfo_t * signal) {
	signal->sigInfo.rssi = radioSininfo.sigInfo.rssi;
	signal->sigInfo.lqi = radioSininfo.sigInfo.lqi;
	return SMPL_SUCCESS;
}
/******************************************************************************
 * @fn          nwk_sendFrame
 *
 * @brief       Send a frame by copying it to the radio Tx FIFO.
 *
 * input parameters
 * @param   pFrameInfo   - pointer to frame to be sent
 * @param   txOption     - do CCA or force frame out.
 *
 * output parameters
 *
 * @return    SMPL_SUCCESS
 *            SMPL_TX_CCA_FAIL Tx failed because of CCA failure.
 *                             Tx FIFO flushed in this case.
 */
smplStatus_t nwk_sendFrame(frameInfo_t *pFrameInfo, unsigned char txOption) {
	smplStatus_t rc;

	/* set the type of device sending the frame in the header */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt), F_TX_DEVICE, sMyTxType);

	if (MRFI_Transmit(&pFrameInfo->mrfiPkt, txOption) == MRFI_TX_RESULT_SUCCESS) {
		rc = SMPL_SUCCESS;
	} else {
		/* Tx failed -- probably CCA. free up frame buffer. We do not have NWK
		 * level retries. Let application do it.
		 */
		rc = SMPL_TX_CCA_FAIL;
	}

	/* TX is done. free up the frame buffer */
	//pFrameInfo->fi_usage = FI_AVAILABLE;

	return rc;
}

/******************************************************************************
 * @fn          nwk_getMyRxType
 *
 * @brief       Get my Rx type. Used to help populate the hops count in the
 *              frame header to try and limit the broadcast storm. Info is
 *              exchanged when linking.
 *
 * input parameters
 *
 * output parameters
 *
 * @return      The address LSB.
 */
unsigned char nwk_getMyRxType(void) {
	return sMyRxType;
}

/******************************************************************************
 * @fn          nwk_getMyAddress
 *
 * @brief       Return a pointer to my address. It comes either from ROM as
 *              set in the configuration file or it was set using the IOCTL
 *              interface -- probably from random bytes.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   pointer to a constant address type object.
 */
addr_t const *nwk_getMyAddress(void) {
	/* This call supports returning a valid pointer before either the
	 * initialization or external setting of the address. But caller needs
	 * to be careful -- if this routine is called immediately it will return
	 * the ROM address. If the application then sets the address using the
	 * IOCTL before doing the SMPL_Init() the original pointer is no longer
	 * valid as it points to the wrong address.
	 */
	return sRAMAddressIsSet ? &sMyRAMAddress : &sMyROMAddress;
}

/******************************************************************************
 * @fn          nwk_setMyAddress
 *
 * @brief       Set my address object if it hasn't already been set. This call
 *              is referenced by the IOCTL support used to change the device
 *              address. It is effective only if the address has not already
 *              been set.
 *
 * input parameters
 *
 * @param   addr  - pointer to the address object to be used to set my address.
 *
 * output parameters
 *
 * @return   Returns non-zero if request is respected, otherwise returns 0.
 */
unsigned char nwk_setMyAddress(addr_t *addr) {
	unsigned char rc = 0;

	if (!sRAMAddressIsSet) {
		memcpy(&sMyRAMAddress, addr, sizeof(addr_t));
		sRAMAddressIsSet = 1; /* RAM address is now valid */
		rc = 1;
	}

	return rc;
}

#ifdef APP_AUTO_ACK
/******************************************************************************
 * @fn          nwk_sendAckReply
 *
 * @brief       Send an acknowledgement reply frame.
 *
 * input parameters
 * @param   frame   - pointer to frame with ack request.
 * @param   port    - port on whcih reply expected.
 *
 * output parameters
 *
 * @return      void
 */
void nwk_sendAckReply(mrfiPacket_t *frame, unsigned char port)
{
	mrfiPacket_t dFrame;
	unsigned char tid = GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_TRACTID_OS);

	/* set the type of device sending the frame in the header */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_TX_DEVICE, sMyTxType);

	/* set the listen type of device sending the frame in the header. */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_RX_TYPE, sMyRxType);

	/* destination address from received frame */
	memcpy(MRFI_P_DST_ADDR(&dFrame), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

	/* source address */
	memcpy(MRFI_P_SRC_ADDR(&dFrame), sMyAddr, NET_ADDR_SIZE);

	/* port is the source the Tx port from the connection object */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_PORT_OS, port);

	/* frame length... */
	MRFI_SET_PAYLOAD_LEN(&dFrame,F_APP_PAYLOAD_OS);

	/* transaction ID taken from source frame */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_TRACTID_OS, tid);

	/* hop count... */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_HOP_COUNT, 1);

	/* set ACK field */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_ACK_RPLY, F_ACK_RPLY_TYPE);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_ACK_REQ, 0);

	/* This is not a forwarded frame */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_FWD_FRAME, 0);

	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_ENCRYPT_OS, 0);

	MRFI_Transmit(&dFrame, MRFI_TX_TYPE_FORCED);

	return;
}
#endif /* APP_AUTO_ACK */

