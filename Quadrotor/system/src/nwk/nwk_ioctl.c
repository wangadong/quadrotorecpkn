/**************************************************************************************************
 Filename:       nwk_ioctl.c

 Description:    This file supports the SimpliciTI IOCTL implmentation. This interface
 gives applications access to the "driver" network level functions
 when necessary.

 **************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include "nwk_ioctl.h"

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

/******************************************************************************
 * LOCAL FUNCTIONS
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_rawSend
 *
 * @brief       Builds an outut frame based on information provided by the
 *              caller. This function allows a raw transmission to the target
 *              if the network address is known. this function is used a lot
 *              to support NWK applications.
 *
 * input parameters
 * @param   info    - pointer to strcuture containing info on how to build
 *                    the outgoing frame.
 * output parameters
 *
 * @return         SMPL_SUCCESS
 *                 SMPL_NOMEM       - no room in output frame queue
 *                 SMPL_TX_CCA_FAIL - CCA failure
 */
smplStatus_t nwk_rawSend(ioctlRawSend_t *info) {
	frameInfo_t *pOutFrame;

	if ((pOutFrame = nwk_buildFrame(info->port, info->msg, info->len, 1))) {
		memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), info->addr, NET_ADDR_SIZE);
		//UPDATE by LISHI on Jun 6, 2010
		return nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED); //changed to Forced from CCA
	}
	return SMPL_NOMEM;
}
//////////////////////////////////////////////////////////////////////////////
//Design by LISHI for ioctl read/write with ACK 
/////////////////////////////////////////////////////////////////////////////
#ifdef APP_AUTO_ACK
/******************************************************************************
 * @fn          nwk_rawSend_withACK
 *
 * @brief       Builds an outut frame based on information provided by the
 *              caller. This function allows a raw transmission to the target
 *              if the network address is known. this function is used a lot
 *              to support NWK applications. this function is used auto ack to
 *              support NWK applications
 *
 * input parameters
 * @param   info    - pointer to strcuture containing info on how to build
 *                    the outgoing frame.
 * output parameters
 *
 * @return         SMPL_SUCCESS
 *                 SMPL_NO_ACK       - no ack from reciever
 *                 SMPL_BAD_PARAM    - bad param
 */
smplStatus_t nwk_rawSend_withACK(ioctlRawSend_t *info)
{
	frameInfo_t *pOutFrame;
	//unsigned char      hops = 1;
	smplStatus_t rc = SMPL_BAD_PARAM;
	unsigned char radioState = MRFI_GetRadioState();

	volatile unsigned char ackTID_Local;
	extern unsigned char ackTID;
	extern unsigned char ackReceived;
	/* Set Receive ACK*/
	ackReceived = 0xFF;
	if ((pOutFrame = nwk_buildAckReqFrame(info->port, info->msg, info->len, 1, &ackTID_Local)))
	{
		memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), info->addr, NET_ADDR_SIZE);
		// UPDATE by LISHI on Jun. 6th, 2010
		rc = nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED); //changed to forced from CCA
	}

	NWK_CHECK_FOR_SETRX(radioState);
	NWK_REPLY_DELAY();
	//NWK_CHECK_FOR_RESTORE_STATE(radioState);
	// add by xt. 2010.8.20
	/* 强行切入IDLE状态 */
	MRFI_RxIdle();


	{
		bspIState_t intState;

		/* If the saved TID hasn't been reset then we never got the ack. */
		BSP_ENTER_CRITICAL_SECTION(intState);
		if (ackTID_Local == ackTID)
		{
			rc = SMPL_SUCCESS;
		}
		else {
			rc = SMPL_NO_ACK;
		}
		// add by xt. 2010.8.20
		ackTID = 0x00;
		BSP_EXIT_CRITICAL_SECTION(intState);
		/* Set Receive Packet */
		ackReceived = 0x00;
		return rc;
	}
}
#endif
/******************************************************************************
 * @fn          nwk_rawReceive
 *
 * @brief       Retriievs specified from from the input frame queue. Additional
 *              information such as source address and hop count may also be
 *              retrieved
 *
 * input parameters
 * @param   info    - pointer to structure containing info on what to retrieve
 *
 * output parameters - actually populated by nwk_retrieveFrame()
 *      info->msg      - application payload copied here
 *      info->len      - length of received application payload
 *      info->addr     - if non-NULL points to memory to be populated with
 *                       source address of retrieved frame.
 *      info->hopCount - if non-NULL points to memory to be populated with
 *                       hop count of retrieved frame.
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_rawReceive(ioctlRawReceive_t *info) {
	//rcvContext_t rcv;

	//rcv.type   = RCV_NWK_PORT;
	//rcv.t.port = info->port;

	return nwk_retrieveFrame(info->msg, &info->len, info->addr);
}

/**
 * Handle radio control functions.
 * @param action radio operation to perform.
 *               currently suppoerted:sleep/unsleep
 * @param val ???//TODO ADD COMMENT
 * @return  Status of operation.
 */
smplStatus_t nwk_radioControl(ioctlAction_t action, void *val) {
	smplStatus_t rc = SMPL_SUCCESS;

	if (action == IOCTL_ACT_RADIO_SLEEP) {
		/* go to sleep mode. */
		MRFI_RxIdle();
		MRFI_Sleep();
	} else if (action == IOCTL_ACT_RADIO_AWAKE) {
		MRFI_WakeUp();
	} else if (action == IOCTL_ACT_RADIO_SIGINFO) {
		nwk_getRadioSignInfo((ioctlRadioSiginfo_t *) val);
	} else if (action == IOCTL_ACT_RADIO_RSSI) {
		*((rssi_t *) val) = MRFI_Rssi();
	} else if (action == IOCTL_ACT_RADIO_RXON) {
		MRFI_RxOn();
	} else if (action == IOCTL_ACT_RADIO_WORON) {

		/* added by us for usage of WOR.  TN. Aug-2010 */

		/* MRFI_WorOn 存在失败的可能性，这里做一个重试机制 */
		unsigned char worOnRetryTimes = 250;
		unsigned char waitBetweenRetries = 20; //in ms

		rc = SMPL_TIMEOUT; //借用一下
		while (worOnRetryTimes) {
			worOnRetryTimes--;
			if (MRFI_WorOn()) {
				rc = SMPL_SUCCESS;
				break;
			}
			NWK_DELAY(waitBetweenRetries);
		}

		/* TODO 可以加个：如果切WOR失败了，闪一个error log灯 */

	} else if (action == IOCTL_ACT_RADIO_RXIDLE) {
		MRFI_RxIdle();
	} else {
		rc = SMPL_BAD_PARAM;
	}
	return rc;
}

/******************************************************************************
 * @fn          nwk_deviceAddress
 *
 * @brief       Set or Get this device address. The Set must be done before
 *              SMPL_Init() for it to take effect. The Get is always legal but
 *              the value could be invalid if it is called before a valid set
 *              call is made.
 *
 * input parameters
 * @param   action  - Gte or Set
 * @param   addr    - pointer to address object containing value on Set
 *
 * output parameters
 * @param   addr    - pointer to address object to receive value on Get.
 *
 * @return   SMPL_SUCCESS
 *           SMPL_BAD_PARAM  Action request illegal or a Set request
 *                           was not respected.
 */
smplStatus_t nwk_deviceAddress(ioctlAction_t action, addr_t *addr) {
	smplStatus_t rc = SMPL_BAD_PARAM;

	if (action == IOCTL_ACT_GET) {
		memcpy(addr, nwk_getMyAddress(), sizeof(addr_t));
		rc = SMPL_SUCCESS;
	} else if (action == IOCTL_ACT_SET) {
		if (nwk_setMyAddress(addr)) {
			rc = SMPL_SUCCESS;
		}
	}

	return rc;
}

