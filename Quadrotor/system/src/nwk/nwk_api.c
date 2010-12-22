/**************************************************************************************************
 Filename:       nwk_api.c

 Description:    This file supports the SimpliciTI appliction layer API.

 **************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include <string.h>

#include "nwk_api.h"
#include "nwk_frame.h"

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/* These defines are in support an application listening for a link frame to
 * terminate after some amount of time. The intention is that this guard be
 * the exception. The intention of the SimpliciTI design is that the
 * temporal contiguity between the listen and the reception of the link frame
 * from the peer be very tight. The SMPL_LinkListen() should be termninated
 * by the reception of the link frame. But in case it does not receive the frame
 * the support below allows intervention by the application.
 */

/* The intention is for user to modify just the following single value */
//#define LINKLISTEN_MILLISECONDS_2_WAIT    (5000)

//#define LINKLISTEN_POLL_PERIOD_MS         (10)
//#define LINKLISTEN_POLL_COUNT             ( (LINKLISTEN_MILLISECONDS_2_WAIT) / (LINKLISTEN_POLL_PERIOD_MS) )

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
static unsigned char sInit_done = 0;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static unsigned char ioctlPreInitAccessIsOK(ioctlObject_t);
/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/***********************************************************************************
 * @fn          SMPL_Init
 *
 * @brief       Initialize the SimpliciTI stack.
 *
 * input parameters
 * @param   f  - Pointer to call back function. Function called by NWK when
 *               user application frame received. The callback is done in the
 *               ISR thread. Argument is Link ID associated with frame. Function
 *               returns 0 if frame is to be kept by NWK, otherwise 1. Frame
 *               should be kept if application will do a SMPL_Receive() in the
 *               user thread (recommended). Pointer may be NULL.
 *
 * output parameters
 *
 * @return   Status of operation:
 *             SMPL_SUCCESS
 *             SMPL_NO_JOIN     No Join reply. AP possibly not yet up.
 *             SMPL_NO_CHANNEL  Only if Frequency Agility enabled. Channel scan
 *                              failed. AP possibly not yet up.
 */
smplStatus_t SMPL_Init(void) {
	if (!sInit_done) {
		/* set up radio. */
		MRFI_Init();

		/* initialize network */
		nwk_frameInit();

		MRFI_WakeUp();
		/* don't turn Rx on if we're an end device that isn't always on. */

		/* All except End Devices are in promiscuous mode */
		MRFI_SetRxAddrFilter((unsigned char *) nwk_getMyAddress());
		MRFI_EnableRxAddrFilter();
	}
	sInit_done = 1;

	return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          SMPL_Ioctl
 *
 * @brief       This routine supplies the SimpliciTI IOCTL support.
 *
 * input parameters
 * @param   object   - The IOCTL target object
 * @param   action   - The IOCTL target action on the object
 * @param   val      - pointer to value. exact forn depends on object type.
 *
 * output parameters
 *
 * @return   Status of action. Value depends on object, action, and result.
 *
 *           SMPL_BAD_PARAM is returned if this API is called before
 *                          initialization and the object is not one of
 *                          the valid exceptions.
 */
smplStatus_t SMPL_Ioctl(ioctlObject_t object, ioctlAction_t action, void *val) {
	smplStatus_t rc;

	/* if init hasn't occurred see if access is still valid */
	if (!sInit_done && !ioctlPreInitAccessIsOK(object)) {
		return SMPL_BAD_PARAM;
	}

	switch (object) {

	case IOCTL_OBJ_ADDR:
		if ((action == IOCTL_ACT_GET) || (action == IOCTL_ACT_SET)) {
			rc = nwk_deviceAddress(action, (addr_t *) val);
		} else {
			rc = SMPL_BAD_PARAM;
		}
		break;

	case IOCTL_OBJ_RAW_IO:
		if (action == IOCTL_ACT_WRITE) {
			//Set by LISHI
#ifdef APP_AUTO_ACK
			rc = nwk_rawSend_withACK((ioctlRawSend_t *)val);
#else
			rc = nwk_rawSend((ioctlRawSend_t *) val);
#endif
		} else if (action == IOCTL_ACT_READ) {
			rc = nwk_rawReceive((ioctlRawReceive_t *) val);
		} else {
			rc = SMPL_BAD_PARAM;
		}
		break;

	case IOCTL_OBJ_RADIO:
		rc = nwk_radioControl(action, val);
		break;

	default:
		rc = SMPL_BAD_PARAM;
		break;
	}

	return rc;
}
/******************************************************************************
 * @fn          ioctlPreInitAccessIsOK
 *
 * @brief       Is the request legal yet? Most requests are not legal before
 *              SMPL_Init().
 *
 * input parameters
 * @param   object   - The IOCTL target object
 *
 * output parameters
 *
 * @return   Returns non-zero if request should be honored for further
 *           processing, otherwise returns 0. This function does not
 *           determine of the object-action pair are valid. It only knows
 *           about exceptions, i.e., those that are valid before the
 *           SMPL_Init() call.
 *
 */
static unsigned char ioctlPreInitAccessIsOK(ioctlObject_t object) {
	unsigned char rc;

	/* Currently the only legal pre-init accesses are the address and
	 * the token objects.
	 */
	switch (object) {
	case IOCTL_OBJ_ADDR:
		rc = 1; /* legal */
		break;

	default:
		rc = 0; /* not legal when init not done */
		break;
	}

	return rc;
}
