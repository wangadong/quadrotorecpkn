/*
 * This header file supports the SimpliciTI IOCTL implmentation.
 * This interface gives applications access to the "driver"
 * network level functions when necessary.
 */
#ifndef NWK_IOCTL_H
#define NWK_IOCTL_H

#include "nwk_frame.h"

/* prototypes */
smplStatus_t nwk_rawSend(ioctlRawSend_t *);

//Design by LISHI
#ifdef APP_AUTO_ACK
smplStatus_t nwk_rawSend_withACK(ioctlRawSend_t *);
#endif

smplStatus_t nwk_rawReceive(ioctlRawReceive_t *);
smplStatus_t nwk_radioControl(ioctlAction_t, void *);
smplStatus_t nwk_deviceAddress(ioctlAction_t, addr_t *);

#endif
