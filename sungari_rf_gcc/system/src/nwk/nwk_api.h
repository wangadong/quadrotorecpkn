/**************************************************************************************************
 Filename:       nwk_api.h

 Description:    This header file supports the SimpliciTI appliction layer API.

 **************************************************************************************************/

#ifndef NWK_API_H
#define NWK_API_H

#include "nwk_ioctl.h"

smplStatus_t SMPL_Init(void);
smplStatus_t SMPL_Ioctl(ioctlObject_t, ioctlAction_t, void *);

#endif
