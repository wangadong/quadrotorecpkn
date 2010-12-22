/*
 * 目前对WOR的研究，最重要结论是：
 * 每个event0周期，Rx在event0长度中占 12.5%时，两点收发稳定
 * （20次内成功率接近100%）
 * 因此目前参数均按 12.5%设置，并设event0长度为 150ms.
 *
 */
#ifndef MRFI_H
#define MRFI_H

#include "BspDef.h"
#include "mrfi_defs.h"
#include "mrfi_spi.h"
#include "smartrf/CC1101/smartrf_CC1101.h"


typedef struct {
  unsigned char frame[MRFI_MAX_FRAME_SIZE];
  unsigned char rxMetrics[MRFI_RX_METRICS_SIZE];
} mrfiPacket_t;

#define WORCTRL_DEFAULT 0xF8
#define MCSM2_DEFAULT   0x07
#define WOREVT0_DEFAULT 0x6B
#define WOREVT1_DEFAULT 0x87

#define WORCTRL_WOR     0x38
#define WOREVT1_WOR     0x14
#define WOREVT0_WOR     0x4F

#define MCSM2_WOR       0x00
#define MCSM0_WOR       0x18

extern const unsigned char mrfiBroadcastAddr[];

//ToCheck No upper layer caller, mrfi.c caller
void Mrfi_WORSettingOn(void);

//ToCheck No upper layer caller, mrfi.c caller
void Mrfi_WORSettingDefault(void);

//ToCheck No upper layer caller,mrfi.c caller
unsigned char MRFI_WorOn(void);

//ToCheck No upper layer caller,  mrfi.c caller
void MRFI_WorOff(void);

//ToCheck nwk_api.c caller
void    MRFI_Init(void);

//ToCheck nwk_frame.c caller
unsigned char MRFI_Transmit(mrfiPacket_t *, unsigned char);

//ToCheck nwk_frame.c caller
void    MRFI_Receive(mrfiPacket_t *);

//ToCheck nwk_frame.c caller
/* populated by code using MRFI */
void    MRFI_RxCompleteISR(void);

//ToCheck nwk_ioctl.c caller
unsigned char MRFI_GetRadioState(void);

//ToCheck nwk_ioctl.c caller
void    MRFI_RxOn(void);

//ToCheck nwk_ioctl.c caller
void    MRFI_RxIdle(void);

//ToCheck nwk_ioctl.c caller
signed char  MRFI_Rssi(void);

//ToCheck No upper layer caller, mrfi.c caller
void    MRFI_SetLogicalChannel(unsigned char);

//ToCheck nwk_api.c caller
unsigned char MRFI_SetRxAddrFilter(unsigned char *);

//ToCheck nwk_api.c caller
void    MRFI_EnableRxAddrFilter(void);

//ToCheck No layer caller
void    MRFI_DisableRxAddrFilter(void);

//ToCheck nwk_ioctl.c caller
void    MRFI_Sleep(void);

//ToCheck nwk_ioctl.c,nwk_api.c caller
void    MRFI_WakeUp(void);

//ToCheck nwk_frame.c caller
unsigned char MRFI_RandomByte(void);

//ToCheck HyenaNode_2232.c,HyenaRoot.c,HyenaUtils.c,nwk_ioctl.c caller
void    MRFI_DelayMs(unsigned short);

//ToCheck nwk_ioctl.c caller
void    MRFI_ReplyDelay(void);

//ToCheck nwk_frame.c caller
void    MRFI_PostKillSem(void);

//ToCheck No upper layer caller, mrfi.c caller
void    MRFI_SetRFPwr(unsigned char);

#endif
