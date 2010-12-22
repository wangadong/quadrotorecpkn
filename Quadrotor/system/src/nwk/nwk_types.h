#ifndef NWK_TYPES_H
#define NWK_TYPES_H

#define NWK_TX_RETRY_COUNT    10
#define NWK_RX_RETRY_COUNT    10

#define NWK_APP_REPLY_BIT  (0x80)

#define NET_ADDR_SIZE      MRFI_ADDR_SIZE   /* size of address in bytes */

#define NWK_FREQ_TBL_SIZE  1

#define SMPL_PORT_USER_BCAST    0x3F

typedef struct {
  unsigned char  addr[NET_ADDR_SIZE];
} addr_t;

typedef unsigned char ccRadioStatus_t;


enum ioctlObject  {
  IOCTL_OBJ_RAW_IO,
  IOCTL_OBJ_RADIO,
  IOCTL_OBJ_ADDR,
};

enum ioctlAction  {
  IOCTL_ACT_SET,
  IOCTL_ACT_GET,
  IOCTL_ACT_READ,
  IOCTL_ACT_WRITE,
  IOCTL_ACT_RADIO_SLEEP,
  IOCTL_ACT_RADIO_AWAKE,
  IOCTL_ACT_RADIO_SIGINFO,
  IOCTL_ACT_RADIO_RSSI,
  IOCTL_ACT_RADIO_RXON,
  IOCTL_ACT_RADIO_RXIDLE,
  IOCTL_ACT_RADIO_SETPWR,
  IOCTL_ACT_RADIO_WORON,         //Set by LISHI
  IOCTL_ACT_ON,
  IOCTL_ACT_OFF,
  IOCTL_ACT_SCAN,
  IOCTL_ACT_DELETE
};

typedef enum ioctlObject   ioctlObject_t;
typedef enum ioctlAction   ioctlAction_t;


typedef struct
{
  addr_t   *addr;
  unsigned char  *msg;
  unsigned char   len;
  unsigned char   port;
} ioctlRawSend_t;

typedef struct
{
  addr_t  *addr;
  unsigned char *msg;
  unsigned char  len;
  unsigned char  port;
  unsigned char  hopCount;
} ioctlRawReceive_t;

/**
 * Signal information support
 */
typedef signed char rssi_t;

typedef struct{
  rssi_t  rssi;
  unsigned char lqi;
} rxMetrics_t;

typedef struct{
  rxMetrics_t  sigInfo;
} ioctlRadioSiginfo_t;


enum smplStatus  {
  SMPL_SUCCESS,
  SMPL_TIMEOUT,
  SMPL_BAD_PARAM,
  SMPL_NOMEM,
  SMPL_NO_FRAME,
  SMPL_NO_CHANNEL,
  SMPL_NO_PEER_UNLINK,
  SMPL_TX_CCA_FAIL,
  SMPL_NO_PAYLOAD,
  SMPL_NO_AP_ADDRESS,
  SMPL_NO_ACK
};

typedef enum smplStatus    smplStatus_t;

/* NWK application frame handling status codes */
enum fhStatus
{
  FHS_RELEASE,   /* handled in interrupt thread */
  FHS_KEEP,      /* handled by background application */
  FHS_REPLAY     /* non-ED case: NWK frame not for me that should be replayed */
};

typedef enum fhStatus   fhStatus_t;


/* Tx options type */
typedef  unsigned short   txOpt_t;

/* Delay loop support. Requires mrfi.h */
#define NWK_DELAY(spin)   MRFI_DelayMs(spin)
#define NWK_REPLY_DELAY() MRFI_ReplyDelay();

/**
 *  Network applications may need to remember radio state because the user
 * application may choose to turn Rx off. These macros help get and restore
 * the radio Rx state. The macros should be in the same code block at the same level.
 * The argument 's' is the 'current' radio state and should be set in the code block
 * with a call to MRFI_GetRadioState() _before_ using the macros.
 *
 * Used extensively by NWK but user applications may use them as well. But it is
 * much more liekly that an application will know the radio state since it likely
 * will have set it with IOCTL calls. Requires mrfi.h.
 */
#define NWK_CHECK_FOR_SETRX(s)  if (s != MRFI_RADIO_STATE_RX)    \
                                {                                \
                                  if (s == MRFI_RADIO_STATE_OFF) \
                                  {                              \
                                    MRFI_WakeUp();               \
                                  }                              \
                                  MRFI_RxOn();                   \
                                }

#define NWK_CHECK_FOR_RESTORE_STATE(s) if (s != MRFI_RADIO_STATE_RX)    \
                                       {                                \
                                         if (s == MRFI_RADIO_STATE_OFF) \
                                         {                              \
                                           MRFI_Sleep();                \
                                         }                              \
                                         else                           \
                                         {                              \
                                           MRFI_RxIdle();               \
                                         }                              \
                                       }
#endif
