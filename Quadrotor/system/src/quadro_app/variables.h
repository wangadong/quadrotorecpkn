#define COMMAND_MAX_LENGTH_IN_BYTES 14
extern float AdValueGyroNick, AdValueGyroRoll, AdValueGyroYaw,AdValueAccRoll, AdValueAccNick , AdValueAccTop;
extern volatile unsigned char ADReady,ADTFinish;
extern unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES];
extern volatile unsigned short voltage[16];
