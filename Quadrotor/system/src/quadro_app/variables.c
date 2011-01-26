#include "variables.h"

long AdValueGyroNick, AdValueGyroRoll, AdValueGyroYaw,AdValueAccRoll, AdValueAccNick , AdValueAccTop;
volatile unsigned char ADReady=0,ADTFinish=1;
unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES];
