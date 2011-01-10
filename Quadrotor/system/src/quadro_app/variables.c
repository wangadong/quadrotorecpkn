#include "variables.h"

int AdValueGyroNick, AdValueGyroRoll, AdValueGyroYaw,AdValueAccRoll, AdValueAccNick , AdValueAccTop;
volatile unsigned char ADReady=0;
unsigned char msg[COMMAND_MAX_LENGTH_IN_BYTES];
