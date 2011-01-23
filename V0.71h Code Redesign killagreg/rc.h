#ifndef _RC_H
#define _RC_H

#include <inttypes.h>

extern void RC_Init (void);
extern volatile int16_t PPM_in[15];		// the RC-Signal
extern volatile int16_t PPM_diff[15];	// the differentiated RC-Signal
extern volatile uint8_t NewPpmData;     // 0 indicates a new recieved PPM Frame
extern volatile int16_t RC_Quality;     // rc signal quality indicator (0 to 200)
#endif //_RC_H
