#ifndef _MM3_H
#define _MM3_H

typedef struct
{
	int8_t X_off;
	int8_t Y_off;
	int8_t Z_off;
	int16_t X_range;
	int16_t Y_range;
	int16_t Z_range;
} MM3_calib_t;

extern MM3_calib_t MM3_calib;

// Initialization of the MM3 communication
void MM3_Init(void);

// should be called cyclic to get actual compass axis readings
void MM3_Update(void);
// this function calibrates the MM3
// and returns immediately if the communication to the MM3-Board is broken.
void MM3_Calibrate(void);

// update compass heading
void MM3_Heading(void);

#endif //_MM3_H

