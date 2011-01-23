#ifndef _MK3MAG_H
#define _MK3MAG_H

typedef struct
{
 int16_t Attitude[2];
 uint8_t UserParam[2];
 uint8_t CalState;
 uint8_t Orientation;
} ToMk3Mag_t;

extern ToMk3Mag_t ToMk3Mag;

// Initialization
void MK3MAG_Init(void);

// should be called cyclic to get actual compass heading
void MK3MAG_Update(void);

#endif //_MK3MAG_H

