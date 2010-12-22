/*
 * hardware i2c slave driver.
 * @author Boreal Mao
 */

#ifndef HWI2CSLAVE2_H_
#define HWI2CSLAVE2_H_

#include "BspDef.h"

//ToCheck  __monitor to critical
critical unsigned char is_I2CFree(void);

void initialize_HWI2CSlave(unsigned char* rb, unsigned char rbLength, unsigned char* tb, unsigned char tbLength,
			unsigned char address);

#endif /* HWI2CSLAVE2_H_ */
