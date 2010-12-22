#include "SMPL.h"

void hyenaInit(void) {
	BSP_Init();

	initLeds();
	initUart();
	initPower_BSP();

	SMPL_Init();
}


