#include <stdlib.h>
#include <avr/pgmspace.h>
#include "mymath.h"

// discrete mathematics

// Sinus with argument in degree at an angular resolution of 1 degree and a discretisation of 13 bit.
const uint16_t pgm_sinlookup[91] PROGMEM = {0, 143, 286, 429, 571, 714, 856, 998, 1140, 1282, 1423, 1563, 1703, 1843, 1982, 2120, 2258, 2395, 2531, 2667, 2802, 2936, 3069, 3201, 3332, 3462, 3591, 3719, 3846, 3972, 4096, 4219, 4341, 4462, 4581, 4699, 4815, 4930, 5043, 5155, 5266, 5374, 5482, 5587, 5691, 5793, 5893, 5991, 6088, 6183, 6275, 6366, 6455, 6542, 6627, 6710, 6791, 6870, 6947, 7022, 7094, 7165, 7233, 7299, 7363, 7424, 7484, 7541, 7595, 7648, 7698, 7746, 7791, 7834, 7875, 7913, 7949, 7982, 8013, 8041, 8068, 8091, 8112, 8131, 8147, 8161, 8172, 8181, 8187, 8191, 8192};

int16_t c_sin_8192(int16_t angle)
{
	int8_t m,n;
	int16_t sinus;

	// avoid negative angles
	if (angle < 0)
	{
		m = -1;
		angle = abs(angle);
	}
	else m = +1;

	// fold angle to intervall 0 to 359
	angle %= 360;

	// check quadrant
	if (angle <= 90) n=1; // first quadrant
	else if ((angle > 90) && (angle <= 180)) {angle = 180 - angle; n = 1;} // second quadrant
	else if ((angle > 180) && (angle <= 270)) {angle = angle - 180; n = -1;} // third quadrant
	else {angle = 360 - angle; n = -1;}	//fourth quadrant
	// get lookup value
	sinus = pgm_read_word(&pgm_sinlookup[angle]);
	// calculate sinus value
	return (sinus * m * n);
}

// Cosinus with argument in degree at an angular resolution of 1 degree and a discretisation of 13 bit.
int16_t c_cos_8192(int16_t angle)
{
	return (c_sin_8192(90 - angle));
}


// Arcustangens returns degree in a range of +/. 180 deg
const uint8_t pgm_atanlookup[346] PROGMEM = {0,1,2,3,4,4,5,6,7,8,9,10,11,11,12,13,14,15,16,17,17,18,19,20,21,21,22,23,24,24,25,26,27,27,28,29,29,30,31,31,32,33,33,34,35,35,36,36,37,37,38,39,39,40,40,41,41,42,42,43,43,44,44,45,45,45,46,46,47,47,48,48,48,49,49,50,50,50,51,51,51,52,52,52,53,53,53,54,54,54,55,55,55,55,56,56,56,57,57,57,57,58,58,58,58,59,59,59,59,60,60,60,60,60,61,61,61,61,62,62,62,62,62,63,63,63,63,63,63,64,64,64,64,64,64,65,65,65,65,65,65,66,66,66,66,66,66,66,67,67,67,67,67,67,67,68,68,68,68,68,68,68,68,69,69,69,69,69,69,69,69,69,70,70,70,70,70,70,70,70,70,71,71,71,71,71,71,71,71,71,71,71,72,72,72,72,72,72,72,72,72,72,72,73,73,73,73,73,73,73,73,73,73,73,73,73,73,74,74,74,74,74,74,74,74,74,74,74,74,74,74,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79};

int16_t c_atan2(int16_t y, int16_t x)
{
	int16_t index, angle;
	int8_t m;

	if (!x && !y) return 0;		//atan2(0, 0) is undefined

	if (y < 0) m = -1;
	else m = 1;

	if (!x) return (90 * m);		// atan2(y,0) = +/- 90 deg

	index = (int16_t)(((int32_t)y * 64) / x);// calculate index for lookup table
	if (index < 0) index = -index;

	if (index < 346) angle = pgm_read_byte(&pgm_atanlookup[index]);	// lookup for 0 deg to 79 deg
	else if (index > 7334) angle = 90;						// limit is 90 deg
	else if (index > 2444) angle = 89;						// 89 deg to 80 deg is mapped via intervalls
	else if (index > 1465) angle = 88;
	else if (index > 1046) angle = 87;
	else if (index > 813) angle = 86;
	else if (index > 664) angle = 85;
	else if (index > 561) angle = 84;
	else if (index > 486) angle = 83;
	else if (index > 428) angle = 82;
	else if (index > 382) angle = 81;
	else angle = 80; // (index>345)

	if (x > 0) return (angle * m);	// 1st and 4th quadrant
	else if ((x < 0) && (m > 0)) return (180 - angle);	// 2nd quadrant
	else return (angle - 180); // ( (x < 0) && (y < 0))	3rd quadrant
}



// integer square root
uint32_t c_sqrt(uint32_t number)
{
	if(!number) return 0;
	uint32_t s1, s2;
	uint8_t iter = 0;
	// initialization of iteration
	s2 = number;
	do // iterative formula to solve x^2 - n = 0
	{
		s1 = s2;
		s2 = number / s1;
		s2 += s1;
		s2 /= 2;
		iter++;
		//if(iter > 40) break;
	}while( ( (s1-s2) > 1) && (iter < 40));
	return s2;
}
