#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>

//##################################################################################################################

typedef struct {
	uint8_t		rxBuffer[512];
	uint16_t	rxIndex;
	uint8_t		rxTmp;

	uint8_t 	UTC_Hour;
	uint8_t		UTC_Min;
	uint8_t		UTC_Sec;
	uint8_t		UTC_MicroSec;

	float 		Latitude;
	float		LatDec;
	char		NS_Indicator;
	float		Longitude;
	float		LonDec;
	char		EW_Indicator;
	float 		MSL_Altitude;

	float		Velocity;

	char 		str[12];

} GPS_t;

extern GPS_t GPS;

//##################################################################################################################
double convertDegMinToDecDeg (float degMin);
void GPS_Parse(void);
void GPS_String(void);
//##################################################################################################################

#endif
