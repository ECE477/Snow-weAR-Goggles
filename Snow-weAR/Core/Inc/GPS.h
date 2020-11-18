#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>

//##################################################################################################################

typedef struct {
	uint8_t		UTC_Hour;
	uint8_t		UTC_Min;
	uint8_t		UTC_Sec;
	uint16_t	UTC_MicroSec;

	float		Latitude;
	float		LatitudeDecimal;  // changed from double
	char		NS_Indicator;
	float		Longitude;
	float		LongitudeDecimal; // changed from double
	char		EW_Indicator;

	float		MSL_Altitude;

} GPGGA_t;

typedef struct {
	uint8_t		rxBuffer[512];
	uint16_t	rxIndex;
	uint8_t		rxTmp;
	float		Velocity;

	GPGGA_t		GPGGA;

} GPS_t;

extern GPS_t GPS;

//##################################################################################################################
double convertDegMinToDecDeg (float degMin);
//##################################################################################################################

#endif
