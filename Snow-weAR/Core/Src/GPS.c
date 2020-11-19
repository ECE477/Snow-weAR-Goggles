#include "../Inc/GPS/GPS.h"
#include "../Inc/GPS/usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define	_GPS_USART					huart2
#define	_GPS_DEBUG					0

GPS_t GPS;

int store[100] = {0};
int count = 0;

//##################################################################################################################
double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );

  return decDeg;
}

void GPS_Parse(void){

	char* str = (char*) GPS.rxBuffer;

	int i=7;
	if(str[3] == 86 && str[4] == 84) {
		int comma = 0;
		while(comma < 6) {
			if(str[i++] == ',') {
				comma++;
			}
		}
		GPS.Velocity = 0;
		while(str[i] != '.') {
			GPS.Velocity = GPS.Velocity*10 + str[i++]-48;
		}
		int velDecimal = 0;
		int divFactor = 1;
		while(str[i] != ',') {
			//GPS.Velocity += (str[i++]-48) / divFactor;
			velDecimal = velDecimal*10 + str[i++]-48;
			divFactor *= 10;
		}
		GPS.Velocity = GPS.Velocity + velDecimal / divFactor;
		return;
	}

	while(str[i-1] != ',') {
		i++;
	}
	GPS.Latitude = 0;
	GPS.LatitudeDecimal = 0;
	GPS.Longitude = 0;
	GPS.LongitudeDecimal = 0;
	GPS.MSL_Altitude = 0;

	GPS.UTC_Hour = 10*(str[i++]-48) + str[i++]-53;
	GPS.UTC_Min = 10*(str[i++]-48) + str[i++]-48;
	GPS.UTC_Sec = 10*(str[i++]-48) + str[i++]-48;
	store[count++] = GPS.UTC_Sec;
	i++;
	GPS.UTC_MicroSec = 100*(str[i++]-48) + 10*(str[i++]-48) + str[i++]-48;
	i++;
	while(str[i] != '.') {
		GPS.Latitude = GPS.Latitude*10 + str[i++]-48;
	}
	i++;
	int divFactor = 1;
	while(str[i] != ',') {
		GPS.LatitudeDecimal = GPS.LatitudeDecimal*10 + str[i++]-48;
		divFactor *= 10;
	}
	GPS.Latitude = GPS.Latitude + GPS.LatitudeDecimal / divFactor;
	i++;
	GPS.NS_Indicator = str[i++];
	i++;
	while(str[i] != '.') {
		GPS.Longitude = GPS.Longitude*10 + str[i++]-48;
	}
	i++;
	divFactor = 1;
	while(str[i] != ',') {
		GPS.LongitudeDecimal = GPS.LongitudeDecimal*10 + str[i++]-48;
		divFactor *= 10;
	}
	i++;
	GPS.EW_Indicator = str[i++];
	GPS.LatitudeDecimal = convertDegMinToDecDeg(GPS.Latitude);
	GPS.LongitudeDecimal = convertDegMinToDecDeg(GPS.Longitude);

	int comma = 0;
	while(comma < 4) {
		if(str[i++] == ',') {
			comma++;
		}
	}
	while(str[i] != '.') {
		GPS.MSL_Altitude = GPS.MSL_Altitude*10 + str[i++]-48;
	}
	int altDecimal = 0;
	divFactor = 1;
	while(str[i] != ',') {
		altDecimal = altDecimal*10 + str[i++]-48;
		divFactor *= 10;
	}
	GPS.MSL_Altitude = GPS.MSL_Altitude + altDecimal / divFactor;
}

//##################################################################################################################
