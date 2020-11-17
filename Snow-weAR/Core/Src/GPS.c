#include "GPS.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define	_GPS_USART					huart2
#define	_GPS_DEBUG					0

GPS_t GPS;

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
//##################################################################################################################
