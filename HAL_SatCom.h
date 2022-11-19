// HAL_SatCom.h

#ifndef _HAL_SatCom_h
#define _HAL_SatCom_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void sendSatComVesselState();

void sendSatComMissionEvent(int mission_index);

char* strtrim(char* str);
void strtrim2(char* str);

#endif

