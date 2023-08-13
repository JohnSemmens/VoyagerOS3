// HAL_Watchdog.h

#ifndef _HAL_WATCHDOG_h
#define _HAL_WATCHDOG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void Watchdog_Init(int timeout);

void Watchdog_Pat();

#endif

