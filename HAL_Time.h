// HAL_Time.h

#ifndef _HAL_TIME_h
#define _HAL_TIME_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


void time_init(void);

void sync_RTC_to_GPS(void);

time_t getTeensy3Time(void);

#endif

