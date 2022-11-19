// HAL_Telemetry.h

#ifndef _HAL_Telemetry_h
#define _HAL_Telemetry_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


class HALTelemetry
{
public:
	void Init();
	bool status;
};


#endif

