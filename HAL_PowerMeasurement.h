// HAL_PowerMeasurement.h

#ifndef _HAL_PowerMeasurement_h
#define _HAL_PowerMeasurement_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define Solar 1
#define BatteryIn 2
#define BatteryOut 3

#include "HAL.h"

class HALPowerMeasure
{
public:
	float Solar_V;
	float Solar_I;
	float BatteryIn_V;
	float BatteryIn_I;
	float BatteryOut_V;
	float BatteryOut_I;


	void read(void);
	void init(void);

	EquipmentStatusType EquipmentStatus;
};

#endif

