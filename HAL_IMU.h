// HAL_IMU.h

#ifndef _HAL_IMU_h
#define _HAL_IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "HAL.h"
#include "MagneticSensorLsm303.h"

class HALIMU
{
public:
	float TemperatureC;
	//float Baro;

	int Pitch; // pitch positive bow up, negative bow down
	int Roll;
	int Heading;
	//int Algorithm_Status;

	void Init();
	void Read();
	MagneticSensorLsm303 compass;
	EquipmentStatusType EquipmentStatus;
};



#endif

