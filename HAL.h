// HAL.h

#ifndef _HAL_h
#define _HAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum EquipmentStatusType {
	Unknown,
	NotFound,
	DataBad,
	Found,
	DataGood
};

#include "HAL_Display.h"
#include "HAL_GPS.h"
#include "HAL_IMU.h"
#include "HAL_PowerMeasurement.h"
#include "HAL_SDCard.h"
#include "HAL_Servo.h"
#include "HAL_Telemetry.h"
#include "HAL_WingAngle.h"

void Scan_I2C_Buses();

#endif
