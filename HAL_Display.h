// HAL_Display.h

#ifndef _HAL_Display_h
#define _HAL_Display_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "HAL.h"

class HALDisplay
{
public:
	EquipmentStatusType EquipmentStatus;

	void Init();
	void Page(char page);
};


#endif

