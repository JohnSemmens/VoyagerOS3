// HAL_SatCom.h

#ifndef _HAL_SatCom_h
#define _HAL_SatCom_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "astronode.h"

class HALSatComms
{
public:
	ASTRONODE astronode;

	void Init();
	void Read();
	uint8_t RSSI;
	uint32_t timeToNextSat;
	time_t timeValue;
	int OutboundMsgCount;

	int TestCounter;

	uint8_t OutboundMsg[50]; // 93 bytes is a reasonable upper limit in payload size, but smaller, the better.

	String LastCommand;

	void sendSatComVesselState();
	void sendSatComMissionEvent(int mission_index);

	void QueueMessage(uint8_t *OutboundMsg, uint8_t OutboundMsgSize);

	EquipmentStatusType EquipmentStatus;
};

#endif

