// LoRaManagement.h

#ifndef _LORAMANAGEMENT_h
#define _LORAMANAGEMENT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum LoRa_Mode_Type
{
	RxNormal, 
	RxLowPower,     // wakeup periodically to listen
	TxShortRange,	
	TxLongRange
};


void Init_LoRa(void);
void Set_LoRa_Mode(LoRa_Mode_Type mode);

#endif

