// TelemetryMessages.h

#ifndef _TELEMETRYMESSAGES_h
#define _TELEMETRYMESSAGES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum TelMessageType {Dummy_0
			//	, SetWing		// wing sail commands are top priority
			//	, SetWing2		// This is a second slot for Set Wing command. Needed to support two setwing commands in quick succesion. 
				, PRL, MCP // then lists
				, LNA, LAT, LPO, LWP, LMI, LWI, LSV, LVS, LPF //, then remaining commands in priority order
				, MCC, MIG, MIS, HLG, HLS, VER, TMG, LCD
				, EQG, CCS, CCG, WC1, WC0, SCS, SCG, DBG
				//, GSV, 
				, LWS // wingsail data
				, PRG // get one parameter
				, PRM // Max Parameter Number
,EndMarker};


void SendMessage(int SerialPortNumber, TelMessageType msg);
void ProcessQueue(int SerialPortNumber);

void QueueMessage(TelMessageType msg);
void SendMissionStep(int CommandPort, int index);

//void WakeupPrefix(int CommandPort,char NextAddr);

#endif

