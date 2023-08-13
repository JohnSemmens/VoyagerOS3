// HAL_SDCard.h

#ifndef _HAL_SDCard_h
#define _HAL_SDCard_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

	void SD_Logging_Init();
	void SD_Logging_OpenFile();
	void SD_Logging_1s(void);
	void SD_Logging_1m(void);
	void SD_Logging_Waypoint(void);

	void OpenSDLogFile(String LogFileName);

	void Check_LogFileSize(void);
	void CloseThenOpenLogFile(void);
	//void Check_GPS_TimeStatus(void);

	void SD_Logging_Event_Decisions(void);
	void SD_Logging_Event_Usage(void);
	void SD_Logging_Event_ParameterChange(int ParameterIndex, char ParameterValue[12]);
	void SD_Logging_Event_Messsage(String message);
	void SD_Logging_Event_MissionStep(int mission_index);

	void SD_Logging_Event_Wingsail_Power(void);
	void SD_Logging_Event_Wingsail_Monitor(String Description);
	//void SD_Logging_Event_GPS_Power(void);

	void LogTime(void);
	void LogTimeHeader(void);
	void dateTime(uint16_t* date, uint16_t* time);
#endif

