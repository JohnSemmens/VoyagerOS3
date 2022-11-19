// 
// 
// 

#include "HAL_Time.h"
#include "TimeLib.h"
#include "HAL_SDCard.h"

extern time_t GPSTime;

void time_init()
{
	// set the Time library to use Teensy 3.0's RTC to keep time
	Serial.println("*** Initialsing Time");

	setSyncProvider(getTeensy3Time);
	if (timeStatus() == timeSet)
		Serial.println("RTC has set the system time");
	else
		Serial.println("Unable to sync with the RTC");

	//time_update();
}

void sync_RTC_to_GPS()
{
	// adjust the system time and the RTC to match the GPS Time.

	long TimeAdjustment = GPSTime -now();

	setTime(GPSTime);			// set the system time to match the GPS Time.
	Teensy3Clock.set(GPSTime);  // set the RTC to match the GPS Time.

	// log an event on the SD Card
	String LogMessage = "RTC synced to GPS. Adjusted by " + String(TimeAdjustment) + " seconds.";
	Serial.println(LogMessage);
	SD_Logging_Event_Messsage(LogMessage);
}


time_t getTeensy3Time()
{
	return Teensy3Clock.get();
}