// 
// 
// 

#include "HAL_SatCom.h"
#include "Navigation.h"
#include "Mission.h"
#include "HAL_Time.h"
#include "TimeLib.h"
#include "WaveMeasurement.h"
#include "Wingsail.h"
#include "USFSmax.h"
#include "HAL_SDCard.h"
#include "Sd.h"

extern NavigationDataType NavData;
extern uint32_t Minute;
extern MissionValuesStruct MissionValues;
extern File LogFile;
extern HALPowerMeasure PowerSensor;
extern WaveClass Wave;
extern WingSailType WingSail;
extern HALWingAngle WingAngleSensor;
extern HALIMU imu;

void sendSatComVesselState()
{
	char FloatString[16];


	LogFile.print(F("MSG"));
	LogTime();
	LogFile.print("""");

	LogFile.print("V");
	LogFile.print(",");
	LogFile.print(year());
	LogFile.print(",");
	LogFile.print(month());
	LogFile.print(",");
	LogFile.print(day());
	LogFile.print(",");
	LogFile.print(hour());
	LogFile.print(",");
	LogFile.print(minute());
	LogFile.print(",");
	LogFile.print(second());
	LogFile.print(",");
	LogFile.print(Minute);
	LogFile.print(",");

	LogFile.print(strtrim(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString)));
	LogFile.print(",");
	LogFile.print(strtrim(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString)));
	LogFile.print(",");
	LogFile.print(NavData.COG);
	LogFile.print(",");
	LogFile.print(NavData.SOG_mps); // m/s
	LogFile.print(",");

	LogFile.print(NavData.ROLL_Avg);
	LogFile.print(",");
	LogFile.print(NavData.HDG_Mag);
	LogFile.print(",");

	LogFile.print(NavData.BTW);
	LogFile.print(",");
	LogFile.print(NavData.DTW);
	LogFile.print(",");
	LogFile.print(NavData.CTE);
	LogFile.print(",");

	LogFile.print(NavData.AWA);
	LogFile.print(",");

	LogFile.print(NavData.CourseType);	
	LogFile.print(",");
	LogFile.print(NavData.MaxCTE);
	LogFile.print(",");

	LogFile.print(strtrim(dtostrf(WingAngleSensor.WingSailAngleSensorPort.temperature, 5, 1, FloatString)));
	LogFile.print(",");
	LogFile.print(strtrim(dtostrf(imu.Baro, 7, 1, FloatString)));
	LogFile.print(",");

	LogFile.print(PowerSensor.BatteryOut_V);
	LogFile.print(",");
	LogFile.print(PowerSensor.BatteryOut_I);
	LogFile.print(",");

	LogFile.print(WingSail.Angle);
	LogFile.print(",");

	LogFile.print(strtrim(dtostrf(Wave.height, 5, 2, FloatString)));
	LogFile.print(",");
	LogFile.print(strtrim(dtostrf(Wave.period, 5, 1, FloatString)));
	LogFile.print(",");

	LogFile.print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 9, 5, FloatString));
	LogFile.print(",");
	LogFile.print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 9, 5, FloatString));
	LogFile.print(",");
	LogFile.print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 9, 5, FloatString));
	LogFile.print(",");
	LogFile.print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 9, 5, FloatString));
	LogFile.print(",");

	LogFile.print("z");

	LogFile.print("""");
	LogFile.println();
}

void sendSatComMissionEvent(int mission_index)
{
	// simulated sat com message - new mission step

	char FloatString[16];

	LogFile.print(F("MSG"));
	LogTime();
	LogFile.print("""");

	LogFile.print("M");
	LogFile.print(",");
	LogFile.print(year());
	LogFile.print(",");
	LogFile.print(month());
	LogFile.print(",");
	LogFile.print(day());
	LogFile.print(",");
	LogFile.print(hour());
	LogFile.print(",");
	LogFile.print(minute());
	LogFile.print(",");
	LogFile.print(second());
	LogFile.print(",");
	LogFile.print(Minute);
	LogFile.print(",");

	LogFile.print(strtrim(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString)));
	LogFile.print(",");
	LogFile.print(strtrim(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString)));
	LogFile.print(",");
	LogFile.print(NavData.COG);
	LogFile.print(",");
	LogFile.print(NavData.SOG_mps); // m/s
	LogFile.print(",");

	LogFile.print(mission_index);
	LogFile.print(",");
	LogFile.print(MissionValues.MissionList[mission_index].cmd);
	LogFile.print(",");
	LogFile.print(MissionValues.MissionList[mission_index].duration);
	LogFile.print(",");
	LogFile.print(MissionValues.MissionList[mission_index].boundary);
	LogFile.print(",");
	LogFile.print(MissionValues.MissionList[mission_index].SteerAWA);
	LogFile.print(",");
	LogFile.print(MissionValues.MissionList[mission_index].TrimTabAngle);
	LogFile.print(",");
	LogFile.print(strtrim(dtostrf(float(MissionValues.MissionList[mission_index].waypoint.lat) / 10000000UL, 10, 5, FloatString)));
	LogFile.print(",");
	LogFile.print(strtrim(dtostrf(float(MissionValues.MissionList[mission_index].waypoint.lng) / 10000000UL, 10, 5, FloatString)));
	LogFile.print(",");
	LogFile.print("z");

	LogFile.print("""");
	LogFile.println();
}


//https://stackoverflow.com/questions/25345598/c-implementation-to-trim-char-array-of-leading-trailing-white-space-not-workin

char* strtrim(char* str) {
	strtrim2(str);
	return str;
}

void strtrim2(char* str) {
	int start = 0; // number of leading spaces
	char* buffer = str;
	while (*str && *str++ == ' ') ++start;
	while (*str++); // move to end of string
	int end = str - buffer - 1;
	while (end > 0 && buffer[end - 1] == ' ') --end; // backup over trailing spaces
	buffer[end] = 0; // remove trailing spaces
	if (end <= start || start == 0) return; // exit if no leading spaces or string is now empty
	str = buffer + start;
	while ((*buffer++ = *str++));  // remove leading spaces: K&R
}