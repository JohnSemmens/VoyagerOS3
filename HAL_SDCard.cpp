
// SD Card Logging 
// 
// V1.0 3/8/2016 John Semmens
// V1.8 29/10/2017 updated for new IMU BNO-055 rather than MPU-9250.
// V1.9 12/11/2017 updated to change BRL to RLB.
// V1.10 15/4/2018 Added Servo output logging SVO and Voltage montitoring VLT.
// V1.11 17/4/2018 bug fix Servo output logging SVO and Voltage montitoring VLT.
// V1.12 28/10/2018 changed to CurrentLocalTime
// V1.13 25/4/2019 extended SAI to include Turn Heading and Target Heading, and added Temp and Press to System.
// V1/14 11/6/2019 added Trim tab Angle to SVO sentence, and Wing Angle to ENV. and IsBTWSailable to NAV
// V1.15 9/7/2019 added Decision Event Value to logging.
// V1.16 11/7/2019 added Heading Error from GPS
// V1.17 8/1/2022 updated WSP to record voltages with 3 digits to observe discharge rates.
// V1.18 19/6/2022 added GPSPwr sentence as Event.
// V1.19 22/7/2023 removed GPS power controls. 

#include "HAL.h"
#include "Sd.h"
#include "HAL_GPS.h"

#include "HAL_SDCard.h"

#include "Navigation.h"
#include "Mission.h"
#include "configValues.h"
#include "CommandState_Processor.h"
#include "WearTracking.h"
#include "Wingsail.h"
#include "Mission.h"
#include "HAL_Display.h"
#include "DisplayStrings.h"
#include "HAL_Time.h"
#include "TimeLib.h"
#include "InternalTemperature.h"

extern File LogFile;

#define  SD_Card_CS_PIN BUILTIN_SDCARD // 10 on Nano, 53 on Mega, E3 on Teensy 3.6   

extern HALGPS gps;
extern StateValuesStruct StateValues;
extern NavigationDataType NavData;
extern long loop_period_us; // microseconds between successive main loop executions

extern bool UseSimulatedVessel; 	// flag to disable the GPS and indicate that the current location is simulated 
extern configValuesType Configuration;
extern HardwareSerial* Serials[];
extern bool SD_Card_Present;
extern uint32_t SSSS;
extern uint32_t Minute;
extern String LogFileName;

extern HALIMU imu;

extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int DecisionEventValue;							// this a value relevant to an event
extern int DecisionEventValue2;							// this a value relevant to an event
extern VesselUsageCountersStruct VesselUsageCounters;
extern HALWingAngle WingAngleSensor;

extern int SteeringServoOutput;
extern HALPowerMeasure PowerSensor;
extern WingSailType WingSail;
extern MissionValuesStruct MissionValues;
extern char Version[];
//extern WaveClass Wave;
extern HALServo servo;

void dateTime(uint16_t* date, uint16_t* time)
{
	// V1.0 1/10/2016 John Semmens
	unsigned int _year = year();
	byte _month = month();
	byte _day = day();
	byte _hour = hour();
	byte _minute = minute();
	byte _second = second();

	// YOUR SKETCH SHOULD UPDATE THESE SIX Values
	// EACH TIME BEFORE CREATING YOUR SD CARD FILE
	*date = FAT_DATE(_year, _month, _day);
	*time = FAT_TIME(_hour, _minute, _second);
}

void LogTimeHeader(void)
{
	// V1.0 1/10/2016 John Semmens
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("YYYY"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("MM"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DD"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("HH"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("mm"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("ss"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SSSS"));
	LogFile.print(Configuration.SDCardLogDelimiter);
}

void SD_Logging_Init() {
	// V1.0 1/10/2016 John Semmens
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 3/11/2016 added flag for the presence of the SD Card

	Serial.println(F("*** Initializing SD card..."));

	// THIS LINE SETS YOUR SKETCH TO SAVE YOUR
	// TIME AND DATE INTO ANY FILE YOU CREATE.
	SdFile::dateTimeCallback(dateTime);

	if (!SD.begin(SD_Card_CS_PIN)) 
	{
		Serial.println(F("*** Initialization failed. Card may not be present."));
		Serial.println();
		SD_Card_Present = false;
	}
	else
	{
		Serial.println(F("*** Initializing SD card complete."));
		Serial.println();
		SD_Card_Present = true;
	}
}

void SD_Logging_OpenFile() {
	// V1.0 10/10/2016 John Semmens
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 30/10/2017 added IMU status to Attitude
	// V1.3 9/1/2022 changed filename to be <boot number>-<minute number>.log

	// set up 8.3 filename
	int BL = String(VesselUsageCounters.BootCounter).length();
	int ML = String(Minute).length();

	String BootString = ("000" + String(VesselUsageCounters.BootCounter)).substring(BL, BL + 3);
	String MinuteString = ("00000" + String(Minute)).substring( ML,  ML + 5);
	LogFileName = BootString + MinuteString + ".log";


	Serial.print(F("Open SD Card Logfile:"));
	Serial.println(LogFileName);
	OpenSDLogFile(LogFileName);
	delay(30);

	// Log_Location  loc - Time, Lat, Lon
	LogFile.print(F("Data"));
	LogTimeHeader();
	LogFile.print(F("Field1"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field2"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field3"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field4"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field5"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field6"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field7"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field8"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field9"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field10"));
	LogFile.println();


	// Log_Location  loc - Time, Lat, Lon
	LogFile.print(F("LOC"));
	LogTimeHeader();
	LogFile.print(F("LAT"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LON"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("COG"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SOG")); //   m/s
	LogFile.println();

	// Log_Attitude att - Time, Compass, Pitch, Roll, 
	LogFile.print(F("ATT"));
	LogTimeHeader();
	LogFile.print(F("HDG_T"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Pitch"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Roll"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Status"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("ROLL_Avg"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("HDG_Mag"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("HDG_Err"));
	LogFile.println();


	// Log_Waypoints way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon
	LogFile.print(F("WAY"));
	LogTimeHeader();
	LogFile.print(F("LAT0"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LON0"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LAT1"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LON1"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.println(F("RLB"));


	// Log_Sailing  sai - Time, CTS, HDG, BTW, WA,   CTE , Max CTE ,TackTime,TurnHDG, TargetHDG
	LogFile.print(F("SAI"));
	LogTimeHeader();
	LogFile.print(F("CTS"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("HDG_T"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("BTW"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("AWA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("CTE"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("MaxCTE"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TackDurn"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TurnHDG"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TargetHDG"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("CourseType"));
	LogFile.println();


	// Log_Navigation nav - Time, BTW, DTW, RLB,Sailable, Wing Angle, AWD
	LogFile.print(F("NAV"));
	LogTimeHeader();
	LogFile.print(F("BTW"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DTW"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("RLB"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Sailable"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("WingAngle"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("AWD"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TWD"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("HDG_T"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("CDA"));
	LogFile.println();

	LogFile.print(F("SYS"));
	LogTimeHeader();
	LogFile.print(F("Loop"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("FileSize"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DeckTemp"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Press"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("CPUTemp"));
	LogFile.println();

	LogFile.print(F("GPS"));
	LogTimeHeader();
	LogFile.print(F("LAT"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LON"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LocationAge"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SimulatedGPS"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.println(F("Loc_Valid"));

	LogFile.print(F("DEC"));
	LogTimeHeader();
	LogFile.print(F("MI"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("CS"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DE"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DER"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DEV"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DEV2"));
	LogFile.println();


	// SVO values
	LogFile.print(F("SVO"));
	LogTimeHeader();
	LogFile.print(F("Steer"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TTA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("ServoPwr"));
	LogFile.println();


	 // VPwr values
	LogFile.print(F("VPwr"));
	LogTimeHeader();
	LogFile.print(F("SolarCell_V"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SolarCell_mA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Charge_V"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Charge_mA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Discharge_V"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Discharge_mA"));
	LogFile.println();


	// Log_Usage  use - Time - 10 min counter, port rudder servo, stbd rudder, trim tab 
	LogFile.print(F("USE"));
	LogTimeHeader();
	LogFile.print(F("10min"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("PortRudder"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("StarboardRudder"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TrimTabCounter"));
	LogFile.println();


	// Log_Usage  ENV - Time - TWD, AWA
	//  Log_Environment = 0x2000
	LogFile.print(F("ENV"));
	LogTimeHeader();
	LogFile.print(F("TWD"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TWS"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("AWA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Temp"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Press"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("WingAngle"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("WA Movement"));
	LogFile.println();


	// Log the mission step  - Time, message
	LogFile.print(F("MIS"));
	LogTimeHeader();
	LogFile.print(F("mission_index"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("cmd"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("duration"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("boundary"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("controlMask"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SteerAWA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TrimTabAngle"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("waypoint.lat"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("waypoint.lng"));
	LogFile.println();

	// Log_change of parameter - Time, parameter index, 
	LogFile.print(F("PRS"));
	LogTimeHeader();
	LogFile.print("ParameterIndex");
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print("ParameterValue");
	LogFile.println();

	// Wing Sail Power
	LogFile.print(F("WSPwr"));
	LogTimeHeader();
	LogFile.print(F("SolarCell_V"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SolarCell_mA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Charge_V"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Charge_mA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Discharge_V"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Discharge_mA"));
	LogFile.println();

	// Wing Sail Monitor
	LogFile.print(F("WSMon"));
	LogTimeHeader();
	LogFile.print(F("Servo_microseconds"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Servo_microseconds_reponse"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("LastResponseTime"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Description"));
	LogFile.println();

	//// Wave Measurement Data 
	//LogFile.print(F("Wave"));
	//LogTimeHeader();
	//LogFile.print(F("MSLP hPa"));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(F("Peak hPA"));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(F("Trough hPA"));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(F("Height m"));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(F("Period s"));
	//LogFile.println();

	// Equipment Data 
	LogFile.print(F("Equip"));
	LogTimeHeader();
	LogFile.print(F("WingAngle"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.println();

	// Performance Data 
	LogFile.print(F("Perf"));
	LogTimeHeader();
	LogFile.print(F("VMG"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("VMC"));
	LogFile.println();

	// log software version and other userful data at the start of each log file
	// log the OS version to the SD Card
	SD_Logging_Event_Messsage(Version);

	// log the boot count to the SD Card
	SD_Logging_Event_Messsage("Boot: " + String(VesselUsageCounters.BootCounter));

	// log the usage values at start up.
	SD_Logging_Event_Usage();

	LogFile.flush();
}

void OpenSDLogFile(String LogFileName) {
	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.

	LogFile = SD.open(LogFileName.c_str(), FILE_WRITE);
};

void LogTime(void)
{
	// V1.0 1/10/2016 John Semmens
	LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(F("20"));
	LogFile.print(year());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(month());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(day());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(hour());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(minute());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(second());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(SSSS);
	LogFile.print(Configuration.SDCardLogDelimiter);
}

void SD_Logging_1s()
{
	// fast logging at 1 second interval
	// log data to the  SD Card data based on selected mask.
	// V1.0 4/8/2016 John Semmens
	// V1.1 3/9/2016 reorganised for logging to use a bit mask.
	// V1.2 1/11/2016 changed ATT to use True Hdg
	// V1.3 30/10/2017 added IMU status to Attitude
	// V1.4 10/12/2017 added Check_LogFileSize to this logging procedure, rather than calling separately.
	// V1.5 1/11/2021 removed mask because it wasn't used.
	// V1.6 30/10/2022 added AWD to NAV

	char FloatString[16];

	// Log_Location  loc - Time, Lat, Lon
	LogFile.print(F("LOC"));
	LogTime();
	LogFile.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.COG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.SOG_mps); // m/s
	LogFile.println();


	// Log_Attitude att - Time, Compass true, Pitch, Roll, 
	LogFile.print(F("ATT"));
	LogTime();
	LogFile.print(NavData.HDG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print((int)imu.Pitch);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print((int)imu.Roll);
	LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(imu.Algorithm_Status);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.ROLL_Avg);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.HDG_Mag);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.HDG_Err);
	LogFile.println();

	// Log_Sailing  sai - Time, CTS, HDG, BTW, WA,   CTE , Max CTE ,TackTime,
	LogFile.print(F("SAI"));
	LogTime();
	LogFile.print(NavData.CTS);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.HDG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.BTW);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.AWA);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.CTE);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.MaxCTE);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.TackDuration);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.TurnHDG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.TargetHDG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CourseTypeToString(NavData.CourseType));
	LogFile.println();


	// Log_Navigation nav - Time, BTW, DTW, BRL,Sailable,Wing angle, AWD
	LogFile.print(F("NAV"));
	LogTime();
	LogFile.print(NavData.BTW);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.DTW);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.RLB);
	LogFile.print(Configuration.SDCardLogDelimiter);
	if (NavData.IsBTWSailable)
		LogFile.print("Y");
	else
		LogFile.print("N");
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.Angle);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.AWD);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.TWD);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.HDG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.CDA);
	LogFile.println();


	// GPS state information
	LogFile.print(F("GPS"));
	LogTime();
	LogFile.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(gps.Location_Age);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(UseSimulatedVessel);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(gps.GPS_LocationIs_Valid(NavData.Currentloc));
	LogFile.println();

	// Log_ServoOut SVO values
	LogFile.print(F("SVO"));
	LogTime();
	LogFile.print(servo.ServoPulseWidth);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.TrimTabAngle);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(servo.PoweredOn);
	LogFile.println();

	// Performance Data 
	LogFile.print(F("Perf"));
	LogTimeHeader();
	LogFile.print(NavData.VMG);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.VMC);
	LogFile.println();

	LogFile.flush();

	// Check if the current log file has reached or exceeded the configured file size limit
	Check_LogFileSize();

	// if GPS Time validity changes to valid then close and open the SD Card Logfile.
	// This is helpful because the timestamps on the previous file will be wrong.
	//Check_GPS_TimeStatus();
}


void SD_Logging_1m()
{
	// medium logging at 1 minute interval
	// V1.0 8/11/2021 John Semmens

	char FloatString[16];

	LogFile.print(F("SYS"));
	LogTime();
	LogFile.print(loop_period_us);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(LogFile.size());
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(strtrim(dtostrf(WingAngleSensor.WingSailAngleSensorPort.temperature, 5, 1, FloatString)));
	LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(dtostrf(imu.Baro, 7, 1, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(InternalTemperature.readTemperatureC(), 5, 1, FloatString));
	LogFile.println();

  // VPwr values
	LogFile.print(F("VPwr"));
	LogTime();
	LogFile.print(PowerSensor.Solar_V);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(PowerSensor.Solar_I);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(PowerSensor.BatteryIn_V);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(PowerSensor.BatteryIn_I);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(PowerSensor.BatteryOut_V);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(PowerSensor.BatteryOut_I);
	LogFile.println();

	// Log_Usage  ENV - Time - TWD, AWA
	LogFile.print(F("ENV"));
	LogTime();
	LogFile.print(NavData.TWD);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.TWS);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.AWA);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(strtrim(dtostrf(WingAngleSensor.WingSailAngleSensorPort.temperature, 5, 1, FloatString)));
	LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(dtostrf(imu.Baro, 7, 1, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.Angle);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingAngleSensor.Movement);
	LogFile.println();


	//// Wave Measurement Data 
	//LogFile.print(F("Wave"));
	//LogTime();
	//LogFile.print(dtostrf(Wave.SLPressure, 7, 2, FloatString));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(dtostrf(Wave.LowPressure, 7, 2, FloatString));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(dtostrf(Wave.HighPressure, 7, 2, FloatString));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(dtostrf(Wave.height, 5, 2, FloatString));
	//LogFile.print(Configuration.SDCardLogDelimiter);
	//LogFile.print(dtostrf(Wave.period, 5, 1, FloatString));
	//LogFile.println();

	// Equipment Data 
	LogFile.print(F("Equip"));
	LogTime();
	LogFile.print(GetEquipmentStatusString(WingAngleSensor.PortStatus));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.println();

	SD_Logging_Waypoint();

	LogFile.flush();
}


void SD_Logging_Waypoint()
{
	// log waypoint details
	// V1.0 8/11/2021 John Semmens

	// this is called once per minute and also at each mission step change

	char FloatString[16];

	// Log_Waypoints way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon
	LogFile.print(F("WAY"));
	LogTime();
	LogFile.print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 10, 5, FloatString));

	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 10, 5, FloatString));

	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 10, 5, FloatString));

	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 10, 5, FloatString));

	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(NavData.RLB);
	LogFile.println();
}

void Check_LogFileSize()
{
	// Check if the current log file has reached or exceeded the configured file size limit (in kb)
	// if the limit is reached then close and open a new file.
	// V1.0 11/10/2016 John Semmens

	if (LogFile.size() >= (uint32_t(Configuration.MaxFileSize) * 1024)) {
		CloseThenOpenLogFile();
		}
}

void CloseThenOpenLogFile(void)
{
	LogFile.close();
	SD_Logging_OpenFile();
};


void SD_Logging_Event_Decisions(void)
{
	// Log_Decisions  dec - Time, command state, decision, decsion reason, and Decision event Value.
	LogFile.print(F("DEC"));
	LogTime();
	LogFile.print(StateValues.mission_index);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CommandStateToString(StateValues.CommandState));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(DecisionEventToString(DecisionEvent));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(DecisionEventReasonToString(DecisionEventReason));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(DecisionEventValue);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(DecisionEventValue2);
	LogFile.println();
}

void SD_Logging_Event_ParameterChange(int ParameterIndex, char ParameterValue[12])
{
	// Log_change of parameter - Time, parameter index, 
	LogFile.print(F("PRS"));
	LogTime();
	LogFile.print(ParameterIndex);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(ParameterValue);
	LogFile.println();
}

void SD_Logging_Event_Messsage(String message)
{
	// Log a message  - Time, message
	LogFile.print(F("MSG"));
	LogTime();
	LogFile.print(message);
	LogFile.println();
	LogFile.flush();
}

void SD_Logging_Event_MissionStep(int mission_index)
{
	char FloatString[16];

	// Log the mission step  - Time, message
	LogFile.print(F("MIS"));
	LogTime();
	LogFile.print(mission_index);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(GetMissionCommandString(MissionValues.MissionList[mission_index].cmd));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].duration);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].boundary);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].controlMask);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].SteerAWA);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].TrimTabAngle);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(MissionValues.MissionList[mission_index].waypoint.lat) / 10000000UL, 10, 5, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(MissionValues.MissionList[mission_index].waypoint.lng) / 10000000UL, 10, 5, FloatString));
	LogFile.println();
}


void SD_Logging_Event_Usage(void)
{
	// Log_Usage  USE - Time, port rudder, starboard rudder, trim tab
	LogFile.print(F("USE"));
	LogTime();
	LogFile.print(VesselUsageCounters.intervalCounter);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(VesselUsageCounters.PortRudderCounter);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(VesselUsageCounters.StarboardRudderCounter);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(VesselUsageCounters.TrimTabCounter);
	LogFile.println();
}


void SD_Logging_Event_Wingsail_Power(void)
{
	char FloatString[16];

	LogFile.print(F("WSPwr"));
	LogTime();
	LogFile.print(dtostrf(WingSail.SolarCell_V, 5, 3, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.SolarCell_mA);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(WingSail.Charge_V, 5, 3, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.Charge_mA);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(WingSail.Discharge_V, 5, 3, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.Discharge_mA);
	LogFile.println();
}

void SD_Logging_Event_Wingsail_Monitor(String Description)
{
	LogFile.print(F("WSMon"));
	LogTime();
	LogFile.print(WingSail.Servo_microseconds);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.Servo_microseconds_reponse);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(WingSail.TimeSinceLastReponse);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(Description);	
	LogFile.println();
}

