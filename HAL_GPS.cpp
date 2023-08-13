// 
// Hardware Abstraction Layer for GPS
// to manage Location and Time
// V1.0 16/6/2021
// V1.1 14/11/2021 added Simulated GPS
// V1.2 22/7/2023 removed GPS power controls.

#include "HAL_GPS.h"

#include "Navigation.h"
#include "location.h"
#include "configValues.h"
#include "TinyGPS++.h"
#include "DisplayStrings.h"
#include "sim_vessel.h"
#include "HAL_Time.h"
#include "TimeLib.h"

extern HALGPS gps;
extern NavigationDataType NavData;
extern HardwareSerial* Serials[];
extern configValuesType Configuration;

extern bool UseSimulatedVessel;			// flag to disable the GPS and indicate that the current location is simulated 
extern sim_vessel simulated_vessel;

extern time_t GPSTime;

extern byte GPSEnablePin;

// The TinyGPS++ object
TinyGPSPlus t_gps;

void HALGPS::Init() {
	// Initialise the GPS 
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 21/10/2018 updated for change from serial to I2C 
	// V1.3 6/4/2019 added delay at the end, because it seems solve a lock up problem.
	// V1.4 13/2/2022 added receive memory buffer as per pjrc.com tiny gps blog article

	(*Serials[Configuration.GPSPort]).begin(9600);

	// Enable the GPS - set the control pin to output and set high initially.
	//pinMode(GPSEnablePin, OUTPUT);
	//PowerMode = Configuration.GPS_PowerMode;   GPS_PowerModeType::Auto_DTB;
	Location_Age = -1;
	//Valid_Duration = -1;

	// setup buffer as 100 bytes
	static char SerialReadBuffer[100];
	(*Serials[Configuration.GPSPort]).addMemoryForRead(SerialReadBuffer, 100);

	Serial.println(F("*** Initialising GPS.."));
	EquipmentStatus = EquipmentStatusType::Unknown;

	// listen for any serial data from the GPS for up to one second.

	char GPS_Msg[50];
	unsigned int MsgIndex = 0;
	unsigned long StartMillis = millis();
	while ( ((millis()- StartMillis) < 1000)  && (MsgIndex < sizeof(GPS_Msg)- 2) )
	{
		if ((*Serials[Configuration.GPSPort]).available())
		{
			char received = (*Serials[Configuration.GPSPort]).read();
			GPS_Msg[MsgIndex++] = received;
		}
	}
	GPS_Msg[MsgIndex] = '\0'; 
	Serial.print(F("GPS Response: "));
	Serial.println(GPS_Msg);

	if (MsgIndex > 0)
		EquipmentStatus = EquipmentStatusType::Found;
	else
		EquipmentStatus = EquipmentStatusType::NotFound;

	Serial.print(F("GPS status: "));
	Serial.println(GetEquipmentStatusString(EquipmentStatus));

	Read();

	//if (Configuration.UseGPSInitString)
	//{ 
	//	Serial.println(F("GPS sending init string. "));
	//	(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_1000_10ms, sizeof(GPS_CFG_TP_1000_10ms));
	//	delay(100);
	//	(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM, sizeof(GPS_CFG_RXM_PSM)); 
	//}
	//else
	//{
	//	Serial.println(F("GPS NOT sending init string. "));
	//}

	Serial.println(F("*** Initialising GPS complete."));
	Serial.println();
};

void HALGPS::Read() {
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 14/4/2018 added GPS directly provided course and speed to Navdata object
	// V1.3 21/10/2018 updated for change from serial to I2C 

	//static long prev_Location_Age;

	Location_Age = t_gps.location.age() / 1000;

	//// clear crazy GPS location age values that can occur at start up, causing lockout problems.
	//if (Location_Age > 1000)
	//{
	//	Location_Age = 0;
	//}


	// if the location age has jumped back to 0 from the max sleep time,
	// then set the start time.
	//if (prev_Location_Age >= Configuration.GPS_Max_Sleep_Time && Location_Age ==0)
	//{
	//	Valid_Start_Time  = millis();
	//}
	//prev_Location_Age = Location_Age;

	//// if the location age is near zero (meaning location is current)
	//// then continiue calculating the valid age.
	//if (Location_Age >= 0 && Location_Age <= 2) 
	//{
	//	Valid_Duration = (millis() - Valid_Start_Time) / 1000;
	//}
	//else 
	//{
	//	Valid_Duration = 0;
	//}

	while ((*Serials[Configuration.GPSPort]).available()) //available() returns the number of new bytes available from the GPS module
	{
		t_gps.encode((*Serials[Configuration.GPSPort]).read()); //Feed the GPS parser	
	}

	// if not using a simulated GPS position (i.e. if real) then populate the NavData
	if (!UseSimulatedVessel)
	{
		NavData.Currentloc.lat = t_gps.location.lat() * 10000000UL;
		NavData.Currentloc.lng = t_gps.location.lng() * 10000000UL;
		NavData.CurrentLocTimeStamp = millis();

		// get course and speed directly from GPS
		NavData.COG = t_gps.course.deg();

		NavData.SOG_knt = (float)t_gps.speed.knots();
		NavData.SOG_mps = (float)t_gps.speed.mps();
	}
	else // populate with simulated data
	{
		NavData.Currentloc.lat = simulated_vessel.Currentloc.lat;
		NavData.Currentloc.lng = simulated_vessel.Currentloc.lng;
		NavData.CurrentLocTimeStamp = millis();

		NavData.COG = simulated_vessel.Heading;
		NavData.SOG_mps = simulated_vessel.SOG_mps;
		NavData.SOG_knt = simulated_vessel.SOG_mps * 1.94384449; // knot/mps;
	}

	// get the date and time from GPS into GPSTime object.
	TimeElements tm;
	tm.Year = t_gps.date.year() - 1970;
	tm.Month = t_gps.date.month();
	tm.Day = t_gps.date.day();
	tm.Hour = t_gps.time.hour();
	tm.Minute = t_gps.time.minute();
	tm.Second = t_gps.time.second();
	GPSTime = makeTime(tm);
	GPSTime += (Configuration.timezone_offset * SECS_PER_HOUR); // apply timezone offset
};


bool HALGPS::GPS_LocationIs_Valid(Location TestLoc) {
	// check if the GPS location is valid (or if the simulated location is valid)
	// V1.1 21/10/2018 Updated to allow GPS_Last_Loc time in ms of zero to considered ok (i.e ">=" vs ">")

	bool valid = false;

	if (UseSimulatedVessel)
	{
		// if simulated, just check for non-zero lat/lon 
		valid = (TestLoc.lat != 0 && TestLoc.lng != 0);
	}
	else
	{
		// if real GPS, check for non=zero lat/lon and valid location provided in last 10 seconds
		valid = (TestLoc.lat != 0 && TestLoc.lng != 0 && Location_Age >= 0 && Location_Age < 10000); // change 100 seconds
	}

	return valid;
};


//void HALGPS::SendConfigurationString(char *GPS_Init_String)
//{
//	(*Serials[Configuration.GPSPort]).print(GPS_Init_String);
//}

//void HALGPS::SendConfigurationString(int MsgNumber)
//{
//	switch(MsgNumber)
//	{
//	case 0: 
//		break;
//
//	case 1:
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_1000_10ms, sizeof(GPS_CFG_TP_1000_10ms));
//		break;
//
//	case 2:
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_2000_10ms, sizeof(GPS_CFG_TP_2000_10ms));
//		break;
//
//	case 3:
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_200ms, sizeof(GPS_CFG_TP_200ms));
//		break;
//
//	case 4:
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM, sizeof(GPS_CFG_RXM_PSM));
//		break;
//
//	case 5:
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM, sizeof(GPS_CFG_RXM_PSM));
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM_OO_100s, sizeof(GPS_CFG_RXM_PSM_OO_100s));
//		break;
//
//	case 6:
//		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM_OO_100s2, sizeof(GPS_CFG_RXM_PSM_OO_100s2));
//		break;
//	default:;
//	}
//}


//void HALGPS::EnableGPS(bool state)
//{
//	Enabled = state;
//	
//	if (state)
//	{
//		digitalWrite(GPSEnablePin, HIGH); // enable the GPS
//	}
//	else
//	{
//		digitalWrite(GPSEnablePin, LOW); // disable the GPS
//	}
//
//	SD_Logging_Event_GPS_Power();
//}

//void HALGPS::updatePowerState()
//{
//	// This expected to be called periodically, such as every 5 seconds.
//	// Update the state of the GPS power depending on current mode and current state of power cycle.
//	switch(Configuration.GPS_PowerMode)
//	{
//	case Off:
//		EnableGPS(false);
//		break;
//
//	case LowPower:
//		if (gps.Location_Age > Configuration.GPS_Max_Sleep_Time ||  gps.Location_Age == -1)
//		{
//			// wakeup
//			EnableGPS(true);
//		}
//
//		if (gps.Valid_Duration > Configuration.GPS_Min_Wake_Time)
//		{
//			// sleep
//			EnableGPS(false);
//		}
//		break;
//
//	case Auto_DTB:
//		if (NavData.DTB < Configuration.DTB_Threshold) // if we are close to a boundary
//		{
//			// ensure the GPS is always on when close to a boundary.
//			EnableGPS(true);
//		}
//		else
//		{
//			// this is a copy of "case LowPower:" above.
//			if (gps.Location_Age > Configuration.GPS_Max_Sleep_Time || gps.Location_Age == -1)
//			{
//				// wakeup
//				EnableGPS(true);
//			}
//
//			if (gps.Valid_Duration > Configuration.GPS_Min_Wake_Time)
//			{
//				// sleep
//				EnableGPS(false);
//			}
//		}
//		break;
//
//	case Normal:
//		EnableGPS(true);
//		break;
//
//	default:;
//	}
//}

//String HALGPS::PowerModeString()
//{
//	// return a display string representing the current GPS Power Mode.
//	String PowerModeStr;
//	switch (Configuration.GPS_PowerMode)
//	{
//	case Off:
//		PowerModeStr = F("Off");
//		break;
//
//	case LowPower:
//		PowerModeStr = F("LowPower");
//		break;
//
//	case Auto_DTB:
//		PowerModeStr = F("Auto_DTB");
//		break;
//
//	case Normal:
//		PowerModeStr = F("Normal");
//		break;
//
//	default:
//		PowerModeStr = F("unkown");
//	}
//	return PowerModeStr;
//}