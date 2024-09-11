// 
// 
//  V1.1 22/7/2023 removed GPS power controls.

#include "HAL_Display.h"
#include "HAL.h"
#include "HAL_Time.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include <SPI.h>
#include "i2c_t3.h"

#include "DisplayStrings.h"
#include "Navigation.h"
#include "configValues.h"
#include "Loiter.h"
#include "CommandState_Processor.h"

#include "Wingsail.h"
#include "WearTracking.h"
#include "location.h"

#include "sim_vessel.h"
#include "sim_weather.h"
#include "TimeLib.h"
#include "BluetoothConnection.h"
#include "InternalTemperature.h"

extern NavigationDataType NavData;
extern StateValuesStruct StateValues;
extern HALIMU imu;
extern HALGPS gps;
extern char Version[];
extern char VersionDate[];
extern time_t GPSTime;

extern HardwareSerial* Serials[];
extern bool SD_Card_Present; // Flag for SD Card Presence
extern configValuesType Configuration;
extern MissionValuesStruct MissionValues;

extern char MessageDisplayLine1[10];
extern char MessageDisplayLine2[10];

extern LoiterStruct LoiterData;

extern HALPowerMeasure PowerSensor;

extern HALWingAngle WingAngleSensor;
extern WingSailType WingSail;

extern VesselUsageCountersStruct VesselUsageCounters;

extern sim_vessel simulated_vessel;
extern int SteeringServoOutput;
extern sim_weather simulated_weather;

extern String LogFileName;
extern uint32_t Minute;
extern int HWConfigNumber;

extern byte BluetoothStatePin;
extern BTStateType BTState;
extern HALServo servo;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

static const byte OLED_Address = 0x3C;

void HALDisplay::Init()
{
	Serial.println(F("*** Initialising OLED Display."));

	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	display.begin(SSD1306_SWITCHCAPVCC, OLED_Address);  // initialize with the I2C addr 0x3C (for the 128x32)
												// init done

												// Show image buffer on the display hardware.
												// Since the buffer is intialized with an Adafruit splashscreen
												// internally, this will display the splashscreen.
	display.display();

	// Clear the buffer.
	display.clearDisplay();
	display.setTextColor(WHITE);

	Wire.beginTransmission(OLED_Address);       // slave addr
	if (Wire.endTransmission() == 0)
	{
		Serial.print(F("Display found at "));
		Serial.println(OLED_Address, HEX);
		Serial.println(F("*** Initialising OLED Display complete."));
		Serial.println();
		EquipmentStatus = EquipmentStatusType::Found;
	}
	else
	{
		Serial.println(F("*** Initialising OLED Display Failed."));
		Serial.println();
		EquipmentStatus = EquipmentStatusType::NotFound;
	}
}

void HALDisplay::Page(char page)
{

	// Update the OLED screen with data in accordance with the current logging level.
	// V1.0 10/12/2017 John Semmens
	// V1.1 20/1/2019 added Mag Accuracy to the Attitude page.
	// V1.2 18/2/2019 added Wear Statistics

	// values for LCD Logging:

	// a: Attitude
	// b: Compass details 2
	// c: Compass details
	// d: display message from the voyager base station
	// e: Equipment
	// f: 
	// g: GPS Data
	// h: Home Location
	// i: Navigation #2
	// j: Wear Statistics
	// k: Wind Situation
	// l: Loiter
	// m: Mission
	// n: Navigation
	// o: Current Mission Step
	// p: GPS Detail Data 
	// q: Sail Navigation Parameters #2
	// r: Time Comparison Screen
	// s: Situation
	// t: Timing display including millis()
	// u: Sat Comms Status
	// v: Version/Time Display 
	// w: Waypoints
	// x: Wing sail 
	// y: Wing Sail Angle Sensor
	// z: System Voltages

	// B: Bluetooth  
	//  
 	// D: Compass Cal Details
	// E: Environment
	// F: WingAngle Fault Detection
	// L: Low Power Nav
	// S: Servo details
	// U: Sail Navigation Parameters #1
	// W: Wingsail Version and Power   
	// 
 	// 0: Off
	// 1: Sailable
	// 2: TWD Calculation
	// 3: Favoured Tack Calculation
	// 4: Simulator Data
	// 5: Steering Command data
	// 6: Wingsail Monitor
	// 7: Wingsail Power
	// 8: Wave Measurement
	// 9: Logfile Details

	char MsgString[16];

	if (EquipmentStatus == EquipmentStatusType::Found)
	{
		// assume the Display may have been disabled, so enable on each command
		display.ssd1306_command(SSD1306_DISPLAYON);

		switch (page)
		{
		case '0':
			// Blank Display **********************************
			// Clear the buffer.
			display.clearDisplay();
			display.display();
			display.ssd1306_command(SSD1306_DISPLAYOFF);
			break;
	
		case 'a':
			// Attitude Display **********************************
			// display compass data  and calibration limits

			// line 1 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(2);
			display.print("M");

			display.print(wrap_360_Int(imu.Heading + Configuration.CompassOffsetAngle));
			display.print(char(0x09));
			display.print(" ");
			display.print("T");
			display.println((int)NavData.HDG);

			//	display.print(char(0x09));
			display.setTextSize(1);
			// line 3
			display.print("P");
			display.print((int)imu.Pitch);
				//	display.print(char(0x09));
			display.print("  ");
			// line 3
			display.print("R");
			display.print((int)imu.Roll);
				//	display.print(char(0x09));
			//display.print(F("  Stat:"));
			//display.println(imu.Algorithm_Status); // 8 is good

			display.setTextSize(1);
			// line 4

			//display.print("  ");
			//display.print((int)imu.Baro); 
			//display.print("hPa ");
			//display.println();

			//display.print(dtostrf(imu.Baro , 7, 1, MsgString));
			//display.print("hPa ");
			display.display();
			break;
	
		case 'b':
			// compass detail Display 2 **********************************
			// Row 1 -- 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print("IMU Hdg: ");
			display.print(NavData.HDG_Raw);

			display.print(" CPC:");
			display.print(NavData.HDG_CPC);
			display.println();

			display.print("   HDG_Mag: ");
			display.print(NavData.HDG_Mag);
			display.println();

			display.print("     Hdg_True:");
			display.println(NavData.HDG_True);

			display.print("CPC ");
			display.print(Configuration.CompassError000);
			display.print("  ");
			display.print(Configuration.CompassError090);
			display.print("  ");
			display.print(Configuration.CompassError180);
			display.print("  ");
			display.print(Configuration.CompassError270);

			display.display();
			break;

		case 'c':
			// compass detail Display 1 **********************************
			// Row 1 -- 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print("Mag Hdg:");
			display.print(NavData.HDG_Mag);

			display.print(" WA:");
			display.print(NavData.AWA);
			display.println();

			display.print("Hdg Err: ");
			display.println(NavData.HDG_Err);

			display.print("Hdg    : ");
			display.println(NavData.HDG);

			display.print("COG: ");
			display.print(NavData.COG);
			display.print(" SOGav: ");
			display.print(NavData.SOG_Avg);
			display.display();
			break;

		case 'd':
			// Display Message  **********************************
			// d = display
			// Row 1 -- line 1 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(2);
			display.println(MessageDisplayLine1);
			// Row 2 -- line 2
			display.println(MessageDisplayLine2);
			display.display();
			break;


		case 'e':
			// Equipment Display **********************************
			display.clearDisplay();
			display.setTextSize(2);
			display.setCursor(0, 0);

			// Row 1 
			display.print(F("SDC:"));
			if (SD_Card_Present)
				display.println(F(" OK"));
			else
				display.println(" Fail");

			// Row 2 -- WingSailAngleSensor
			display.print(F("WA :"));
			if (WingAngleSensor.PortStatus)
				display.print(F(" OK"));
			else
				display.print(F(" Fail"));
			display.display();
			break;

		case 'f':
			break;

		case 'g':
			// GPS Display **********************************
			// Row 1 -- current lat/lon
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(2);
			display.print("P");
			display.println(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));
			display.print(" ");
			display.println(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));

			//// Row 2 -- COG/SOG
			//display.print("COG:");
			//display.print(NavData.COG);
			//display.print("  ");

			//display.print("SOG:");
			//display.println(NavData.SOG);

			display.display();
			break;
	

		case  'h':
			// Home Location Display **********************************
			// h = Home Location 
			// Row 1 -- Home 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(2);
			display.print("H");

			// Row 2 -- Home lat/lon
			display.println(dtostrf(float(StateValues.home.lat) / 10000000UL, 9, 5, MsgString));

			display.setTextSize(1);
			if (StateValues.home_is_set)
			{
				display.print("Yes");
			}
			else
			{
				display.print("NO");
			}

			display.setTextSize(2);
			display.println(dtostrf(float(StateValues.home.lng) / 10000000UL, 9, 5, MsgString));

			display.display();
			break;


		case 'i':
			// Navigation Display #2 **********************************
			// Row 1 -- 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			display.print(F("HDG: "));
			display.print(NavData.HDG);
			display.print(" ");
			display.print(F("THG:"));
			display.print(NavData.TargetHDG);
			display.println();

			// Row 2 -- 

			display.print(F("DTW: "));
			display.print(NavData.DTW);
			display.print(" ");
			display.print(F("MaxCTE:"));
			display.print(NavData.MaxCTE);
			display.println();

			// Row 3 --
			display.print(F("BTW: "));
			display.print(NavData.BTW);
			display.print(" ");
			display.print(F("CTE: "));
			display.print(NavData.CTE);
			display.println();

			// Row 4 -- DTW,CTE
			display.print(F("CTS: "));
			display.print(NavData.CTS);
			display.print(" ");
			display.print(F("Adj: "));
			display.print(NavData.CTE_Correction);
			display.println();

			display.display();
			break;


		case 'j':
			// Display Wear Statistics   **********************************
			// j = Wear Statistics
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 
			display.print(F("min: "));
			display.print(VesselUsageCounters.intervalCounter * 10);
			display.print(F(" boot: "));
			display.print(VesselUsageCounters.BootCounter);
			display.println();

			// Row 2 
			display.print(F("Port Rudder:"));
			display.print(VesselUsageCounters.PortRudderCounter);
			display.println();

			// Row 3 
			display.print(F("Stbd Rudder:"));
			display.print(VesselUsageCounters.StarboardRudderCounter);
			display.println();

			// Row 4 
			display.print(F("Trim Tab:"));
			display.print(VesselUsageCounters.TrimTabCounter);
			display.println();

			display.display();
			break;


		case 'k':
			// Wind Situation Display **********************************
			// k=Wind Situation
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Angle
			display.print(F("WS Ang:"));
			display.print(WingAngleSensor.Angle);

			display.print(F(" AWA:"));
			display.print(NavData.AWA);
			display.println();

			// Row 2 HDG
			display.print(F("HDG:"));
			display.print(NavData.HDG);

			display.print(F(" "));
			display.print(CourseTypeToString(NavData.ManoeuvreState));

			display.println();

			// Row 3 -- TWD TWS
			display.print(F("TWD: "));
			display.print((int)NavData.TWD);
			display.print(F(" TWS: "));
			display.print((int)NavData.TWS);
			display.print(F("m/s"));
			display.println();

			// Row 4 --  CTS
			display.print(F("TurnHDG:"));
			display.print(NavData.TurnHDG);
			display.print(F(" CTS:"));
			display.print(NavData.CTS);
			display.println();

			display.display();
			break;

		case 'l':
			// Display Message  **********************************
			// l = Loiter
			// Row 1 -- Loiter State
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			display.print(F("Loiter: "));
			switch (LoiterData.LoiterState)
			{
			case lsNotLoitering:
				display.println(F("NotLoitering"));
				break;

			case lsApproach:
				display.println(F("Approach"));
				break;

			case lsPortSide:
				display.println(F("PortSide"));
				break;

			case lsStarboardSide:
				display.println(F("StbdSide"));
				break;
			default:;
			}

			// Row 2 -- Loiter Point
			display.print("L");
			display.print(dtostrf(float(LoiterData.loiterCentreLocation.lat) / 10000000UL, 9, 5, MsgString));

			display.print("  ");
			display.print(dtostrf(float(LoiterData.loiterCentreLocation.lng) / 10000000UL, 9, 5, MsgString));

			// Row 3 -- Bearing and Distance
			display.print("BTW:");
			display.print(LoiterData.BTW);
			display.print(" Dist:");
			display.println(LoiterData.DTW);

			// Row 4 -- Manoeuvre and Turn Heading 
			display.print("Move:");
			switch (NavData.Manoeuvre)
			{
			case mtNotDefined:
				display.print("N/A");
				break;
			case mtTack:
				display.print("Tack");
				break;
			case mtGybe:
				display.print("Gybe");
				break;
			default:;
			}
			display.print(" TurnHDG:");
			display.println(NavData.TurnHDG);

			display.display();
			break;

		case 'm':
			// Mission Display **********************************
			// m = Command
			// Row 1 -- BTW, DTW
			display.clearDisplay();
			display.setTextSize(1);
			display.setCursor(0, 0);

			display.print(F("BTW:"));
			display.print(NavData.BTW);
			display.print(" ");
			display.print(F("DTW:"));
			display.println(NavData.DTW);

			display.setTextSize(1);

			// Row 2--  HDG True
			display.print(F("HDG:"));
			display.print((int)NavData.HDG);
			display.print(" ");

			display.print(F("LOC: "));
			if (gps.GPS_LocationIs_Valid(NavData.Currentloc)) {
				display.print("OK");
			}
			else {
				display.print("NO");
			}
			display.println();

			// Row 3--  Mission Index
			display.print(F("MI:"));
			display.print(StateValues.mission_index);
			display.print("/");
			display.print(MissionValues.mission_size);
			display.print(" ");
			display.print(F("MaxCTE:"));
			display.print(NavData.MaxCTE);
			display.println();

			// Row 4 -- Command State
			display.print(StateValues.CommandState);
			display.print(" ");
			switch (StateValues.CommandState)
			{
			case  vcsIdle:
				display.print(F("Idle          "));
				break;

				//case  vcsFullManual:
				//	display.print(F("Full Manual   "));
				//	break;

				//case  vcsPartialManual:
				//	display.print(F("Part Manual   "));
				//	break;

			case  vcsResetMissionIndex:
				display.print(F("ResetMissionIndex"));
				break;

			case  vcsSetHome:
				display.print(F("Set Home       "));
				break;

			case  vcsSteerMagneticCourse:
				display.print(F("Steer Compass "));
				display.print(StateValues.SteerCompassBearing);
				break;

			case  vcsSteerWindCourse:
				display.print(F("Steer Wind "));
				display.print(StateValues.SteerWindAngle);
				display.print(" ");
				display.print(Configuration.TrimTabDefaultAngle);
				break;

			case  vcsFollowMission:
				display.print(F("Follow Mission"));
				break;

			case  vcsReturnToHome:
				display.print(F("Return To Home"));
				break;

			case vcsLoiter:
				display.print(F("Loiter Here    "));
				break;

			default:
				display.print(F("Unknown"));
			}
			display.display();
			break;


		case 'n':
			// Navigation Display **********************************

			// Row 1 -- current lat/lon
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));

			display.print(" ");
			display.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));
			display.println();

			// Row 2 -- prev WP
			display.print(F("THG:"));
			display.print(NavData.TargetHDG);
			display.print("  ");

			display.print(F("BTW:"));
			display.print(NavData.BTW);
			display.print(" ");

			display.print("M:");
			display.println(StateValues.mission_index);

			// Row 3 -- next WP
			display.print(F("HDG: "));
			display.print(NavData.HDG);
			display.print("  ");

			display.print(F("Past WP:"));
			if (NavData.PastWP)
			{
				display.print(F("YES"));
			}
			else
			{
				display.print(F("NO "));
			}
			display.println();

			// Row 4 -- DTW,CTE
			display.print(F("DTW: "));
			display.print(NavData.DTW);
			display.print("  ");

			display.print(F("CTE: "));
			display.println(NavData.CTE);

			display.display();
			break;

		case 'o':
			// Current Mission Step Display **********************************
			// Row 1 -- mission index and duration
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print("M:");
			display.print(StateValues.mission_index);

			display.print(" Cmd:"); // 
			display.print(GetMissionCommandString(MissionValues.MissionList[StateValues.mission_index].cmd));
			display.println();

			// Row 2 -- line 2
			display.print("Dur:"); // in minutes
			display.print(MissionValues.MissionList[StateValues.mission_index].duration);

			display.print(" MT:  "); // in seconds
			display.print((millis() - MissionValues.MissionCommandStartTime) / 1000);
			display.println();

			// Row 3 -- line 3
			display.print("AWA: ");
			display.print(MissionValues.MissionList[StateValues.mission_index].SteerAWA);
			display.print(" TTA: ");
			display.print(MissionValues.MissionList[StateValues.mission_index].TrimTabAngle);
			display.println();

			// Row 4 -- 
			display.print("C:");
			display.print(NavData.CTS);
			display.print(" H:");
			display.print(NavData.HDG);
			display.print(" A:");
			display.print(NavData.AWA);
			display.println();

			display.display();
			break;

		case 'p':
			// GPS Detail Display **********************************

			// Row 1 -- current lat/lon
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));
			display.print(" ");
			display.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));
			display.println();

			// Row 2 -- COG/SOG
			display.print("COG:");
			display.print(NavData.COG);
			display.print(" ");

			display.print("SOG:");
			display.print(NavData.SOG_mps);
			display.println(" m/s");

			// Row 3 -- Time date
			display.print(hour());
			display.print(":");
			display.print(minute());
			display.print(":");
			display.print(second());

			display.print(" ");

			display.print(day());
			display.print("/");
			display.print(month());
			display.print("/");
			display.println(year());

			// Row 4 -- state
		//	display.print("state:");
		//	display.print(gps.PowerModeString());

			display.display();
			break;

		case 'q':
			// 	Sail Navigation Parameters #2 **********************************
			// q: Sail Navigation Parameters
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- BTW AWA
			display.print(F("TWD:"));
			display.print(NavData.TWD);

			display.print(F(" CTE:"));
			display.print(NavData.CTE);
			display.println();

			// Row 2 -- 
			display.print(F("PL:"));
			display.print(NavData.PortLayline);
			display.print(" ");
			display.print(NavData.PortLaylineRunning);

			display.print(F(" SL:"));
			display.print(NavData.StarboardLayline);
			display.print(" ");
			display.print(NavData.StarboardLaylineRunning);
			display.println();

			// Row 3 --
			display.print(F("CTS:"));
			display.print(NavData.CTS);

			display.print(F(" Tack:"));
			switch (NavData.CourseType)
			{
			case SteeringCourseType::ctDirectToWayPoint:
				display.print(F("Direct"));
				break;

			case SteeringCourseType::ctPortTack:
				display.print(F("Port"));
				break;

			case SteeringCourseType::ctStarboardTack:
				display.print(F("Stdb"));
				break;

			case SteeringCourseType::ctPortTackRunning:
				display.print(F("PortRun"));
				break;

			case SteeringCourseType::ctStarboardTackRunning:
				display.print(F("StdbRun"));
				break;

			default:;
			}
			display.println();

			// Row 4 --  
			display.print(F("BTW:"));
			display.print(NavData.BTW);

			display.print(F(" Sailable:"));
			if (NavData.IsBTWSailable)
				display.print("Y");
			else
				display.print("N");

			display.println();
			display.display();
			break;

		case 'r':
			// Time Comparison Screen **********************************

			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- System Time
			display.print("SYS: ");

			display.print(hour());
			display.print(":");
			display.print(minute());
			display.print(":");
			display.print(second());

			display.print(" ");

			display.print(day());
			display.print("/");
			display.print(month());
			display.println();

			// Row 2 -- 
			display.print("GPS: ");

			TimeElements tm;
			breakTime(GPSTime, tm); // break up the GP time into pieces.

			display.print(tm.Hour);
			display.print(":");
			display.print(tm.Minute);
			display.print(":");
			display.print(tm.Second);

			display.print(" ");

			display.print(tm.Day);
			display.print("/");
			display.print(tm.Month);
			display.println();

			// Row 3 -- 

			display.print(F("LOC: "));
			if (gps.GPS_LocationIs_Valid(NavData.Currentloc)) {
				display.print("OK");
			}
			else {
				display.print("NO");
			}
			display.println();

			// Row 4 --



			display.display();
			break;

		case 's':
			// Situation Display **********************************
			// s = Situation
			//BTW, DTW,-- CTS, CTE, -- CDA, MI ,-- WA (wind angle), HDG True
			// Row 1 -- BTW, DTW
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print(F("BTW:"));
			display.print(NavData.BTW);
			display.print(" ");

			display.print(F("DTW:"));
			display.println(NavData.DTW);

			// Row 2 -- CTS, CTE
			display.print(F("CTS:"));
			display.print(NavData.CTS);
			display.print(" ");

			display.print(F("CTE: "));
			display.println(NavData.CTE);

			// Row 3 -- CDA MI
			display.print(F("CDA: "));
			display.print(NavData.CDA);
			display.print("  ");

			display.print(F("MI:"));
			display.println(StateValues.mission_index);

			// Row 4 --  HDG True, WA (wind Angle)
			display.print(F("HDG: "));
			display.print((int)NavData.HDG);
			display.print("  ");

			display.print(F("WA: "));
			display.println(NavData.AWA);

			display.display();
			break;


		case 't':
			// Display Timing information  **********************************
			// t = timing
			// Row 1 -- line 1 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			display.print("millis(): ");  // in seconds
			display.println(millis() / 1000);

			// Row 2 -- line 2
			display.print("MC Start: "); // in seconds
			display.println(MissionValues.MissionCommandStartTime / 1000);

			// Row 3 -- line 3
			display.print("Elapsed:  "); // in seconds
			display.println((millis() - MissionValues.MissionCommandStartTime) / 1000);

			// Row 4 -- 
			display.print("Duration m: "); // in minutes
			display.println(MissionValues.MissionList[StateValues.mission_index].duration);
			display.display();
			break;

		//case 'u':
		//	// 	Sat Comms **********************************
		//	// u: Sail Navigation Parameters
		//	display.clearDisplay();
		//	display.setCursor(0, 0);
		//	display.setTextSize(1);

		//	// Row 1 -- astronode
		//	display.print("RSSI:");
		//	display.print(SatComm.RSSI);
		//	display.print(" Msgs:");
		//	display.print(SatComm.OutboundMsgCount);
		//	display.print(" Next:");
		//	display.print(SatComm.timeToNextSat);
		//	display.println();

		//	// Row 2 -- 
		//	char timestamp[20];
		//	strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&SatComm.timeValue));
		//	display.print(timestamp);
		//	display.println();

		//	// Row 3 --
		//	display.print("Cmd: ");
		//	display.print(SatComm.LastCommand);
		//	display.println();

		//	// Row 4 --  
		//	//display.print(SatComm.TestCounter);
		//	display.display();
		//	break;

		case 'v':
			// Version Display **********************************
			// display Version Information for Startup Screen
			display.clearDisplay();
			display.setTextSize(2);
			display.setCursor(0, 0);
			display.println(Version);
			display.println(VersionDate);
			display.display();
			break;

		case 'w':
			// Waypoint Display **********************************
			// w = Waypoints
			// Row 1 -- current lat/lon
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);
			display.print("L");
			display.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));

			display.print("  ");
			display.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));

			// Row 2 -- prev_WP lat/lon

			display.print("P");
			display.print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 9, 5, MsgString));

			display.print("  ");
			display.print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 9, 5, MsgString));

			// Row 3 -- next_WP
			display.print("N");
			display.print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 9, 5, MsgString));

			display.print("  ");
			display.print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 9, 5, MsgString));

			// Row 4
			display.print("M:");
			display.print(StateValues.mission_index);
			display.print("   ");

			display.print(F("Past WP:"));
			if (NavData.PastWP)
			{
				display.print("YES");
			}
			else
			{
				display.print("NO ");
			}
			display.display();
			break;

		case 'x':
			// Wingsail Display **********************************
			// x= WingSail 
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Angle
			display.print(F("WS Angle:"));
			display.print(WingSail.Angle); //WingSailAngleSensor.MagneticAngle

			// show which tack we're on
			display.print(" ");
			switch (WingSail.Tack)
			{
			case wsPortTack:
				display.print(F("Port Tk"));
				break;
			case wsStarboardTack:
				display.print(F("Stbd Tk"));
				break;
			case wsHeadToWind:
				display.print(F("Hd to Wnd"));
				break;
			default:;
			}
			display.println();

			//// Row 2 -- Wingsail state (fwd/idle/rev)
			display.print(F("State: "));
			switch (WingSail.State)
			{
			case wsForward:
				display.print(F("Forward"));
				break;
			case wsIdle:
				display.print(F("Idle"));
				break;
			case wsReverse:
				display.print(F("Reverse"));
				break;
			default:;
			}
			display.println();

			//// Row 3 -- Trim Tab Angle
			display.print(F("Tab Angle: "));
			display.print(WingSail.TrimTabAngle);
			display.println();

			//// Row 4 -- Servo us
			display.print(F("Servo us: "));
			display.print(WingSail.Servo_microseconds);
			display.println(" ");

			display.display();
			break;

		case 'y':
			// Wingsail Angle Sensor Display **********************************
			// y= WingSail AngleSensor
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Angle
			display.print(F("WS Angle:"));
			display.print(WingAngleSensor.Angle);
			display.print(F(" Dv:"));
			display.print(WingAngleSensor.Deviation);
			display.println();

			// Row 2 -- current mx / my
			display.print("mx: ");
			display.print((int)WingAngleSensor.WingSailAngleSensorPort.WingSailAngleSensor->mx);
			display.print(" my: ");
			display.println((int)WingAngleSensor.WingSailAngleSensorPort.WingSailAngleSensor->my);

			//// Row 3 --  Scale XY
			display.print("Scale X:");
			display.print(Configuration.WingAngle_mXScale);
			display.print(" Y:");
			display.print(Configuration.WingAngle_mYScale);
			display.println();

			//// Row 4 -- display cal mode
			display.print(F("Cal mode: "));
			if (WingAngleSensor.WingSailAngleSensorPort.WingSailAngleSensor->MagneticCompassCalibrationMode)
				display.print(F("Yes"));
			else
				display.print(F("No"));
			display.println();

			display.display();
			break;

		case 'z':
			// Display System Voltages   **********************************
			// z = Voltages
			display.clearDisplay();
			display.setTextSize(1);
			display.setCursor(0, 0);

			// Row 1 -- Voyager Power
			display.println("Voyager Power:");

			// Row 2 -- line 2
			display.print("Solr:");
			display.print(dtostrf(PowerSensor.Solar_V, 6, 2, MsgString));
			display.print("V");
			display.print(dtostrf(PowerSensor.Solar_I, 6, 1, MsgString));
			display.println("mA");

			// Row 3 -- line 3
			display.print("B In:");
			display.print(dtostrf(PowerSensor.BatteryIn_V, 6, 2, MsgString));
			display.print("V");
			display.print(dtostrf(PowerSensor.BatteryIn_I, 6, 1, MsgString));
			display.println("mA");

			// Row 4 -- line 4
			display.print("BOut:");
			display.print(dtostrf(PowerSensor.BatteryOut_V, 6, 2, MsgString));
			display.print("V");
			display.print(dtostrf(PowerSensor.BatteryOut_I, 6, 1, MsgString));
			display.println("mA");

			display.display();
			break;

//******************************************************************************************
		// Uppercase Letters
		// 
		// Bluetooth Status Display **********************************
		case 'B':
				display.clearDisplay();
				display.setCursor(0, 0);
				display.setTextSize(1);

				// Row 1 -- Bluetooth Status
				display.print(F("BT Port "));
				display.print(Configuration.BluetoothPort);
				display.print(": ");
				display.print(Configuration.BTPortBaudRate);
				display.println(" Baud");

				// Row 2 -- Bluetooth
				display.print(F("Status Pin: "));
				display.print(BluetoothStatePin);
				display.print(": ");
				display.println(digitalRead(BluetoothStatePin));

				// Row 3 -- Bluetooth status
				display.print(F("State:"));
				display.print(BTState);
				display.print(" ");
				display.println(GetBTStatus(BTState));

				// Row 4 -- Bluetooth MAC
				display.print("MAC: ");
				display.println(Configuration.BT_MAC_Address);

				display.display();
				break;	

				// Compass Calibration Detail Display **********************************
		case 'D':
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- CPU Temp
			display.print("Comp Cal Enabled:");

			imu.compass.CalibrateMode = true; // ** enable calibration when the page is displayed **

			if (imu.compass.CalibrateMode)
				display.print("y");
			else
				display.print("n");

			display.println();

			// Row 2 -- min
			display.print("Min:");
			display.print(imu.compass.cal_running_min.x);
			display.print(",");
			display.print(imu.compass.cal_running_min.y);
			display.print(",");
			display.print(imu.compass.cal_running_min.z);
			display.println();

			// Row 3 -- current
			display.print("imu:");
			display.print(imu.compass.magnetometer.x);
			display.print(",");
			display.print(imu.compass.magnetometer.y);
			display.print(",");
			display.print(imu.compass.magnetometer.z);
			display.println();

			// Row 4 -- max
			display.print("Max:");
			display.print(imu.compass.cal_running_max.x);
			display.print(",");
			display.print(imu.compass.cal_running_max.y);
			display.print(",");
			display.print(imu.compass.cal_running_max.z);
			display.println();

			display.display();
			break;




		// Environment ***********************
		case 'E':
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- CPU Temp
			display.print("CPU:  ");
			display.print(dtostrf(InternalTemperature.readTemperatureC(), 5, 1, MsgString));
			display.print("C");
			display.println();

			// Row 2 - Wing Angle Sensor Temp
			display.print("Deck: ");
			display.print(dtostrf(WingAngleSensor.WingSailAngleSensorPort.temperature, 5, 1, MsgString));
			display.print("C");
			display.println();

			// Row 3 - Baro
		//	display.print("Pres:  ");
		//	display.print(dtostrf(imu.Baro, 6, 1, MsgString));
		//	display.print("hPa");
		//	display.println();

			display.display();
			break;

		// WingAngle Fault Detection ***********************
		case 'F':
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Angle
			display.print(F("WingAngle: "));
			display.print(WingAngleSensor.Angle);
			display.println();

			// Row 2 -- Peak Angle Port
			display.print(F("PeakPort: "));
			display.print(WingAngleSensor.PeakAnglePort);
			display.println();

			// Row 3 -- Peak Angle Starboard
			display.print(F("PeakStbd: "));
			display.print(WingAngleSensor.PeakAngleStbd);
			display.println();


			// Row 4 -- Angle Movement
			display.print(F("Mvmt: "));
			display.print(WingAngleSensor.Movement);
			display.print(F(" "));
			display.print(GetEquipmentStatusString(WingAngleSensor.PortStatus));
			display.println();

			display.display();
			break;


		// Low Power Nav Display **********************************
		case 'L':
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Max CTE, CTE

			display.print(F("MaxCTE:"));
			display.print(NavData.MaxCTE);
			display.print(" ");
			display.print(F("CTE: "));
			display.print(NavData.CTE);		
			display.println();

			// Row 2 -- DTW, DTB

			display.print(F("DTW: "));
			display.print(NavData.DTW);
			display.print(" ");
			display.print(F("DTB: "));
			display.print(NavData.DTB);
			display.println();

			// Row 3 --
			//display.print(F("GPS: "));
			//if (gps.Enabled)
			//	display.print("On ");
			//else
			//	display.print("Off");
			//display.print(" ");
			//display.print(F("Valid:"));
			//display.print(gps.Valid_Duration);
			//display.print("s");
			display.println();

			// Row 4 --
			display.print(F("LOC:"));
			if (gps.GPS_LocationIs_Valid(NavData.Currentloc)) {
				display.print("Y");
			}
			else {
				display.print("N");
			}

			display.print(F(" LocAge:"));
			display.print(gps.Location_Age);
			//display.print(F(" S:"));
		//	display.print(Configuration.GPS_Setttle_Time);

			display.println();

			display.display();
			break;

		// steering Servo details
		case 'S':
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Servo Pulse width
			display.print(F("Servo: "));
			display.print(servo.ServoPulseWidth);
			display.print(F("us"));
			display.println();

			// Row 2 -- Battery Current
			display.print(F("Batt: "));
			display.print(dtostrf(PowerSensor.BatteryOut_I, 6, 1, MsgString));
			display.print("mA");
			display.println();

			// Row 3 -- 
			display.print(F("Power: "));
			display.print((servo.PoweredOn)?"On":"Off");   // (condition) ? expressionTrue : expressionFalse;
			display.println();

			display.display();
			break;


		case 'U':
			// 	Sail Navigation Parameters **********************************
			// u: Sail Navigation Parameters
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- BTW AWA
			display.print(F("BTW:"));
			display.print(NavData.BTW);

			display.print(F(" Sailable:"));
			if (NavData.IsBTWSailable)
				display.print("Y");
			else
				display.print("N");
			display.println();

			// Row 2 -- 
			display.print(F("TWD:"));
			display.print(NavData.TWD);

			display.print(F(" CTS:"));
			display.print(NavData.CTS);

			display.println();

			// Row 3 --
			display.print(F("AWA:"));
			display.print(NavData.AWA);

			display.print(" ");
			switch (NavData.PointOfSail)
			{
			case PointOfSailType::psNotEstablished:
				display.print(F("N/A"));
				break;

			case PointOfSailType::psPortTackBeating:
				display.print(F("Port Beat"));
				break;

			case PointOfSailType::psPortTackRunning:
				display.print(F("Port Run"));
				break;

			case PointOfSailType::psStarboardTackBeating:
				display.print(F("Stbd Beat"));
				break;

			case PointOfSailType::psStarboardTackRunning:
				display.print(F("Stbd Run"));
				break;

			default:;
			}
			display.println();

			// Row 4 --  
			display.print(F("HDG:"));
			display.print(NavData.HDG);
			display.print(F(" AWATW: "));
			display.print((int)NavData.WindAngleToWaypoint);
			display.println();
			display.display();
			break;



		// Wingsail Version and Power Display **********************************
		case 'W':
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- wing sail
			display.print(F("Wing: "));
			display.println();

			// Row 2 -- version
			display.print(WingSail.VersionDate);
			display.println();
			 
			// Row 3 -- power

			display.print(dtostrf(WingSail.Discharge_V, 6, 2, MsgString));
			display.print("V ");
			display.print(dtostrf(WingSail.Discharge_mA, 6, 1, MsgString));
			display.print("mA");
			display.println();

			display.display();
			break;

		case  '1':
			// IsSailable Display **********************************
			// 1= IsSailable
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Angle
			display.print(F("AWATW: "));
			display.print((int)NavData.WindAngleToWaypoint);

			display.print(F(" AWA:"));
			display.print(NavData.AWA);
			display.println();

			// Row 2 HDG
			display.print(F("HDG:"));
			display.print(NavData.HDG);

			display.print(F(" Margin: "));
			display.print(Configuration.SailableAngleMargin);

			display.println();

			// Row 3 -- Config
			display.print(F("MinUp: "));
			display.print(Configuration.MinimumAngleUpWind);
			display.print(F(" MinDn: "));
			display.print(Configuration.MinimumAngleDownWind);
			display.println();

			// Row 4 --  
			display.print(F("BTW:"));
			display.print(NavData.BTW);

			display.print(F(" Sailable:"));
			if (NavData.IsBTWSailable)
				display.print("Y");
			else
				display.print("N");
			display.println();
			display.display();
			break;

		case '2':
			// TWD True Wind Direction Calculation Display **********************************
			// 2 = TWD Calculation
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Angle, AWA
			display.print(F("WS Ang:"));
			display.print(WingAngleSensor.Angle);

			display.print(F(" AWA:"));
			display.print(NavData.AWA);
			display.println();

			// Row 2 HDG AWD
			display.print(F("HDG:"));
			display.print(NavData.HDG);

			display.print(F(" AWD:"));
			display.print(NavData.AWD);
			display.println();

			// Row 3 -- TWD 
			display.print(F("TWD: "));
			display.print((int)NavData.TWD);
			display.println();

			// Row 4 --  TWD Offset
			display.print(F("TWD Offset:"));
			display.print(NavData.TWD_Offset);
			display.println();
			display.display();
			break;

		case '3':
			//Favoured Tack Calculation  **********************************
			// 3=Favoured Tack Calculation
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- HDG, BTW
			display.print(F("HDG:"));
			display.print(NavData.HDG);
			display.print(F(" BTW:"));
			display.print(NavData.BTW);
			display.print(F(" PB Hold:"));
			display.print(NavData.PastBoundaryHold ? "Y" : "N");
			display.println();

			// Row 2 TWD
			display.print(F("TWD: "));
			display.print((int)NavData.TWD);
			display.println();

			// Row 3 Favoured Tack
			display.print(F("Fav Tack: "));
			switch (NavData.FavouredTack)
			{
			case SteeringCourseType::ctDirectToWayPoint:
				display.print(F("Direct"));
				break;

			case SteeringCourseType::ctPortTack:
				display.print(F("Port"));
				break;

			case SteeringCourseType::ctStarboardTack:
				display.print(F("Stdb"));
				break;

			case SteeringCourseType::ctPortTackRunning:
				display.print(F("PortRun"));
				break;

			case SteeringCourseType::ctStarboardTackRunning:
				display.print(F("StdbRun"));
				break;

			default:;
			}

			display.println();

			// Row 4 --  Course Type
			display.print(F("Course Type: "));
			switch (NavData.CourseType)
			{
			case SteeringCourseType::ctDirectToWayPoint:
				display.print(F("Direct"));
				break;

			case SteeringCourseType::ctPortTack:
				display.print(F("Port"));
				break;

			case SteeringCourseType::ctStarboardTack:
				display.print(F("Stdb"));
				break;

			case SteeringCourseType::ctPortTackRunning:
				display.print(F("PortRun"));
				break;

			case SteeringCourseType::ctStarboardTackRunning:
				display.print(F("StdbRun"));
				break;

			default:;
			}
			display.println();
			display.display();
			break;

		case '4':
			// Simulation  Data Display **********************************
			// 1= IsSailable
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- time
			display.print(F("ms: "));
			display.print(simulated_vessel.update_time_ms);

			display.println();

			// Row 2 HDG
			display.print(F("HDG:"));
			display.print(NavData.HDG);

			display.print(F(" Wind:"));
			display.print(simulated_weather.WindDirection);
			display.println();

			// Row 3 -- steeering
			display.print(F("SVO:"));
			display.print(SteeringServoOutput);
			display.print(F(" Ctr:"));
			display.print((int)Configuration.pidCentre);
			display.println();

			// Row 4 --  
			display.print(F("SOG: "));
			display.print(simulated_vessel.SOG_mps);
			display.print(F("WS Angle:"));
			display.print(WingSail.Angle);
			display.println();
			display.display();
			break;

		case '5':
			// Steering Command  Data Display **********************************
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 --CTS
			display.print(F("CTS: "));
			display.print(NavData.CTS);
			display.print(" ");


			display.println();

			// Row 2 -- Manoeuvre and Turn Heading 
			display.print("Mv:");
			display.print(CourseTypeToString(NavData.ManoeuvreState).substring(0,4));

			display.print(" TnHDG:");
			display.print(NavData.TurnHDG);
			display.println();

			// Row 3 Targethdg -  HDG

			display.print(F("TgHDG:"));
			display.print(NavData.TargetHDG);
			display.print("  ");

			display.print(F("HDG:"));
			display.print(NavData.HDG);
			display.println();

			// Row 4 --  
			display.print(F("WS Angle:"));
			display.print(WingSail.Angle);
			display.println();

			display.display();
			break;


		case '6':
			// Wingsail Monitor  Data Display **********************************
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Servo command - servo response
			display.print(F("Set: "));
			display.print(WingSail.Servo_microseconds);
			display.print(" Get:");
			display.print(WingSail.Servo_microseconds_reponse);

			display.println();

			// Row 2 --  
			display.print("Cmd Time: ");
			display.print( (millis() - WingSail.LastCommandTime) / 1000 );
			display.println();

			// Row 3 --
			display.print("Req Time: ");
			display.print( (millis() - WingSail.LastRequestTime) / 1000 );

			display.println();
			display.print("Rsp Time: ");
			display.print((millis() - WingSail.LastResponseTime) / 1000);

			// Row 4 --  
			display.println();

			display.display();
			break;


		case '7':
			// Wingsail Power Monitor  Data Display **********************************
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 -- Wingsail Power
			display.println("Wingsail Power:");
			// Row 2 -- line 2
			display.print("Solr:");
			display.print(dtostrf(WingSail.SolarCell_V, 6, 3, MsgString));
			display.print("V");
			display.print(dtostrf(WingSail.SolarCell_mA, 6, 1, MsgString));
			display.println("mA");

			// Row 3 -- line 3
			display.print("B In:");
			display.print(dtostrf(WingSail.Charge_V, 6, 3, MsgString));
			display.print("V");
			display.print(dtostrf(WingSail.Charge_mA, 6, 1, MsgString));
			display.println("mA");

			// Row 4 -- line 4
			display.print("BOut:");
			display.print(dtostrf(WingSail.Discharge_V, 6, 3, MsgString));
			display.print("V");
			display.print(dtostrf(WingSail.Discharge_mA, 6, 1, MsgString));
			display.println("mA");

			display.display();
			break;


		case '8':
			// Wave Measurement  Data Display **********************************
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 --  Wave Measurement
			display.print("Peak:  ");
			//display.print(dtostrf(Wave.LowPressure, 7, 2, MsgString));
			display.println("hPa");

			// Row 2 -- line 2
			display.print("MSLP:  ");
			//display.print(dtostrf(Wave.SLPressure, 7, 2, MsgString));
			display.println("hPa");

			// Row 3 -- line 3
			display.print("Trough:");
			//display.print(dtostrf(Wave.HighPressure, 7, 2, MsgString));
			display.println("hPa");

			// Row 4 -- line 4
			display.print("Wave: ");
			//display.print(dtostrf(Wave.height, 5, 2, MsgString));
			display.print("m ");
			//display.print(dtostrf(Wave.period, 5, 1, MsgString));
			display.print("s");

			display.display();
			break;

		case '9':
			// logfile details  Data Display **********************************
			display.clearDisplay();
			display.setCursor(0, 0);
			display.setTextSize(1);

			// Row 1 --  Boot Number
			display.print("Boot: ");
			display.print(VesselUsageCounters.BootCounter);

			display.print("  CPU:");
			display.print(F_CPU / 1000000);
			display.print("MHz");

			display.println();

			// Row 2 -- line 2
			display.print("Minute:  ");
			display.print(Minute);
			display.println();

			// Row 3 -- line 3
			display.print("");
			display.print(LogFileName);
			display.println();

			// Row 4 -- line 4
			display.print("Config: ");
			display.print(Configuration.EEPROM_Storage_Version);
			display.println();

			display.display();
			break;



	default:;

	} //switch
	};// if (EquipmentStatus == EquipmentStatusType::Found) - real

}