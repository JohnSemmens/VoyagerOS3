// Manage Configuration values stored in the EEPROM.
// This covers the storage structures, and also checks the validity of the stored sturcture versus the expect structure.
// 
// V1.1 11/10/2016 added MaxFileSize.
// V1.2 31/10/2016 added MagnetVariation
// V1.3 13/11/2016 added Steering PID values to Config
// V1.4 6/12/2017 added Motor control parameters
// V1.5 17/1/2018 added default compass calibration values
// V1.6 2/4/2018 added Voltage Measurement parameters.
// V1.7 31/10/2018 added TrimTabScale Scale and Offset
// V1.8 19/6/2021 Updated to revert to using the default USB serial for serial logging.
//					Added auto save of default config, if the config version flag is invalidated.
// V1.9 11/12/2021 added UseGPSInitString
// V1.10 4/2/2022 added support for different default config settings based on HardWare Config setting.

#include "configValues.h"
#include <EEPROM.h>
#include "eepromAnything.h"
#include "Mission.h"
#include "CLI.h"
#include "CommandState_Processor.h"
#include "PID_v1.h"
#include "WearTracking.h"
#include "Navigation.h"


extern configValuesType Configuration;
extern MissionCommand MissionList[MaxMissionCommands];
extern MissionValuesStruct MissionValues;
extern StateValuesStruct StateValues;
extern HardwareSerial *Serials[];
extern NavigationDataType NavData;
extern int HWConfigNumber;

extern WearCounter PortRudderUsage;
extern WearCounter StarboardRudderUsage;
extern WearCounter TrimTabUsage;
extern VesselUsageCountersStruct VesselUsageCounters;

// Calculate the base address of each structure in the EEPROM.
// This done by getting the size of the previous object and adding it to the address of the previous object.
static const int VesselUsageCountersAddress = 0;
static const int sizeof_VesselUsageCounters = sizeof(VesselUsageCounters);

static const int ConfigurationAddress = sizeof_VesselUsageCounters + VesselUsageCountersAddress;
static const int sizeof_Configuration = sizeof(Configuration);

static const int MissionValuesAddress = sizeof_Configuration + ConfigurationAddress;
static const int sizeof_MissionValues = sizeof(MissionValues);

static const int StateValuesAddress = sizeof_MissionValues + MissionValuesAddress;
static const int sizeof_StateValues = sizeof(StateValues);

static const int TotalEEPROMStorage = sizeof_Configuration + sizeof_MissionValues + sizeof_StateValues + sizeof_VesselUsageCounters;


void Save_EEPROM_VesselUsage(void)
{
	VesselUsageCounters.PortRudderCounter = PortRudderUsage.Counter ;
	VesselUsageCounters.StarboardRudderCounter = StarboardRudderUsage.Counter;
	VesselUsageCounters.TrimTabCounter = TrimTabUsage.Counter;

	EEPROM_write(VesselUsageCountersAddress, VesselUsageCounters);
};

void Load_EEPROM_VesselUsage(void)
{
	EEPROM_read(VesselUsageCountersAddress, VesselUsageCounters);
	delay(30);

	// check if the init byte has been initialised previously.
	// This is done by using an abitrary value that would not occur randomly e.g. 15. more likely uninitialised values would be either 00,FF,55,AA.
	// I have not verified this on this processor.
	// if not then it implies that it has never been saved.
	if (VesselUsageCounters.init_flag != 15)
	{
		VesselUsageCounters.init_flag = 15;
		VesselUsageCounters.intervalCounter = 0;
		VesselUsageCounters.PortRudderCounter = 0;
		VesselUsageCounters.StarboardRudderCounter = 0;
		VesselUsageCounters.TrimTabCounter = 0;
		VesselUsageCounters.BootCounter = 0;
		VesselUsageCounters.SpareCounter2 = 0;
		VesselUsageCounters.SpareCounter3 = 0;
	};

	PortRudderUsage.Counter = VesselUsageCounters.PortRudderCounter;
	StarboardRudderUsage.Counter = VesselUsageCounters.StarboardRudderCounter;
	TrimTabUsage.Counter = VesselUsageCounters.TrimTabCounter;
}

void Save_EEPROM_ConfigValues(void)
{
	// set the EEEPROM structure storage version number before saving.
	Configuration.EEPROM_Storage_Version = EEPROM_Storage_Version_Const;

	EEPROM_write(ConfigurationAddress, Configuration);
}

void Save_EEPROM_ConfigValues_LeaveVersion(void)
{
	// save the config values, but do not restore the config version.
	// This is to support forcing a restore to default values

	EEPROM_write(ConfigurationAddress, Configuration);
}

void Load_EEPROM_ConfigValues(void)
{
	EEPROM_read(ConfigurationAddress, Configuration);
	delay(30);
}

void load_EEPROM_Mission(void)
{
	// load the mission structure from the EEPROM.
	// perform a simple validation check to ensure that the number of mission steps is within the maximum allowed.
	// V1.0 8/10/2016 John Semmens

	EEPROM_read(MissionValuesAddress, MissionValues);
	delay(30);

	// check if number of mission steps looks valid; otherwise zero it out.
	if (MissionValues.mission_size > MaxMissionCommands)
		MissionValues.mission_size = 0;
}

void Save_EEPROM_Mission(void)
{
	EEPROM_write(MissionValuesAddress, MissionValues);
}

void Load_EEPROM_StateValues(void)
{
	// load the vessel state values. Perform a simple validity check by only setting current mission values if there is a mission.
	EEPROM_read(StateValuesAddress, StateValues);
	delay(30);

	if (MissionValues.mission_size == 0)
	{
		StateValues.mission_index = 0;
		StateValues.StartingMission = false;
		//StateValues.CommandState = VesselCommandStateType::vcsIdle;
	}
}

void Save_EEPROM_StateValues(void)
{
	EEPROM_write(StateValuesAddress, StateValues);
}

bool EEPROM_Storage_Version_Valid(void)
{
	// return true or false to indicate if the cureent stored data structures have a version consistent with the current software
	// V1.0 28/8/2016 John Semmens.
	return (EEPROM_Storage_Version_Const == Configuration.EEPROM_Storage_Version);
}

void Load_Config_default_values(void)
{
	// load default Config values when the stored values are invalid.
	// V1.0 28/9/2016 John Semmens.
	// V1.1 11/10/2016 added MaxFileSize.
	// V1.2 29/10/2017 remove items related to MPU-2950
	// V1.3 30/10/2017 added compass offset angle
	// V1.4 6/12/2017 added Motor speed mapping 
	// V1.5 17/1/2018 added compass calibration defaults
	// V1.6 18/2/2017 updated default values for motor speed
	// V1.7 27/4/2018 updated TargetHeadingFilterConstant following on-water trials.
	// V1.8 28/10/2018 added timezone_offset.
	// V1.9 21/2/2019 added TackingMethod
	// V1.10 11/12/2021 added UseGPSInitString
	// V1.11 4/2/2022 added support for different default config settings based on HardWare Config setting.

	Configuration.TackingMethod = ManoeuvreType::mtGybe;
	Configuration.MinimumAngleDownWind = 20; // degrees off dead downwind
	Configuration.WPCourseHoldRadius = 10; // metres radius from WP - hold course

	Configuration.RTHTimeManualControl = 300; // seconds  -- 5 min - return to home if time expires with no command in manual control.
	Configuration.DefaultMaxCTE = 20; // 20 metres. used for Return to Home for example
	Configuration.SaveStateValues = true;

	Configuration.WindAngleCalibrationOffset = 0; // degrees
	Configuration.DisplayScreenView = 'm'; // mission overview/ checklist
	Configuration.SDCardLogDelimiter = '\t';  //tab --  perhaps  ',' or '\t'
	Configuration.TargetHeadingFilterConstant = 0.02;

	Configuration.SailableAngleMargin = 25; // was 45°, changed to 35° 15/4/2020 , changed to 25° 5/1/2020

	Configuration.timezone_offset = 10; // offset to our timezone from UTC/GPS time +10 hours for EST, and +11 for EDT (summer time).

	Configuration.MagnetVariation = 12; // about 12 degrees East for Port Philip

	Configuration.MaxFileSize = 2048; //kb.  was 1024 kb 

	Configuration.Servo_Channel_Steering = 0;		// channel number
	Configuration.Servo_Channel_Steering_Stbd = 1;	// channel number
	Configuration.Servo_Channel_Motor = 3;			// channel number

	Configuration.UseGPSInitString = false;

	Configuration.DualRudder = false; // Single Rudder only

	Configuration.LoRaPort = 3; // Serial Port3
	Configuration.SatCommsPort = 4; // serial port 4
	Configuration.GPSPort = 1; // serial port 1
	Configuration.BluetoothPort = 2; // Serial Port2

	Configuration.LoRaPortBaudRate = 9600; // Baud Rate  9600 Baud
	Configuration.SatCommsPortBaudRate = 9600;   // Baud Rate
	Configuration.BTPortBaudRate = 9600; // Baud Rate

	// Steering PID
	//adjust PID 25/4/2018 8,0,0
	Configuration.pidKp = 8; // Proportional //adjust PID 25/4/2018
	Configuration.pidKi = 0; // Integral
	Configuration.pidKd = 0; // Differential
	Configuration.pidOutputmin = -400;  // steering PID output limits - microseconds - min was 1100us
	Configuration.pidOutputmax = +400;  // steering PID output limits - microseconds - max was 1900us
	Configuration.pidDirection = REVERSE; // Direction: DIRECT 0 or REVERSE 1
	Configuration.pidCentre = 1500; // 
	Configuration.SteeringFilterConstant = 0.15;

	Configuration.UseMotor = false; // set default to true during early trials

	Configuration.LoiterRadius = 15; // metres

	Configuration.TWD_Offset = 30; // degrees. about 30 degrees

	Configuration.CTE_CorrectionGain = 20; // °  degrees. apply 20° correction to the CTS when CTE/CTEmax = 1

	Configuration.GPS_PowerMode = GPS_PowerModeType::Auto_DTB;
	Configuration.DTB_Threshold = 200; // metres - Distance to Boundary threshold. move to low power nav mode when DTB exceeds threshold
	Configuration.GPS_Max_Sleep_Time = 60;  // seconds - sleep time for GPS while in low power nav mode.
	Configuration.GPS_Min_Wake_Time = 12;	// seconds - wake time for GPS while in low power nav mode. This must be longer than Settle time.
	Configuration.GPS_Setttle_Time = 10;  // seconds. Time to wait after GPS location becomes valid before using location data.
										 // this is required the because the Ebyte GPS is intially a few metres (maybe 10m) off to one side when it wakes.
										 // it then takes a few seconds to settle into the correct location.

	// set default config according to HW Config setting for the following config items.
	switch (HWConfigNumber)
	{
	case 0:
		// Voyager 3.0
		strcpy(Configuration.VesselName, "Voyager 3.0");
		Configuration.CompassOffsetAngle = 0; // degrees

		// Compass USFS Max
		// Cardinal corrections
		Configuration.CompassError000 = -05;
		Configuration.CompassError090 = +65;
		Configuration.CompassError180 = +45;
		Configuration.CompassError270 = +01;

		// Wingsail Magnetic Angle Sensor MPU9250 
		// Cardinal corrections
		Configuration.WingAngleError000 = +2;
		Configuration.WingAngleError090 = +2;
		Configuration.WingAngleError180 = -4;
		Configuration.WingAngleError270 = -6;
		// Scale factors
		Configuration.WingAngle_mXScale = 2350;
		Configuration.WingAngle_mYScale = 3650;

		// Scale and Offset for mapping trim tab angle to the Servo input signal in microseconds
		Configuration.TrimTabScale = 15; // us/degree.  
		Configuration.TrimTabOffset = 1480; // us
		Configuration.TrimTabDefaultAngle = 15; // degrees
		Configuration.MinimumAngleUpWind = 30; // degrees off head to wind. was 40. 35 seems ok. maybe 30 for V3.0.

		strcpy(Configuration.BT_MAC_Address, "113EE2A6E373");
	//	strcpy(Configuration.BT_MAC_Address, "11899aa11fa1");
		break;

	case 1:
		// Voyager 2.5/Voyager 2.6
		strcpy(Configuration.VesselName, "Voyager 2.6");
		Configuration.CompassOffsetAngle = 90; // degrees

		// Compass USFS Max
		// Cardinal corrections // valaidated as at 5/11/2022
		Configuration.CompassError000 = +20;
		Configuration.CompassError090 = -25;
		Configuration.CompassError180 = +20;
		Configuration.CompassError270 = +30; // was +60;

		// Wingsail Magnetic Angle Sensor MPU9250 
		// Cardinal corrections
		Configuration.WingAngleError000 = +3;
		Configuration.WingAngleError090 = +4;
		Configuration.WingAngleError180 = +0;
		Configuration.WingAngleError270 = +9;
		// Scale factors
		Configuration.WingAngle_mXScale = 5100;
		Configuration.WingAngle_mYScale = 7000;

		// Scale and Offset for mapping trim tab angle to the Servo input signal in microseconds
		Configuration.TrimTabScale = 15; // us/degree.  
		Configuration.TrimTabOffset = 1480; // us
		Configuration.TrimTabDefaultAngle = 15; // degrees
		Configuration.MinimumAngleUpWind = 35; // degrees off head to wind. was 40. 35 seems ok. maybe 30 for V3.0.

		strcpy(Configuration.BT_MAC_Address, "113EE2A6E37A");
		//strcpy(Configuration.BT_MAC_Address, "11899aa11fa1");
		break;

	default:;
	}
		 
};

void Load_ConfigValues()
{
	// Load the Config values from EEPROM
	// perform a simple check on the validity of the stored data by using a stored structure version number.
	// V1.0 8/10/2016 John Semmens
	// V1.1 23/10/2016 Updated to provide option to display messages.
	// V1.2 19/6/2021 Updated to revert to using the default USB serial for serial logging.

	Serial.println(F("*** Initialising Config from EEPROM"));

	Load_EEPROM_ConfigValues();
	Serial.print(F("Current EEPROM Structure Version: "));
	Serial.print(Configuration.EEPROM_Storage_Version);
	Serial.print(F(". Expected: "));
	Serial.print(EEPROM_Storage_Version_Const);
	Serial.print(F(". "));
	Serial.println((EEPROM_Storage_Version_Valid() ? "OK." : "Loading Default Values."));

	// load other structures from the EEPROM provided the structure storage version.
	if (EEPROM_Storage_Version_Valid())
	{
		Serial.print(F("Loading Mission.."));

		load_EEPROM_Mission();

		Serial.print(MissionValues.mission_size);
		Serial.println(F(" steps."));

		//Serial.print("Configuration.CommandPort:");
		//Serial.println(Configuration.CommandPort);

		// send the mission command list to the serials device.
		CLI_Processor(Configuration.LoRaPort,"mcl,");

		if (Configuration.SaveStateValues)
		{
			Serial.println(F("Loading State Values."));
			Load_EEPROM_StateValues();
		}
	}
	else
	{
		Serial.println("Loading Default Config Values...");
		// the stored configuration is invalid, so load some reasonable default values
		Load_Config_default_values();
		Serial.println("Default Config Values Loaded.");


		Save_EEPROM_ConfigValues(); // temporarily save config
		Serial.println("Default Config Values Saved.");

		// reset all mission values and flags, save 
		MissionValues.mission_size = 0;
		StateValues.mission_index = 0;
		StateValues.StartingMission = true;
		NavData.next_WP_valid = false;
		Save_EEPROM_Mission();
	}
	Serial.println(F("*** Initialising Config complete."));
	Serial.println();
};


void ReadHWConfigNumber()
{
    // read the value of the Config jumper on Digital Pin 16/A2

	// set D16 as input with internal pull up.
	pinMode(PIN_A2, INPUT_PULLUP);
	HWConfigNumber = digitalRead(PIN_A2);
}

