// Command Processor 
// Interpret commands received through the main serial port
// V1.0 22/12/2015
// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 8/10/2016 updated to add mission command: ctLoiterUntil
// V1.3 18/10/2016 added command scg - Command State Get.
// V1.4 23/10/2016 expanded config parameter list up to 44.
// V1.5 12/11/2016 corrected logging masks to be long rather than int.
// v1.6 29/10/2017 updated for change from MPU-2950 to BNO-055
// V1.7 12/11/2017 added hlc command to clear home location
//				   added mcp command for retrieving the mission from the autopilot for plotting
// V1.8 24/1/2018 added parameter LoiterRadius for use in the "Loiter Here" command
// V1.9 2/4/2018 added Parameters for Voltage Measurement scale factors.
//					added vmg, Get Voltage Measurements
// V1.10 24/4/2018 added parameter pidCentre and bug fix for Target Heading filter constant - float not int
// V1.11 5/5/2019 added support for parameter for the default Trim Tab angle. 
// V1.12 29/6/2019 added CCS and CCG for Setting Campass Cal, and Getting Compass Status
// V1.13 16/7/2019 adding the new Mission command for Steering a Wind Angle
// V1.14 4/8/2021 updated to support full addressing
// V1.15 13/10/2021 add support for reply from wing sail
// V1.16 22/5/2022 removing relaying and addressing.

#include "CommandState_Processor.h"
#include "Mission.h"
#include "CLI.h"
#include "configValues.h"
#include "HAL_GPS.h"
#include "LEDHeartBeat.h"
#include "Navigation.h"
#include "Steering.h"
#include "HAL_WingAngle.h"
#include "HAL_Telemetry.h"
#include "HAL_SDCard.h"
#include "Wingsail.h"
#include "TelemetryMessages.h"
#include "location.h"
#include "sim_vessel.h"
#include "DisplayStrings.h"
#include "LoRaManagement.h"
#include "HAL_Time.h"
#include "HAL_SatCom.h"

extern NavigationDataType NavData;
extern HALGPS gps;
extern char Version[];
extern char VersionDate[];
extern configValuesType Configuration;
extern MissionValuesStruct MissionValues;
extern StateValuesStruct StateValues;
extern bool UseSimulatedVessel;
extern HardwareSerial *Serials[];

extern char MessageDisplayLine1[10];
extern char MessageDisplayLine2[10];

extern HALWingAngle WingAngleSensor;

extern bool SD_Card_Present; // Flag for SD Card Presence
extern WingSailType WingSail;

extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int DecisionEventValue;
extern int DecisionEventValue2;
extern int LastParameterIndex;

extern sim_vessel simulated_vessel;

// Command Line Interpreter - Global Variables
char CLI_Msg[60];
unsigned int CLI_i = 0;

// temporary test code to investigate init strings for the NEO-8M GPS.
int GPSInitMessageNumber;

// Collect the characters into a command string until end end of line,
// and then process it.
// V1.0 22/12/2015
void CLI_Process_Message(int CommandPort)
{
	// This is called from the medium loop, about 1 sec.
	// Accumulate characters in a command string up to a CR or LF or buffer fills. 
	while ((*Serials[CommandPort]).available())
	{
		char received = (*Serials[CommandPort]).read();
		CLI_Msg[CLI_i++] = received;
		// Process message when new line character is received
		if (received == '\n'  || CLI_i >= sizeof(CLI_Msg) - 2) // || received == '\r'
		{
			CLI_i = CLI_i - 2; // move pointer back before the trailing CrLf

			CLI_Msg[CLI_i] = '\0';
			CLI_i = 0;

			if (strlen(CLI_Msg) >= 3) // check the command is long enough
				CLI_Processor(CommandPort);
		}
	}
}

void CLI_Processor(int CommandPort, String Command)
{
	Command.toCharArray(CLI_Msg, Command.length()); 
	CLI_Processor(CommandPort);
	CLI_i = 0;
}

// Process the Command String.
// Split into command and parameters separated by commas. 
// V1.0 22/12/2015
// V1.1 13/01/2018 added support for a Vessel Command Parameter. i.e. steer a magnetic heading
// V1.2 1/12/2018 changed strcpy to strncpy to guard against corrupting memory with long strings

void CLI_Processor(int CommandPort)
{

	//char Fullcmd[8] = "";
	char cmd[4] = "";
	char param1[12] = "";
	char param2[12] = "";
	char param3[12] = "";
	char param4[12] = "";
	char param5[12] = "";
	char param6[12] = "";
	char param7[12] = "";

	strcat(CLI_Msg, ",");

	//Serial.println(CLI_Msg);

	// Split into command and parameters separated by commas. 
	strncpy(cmd, strtok(CLI_Msg, ","), sizeof(cmd) - 1);
	strncpy(param1, strtok(NULL, ","), sizeof(param1) - 1);
	strncpy(param2, strtok(NULL, ","), sizeof(param2) - 1);
	strncpy(param3, strtok(NULL, ","), sizeof(param3) - 1);
	strncpy(param4, strtok(NULL, ","), sizeof(param4) - 1);
	strncpy(param5, strtok(NULL, ","), sizeof(param5) - 1);
	strncpy(param6, strtok(NULL, ","), sizeof(param6) - 1);
	strncpy(param7, strtok(NULL, ","), sizeof(param7) - 1);

	// ===============================================
	// Command ech: Echo the command and parameters
	// ===============================================
	if (!strncmp(cmd, "ech", 3))
	{
		(*Serials[CommandPort]).print(cmd);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param1);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param2);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param3);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param4);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param5);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param6);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param7);
		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// Command mcs: Mission Command Set
	// ===============================================
	// set a Mission Command
	// Parameter 1: Mission Sequence Number 
	// Parameter 2: Mission Command 
	// Parameter 3: MC Param 1
	// Parameter 4: MC Param 2
	// Parameter 5: MC Param 3  
	// Parameter 6: MC Param 4
	// Parameter 7: MC Param 5
	if (!strncmp(cmd, "mcs", 3))
	{
		// set the sequence number of the command starting with zero
		int Mission_cmd_ptr = atoi(param1);

		// Advance the mission size to encompass the specified position
		if ((Mission_cmd_ptr + 1) > MissionValues.mission_size) {
			MissionValues.mission_size = Mission_cmd_ptr + 1;
		}

		MissionCommandType mc = MissionCommandType(atoi(param2));
		MissionValues.MissionList[Mission_cmd_ptr].cmd = mc;

		switch (mc)
		{
		case ctGotoWaypoint:
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lat = atof(param3) * 10000000UL;  //Latitude  * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lng = atof(param4) * 10000000UL;  //Longitude * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].boundary = atoi(param5);      // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param6);      // Control Mask
			break;

		case ctLoiter:
		case ctLoiterUntil:
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lat = atof(param3) * 10000000UL;  //Latitude  * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lng = atof(param4) * 10000000UL;  //Longitude * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].boundary = atoi(param5);	   // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param6);      // Control Mask
			MissionValues.MissionList[Mission_cmd_ptr].duration = atoi(param7);	   // minutes
			break;

		case ctReturnToHome:
			MissionValues.MissionList[Mission_cmd_ptr].boundary = atoi(param3);	   // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param4);      // Control Mask
			break;

		case ctSteerWindCourse:
			MissionValues.MissionList[Mission_cmd_ptr].SteerAWA = atoi(param3);		// SteerAWA degrees
			MissionValues.MissionList[Mission_cmd_ptr].TrimTabAngle = atoi(param4);  //TrimTabAngle degrees
			MissionValues.MissionList[Mission_cmd_ptr].duration = atoi(param5);      // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param6);      // Control Mask
			break;

		default:;
		}
	}

	// ===============================================
	// Command scs: Set Command State
	// ===============================================
	// Set the vessel into a specifed command State
	// Parameter 1: Command State (enumerated Type) 
		//vcsIdle,					// idle state, feathered settings for sails.
		//vcsFullManual,				// The vessel is under manual command via RC
		//vcsPartialManual,				// The vessel is under manual command via RC
		//vcsFollowMission,		// The vessel is under automatic control following the mission list.
		//vcsSteerMagneticCourse,	// The vessel is steering a course relative to the compass
		//vcsSteerWindCourse,		// The vessel is steering a course relative to the wind.
		//vcsReturnToHome,			// return to the preset home location
		//vcsSetHome,				// set home location
		//vcsResetMissionIndex		// Reset the Mission Index, to force a restsart of the mission.		
		//vcsLoiter				// Loiter here.

	if (!strncmp(cmd, "scs", 3))
	{
		StateValues.CommandState = VesselCommandStateType(atoi(param1));
		// set both steering angles to the command parameter
		StateValues.SteerCompassBearing = wrap_360_Int(atoi(param2));
		StateValues.SteerWindAngle = atoi(param2);

		// Only set the TrimTabDefaultAngle if the command is vcsSteerWindCourse, otherwise it will interfere normal mission operation
		if (StateValues.CommandState == VesselCommandStateType::vcsSteerWindCourse)
		{
			Configuration.TrimTabDefaultAngle = atoi(param3);
			DecisionEventValue2 = Configuration.TrimTabDefaultAngle;
		}

		// start command timer. Record the start time of each new command	
		MissionValues.MissionCommandStartTime = millis();

		// save the state
		Save_EEPROM_StateValues();

		DecisionEvent = DecisionEventType::deChangeCommandState;
		DecisionEventReason = DecisionEventReasonType::rManualIntervention;
		DecisionEventValue = atoi(param2);

		// explictly log the commands to set the steering values for wind or compass.
		SD_Logging_Event_Decisions();

		// print the command state
		QueueMessage(TelMessageType::SCS);
	}

	// ===============================================
	// Command scg: Get Command State
	// ===============================================
	// get the current Command State.
	if (!strncmp(cmd, "scg", 3))
	{
		QueueMessage(TelMessageType::SCG);
	}

	// ===============================================
	// Command mis: Mission Command Index Set
	// ===============================================
	// Set the index to the one less the desired step. i.e. set to zero to commence with step 1.
	// This is usually 0 to start the mission
	// Parameter 1: Mission Sequence Number  
	if (!strncmp(cmd, "mis", 3))
	{
		StateValues.mission_index = atoi(param1);

		// save the state
		Save_EEPROM_StateValues();

		// since we are forcing a change in the mission index, then force a restart at that index.
		StateValues.StartingMission = true;
		NavData.next_WP_valid = false;

		// update next WP for display purposes.
		set_next_WP_for_display();

		QueueMessage(TelMessageType::MIS);
	}

	// ===============================================
	// Command mig: Mission Command Index Get
	// ===============================================
	// Get the current value of mission command index 
	// No parameters
	if (!strncmp(cmd, "mig", 3))
	{
		QueueMessage(TelMessageType::MIG);
	}

	// ===============================================
	// Command mcl: Mission Command List
	// ===============================================
	// return the whole Mission Command list; no parameters needed.
	// No parameters
	if (!strncmp(cmd, "mcl", 3))
	{
		(*Serials[CommandPort]).print(F("Mission Command List:"));
		(*Serials[CommandPort]).println();
		char FloatFormatString[16];

		if (MissionValues.mission_size == 0) {
			(*Serials[CommandPort]).println(F("Empty."));
		}

		// loop through the mission list array
		for (int i = 0; i < MissionValues.mission_size; i++)
		{
			(*Serials[CommandPort]).print(i);
			(*Serials[CommandPort]).print(":");

			MissionCommandType mc = MissionValues.MissionList[i].cmd;

			switch (mc)
			{
			case ctGotoWaypoint:
				(*Serials[CommandPort]).print(F("GotoWaypoint:"));
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(",");
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(F(",Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).println();
				break;

			case ctLoiter:
				(*Serials[CommandPort]).print(F("Loiter:"));
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(",");
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(F(",Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).print(F(",Duration: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
				(*Serials[CommandPort]).println();
				break;

			case ctLoiterUntil:
				(*Serials[CommandPort]).print(F("Loiter Until:"));
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(",");
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(F(",Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).print(F(",Time: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
				(*Serials[CommandPort]).println();
				break;

			case ctReturnToHome:
				(*Serials[CommandPort]).print(F("ReturnToHome,Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).println();
				break;

			case ctSteerWindCourse:
				(*Serials[CommandPort]).print(F("SteerWindCourse,AWA:"));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].SteerAWA);
				(*Serials[CommandPort]).print(F(",TrimTabAngle:"));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].TrimTabAngle);
				(*Serials[CommandPort]).print(F(",Duration: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).println();
				break;

			default:
				(*Serials[CommandPort]).print(F("Unknown command"));
			}
		}
	}

	// ===============================================
	// Command mcp: Mission Command List for Plotting
	// ===============================================
	// return the whole Mission Command list; no parameters needed.
	// No parameters
	if (!strncmp(cmd, "mcp", 3))
	{
		QueueMessage(TelMessageType::MCP);
	}

	// ===============================================
	// Command mcc:  Mission Command List - Clear All
	// ===============================================
	// clear the Mission Command List
	// No parameters
	if (!strncmp(cmd, "mcc", 3))
	{
		// reset all mission values and flags
		MissionValues.mission_size = 0;
		StateValues.mission_index = 0;
		StateValues.StartingMission = true;
		NavData.next_WP_valid = false;

		QueueMessage(TelMessageType::MCC);
	}

	// ===============================================
	// Command lcs: Set Current Location, and disable GPS. 
	// i.e. use a simulated GPS. 
	// ===============================================
	// Set Current Location
	// Parameter 1: simulated Lat 
	// Parameter 2: simulated Lon 
	if (!strncmp(cmd, "lcs", 3))
	{
		UseSimulatedVessel = true;

		simulated_vessel.Currentloc.lat = atof(param1) * 10000000UL;  //Latitude  * 10**7
		simulated_vessel.Currentloc.lng = atof(param2) * 10000000UL;  //Longitude * 10**7
		simulated_vessel.Heading = atoi(param3);
	}

	// ===============================================
	// Command HLS, Set Home Location
	// ===============================================
	// Parameter 1: Range from current location - metres
	// Parameter 2: True Bearing from current location - degrees
	// 
	if (!strncmp(cmd, "hls", 3))
	{
		// get the current location 
		StateValues.home = NavData.Currentloc;

		float distance = atof(param1); // metres
		float bearing = atof(param2);  // degrees

		location_update(StateValues.home, bearing, distance);
		StateValues.home_is_set = true;

		// save the state
		Save_EEPROM_StateValues();

		SD_Logging_Event_Messsage(F("Set Home"));

		QueueMessage(TelMessageType::HLG);
	}

	// ===============================================
	// Command HLG, Get Home Location
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "hlg", 3))
	{
		QueueMessage(TelMessageType::HLG);
	}

	// ===============================================
	// Command HLC, Clear Home Location
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "hlc", 3))
	{
		// Clear the flag
		StateValues.home_is_set = false;

		SD_Logging_Event_Messsage(F("Clear Home"));

		char MsgString[16];

		(*Serials[CommandPort]).print("hom:");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lat) / 10000000UL, 10, 5, MsgString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lng) / 10000000UL, 10, 5, MsgString));
		(*Serials[CommandPort]).print(",");
		StateValues.home_is_set ? (*Serials[CommandPort]).print("Set:true") : (*Serials[CommandPort]).print("Set:false");
		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// Command ver, Get Software Version
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "ver", 3))
	{
		QueueMessage(TelMessageType::VER);
	}

	// ===============================================
	// Command TMG, Get Current Time
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "tmg", 3))
	{
		QueueMessage(TelMessageType::TMG);
	}

	// ===============================================
	// Command lcd, Set LCD Logging Level.
	// ===============================================
	// Parameter 1: Logging Level: 0,1,2,3
	// 
	if (!strncmp(cmd, "lcd", 3))
	{
		Configuration.DisplayScreenView = *param1;
	
		if (Configuration.DisplayScreenView == 'W')
		{
			CheckWingSailPower();
		}

		QueueMessage(TelMessageType::LCD);
	}

	// ===============================================
	// Command rst, Restore Config back to default at next power up
	// ===============================================
	// No Parameter
	// 
	if (!strncmp(cmd, "rst", 3))
	{
		Configuration.EEPROM_Storage_Version--; // force the config ID to be invalid, then save it.

		// save the Calibration values, using the save procedure without restoring the config version number
		Save_EEPROM_ConfigValues_LeaveVersion();

		// .......

		// Then on next power up the config should revert to the default values
	}

	// ===============================================
	// Command sat: initiate a simulated Sat Com Vessel message. 
	// i.e. it stores the simulated message on the SD Card
	// ===============================================
	if (!strncmp(cmd, "stv", 3))
	{
		sendSatComVesselState();
	}

	// ===============================================
	// Command sat: initiate a simulated Sat Com Mission Step message. 
	// i.e. it stores the simulated message on the SD Card
	// ===============================================
	if (!strncmp(cmd, "stm", 3))
	{
		sendSatComMissionEvent(StateValues.mission_index);
	}

	// ===============================================
	// Command eqg, Get Equipment Status
	// ===============================================
	//  No parameters
	// 
	// Return the status of the Wing Angle Sensor, SD Card.

	if (!strncmp(cmd, "eqg", 3))
	{
		QueueMessage(TelMessageType::EQG);
	}

	// ===============================================
	// Command ccs,  Compass calibration Save
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "ccs", 3))
	{
		QueueMessage(TelMessageType::CCS);

		//		IMU_Save_Cal();
	}

	// ===============================================
	// Command ccg,  Compass calibration Get Status
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "ccg", 3))
	{
		QueueMessage(TelMessageType::CCG);
	}

	// ===============================================
	// Command wc1,  Wingsail Angle Sensor calibration ON
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "wc1", 3))
	{
		QueueMessage(TelMessageType::WC1);

		// enable calibration mode. Fast reads and maintain enable setting of the min/max calibration values
		WingAngleSensor.WingSailAngleSensorPort.WingSailAngleSensor->MagneticCompassCalibrationMode = true;


		Configuration.WingAngle_mXScale = 0;
		Configuration.WingAngle_mYScale = 0;

		// init the Calibration limits
		WingAngleSensor.WingSailAngleSensorPort.IMU_Mag_Init();
	}

	// ===============================================
	// Command wc0,  Wingsail Angle Sensor calibration OFF
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "wc0", 3))
	{
		QueueMessage(TelMessageType::WC0);

		// disable calibation mode
		WingAngleSensor.WingSailAngleSensorPort.WingSailAngleSensor->MagneticCompassCalibrationMode = false;

		// save the Calibration values
		Save_EEPROM_ConfigValues();
	}


	// ===============================================
	// Command sav, Save Data to EEPROM
	// ===============================================
	//  Parameter 1: Entity to be saved: m-Mission, c-Config, s-State , y-Save State-On, n-Save-State-Off
	// 
	if (!strncmp(cmd, "sav", 3))
	{
		(*Serials[CommandPort]).print(F("SAV, Save to EEPROM: "));

		switch (*param1)
		{
		case 'm':
			(*Serials[CommandPort]).print(F("Mission"));
			Save_EEPROM_Mission();
			break;

		case 'c':
			(*Serials[CommandPort]).print(F("Configuration"));
			Save_EEPROM_ConfigValues();
			break;

		case 's':
			(*Serials[CommandPort]).print(F("State"));
			Save_EEPROM_StateValues();
			break;

		case 'y':
			(*Serials[CommandPort]).print(F("Save State-Yes"));
			Configuration.SaveStateValues = true;
			break;

		case 'n':
			(*Serials[CommandPort]).print(F("Save State-No"));
			Configuration.SaveStateValues = false;
			break;

		default:
			(*Serials[CommandPort]).print(F("unknown"));
		}
		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// 	Command dsp, Display messages on LCD/OLED
	// ===============================================
	// Paramters: Line1, Line2
	// 
	if (!strncmp(cmd, "dsp", 3))
	{
		strcpy(MessageDisplayLine1, param1);
		strcpy(MessageDisplayLine2, param2);
	}

	// ===============================================
	// 	Command gps, set GPS Power Mode
	// ===============================================
	// Paramters: Line1, Line2
	// 
	if (!strncmp(cmd, "gps", 3))
	{
		Configuration.GPS_PowerMode = (GPS_PowerModeType)atoi(param1);
	}

	// ===============================================
	// 	prl, Parameter List   
	// ===============================================
	// Parameters: none.
	// this initiates listing all parameters
	if (!strncmp(cmd, "prl", 3))
	{
		QueueMessage(TelMessageType::PRL);
	}

	// ===============================================
// 	prm, Max Parameter Number 
// ===============================================
// Parameters: none.
// this initiates listing all parameters
	if (!strncmp(cmd, "prm", 3))
	{
		QueueMessage(TelMessageType::PRM);
	}

	// ===============================================
	// 	prg, Parameter Get   
	// ===============================================
	// Parameters: Parameter Index Number
	// 
	if (!strncmp(cmd, "prg", 3))
	{
		LastParameterIndex = atoi(param1); // set a global variable (sorry for that) with the single parameter number to be used next.
		QueueMessage(TelMessageType::PRG);
	}

	// ===============================================
	// 	prs, Parameter Set   
	// ===============================================
	// Parameters: Parameter Index Number, Parameter Value
	// 
	if (!strncmp(cmd, "prs", 3))
	{
		LastParameterIndex = atoi(param1);
		QueueMessage(TelMessageType::PRG);

		// log the change to the SD Card.
		SD_Logging_Event_ParameterChange(LastParameterIndex, param2);

		switch (LastParameterIndex)
		{
		case 1:
			Configuration.CompassOffsetAngle = atof(param2);
			break;

		case 2:
			Configuration.TrimTabOffset = atoi(param2);
			break;

		case 3:
			Configuration.TrimTabScale = atoi(param2);
			break;

		case 4:
			Configuration.TrimTabDefaultAngle = atoi(param2);
			break;

		case 5:
			Configuration.SailableAngleMargin = atoi(param2);
			break;

		case 6:
			Configuration.LoiterRadius = atol(param2);
			break;

		case 7:
			Configuration.timezone_offset = atoi(param2);
			break;

		case 8:
			Configuration.DisplayScreenView = *param2;
			break;

		case 9:
			// temporary test code to investigate init strings for the NEO-8M GPS.
			GPSInitMessageNumber = atoi(param2);
			gps.SendConfigurationString(GPSInitMessageNumber);
			break;

		case 10:
			Configuration.UseGPSInitString = atoi(param2);
			break;

		case 11:
			Configuration.SaveStateValues = atoi(param2);
			break;

		case 12:
			Configuration.MinimumAngleUpWind = atoi(param2);
			break;

		case 13:
			Configuration.MinimumAngleDownWind = atoi(param2);
			break;

		case 14:
			Configuration.WindAngleCalibrationOffset = atoi(param2);
			break;

		case 15:
			Configuration.WPCourseHoldRadius = atoi(param2);
			break;

		case 16:
			if (!strncmp(param2, "t", 1)) { Configuration.SDCardLogDelimiter = '\t'; }
			if (!strncmp(param2, ",", 1)) { Configuration.SDCardLogDelimiter = ','; }
			break;

		case 17:
			Configuration.RTHTimeManualControl = atoi(param2);
			break;

		case 18:
			Configuration.TWD_Offset = atoi(param2);
			break;

		case 19:
			Configuration.TargetHeadingFilterConstant = atof(param2);
			break;

		case 20:
			Configuration.MaxFileSize = atoi(param2);
			break;

		case 21:
			Configuration.MagnetVariation = atof(param2);
			break;

		case 22:
			Configuration.LoRaPort = atoi(param2);
			break;

		case 23:
			Configuration.SatCommsPort = atoi(param2);
			break;

		case 24:
			Configuration.LoRaPortBaudRate = atol(param2);
			break;

		case 25:
			Configuration.SatCommsPortBaudRate = atol(param2);
			break;

		case 26:
			break;

		case 27:
			break;

		case 28:
			break;

		case 29:
			Configuration.Servo_Channel_Steering = atoi(param2);
			break;

		case 30:
			Configuration.Servo_Channel_Steering_Stbd = atoi(param2);
			break;

		case 31:
		//	Configuration.Servo_Channel_Sail = atoi(param2);
			break;

		case 32:
			Configuration.Servo_Channel_Motor = atoi(param2);
			break;

		case 33:
			Configuration.UseMotor = atoi(param2);
			break;

		case 34:
			Configuration.DualRudder = atoi(param2);
			break;

		case 35:
			Configuration.SteeringFilterConstant = atof(param2);
			SteeringPID_Init(); // update filter immediately
			break;

		case 36:
			Configuration.pidDirection = atoi(param2);
			SteeringPID_Init(); // update PID immediately
			break;

		case 37:
			Configuration.pidKp = atol(param2);
			SteeringPID_Init(); // update PID immediately
			break;

		case 38:
			Configuration.pidKi = atol(param2);
			SteeringPID_Init(); // update PID immediately
			break;

		case 39:
			Configuration.pidKd = atol(param2);
			SteeringPID_Init(); // update PID immediately
			break;

		case 40:
			Configuration.pidOutputmin = atol(param2);
			SteeringPID_Init(); // update PID immediately
			break;

		case 41:
			Configuration.pidOutputmax = atol(param2);
			SteeringPID_Init(); // update PID immediately
			break;

		case 42:
			Configuration.pidCentre = atol(param2);
			break;


		case 43:
			Configuration.WingAngleError000 = atoi(param2);
			break;

		case 44:
			Configuration.WingAngleError090 = atoi(param2);
			break;

		case 45:
			Configuration.WingAngleError180 = atoi(param2);
			break;

		case 46:
			Configuration.WingAngleError270 = atoi(param2);
			break;

		case 47:
			Configuration.WingAngle_mXScale = atoi(param2);
			break;

		case 48:
			Configuration.WingAngle_mYScale = atoi(param2);
			break;

		case 49:
			Configuration.CompassError000 = atoi(param2);
			break;

		case 50:
			Configuration.CompassError090 = atoi(param2);
			break;

		case 51:
			Configuration.CompassError180 = atoi(param2);
			break;

		case 52:
			Configuration.CompassError270 = atoi(param2);
			break;

		case 53:
			Configuration.DTB_Threshold = atoi(param2);
			break;

		case 54:
			Configuration.GPS_Max_Sleep_Time = atoi(param2);
			break;

		case 55:
			Configuration.GPS_Min_Wake_Time = atoi(param2);
			break;

		default:;
		}

	}

	// ********** Start - LoRa Polling Commands *****************************************************************************

	// ===============================================
	// Command: LAP - (LoRa) Nav - 	responses: CTE, DTW, BTW, CDA, LAT, LON, COG, SOG, HDG
	// ===============================================
	// No Parameters

	if (!strncmp(cmd, "lna", 3))
	{
		QueueMessage(TelMessageType::LNA);
	}

	// ===============================================
	// Command: LAT - (LoRa) Attitude:	responses: HDG, Pitch, Roll, Roll_avg
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lat", 3))
	{
		QueueMessage(TelMessageType::LAT);
	}

	// ===============================================
	// Command: LPO - (LoRa) Power:	responses: Voyager Power V,I, Wingsail Power V, I
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lpo", 3))
	{
		//Serial.println(Fullcmd);
		QueueMessage(TelMessageType::LPO);
	}

	// ===============================================
	// Command: LWP - (LoRa) Waypoint:	responses: Prev WP, Next WP, Max CTE
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lwp", 3))
	{
		QueueMessage(TelMessageType::LWP);
	}

	// ===============================================
	// Command: LMI - (LoRa) current Mission Step Responses: MI, Cmd, Duration, SteerAWA, TTA
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lmi", 3))
	{
		QueueMessage(TelMessageType::LMI);
	}

	// ===============================================
	// Command: LWI - (LoRa) Wind data: Responses AWA / TWD,  TWS, WA / TTA
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lwi", 3))
	{
		QueueMessage(TelMessageType::LWI);
	}

	// ===============================================
	// Command: LVS - (LoRa) GetVessel State - Response: Vessel State
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lvs", 3))
	{
		QueueMessage(TelMessageType::LVS);
	}

	// ===============================================
	// Command: LPF - (LoRa) Get Sailing Performance 
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lpf", 3))
	{
		QueueMessage(TelMessageType::LPF);
	}

	// ===============================================
	// Command: LSV - (LoRa) Servo positions - Response: Rudder us, Trim Tab us
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lsv", 3))
	{
		QueueMessage(TelMessageType::LSV);
	}

	// ===============================================
	// Command: lws - (LoRa) Wingsail Power Data
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lws", 3))
	{
		QueueMessage(TelMessageType::LWS);
	}
}
	// ********** End - LoRa Polling Commands *****************************************************************************


void ListParameter(int CommandPort,int ParameterIndex)
{
	// function to list the value of a single parameter specified by its parameter number
	// V1.1 11/10/2016 added MaxFileSize.

	Set_LoRa_Mode(LoRa_Mode_Type::TxShortRange);

	(*Serials[CommandPort]).print(F("prg,"));
	(*Serials[CommandPort]).print(ParameterIndex);
	(*Serials[CommandPort]).print(",");

	switch (ParameterIndex)
	{
	case 1:
		(*Serials[CommandPort]).print(F("Mag_Offset,"));
		(*Serials[CommandPort]).print(Configuration.CompassOffsetAngle);
		break;

	case 2:
		(*Serials[CommandPort]).print(F("TrimTabOffset,"));
		(*Serials[CommandPort]).print(Configuration.TrimTabOffset);
		break;

	case 3:
		(*Serials[CommandPort]).print(F("TrimTabScale,"));
		(*Serials[CommandPort]).print(Configuration.TrimTabScale);
		break;

	case 4:
		(*Serials[CommandPort]).print(F("TrimTab Default Angle,"));
		(*Serials[CommandPort]).print(Configuration.TrimTabDefaultAngle);
		break;

	case 5:
		(*Serials[CommandPort]).print(F("Sailable Angle Margin,"));
		(*Serials[CommandPort]).print(Configuration.SailableAngleMargin);
		break;

	case 6:
		(*Serials[CommandPort]).print(F("LoiterRadius,"));
		(*Serials[CommandPort]).print(Configuration.LoiterRadius);
		break;

	case 7:
		(*Serials[CommandPort]).print(F("timezone_offset,"));
		(*Serials[CommandPort]).print(Configuration.timezone_offset);
		break;

	case 8:
		(*Serials[CommandPort]).print(F("LCDScreenView,"));
		(*Serials[CommandPort]).print(Configuration.DisplayScreenView);
		break;

	case 9:
		// temporary test code to investigate init strings for the NEO-8M GPS.
		(*Serials[CommandPort]).print(F("GPSInitMessageNumber,"));
		(*Serials[CommandPort]).print(GPSInitMessageNumber);
		break;

	case 10:
		(*Serials[CommandPort]).print(F("UseGPSInitString,"));
		Configuration.UseGPSInitString ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 11:
		(*Serials[CommandPort]).print(F("SaveStateValues,"));
		Configuration.SaveStateValues ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 12:
		(*Serials[CommandPort]).print(F("MinimumAngleUpWind,"));
		(*Serials[CommandPort]).print(Configuration.MinimumAngleUpWind);
		break;

	case 13:
		(*Serials[CommandPort]).print(F("MinimumAngleDownWind,"));
		(*Serials[CommandPort]).print(Configuration.MinimumAngleDownWind);
		break;

	case 14:
		(*Serials[CommandPort]).print(F("WindAngleCalibrationOffset,"));
		(*Serials[CommandPort]).print(Configuration.WindAngleCalibrationOffset);
		break;

	case 15:
		(*Serials[CommandPort]).print(F("WPCourseHoldRadius,"));
		(*Serials[CommandPort]).print(Configuration.WPCourseHoldRadius);
		break;

	case 16:
		(*Serials[CommandPort]).print(F("SDCardDelimiter,"));

		if (Configuration.SDCardLogDelimiter == '\t') { (*Serials[CommandPort]).print(F("Tab")); }
		if (Configuration.SDCardLogDelimiter == ',') { (*Serials[CommandPort]).print(","); }
		break;

	case 17:
		(*Serials[CommandPort]).print(F("RTHTimeManualControl,"));
		(*Serials[CommandPort]).print(Configuration.RTHTimeManualControl);
		break;

	case 18:
		(*Serials[CommandPort]).print(F("AWA to TWD_Offset Angle,"));
		(*Serials[CommandPort]).print(Configuration.TWD_Offset);
		break;

	case 19:
		(*Serials[CommandPort]).print(F("TargetHeadingFilterConstant,"));
		(*Serials[CommandPort]).print(Configuration.TargetHeadingFilterConstant);
		break;

	case 20:
		(*Serials[CommandPort]).print(F("MaxFileSize,"));
		(*Serials[CommandPort]).print(Configuration.MaxFileSize);
		break;

	case 21:
		(*Serials[CommandPort]).print(F("MagneticVariation,"));
		(*Serials[CommandPort]).print(Configuration.MagnetVariation);
		break;

	case 22:
		(*Serials[CommandPort]).print(F("LoRaPort,"));
		(*Serials[CommandPort]).print(Configuration.LoRaPort);
		break;

	case 23:
		(*Serials[CommandPort]).print(F("SatCommsPort,"));
		(*Serials[CommandPort]).print(Configuration.SatCommsPort);
		break;

	case 24:
		(*Serials[CommandPort]).print(F("LoRaPortBaudRate,"));
		(*Serials[CommandPort]).print(Configuration.LoRaPortBaudRate);
		break;

	case 25:
		(*Serials[CommandPort]).print(F("SatCommsPortBaudRate,"));
		(*Serials[CommandPort]).print(Configuration.SatCommsPortBaudRate);
		break;

	case 26:
		break;

	case 27:
		break;

	case 28:
		break;

	case 29:
		(*Serials[CommandPort]).print(F("Servo_Channel_Steering,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Steering);
		break;

	case 30:
		(*Serials[CommandPort]).print(F("Servo_Channel_Steering_Stbd,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Steering_Stbd);
		break;

	case 31:
	//	(*Serials[CommandPort]).print(F("Servo_Channel_Sail,"));
	//	(*Serials[CommandPort]).print(Configuration.Servo_Channel_Sail);
		break;

	case 32:
		(*Serials[CommandPort]).print(F("Servo_Channel_Motor,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Motor);
		break;

	case 33:
		(*Serials[CommandPort]).print(F("UseMotor,"));
		Configuration.UseMotor ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 34:
		(*Serials[CommandPort]).print(F("DualRudder,"));
		Configuration.DualRudder ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 35:
		(*Serials[CommandPort]).print(F("SteeringFilter,"));
		(*Serials[CommandPort]).print(Configuration.SteeringFilterConstant);
		break;

	case 36:
		(*Serials[CommandPort]).print(F("pidDirection,"));
		if (Configuration.pidDirection == 0) {
			(*Serials[CommandPort]).print("Direct");
		} else{
			(*Serials[CommandPort]).print("Reverse");
		}
		break;

	case 37:
		(*Serials[CommandPort]).print(F("pidKp,"));
		(*Serials[CommandPort]).print(Configuration.pidKp);
		break;

	case 38:
		(*Serials[CommandPort]).print(F("pidKi,"));
		(*Serials[CommandPort]).print(Configuration.pidKi);
		break;

	case 39:
		(*Serials[CommandPort]).print(F("pidKd,"));
		(*Serials[CommandPort]).print(Configuration.pidKd);
		break;

	case 40:
		(*Serials[CommandPort]).print(F("pidOutputmin,"));
		(*Serials[CommandPort]).print(Configuration.pidOutputmin);
		break;

	case 41:
		(*Serials[CommandPort]).print(F("pidOutputmax,"));
		(*Serials[CommandPort]).print(Configuration.pidOutputmax);
		break;

	case 42:
		(*Serials[CommandPort]).print(F("pidCentre,"));
		(*Serials[CommandPort]).print(Configuration.pidCentre);
		break;

	case 43:
		(*Serials[CommandPort]).print(F("WingAngleError000,"));
		(*Serials[CommandPort]).print(Configuration.WingAngleError000);
		break;

	case 44:
		(*Serials[CommandPort]).print(F("WingAngleError090,"));
		(*Serials[CommandPort]).print(Configuration.WingAngleError090);
		break;

	case 45:
		(*Serials[CommandPort]).print(F("WingAngleError180,"));
		(*Serials[CommandPort]).print(Configuration.WingAngleError180);
		break;

	case 46:
		(*Serials[CommandPort]).print(F("WingAngleError270,"));
		(*Serials[CommandPort]).print(Configuration.WingAngleError270);
		break;


	case 47:
		(*Serials[CommandPort]).print(F("WingAngle_mXScale,"));
		(*Serials[CommandPort]).print(Configuration.WingAngle_mXScale);
		break;

	case 48:
		(*Serials[CommandPort]).print(F("WingAngle_mYScale,"));
		(*Serials[CommandPort]).print(Configuration.WingAngle_mYScale);
		break;

	case 49:
		(*Serials[CommandPort]).print(F("CompassError000,"));
		(*Serials[CommandPort]).print(Configuration.CompassError000);
		break;

	case 50:
		(*Serials[CommandPort]).print(F("CompassError090,"));
		(*Serials[CommandPort]).print(Configuration.CompassError090);
		break;

	case 51:
		(*Serials[CommandPort]).print(F("CompassError180,"));
		(*Serials[CommandPort]).print(Configuration.CompassError180);
		break;

	case 52:
		(*Serials[CommandPort]).print(F("CompassError270,"));
		(*Serials[CommandPort]).print(Configuration.CompassError270);
		break;

	case 53: 
		(*Serials[CommandPort]).print(F("DTB_Threshold,"));
		(*Serials[CommandPort]).print(Configuration.DTB_Threshold);
		break;

	case 54:
		(*Serials[CommandPort]).print(F("GPS_Max_Sleep_Time,"));
		(*Serials[CommandPort]).print(Configuration.GPS_Max_Sleep_Time);
		break;

	case 55:
		(*Serials[CommandPort]).print(F("GPS_Min_Wake_Time,"));
		(*Serials[CommandPort]).print(Configuration.GPS_Min_Wake_Time);
		break;

	default:
		(*Serials[CommandPort]).print(F("Unknown"));
	}

	(*Serials[CommandPort]).println();
}

void ShowCommandState(int CommandPort, VesselCommandStateType cs)
{
	// function to print a string representation for the Vessel Command state
	// V1.1 12/1/2018 added csLoiter
	switch (cs)
	{
	case vcsIdle:
		(*Serials[CommandPort]).print(F("Idle"));
		break;

	case vcsFollowMission:
		(*Serials[CommandPort]).print(F("FollowMission"));
		break;

	case vcsSteerMagneticCourse:
		(*Serials[CommandPort]).print(F("SteerMagneticCourse"));
		break;

	case vcsSteerWindCourse:
		(*Serials[CommandPort]).print(F("SteerWindCourse"));
		break;

	case vcsReturnToHome:
		(*Serials[CommandPort]).print(F("ReturnToHome"));
		break;

	case vcsSetHome:
		(*Serials[CommandPort]).print(F("SetHome"));
		break;

	case vcsResetMissionIndex:
		(*Serials[CommandPort]).print(F("ResetMissionIndex"));
		break;

	case vcsLoiter:
		(*Serials[CommandPort]).print(F("Loiter Here"));
		break;

	default:
		(*Serials[CommandPort]).print(F("unknown"));
	}

}