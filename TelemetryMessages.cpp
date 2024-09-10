// 
// 
// V1.01 4/8/2021 updated to support full addressing

#include "TelemetryMessages.h"
#include "HAL.h"
#include "Navigation.h"
#include "HAL_IMU.h"
//#include "TelemetryLogging.h"
#include "Wingsail.h"
#include "Mission.h"
#include "Wingsail.h"
#include "HAL_WingAngle.h"
#include "configValues.h"
#include "HAL_GPS.h"
#include "CLI.h"
#include "DisplayStrings.h"
#include "HAL_PowerMeasurement.h"
#include "LoRaManagement.h"
#include "HAL_Time.h"
#include "TimeLib.h"

extern HardwareSerial* Serials[];
extern NavigationDataType NavData;
extern HALIMU imu;

extern HALPowerMeasure PowerSensor;

extern StateValuesStruct StateValues;
extern WingSailType WingSail;
extern int SteeringServoOutput;
extern MissionValuesStruct MissionValues;
extern char Version[];
extern char VersionDate[];
//extern Time CurrentLocalTime;
extern bool SD_Card_Present; // Flag for SD Card Presence
extern configValuesType Configuration;
extern HALWingAngle WingAngleSensor;

extern byte MessageArray[EndMarker + 1];
extern bool MessageToSend;
extern int LastParameterIndex;
extern uint32_t LastMessageSendTime;

static const int MaxParameterIndex = 55;

void SendMessage(int CommandPort, TelMessageType msg)
{
	char FloatString[16];

	switch (msg)
	{
	case TelMessageType::LNA:

		(*Serials[CommandPort]).print(F("lna,"));
		(*Serials[CommandPort]).print(NavData.CTE);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.DTW);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.BTW);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.IsBTWSailable ? 'Y' : 'N');

		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.COG);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.SOG_mps);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.HDG);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LAT:

		(*Serials[CommandPort]).print(F("lat,"));
		(*Serials[CommandPort]).print(NavData.HDG);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print((int)imu.Pitch);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print((int)imu.Roll);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.ROLL_Avg);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.VMG);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

		// swapped back to Bluetooth
	//case TelMessageType::SetWing:
	//case TelMessageType::SetWing2:
	//	// send the serial message for the servo 
	//	(*Serials[Configuration.LoRaPort]).println(F("wakeup")); // The wingsail needs prompting to wakeup, and it misses characters.
	//	(*Serials[Configuration.LoRaPort]).print("s,");	// direct to wingsail, use the short message version
	//	(*Serials[Configuration.LoRaPort]).println(WingSail.Servo_microseconds);
	//	MessageArray[msg] = 0; // clear the flag
	//	break;

	//case TelMessageType::GSV: // request wingsail servo position
	//	(*Serials[Configuration.LoRaPort]).println(F("wakeup")); // The wingsail needs prompting to wakeup, and it misses characters.
	//	(*Serials[CommandPort]).print("wwvv");		// next address, final address,previous address,source address
	//	(*Serials[CommandPort]).println(F("gsv"));
	//	MessageArray[msg] = 0; // clear the flag
	//	break;

	case TelMessageType::LWS: // wingsail power
		(*Serials[CommandPort]).print(F("lws,"));
		(*Serials[CommandPort]).print(WingSail.Discharge_V);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.Discharge_mA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.TimeSinceLastPowerReponse);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0; // clear the flag
		break;

	case TelMessageType::LPO:
		(*Serials[CommandPort]).print(F("lpo,"));
		(*Serials[CommandPort]).print(PowerSensor.Solar_V);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(PowerSensor.Solar_I);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(PowerSensor.BatteryIn_V);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(PowerSensor.BatteryIn_I);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(PowerSensor.BatteryOut_V);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(PowerSensor.BatteryOut_I);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LWP:
		(*Serials[CommandPort]).print(F("lwp,"));
		(*Serials[CommandPort]).print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.MaxCTE);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LMI:
		(*Serials[CommandPort]).print(F("lmi,"));
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.mission_size);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].cmd);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].duration);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].SteerAWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].TrimTabAngle);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LWI:
		(*Serials[CommandPort]).print(F("lwi,"));
		(*Serials[CommandPort]).print(NavData.AWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.TWD);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.TWS);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.Angle);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.TrimTabAngle);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LSV:
		(*Serials[CommandPort]).print(F("lsv,"));
		(*Serials[CommandPort]).print(SteeringServoOutput);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.Servo_microseconds);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LVS:
		(*Serials[CommandPort]).print(F("lvs,"));
		(*Serials[CommandPort]).print(StateValues.CommandState);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CommandStateToString(StateValues.CommandState));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LPF:
		(*Serials[CommandPort]).print(F("lpf,"));
		(*Serials[CommandPort]).print(NavData.AWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.SOG_mps);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.VMG);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(Configuration.TrimTabDefaultAngle);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::MCC:
		(*Serials[CommandPort]).print(F("MSG,Mission Command List Cleared."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::MIG:
		(*Serials[CommandPort]).print(F("MSG,Mission Command Index is: "));
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::MIS:
		(*Serials[CommandPort]).print(F("MSG,Mission Command Index Set to:"));
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::HLG:
	case TelMessageType::HLS:
		(*Serials[CommandPort]).print("hlg,");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		StateValues.home_is_set ? (*Serials[CommandPort]).print("Set:true") : (*Serials[CommandPort]).print("Set:false");
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.DTH);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.BTH);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::VER:
		(*Serials[CommandPort]).print("ver,");
		(*Serials[CommandPort]).print(Version);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).println(VersionDate);
		MessageArray[msg] = 0;
		break;

	case TelMessageType::TMG:
		(*Serials[CommandPort]).print(F("tim,"));
		(*Serials[CommandPort]).print(year(), DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(month(), DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(day(), DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(hour(), DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(minute(), DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(second(), DEC);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LCD:
		(*Serials[CommandPort]).print(F("MSG,Display Screen View set to: "));
		(*Serials[CommandPort]).print(Configuration.DisplayScreenView);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::EQG:
		(*Serials[CommandPort]).print(F("eqg"));
		(*Serials[CommandPort]).print(F(",SDCard:"));
		if (SD_Card_Present)
			(*Serials[CommandPort]).print(F("OK"));
		else
			(*Serials[CommandPort]).print(F("Fail"));

		(*Serials[CommandPort]).print(",WingAngle:");
		if (WingAngleSensor.PortStatus)
			(*Serials[CommandPort]).print(F("OK"));
		else
			(*Serials[CommandPort]).print(F("Fail"));

		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::CCS:
		(*Serials[CommandPort]).print(F("MSG,Compass Calibration Save."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::CCG:
		(*Serials[CommandPort]).print(F("MSG,Compass Status:"));
		//(*Serials[CommandPort]).print(imu.Algorithm_Status);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::WC0:
		(*Serials[CommandPort]).print(F("MSG,Wingsail Angle Sensor Calibration: Completed & Saved."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::WC1:
		(*Serials[CommandPort]).print(F("MSG,Wingsail Angle Sensor Calibration: Begin."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::SCS:
	case TelMessageType::SCG:
		(*Serials[CommandPort]).print(F("MSG,Command State set to: "));
		ShowCommandState(CommandPort, StateValues.CommandState);
		// if the command includes a parameter, then print that as well
		switch (StateValues.CommandState)
		{
		case vcsSteerWindCourse:
			(*Serials[CommandPort]).print(",");
			(*Serials[CommandPort]).print(StateValues.SteerWindAngle);
			break;

		case vcsSteerMagneticCourse:
			(*Serials[CommandPort]).print(",");
			(*Serials[CommandPort]).print(StateValues.SteerCompassBearing);
			break;
		default:;
		}
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::DBG:
		(*Serials[CommandPort]).print(F("MSG,10 minute Debug."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::PRG:
		ListParameter(CommandPort, LastParameterIndex);
		MessageArray[msg] = 0;
		break;

	case TelMessageType::PRM:
		(*Serials[CommandPort]).print(F("prm,"));
		(*Serials[CommandPort]).print(MaxParameterIndex);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

// Message with lists
	case TelMessageType::MCP: // Mission List
		SendMissionStep(CommandPort, MissionValues.mission_size - MessageArray[msg] );
		MessageArray[msg]--;
		break;

	case TelMessageType::PRL: // parameter list
		ListParameter(CommandPort, MaxParameterIndex - MessageArray[msg] + 1);
		MessageArray[msg]--;
		break;

	default:;
	}
}

void ProcessQueue(int SerialPortNumber)
{
	// called from the telemetry loop // 1 second
	// find the first non-zero flag in the message array, send the corresponding message, and clear the flag
	
	const uint32_t SendGapTime = 2000; // ms

	int msg = 0;
	while ((MessageArray[msg] == 0) && (msg < EndMarker))
	{
		msg++;
	};

	if ((msg < EndMarker) && ((LastMessageSendTime + SendGapTime) < millis()) ) // don't enter if we are still within the gap time of the last send
	{
		// if we are here, its because we found something before hitting the endMarker
		Set_LoRa_Mode(LoRa_Mode_Type::TxShortRange);

		SendMessage(SerialPortNumber, (TelMessageType) msg);

		Set_LoRa_Mode(LoRa_Mode_Type::RxLowPower);
		LastMessageSendTime = millis();

		// check if there are no messages and clear the MessageToSend flag if true
		msg = 0;
		// check each slot, and then exit if one is occupied, or at the end.
		while ((MessageArray[msg] == 0) && (msg < EndMarker))
		{
			msg++;
		};

		if (msg == EndMarker) // if end the no messages waiting, so clear flag.
		{
			MessageToSend = false;
		}		
	};
}

void QueueMessage(TelMessageType msg)
{
	// Place the message on the message queue.
	// Its not really a queue however. Its more like a checklist, where each message is represented in priority order.
	// Then each message is flagged with a 1, if its to be sent.
	// The high priority messages are at the top.
	// The "list" message are flagged by setting their to count of items in the list. 
	// These are placed at the top of the list, even though they are not high priority. 
	// The issue is that they need to get processed before other messages or they may take way too long to clear.

	switch (msg) 
	{
	    case TelMessageType::MCP: // Mission List -- special case, because the rsponse is a list
			MessageArray[msg] = MissionValues.mission_size;
			break;

		case TelMessageType::PRL: // parameter list  -- special case, because the rsponse is a list
			MessageArray[msg] = MaxParameterIndex;
			break;

		default: //	process all other message types (i.e. those that don't return a list or don't need special treatment).
			MessageArray[msg] = 1;	
	}

	MessageToSend = true; // set flag to signal there's a message waiting.
}

void SendMissionStep(int CommandPort, int i)
{
	char FloatFormatString[16];

	(*Serials[CommandPort]).print("mcp,");
	MissionCommandType mc = MissionValues.MissionList[i].cmd;

	switch (mc)
	{
	case ctGotoWaypoint:
		(*Serials[CommandPort]).print(ctGotoWaypoint);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).println();
		break;

	case ctLoiter:
		(*Serials[CommandPort]).print(ctLoiter);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
		(*Serials[CommandPort]).println();
		break;

	case ctLoiterUntil:
		(*Serials[CommandPort]).print(ctLoiterUntil);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
		(*Serials[CommandPort]).println();
		break;

	case ctReturnToHome:
		(*Serials[CommandPort]).print(ctReturnToHome);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).println();
		break;

	case ctSteerWindCourse:
		(*Serials[CommandPort]).print(ctSteerWindCourse);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].SteerAWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].TrimTabAngle);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).println();
		break;

	default:
		(*Serials[CommandPort]).print(F("Unknown command"));
	}

}
