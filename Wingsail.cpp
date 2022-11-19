// 
// 
// 

#include "Wingsail.h"
#include "TelemetryMessages.h"
#include "configValues.h"
#include "CommandState_Processor.h"
#include "Navigation.h"
#include "Loiter.h"
#include "HAL_GPS.h"
#include "WearTracking.h"
#include "Mission.h"
#include "HAL_WingAngle.h"

extern HardwareSerial *Serials[];

extern configValuesType Configuration;		// stucture holding Configuration values; preset variables
extern WingSailType WingSail;
extern StateValuesStruct StateValues;
extern NavigationDataType NavData;
extern LoiterStruct LoiterData;
extern WearCounter TrimTabUsage;
extern MissionValuesStruct MissionValues;
extern HALGPS gps;
extern HALWingAngle WingAngleSensor;		// HAL WingSail Angle Sensor object

const uint16_t Deadband = 10; // us

void wingsail_init(void)
{
	// set to forward, at least until we have control over the state. 
	WingSail.State = wsForward;
	wingsail_update();
};

void wingsail_update(void)
{
	// called every 5 seconds normally or 1 second if in manual mode.
	// also called in 50ms loop to track changes rapidly.

	// sailing mode 
	// idle/forward/manual
	switch(StateValues.CommandState)
	{
	case vcsFollowMission:
		// We're following mission and not steering a wind course as a mission step
		if (MissionValues.MissionList[StateValues.mission_index].cmd != MissionCommandType::ctSteerWindCourse)
		{
			if (gps.GPS_LocationIs_Valid(NavData.Currentloc) && NavData.next_WP_valid)
			{
				// set sail to forward sailing mode
				WingSail.State = wsForward;
				AutoSetWingSail(WingSail.State);
			}
			else
			{
				// set sail to idle
				WingSail.State = wsIdle;
				AutoSetWingSail(WingSail.State);
			}
		}
		else // We are following mission and we are steering a wind course as a mission step
		{
			// explicitly set the TrimTab Angle
			SetTrimTabAngle(MissionValues.MissionList[StateValues.mission_index].TrimTabAngle);
		}

		break;

	case vcsLoiter:
		if (gps.GPS_LocationIs_Valid(NavData.Currentloc) &&
			(LoiterData.LoiterState != LoiterStateType::lsNotLoitering)
			)
		{
			// set sail to forward sailing mode
			WingSail.State = wsForward;
			AutoSetWingSail(WingSail.State);
		}
		else
		{
			// set sail to idle
			WingSail.State = wsIdle;
			AutoSetWingSail(WingSail.State);
		}
		break;

	case vcsReturnToHome:
		if (gps.GPS_LocationIs_Valid(NavData.Currentloc) && StateValues.home_is_set)
		{
			// set sail to forward sailing mode
			WingSail.State = wsForward;
			AutoSetWingSail(WingSail.State);
		}
		else
		{
			// set sail to idle
			WingSail.State = wsIdle;
			AutoSetWingSail(WingSail.State);
		}
		break;

	//case vcsPartialManual:
	case vcsSetHome:
	case vcsResetMissionIndex:
	case vcsSteerMagneticCourse:
	case vcsSteerWindCourse:
		WingSail.State = wsForward;
		AutoSetWingSail(WingSail.State);
		break;

	//case vcsFullManual:
	//	// pass through the RC Input channels to the BT Serial Servo
	////	SailIn_us = RC_IN.Channel[Configuration.RC_IN_Channel_Sail];
	////	WingSailServo(SailIn_us);
	//	break;

	case vcsIdle:
		WingSail.State = wsIdle;
		AutoSetWingSail(WingSail.State);
		break;

	default:;
		WingSail.State = wsIdle;
		AutoSetWingSail(WingSail.State);
	};
};

void AutoSetWingSail(WingSailStateType WingSailState)
{
	// called every 5 seconds normally or 1 second if in manual mode.
	// set the trim tab in accordance with current conditions, and current state
	WingSail.TrimTabAngle = CalcTrimTabAngle(WingSail.Angle, WingSailState);
	SetTrimTabAngle(WingSail.TrimTabAngle);
}

void SetTrimTabAngle(int TrimTabAngle)
{
	// set the trim tab angle 
	WingSail.Servo_microseconds = TrimTabAngle_to_us(TrimTabAngle);
	WingSailServo(WingSail.Servo_microseconds);
}


int CalcTrimTabAngle(int WingSailAngle, WingSailStateType WingSailState)
{
	// return the required angle to apply to the TrimTab to drive the vessel in accordance with
	// wingsail state, and the current wind angle.
	// called every 5 seconds normally or 1 second if in manual mode.
	// V1.1 update to add Config TrimTabDefaultAngle

	IdentifyCurrentTack(WingSailAngle);

	int TrimTabAngle = 0;

	switch (WingSailState) {
	case WingSailStateType::wsIdle:
		TrimTabAngle = 0;
		break;

	case WingSailStateType::wsForward:
		switch (WingSail.Tack) {
		case WingSailTackType::wsPortTack:
			TrimTabAngle = -Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsStarboardTack:
			TrimTabAngle = +Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsHeadToWind:
		default:
			TrimTabAngle = 0;
		};
		break;

	case WingSailStateType::wsReverse:
		switch (WingSail.Tack) {
		case WingSailTackType::wsPortTack:
			TrimTabAngle = +Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsStarboardTack:
			TrimTabAngle = -Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsHeadToWind:
		default:
			TrimTabAngle = 0;
		};
		break;

	default:
		TrimTabAngle = 0;
	}
	return TrimTabAngle;
}


int TrimTabAngle_to_us(int TrimTabAngle)
{
	// convert requested trim tab angle to the corresponding Servo Signal in microseconds.
	int ServoMicroSeconds = (Configuration.TrimTabScale * TrimTabAngle) + Configuration.TrimTabOffset;

	return ServoMicroSeconds;
}

void WingSailServo(int SailIn_us)
{
	// send the serial message to the Servo
	static uint16_t prev_SailIn_us;

	if (abs(SailIn_us - prev_SailIn_us) >= Deadband)  
	{
		prev_SailIn_us = SailIn_us;
		WingSail.Servo_microseconds = SailIn_us;

		// send message to Wingsail servo
		//QueueMessage(TelMessageType::SetWing);
		(*Serials[Configuration.BluetoothPort]).println("wake");
		(*Serials[Configuration.BluetoothPort]).println("wake");
		(*Serials[Configuration.BluetoothPort]).println("");
		//(*Serials[Configuration.BluetoothPort]).print("srv,");
		//(*Serials[Configuration.BluetoothPort]).println(WingSail.Servo_microseconds);
		(*Serials[Configuration.BluetoothPort]).println("");
		(*Serials[Configuration.BluetoothPort]).print("srv,");
		(*Serials[Configuration.BluetoothPort]).println(WingSail.Servo_microseconds);

		TrimTabUsage.TrackServoUsage(WingSail.Servo_microseconds);

		// record the time of sending command
		WingSail.LastCommandTime = millis();
	}
};

void IdentifyCurrentTack(int WingSailAngle)
{
	// calculate whether the sail is on port tack or starboard tack based on Wing Angle.
	// This is useful for identifying transitions of the wingsail through manoeuvres.

	// Wingsail Angle varies from -180 to 0 to +180

	// V1.0 5/2/2018 John Semmens.
	const int HeadToWindAngle = 3; // was 5 now 3 degrees 25/6/2020

	// port tack is more than x degrees off
	if (WingSailAngle <= -HeadToWindAngle)
	{
		WingSail.Tack = wsPortTack;
	}

	// starboard tack is more than x degrees off
	if (WingSailAngle >= +HeadToWindAngle)
	{
		WingSail.Tack = wsStarboardTack;
	}

	//  Head to Wind is within x degrees of zero.
	if (WingSailAngle > -HeadToWindAngle && WingSailAngle < HeadToWindAngle)
	{
		WingSail.Tack = wsHeadToWind;
	}
};

void Wingsail_TrackTackChange(void)
{
	// called in the fast loop 50ms
	
	static WingSailTackType WingSailTackPrev;

	IdentifyCurrentTack(WingSail.Angle); // convert WingSail Angle to a WingSail.Tack state

	// if there's a change of Wingsail Tack Status, then send an update to the wingsail
	if (WingSailTackPrev != WingSail.Tack)
	{
		WingSailTackPrev = WingSail.Tack;
		wingsail_update();
	}
};


void CheckWingSailServo(void)
{
	// Check the wingsail in a separate thread. called every 5 seconds
	// Query current position, and if it doesn't match the command position, then send another command

	WingSail.TimeSinceLastCommand = (millis() - WingSail.LastCommandTime) / 1000;
	WingSail.TimeSinceLastRequest = (millis() - WingSail.LastRequestTime) / 1000;
	WingSail.TimeSinceLastReponse = (millis() - WingSail.LastResponseTime) / 1000;
	WingSail.TimeSinceLastPowerReponse = (millis() - WingSail.LastPowerResponseTime) / 1000;

	// if a command has been recently sent AND we haven't checked in the laat 60 seconds then check again
	if (WingSail.TimeSinceLastCommand > 5 && WingSail.TimeSinceLastRequest > 60)
	{
		// send query to Wingsail servo
		//QueueMessage(TelMessageType::GSV);
		(*Serials[Configuration.BluetoothPort]).println("wake");
		(*Serials[Configuration.BluetoothPort]).println("wake");
		(*Serials[Configuration.BluetoothPort]).println("");
		(*Serials[Configuration.BluetoothPort]).println("");
		(*Serials[Configuration.BluetoothPort]).println("gsv");

		// record the time of sending command
		WingSail.LastRequestTime = millis();
	}

	// if we've recently had a response, AND the response doesn't match the command, AND its at least 60 seconds since last command
	// then send another command
	if (WingSail.TimeSinceLastReponse > 5 && WingSail.TimeSinceLastCommand > 60 && (WingSail.Servo_microseconds != WingSail.Servo_microseconds_reponse))
	{
		// send message to Wingsail servo
		//QueueMessage(TelMessageType::SetWing);
		(*Serials[Configuration.BluetoothPort]).println("wake");
		(*Serials[Configuration.BluetoothPort]).println("wake");
		(*Serials[Configuration.BluetoothPort]).println("");
		(*Serials[Configuration.BluetoothPort]).println("");
		(*Serials[Configuration.BluetoothPort]).print("srv,");
		(*Serials[Configuration.BluetoothPort]).println(WingSail.Servo_microseconds);

		// record the time of sending command
		WingSail.LastCommandTime = millis();

		SD_Logging_Event_Wingsail_Monitor("Corrrection");
	}
};


void CheckWingSailPower(void)
{
	(*Serials[Configuration.BluetoothPort]).println("wake");
	(*Serials[Configuration.BluetoothPort]).println("wake");
	(*Serials[Configuration.BluetoothPort]).println("");
	(*Serials[Configuration.BluetoothPort]).println("");
	(*Serials[Configuration.BluetoothPort]).println("pow");
}

void CheckWingSailVersion(void)
{
	(*Serials[Configuration.BluetoothPort]).println("wake");
	(*Serials[Configuration.BluetoothPort]).println("wake");
	(*Serials[Configuration.BluetoothPort]).println("");
	(*Serials[Configuration.BluetoothPort]).println("");
	(*Serials[Configuration.BluetoothPort]).println("ver");
}