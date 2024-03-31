// Handle the low level Steering
// use a PID to control the Steering servo(s) based on a target heading error.
// only operate the operate the automated steering if location is valid 
// or use the RC steering channel if in a manual type mode.

// V1.0 13/11/2016 
// V1.1 25/4/2018 updated 

#include "Steering.h"
#include "CommandState_Processor.h"
#include "configValues.h"
#include "location.h"
#include "Navigation.h"
#include "PID_v1.h"
#include "HAL_GPS.h"
#include "Loiter.h"
#include "WearTracking.h"
#include "HAL_Servo.h"

extern StateValuesStruct StateValues;
extern configValuesType Configuration;
extern NavigationDataType NavData;
extern int SteeringServoOutput;
extern LoiterStruct LoiterData;
extern WearCounter PortRudderUsage;
extern WearCounter StarboardRudderUsage;
extern HALGPS gps;
extern HALServo servo;

// Declare the Steeering PID 
double pidTargetHdgError = 0;
double pidActualHdgError, pidServoOutput;
PID SteeringPID(&pidActualHdgError, &pidServoOutput, &pidTargetHdgError, 5, 0, 0, DIRECT);

LowPassFilter SteeringFilter;
double SteeringServoOutput_LPF;


void SteeringFastUpdate(void)
{
	// Called from the Fast Loop. 25ms
	// 
	// This procedures operates the rudder.
	// It supports a single rudder or dual rudder.
	// The source of Rudder control is based on current CommandState.
	// For manual control type modes, use the RC input steering channel.
	// For automatically controlled modes, use the output of the Steering PID.
	// Calculate a heading error and then operate the rudder servo via the steering PID.
	// V1.0 14/11/2016 John Semmens
	// V1.2 24/4/2018 added offset to PID output by adding value: Configuration.pidCentreOffset
	// V1.3 28/4/2018 added criteria to loitering to only steer while on the approach or inbound phases.
	// V1.4 18/2/2019 added wear tracking for the Servos
	// V1.5 13/11/2021 added Lowpass filter to final signal to servo to remove jitter.

	switch (StateValues.CommandState)
	{
	case vcsFollowMission:
		if (gps.GPS_LocationIs_Valid(NavData.Currentloc) && NavData.next_WP_valid) 
		{
			// calculate the heading error  -180 to +180
			pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

			SteeringPID.Compute();
			SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		}
		break;

	case vcsLoiter:
		if ( gps.GPS_LocationIs_Valid(NavData.Currentloc) &&
			(LoiterData.LoiterState != LoiterStateType::lsNotLoitering)
			)
		{
			// only steer if we are loitering.

			// calculate the heading error  -180 to +180
			pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

			SteeringPID.Compute();
			SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		}
		break;

	case vcsReturnToHome:
		if (gps.GPS_LocationIs_Valid(NavData.Currentloc) && StateValues.home_is_set) 
		{
			// calculate the heading error  -180 to +180
			pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

			SteeringPID.Compute();
			SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		}
		break;

	case vcsSteerMagneticCourse:
	case vcsSteerWindCourse:
		pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

		SteeringPID.Compute();
		SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		break;

	//case vcsFullManual:
	//case vcsPartialManual:
	//	// pass through the RC Input channels to the Servo Ouput Channels to allow manaual steering and motor control
	//	//SteeringServoOutput = RC_IN.Channel[Configuration.RC_IN_Channel_Steering];
	//	break;

	case vcsSetHome:
	case vcsResetMissionIndex:
		break;
	default:	;
	}

	// apply a low pass filter to SteeringServoOutput to remove jitter, yielding SteeringServoOutput_LPF
	SteeringServoOutput_LPF = SteeringFilter.Filter(SteeringServoOutput);

	servo.Servo_Out(SteeringServoOutput_LPF);
	PortRudderUsage.TrackServoUsage(SteeringServoOutput_LPF);

	//if (Configuration.DualRudder)
	//{
	//	if (NavData.ROLL_Avg > 5) 
	//	{ // operate the Starboard Rudder
	//		servo.Servo_Out(Configuration.Servo_Channel_Steering_Stbd, SteeringServoOutput_LPF);
	//		StarboardRudderUsage.TrackServoUsage(SteeringServoOutput_LPF);
	//	}
	//	else
	//	{ // operate the  Port Rudder
	//		servo.Servo_Out(Configuration.Servo_Channel_Steering, SteeringServoOutput_LPF);
	//		PortRudderUsage.TrackServoUsage(SteeringServoOutput_LPF);
	//	}
	//}
	//else
	//{ // operate a single Rudder
	//	servo.Servo_Out(Configuration.Servo_Channel_Steering, SteeringServoOutput_LPF);
	//	PortRudderUsage.TrackServoUsage(SteeringServoOutput_LPF);
	//}
}

void SteeringPID_Init(void)
{
	// initial the Steering PID using config values and start it.
	// V1.0 John Semmens
	// V1.1 25/4/2018 added the Centre value. This required when using PID set to P only (with no ID)
	// V1.2 13/11/2021 added initialisation of the Steering filter low pass filter

	SteeringPID.SetControllerDirection(Configuration.pidDirection);
	SteeringPID.SetTunings(Configuration.pidKp, Configuration.pidKi, Configuration.pidKd);
	SteeringPID.SetOutputLimits(Configuration.pidOutputmin, Configuration.pidOutputmax);
	SteeringPID.SetMode(AUTOMATIC); // switch on the PID

	SteeringFilter.FilterConstant = Configuration.SteeringFilterConstant;
}

