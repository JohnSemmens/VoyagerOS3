// 
// 
// 

#include "DisplayStrings.h"

String CommandStateToString(VesselCommandStateType CommandState)
{
	// function to return a string version of the CommandState enumnerated type
	// V1.0 21/4/2019 John Semmens

	String CommandStateAsString;
	switch (CommandState)
	{
	case VesselCommandStateType::vcsFollowMission:
		CommandStateAsString = F("FollowMission");
		break;

	//case VesselCommandStateType::vcsFullManual:
	//	CommandStateAsString = F("FullManual");
	//	break;

	case VesselCommandStateType::vcsIdle:
		CommandStateAsString = F("Idle");
		break;

	case VesselCommandStateType::vcsLoiter:
		CommandStateAsString = F("Loiter");
		break;

	//case VesselCommandStateType::vcsPartialManual:
	//	CommandStateAsString = F("PartialManual");
	//	break;

	case VesselCommandStateType::vcsResetMissionIndex:
		CommandStateAsString = F("ResetIndex");
		break;

	case VesselCommandStateType::vcsReturnToHome:
		CommandStateAsString = F("ReturnHome");
		break;

	case VesselCommandStateType::vcsSetHome:
		CommandStateAsString = F("SetHome");
		break;

	case VesselCommandStateType::vcsSteerMagneticCourse:
		CommandStateAsString = F("SteerMag");
		break;

	case VesselCommandStateType::vcsSteerWindCourse:
		CommandStateAsString = F("SteerWind");
		break;

	default:
		CommandStateAsString = F("Invalid");
	}
	return  CommandStateAsString;
}


String DecisionEventToString(DecisionEventType DecisionEvent)
{
	// function to return a string version of the DecisionEvent enumnerated type
	// V1.0 21/4/2019 John Semmens
	// V1.1 13/6/2019 added deHoldCourse

	String DecisionEventAsString;
	switch (DecisionEvent)
	{
	case DecisionEventType::deEndOfMission:
		DecisionEventAsString = F("EndOfMission");
		break;

	case DecisionEventType::deIncrementMissionIndex:
		DecisionEventAsString = F("IncrementMissionIndex");
		break;

	case DecisionEventType::dePastWaypoint:
		DecisionEventAsString = F("PastWaypoint");
		break;

	case DecisionEventType::deTackToPort:
		DecisionEventAsString = F("toPortTack");
		break;

	case DecisionEventType::deTackToStarboard:
		DecisionEventAsString = F("toStarboardTack");
		break;

	case DecisionEventType::deTackToPortRunning:
		DecisionEventAsString = F("toPortTackRunning");
		break;

	case DecisionEventType::deTackToStarboardRunning:
		DecisionEventAsString = F("toStarboardTackRunning");
		break;

	case DecisionEventType::deChangeCommandState:
		DecisionEventAsString = F("ChangeCommandState");
		break;

	case DecisionEventType::deHoldCourse:
		DecisionEventAsString = F("HoldCourse");
		break;

	default:
		DecisionEventAsString = F("Invalid");
	}
	return DecisionEventAsString;
}

String DecisionEventReasonToString(DecisionEventReasonType DecisionEventReason)
{
	// function to return a string version of the DecisionEvent enumnerated type
	// V1.0 21/4/2019 John Semmens
	// V1.1 13/6/2019 added rApproachingWP

	String DecisionEventReasonAsString;
	switch (DecisionEventReason)
	{
	case DecisionEventReasonType::rFavouredTack:
		DecisionEventReasonAsString = F("FavouredTack");
		break;

	case DecisionEventReasonType::rNoChange:
		DecisionEventReasonAsString = F("NoChange");
		break;

	case DecisionEventReasonType::rPastBoundary:
		DecisionEventReasonAsString = F("PastBoundary");
		break;

	case DecisionEventReasonType::rPastDuration:
		DecisionEventReasonAsString = F("PastDuration");
		break;

	case DecisionEventReasonType::rPastTime:
		DecisionEventReasonAsString = F("PastTime");
		break;

	case DecisionEventReasonType::rPastWaypoint:
		DecisionEventReasonAsString = F("PastWaypoint");
		break;

	case DecisionEventReasonType::rUnkown:
		DecisionEventReasonAsString = F("Unkown");
		break;

	case DecisionEventReasonType::rLimitToSailingCourse:
		DecisionEventReasonAsString = F("LimitToSailingCourse");
		break;

	case DecisionEventReasonType::rPastLoiterBoundary:
		DecisionEventReasonAsString = F("PastLoiterBoundary");
		break;

	case DecisionEventReasonType::rNone:
		DecisionEventReasonAsString = F("None");
		break;

	case DecisionEventReasonType::rManualIntervention:
		DecisionEventReasonAsString = F("ManualIntervention");
		break;

	case DecisionEventReasonType::rApproachingWP:
		DecisionEventReasonAsString = F("ApproachingWP");
		break;

	default:
		DecisionEventReasonAsString = F("Invalid");
	}
	return DecisionEventReasonAsString;
}

String CourseTypeToString(SteeringCourseType CourseType)
{
	// function to return a string version of the CourseType enumnerated type
	// V1.0 5/5/2019 John Semmens
	// V1.1 10/1/2021 filled out additional new running values.

	String CourseTypeAsString;

	switch (CourseType)
	{
	case SteeringCourseType::ctDirectToWayPoint:
		CourseTypeAsString = F("DirectToWayPoint");
		break;

	case SteeringCourseType::ctPortTack:
		CourseTypeAsString = F("PortTack");
		break;

	case SteeringCourseType::ctStarboardTack:
		CourseTypeAsString = F("StarboardTack");
		break;


	case SteeringCourseType::ctPortTackRunning:
		CourseTypeAsString = F("PortTackRun");
		break;

	case SteeringCourseType::ctStarboardTackRunning:
		CourseTypeAsString = F("StbdTackRun");
		break;

	default:
		CourseTypeAsString = F("Invalid");
	}

	return CourseTypeAsString;
}


String CourseTypeToString(ManoeuvreStateType ManoeuvreState)
{
	// function to return a string version of the ManoeuvreState enumnerated type
	// V1.0 29/5/2019 John Semmens
	// V1.1 29/6/2022 added mstCommenceToPort and mstCommenceToStbd for Asymetric Gybe
	String ManoeuvreStateAsString;

	switch (ManoeuvreState)
	{
	case ManoeuvreStateType::mstCommenceToPort:
		ManoeuvreStateAsString = F("ToPort");
		break;

	case ManoeuvreStateType::mstCommenceToStbd:
		ManoeuvreStateAsString = F("ToStbd");
		break;

	case ManoeuvreStateType::mstComplete:
		ManoeuvreStateAsString = F("Complete");
		break;

	case ManoeuvreStateType::mstNone:
		ManoeuvreStateAsString = F("None");
		break;

	case ManoeuvreStateType::mstRunningToPort:
		ManoeuvreStateAsString = F("RunToPort");
		break;

	case ManoeuvreStateType::mstRunningToStbd:
		ManoeuvreStateAsString = F("RunToStbd");
		break;

	case ManoeuvreStateType::mstApproachPort:
		ManoeuvreStateAsString = F("ApprchPort");
		break;

	case ManoeuvreStateType::mstApproachStbd:
		ManoeuvreStateAsString = F("ApprchStbd");
		break;

	default:
		ManoeuvreStateAsString = F("Invalid");
	}

	return ManoeuvreStateAsString;
}





String GetMissionCommandString(MissionCommandType cmd)
{
	// function to return a string version of the MissionCommandType enumnerated type
	// V1.0 29/5/2019 John Semmens
	String MissionCommandString;

	switch (cmd)
	{
	case MissionCommandType::ctGotoWaypoint:
		MissionCommandString = F("Waypoint");
		break;

	case MissionCommandType::ctLoiter:
		MissionCommandString = F("Loiter");
		break;

	case MissionCommandType::ctLoiterUntil:
		MissionCommandString = F("Loiter'til");
		break;

	case MissionCommandType::ctReturnToHome:
		MissionCommandString = F("Home");
		break;

	case MissionCommandType::ctSteerWindCourse:
		MissionCommandString = F("SteerWind");
		break;

	default:
		MissionCommandString = F("unknown");
	}

	return MissionCommandString;
}


String GetEquipmentStatusString(EquipmentStatusType status)
{

	String ResultString;

	switch (status) {
	case EquipmentStatusType::Unknown:
		ResultString = "Unknown";
		break;

	case EquipmentStatusType::NotFound:
		ResultString = "NotFound";
		break;

	case EquipmentStatusType::Found:
		ResultString = "Found";
		break;

	case EquipmentStatusType::DataBad:
		ResultString = "DataBad";
		break;

		case EquipmentStatusType::DataGood:
		ResultString = "DataGood";
		break;

	default:
		ResultString = "StatusUnknown";
	}

	return ResultString;
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
