// DisplayStrings.h

#ifndef _DISPLAYSTRINGS_h
#define _DISPLAYSTRINGS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

	#include "Mission.h"
	#include "CommandState_Processor.h"
	#include "Navigation.h"
	#include "HAL.h"

// enumerated type for Decision Events for the purpose of logging
enum DecisionEventType {
	deIncrementMissionIndex,
	dePastWaypoint,
	deEndOfMission,
	deTackToPort,
	deTackToStarboard,
	deTackToPortRunning,
	deTackToStarboardRunning,
	deChangeCommandState,
	deHoldCourse
};

enum DecisionEventReasonType {
	rPastWaypoint,
	rPastTime,
	rPastDuration,
	rPastBoundary,
	rFavouredTack,
	rNoChange,
	rLimitToSailingCourse,
	rPastLoiterBoundary,
	rUnkown,
	rNone,
	rManualIntervention,
	rApproachingWP
};


	String CommandStateToString(VesselCommandStateType CommandState);
	String DecisionEventToString(DecisionEventType DecisionEvent);
	String DecisionEventReasonToString(DecisionEventReasonType DecisionEventReason);
	String CourseTypeToString(SteeringCourseType CourseType);
	String CourseTypeToString(ManoeuvreStateType ManoeuvreState);

	String GetMissionCommandString(MissionCommandType cmd);
	String GetEquipmentStatusString(EquipmentStatusType status);

#endif

