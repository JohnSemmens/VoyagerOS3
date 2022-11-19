// Wingsail.h

#ifndef _WINGSAIL_h
#define _WINGSAIL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum WingSailTackType {
	wsHeadToWind,
	wsPortTack,
	wsStarboardTack
};

enum WingSailStateType {
	wsIdle,
	wsForward,
	wsReverse
};

struct WingSailType {
	int Angle;
	int TrimTabAngle;
	WingSailStateType State;
	int Servo_microseconds;
	int Servo_microseconds_reponse;
	WingSailTackType Tack;

	uint32_t LastCommandTime;  // ms
	uint32_t LastRequestTime;  // ms
	uint32_t LastResponseTime; // ms
	uint32_t LastPowerResponseTime; // ms

	long TimeSinceLastCommand; //s
	long TimeSinceLastRequest; //s
	long TimeSinceLastReponse; //s
	long TimeSinceLastPowerReponse; //s

	float SolarCell_V;
	float SolarCell_mA;
	float Charge_V;
	float Charge_mA;
	float Discharge_V;
	float Discharge_mA;

	//char Version[30];
	char VersionDate[30];
};

void wingsail_init(void);

void wingsail_update(void);
void AutoSetWingSail(WingSailStateType WingSailState);
void SetTrimTabAngle(int TrimTabAngle);

int CalcTrimTabAngle(int WingSailAngle, WingSailStateType WingSailState);

int TrimTabAngle_to_us(int TrimTabAngle);

void WingSailServo(int SailIn_us);

void IdentifyCurrentTack(int WingSailAngle);

void Wingsail_TrackTackChange(void);

void CheckWingSailServo(void);

void CheckWingSailPower(void);
void CheckWingSailVersion(void);

#endif

